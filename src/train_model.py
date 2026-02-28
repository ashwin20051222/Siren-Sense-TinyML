#!/usr/bin/env python3
"""
train_model.py — Training Pipeline for Emergency Sound Detection

Loads audio from dataset/, extracts Mel Spectrograms, trains a
DepthwiseSeparable CNN, and exports an INT8-quantized TFLite model.

Dataset expected layout:
    dataset/
        emergency/          # ambulance, fire truck, police sirens, etc.
        non_emergency/      # traffic, birds, rain, speech, etc.

Usage:
    python3 src/train_model.py
    python3 src/train_model.py --epochs 50 --batch-size 32

Author:  Emergency Sound Detection Project
License: MIT
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
from pathlib import Path
from typing import List, Optional, Tuple

# Force TensorFlow to use legacy Keras 2 (tf-keras) — required for
# TFLite conversion and stable training with TF 2.16+/2.20+
os.environ["TF_USE_LEGACY_KERAS"] = "1"
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"  # Suppress TF info logs

import librosa
import matplotlib
matplotlib.use("Agg")  # Non-interactive backend — must be set before pyplot import
import matplotlib.pyplot as plt
import numpy as np
from scipy.io import wavfile
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report, confusion_matrix

import tensorflow as tf
import tf_keras as keras
from tf_keras import layers

# ---------------------------------------------------------------------------
# Constants  (Feature parameters — keep identical to inference_pi.py!)
# ---------------------------------------------------------------------------
SAMPLE_RATE: int = 16_000
DURATION: float = 3.0                       # 3 seconds for siren context
NUM_SAMPLES: int = int(SAMPLE_RATE * DURATION)  # 48 000

N_MELS: int = 128           # Mel frequency bands
N_FFT: int = 1024           # FFT window size (larger for better freq resolution)
HOP_LENGTH: int = 256       # Hop between STFT frames
# Expected output shape: (128, ceil(48000/256) + 1) ≈ (128, 188)

# Legacy MFCC params (still used for supplementary features)
N_MFCC: int = 13

PROJECT_ROOT: Path = Path(__file__).resolve().parent.parent
DATASET_DIR: Path = PROJECT_ROOT / "dataset"
MODELS_DIR: Path = PROJECT_ROOT / "models"

# Logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)


# ===================================================================
# 1. DATA LOADING
# ===================================================================

def load_dataset(
    dataset_dir: Path,
) -> Tuple[np.ndarray, np.ndarray, List[str]]:
    """
    Walk dataset/<label>/ directories and load WAV files.

    Returns:
        audio_data:   np.ndarray of shape (N, NUM_SAMPLES) float32
        labels:       np.ndarray of shape (N,) int
        class_names:  sorted list of label strings
    """
    class_names = sorted(
        d.name for d in dataset_dir.iterdir() if d.is_dir() and not d.name.startswith(".")
    )
    if not class_names:
        logger.error("No class directories found in %s", dataset_dir)
        sys.exit(1)

    logger.info("Found %d classes: %s", len(class_names), class_names)
    label_to_idx = {name: idx for idx, name in enumerate(class_names)}

    audio_data: List[np.ndarray] = []
    labels: List[int] = []
    skipped = 0

    for class_name in class_names:
        class_dir = dataset_dir / class_name
        wav_files = sorted(
            list(class_dir.glob("*.wav"))
            + list(class_dir.glob("*.WAV"))
            + list(class_dir.glob("*.mp3"))
        )
        logger.info("  %-20s : %d files", class_name, len(wav_files))

        for wav_path in wav_files:
            try:
                # Use librosa for robust loading (handles varied formats/sample rates)
                audio, sr = librosa.load(str(wav_path), sr=SAMPLE_RATE, mono=True)

                # Pad or truncate to exactly DURATION seconds
                if len(audio) < NUM_SAMPLES:
                    audio = np.pad(audio, (0, NUM_SAMPLES - len(audio)))
                else:
                    audio = audio[:NUM_SAMPLES]

                audio_data.append(audio.astype(np.float32))
                labels.append(label_to_idx[class_name])

            except Exception as exc:
                logger.warning("Skipping %s: %s", wav_path.name, exc)
                skipped += 1

    if skipped:
        logger.warning("Skipped %d corrupted/unreadable files.", skipped)

    logger.info("Loaded %d samples total.", len(audio_data))
    return np.array(audio_data, dtype=np.float32), np.array(labels), class_names


# ===================================================================
# 2. FEATURE EXTRACTION (MEL SPECTROGRAM)
# ===================================================================

def extract_mel_spectrogram(audio: np.ndarray) -> np.ndarray:
    """
    Convert audio waveform to a log-Mel Spectrogram.

    Args:
        audio: np.ndarray of shape (NUM_SAMPLES,) float32 in [-1, 1].

    Returns:
        mel_spec: np.ndarray of shape (N_MELS, T) where T ≈ 188.
    """
    mel_spec = librosa.feature.melspectrogram(
        y=audio,
        sr=SAMPLE_RATE,
        n_mels=N_MELS,
        n_fft=N_FFT,
        hop_length=HOP_LENGTH,
        fmax=SAMPLE_RATE // 2,
    )
    # Convert to log scale (dB)
    log_mel = librosa.power_to_db(mel_spec, ref=np.max)
    return log_mel


def extract_all_features(audio_data: np.ndarray) -> np.ndarray:
    """Extract Mel Spectrograms for an entire dataset.

    Returns:
        np.ndarray of shape (N, N_MELS, T, 1) ready for Conv2D input.
    """
    features = []
    for i, audio in enumerate(audio_data):
        mel = extract_mel_spectrogram(audio)
        features.append(mel)
        if (i + 1) % 100 == 0:
            logger.info("  Extracted features: %d / %d", i + 1, len(audio_data))

    features_arr = np.array(features, dtype=np.float32)

    # ---- Normalize per-sample to zero-mean / unit-variance ----
    # Raw log-mel dB values (range ~[-80, 0]) cause training to collapse.
    for i in range(len(features_arr)):
        m = features_arr[i].mean()
        s = features_arr[i].std()
        if s > 1e-6:
            features_arr[i] = (features_arr[i] - m) / s
        else:
            features_arr[i] = features_arr[i] - m
    logger.info("Features normalised (per-sample zero-mean / unit-var).")

    # Add channel dimension → (N, N_MELS, T, 1)
    features_arr = features_arr[..., np.newaxis]
    logger.info("Feature array shape: %s", features_arr.shape)
    return features_arr


# ===================================================================
# 3. DATA AUGMENTATION
# ===================================================================

def augment_white_noise(
    audio: np.ndarray, noise_factor: float = 0.005
) -> np.ndarray:
    """Add slight white noise to an audio waveform."""
    noise = np.random.randn(*audio.shape).astype(np.float32) * noise_factor
    return audio + noise


def augment_time_shift(
    audio: np.ndarray, max_shift: int = 4800
) -> np.ndarray:
    """Randomly shift audio left or right (circular). Default ~300ms at 16kHz."""
    shift = np.random.randint(-max_shift, max_shift)
    return np.roll(audio, shift)


def augment_pitch_shift(
    audio: np.ndarray, sr: int = SAMPLE_RATE
) -> np.ndarray:
    """Randomly pitch-shift the audio by ±2 semitones."""
    n_steps = np.random.uniform(-2.0, 2.0)
    return librosa.effects.pitch_shift(y=audio, sr=sr, n_steps=n_steps)


def augment_time_stretch(
    audio: np.ndarray, rate: Optional[float] = None
) -> np.ndarray:
    """Randomly time-stretch the audio (0.8x to 1.2x speed)."""
    if rate is None:
        rate = np.random.uniform(0.8, 1.2)
    stretched = librosa.effects.time_stretch(y=audio, rate=rate)
    # Pad or truncate to original length
    if len(stretched) < NUM_SAMPLES:
        stretched = np.pad(stretched, (0, NUM_SAMPLES - len(stretched)))
    else:
        stretched = stretched[:NUM_SAMPLES]
    return stretched.astype(np.float32)


def augment_volume(
    audio: np.ndarray, gain_db_range: float = 6.0
) -> np.ndarray:
    """Randomly change volume by ±gain_db_range dB."""
    gain_db = np.random.uniform(-gain_db_range, gain_db_range)
    gain = 10.0 ** (gain_db / 20.0)
    return (audio * gain).astype(np.float32)


def spec_augment(
    mel_spec: np.ndarray,
    num_freq_masks: int = 2,
    freq_mask_width: int = 10,
    num_time_masks: int = 2,
    time_mask_width: int = 15,
) -> np.ndarray:
    """Apply SpecAugment (frequency + time masking) to a Mel spectrogram.

    Args:
        mel_spec: shape (N_MELS, T, 1) or (N_MELS, T).
    Returns:
        Masked spectrogram of the same shape.
    """
    spec = mel_spec.copy()
    squeeze = False
    if spec.ndim == 3:
        squeeze = True
        spec = spec[:, :, 0]

    n_mels, t_frames = spec.shape

    # Frequency masks
    for _ in range(num_freq_masks):
        f = np.random.randint(0, min(freq_mask_width, n_mels))
        f0 = np.random.randint(0, max(1, n_mels - f))
        spec[f0 : f0 + f, :] = 0.0

    # Time masks
    for _ in range(num_time_masks):
        t = np.random.randint(0, min(time_mask_width, t_frames))
        t0 = np.random.randint(0, max(1, t_frames - t))
        spec[:, t0 : t0 + t] = 0.0

    if squeeze:
        spec = spec[..., np.newaxis]
    return spec


def spec_augment_features(
    features: np.ndarray, labels: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """Apply SpecAugment to all feature spectrograms (in-place copy).

    Generates one SpecAugment-ed copy per sample and appends to the dataset.
    """
    logger.info("Applying SpecAugment to features …")
    aug_features = []
    for i in range(len(features)):
        aug_features.append(spec_augment(features[i]))
    aug_features = np.array(aug_features, dtype=np.float32)
    return (
        np.concatenate([features, aug_features], axis=0),
        np.concatenate([labels, labels], axis=0),
    )


def mixup_batch(
    X: np.ndarray, y: np.ndarray, alpha: float = 0.2
) -> Tuple[np.ndarray, np.ndarray]:
    """Create a mixup-augmented copy of the dataset.

    Blends random pairs with a Beta-distributed weight.
    Returns float labels suitable for categorical cross-entropy.
    """
    logger.info("Applying Mixup augmentation (alpha=%.2f) …", alpha)
    n = len(X)
    indices = np.random.permutation(n)
    lam = np.random.beta(alpha, alpha, size=n).astype(np.float32)

    # Broadcast lambda for feature dimensions
    lam_x = lam.reshape(-1, 1, 1, 1) if X.ndim == 4 else lam.reshape(-1, 1)

    X_mix = (lam_x * X + (1 - lam_x) * X[indices]).astype(np.float32)
    y_mix = (lam * y + (1 - lam) * y[indices]).astype(np.float32)
    return X_mix, y_mix


def augment_dataset(
    audio_data: np.ndarray,
    labels: np.ndarray,
    augment_factor: int = 5,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Apply augmentation and return an expanded dataset.

    Uses noise, time shift, pitch shift, time stretch, and volume
    perturbation.

    Args:
        audio_data:     shape (N, NUM_SAMPLES)
        labels:         shape (N,)
        augment_factor: number of augmented copies per original sample

    Returns:
        augmented audio and labels (original + augmented)
    """
    logger.info("Augmenting dataset (factor=%d) …", augment_factor)
    aug_audio: List[np.ndarray] = []
    aug_labels: List[int] = []

    augmenters = [
        augment_white_noise,
        augment_time_shift,
        augment_pitch_shift,
        augment_time_stretch,
        augment_volume,
    ]

    for i in range(len(audio_data)):
        for _ in range(augment_factor):
            sample = audio_data[i].copy()
            # Apply 1-3 random augmentations
            n_aug = np.random.randint(1, 4)
            chosen = np.random.choice(len(augmenters), size=n_aug, replace=False)
            for idx in chosen:
                try:
                    sample = augmenters[idx](sample)
                except Exception:
                    pass  # Skip if augmentation fails
            aug_audio.append(sample)
            aug_labels.append(labels[i])

        if (i + 1) % 100 == 0:
            logger.info("  Augmented: %d / %d", i + 1, len(audio_data))

    combined_audio = np.concatenate(
        [audio_data, np.array(aug_audio, dtype=np.float32)], axis=0
    )
    combined_labels = np.concatenate(
        [labels, np.array(aug_labels)], axis=0
    )
    logger.info(
        "Dataset after augmentation: %d samples (was %d).",
        len(combined_labels), len(labels),
    )
    return combined_audio, combined_labels


# ===================================================================
# 4. MODEL ARCHITECTURE
# ===================================================================

def build_model(num_classes: int, input_shape: Optional[Tuple[int, ...]] = None) -> keras.Model:
    """
    Build a deeper CNN for emergency sound detection.

    7 DepthwiseSeparable Conv blocks on Mel Spectrograms.
    Target: < 1.5M trainable parameters for enhanced accuracy.

    Args:
        num_classes:  Number of output classes (typically 2: emergency / non_emergency).
        input_shape:  Shape of one Mel Spectrogram (N_MELS, T, 1).

    Returns:
        Compiled Keras model.
    """
    if input_shape is None:
        # Compute T dynamically
        dummy = np.zeros(NUM_SAMPLES, dtype=np.float32)
        t_frames = extract_mel_spectrogram(dummy).shape[1]
        input_shape = (N_MELS, t_frames, 1)

    logger.info("Model input shape: %s", input_shape)

    model = keras.Sequential([
        layers.Input(shape=input_shape),

        # Block 1: Depthwise Separable Conv — 64 filters
        layers.DepthwiseConv2D(
            kernel_size=(3, 3), padding="same", activation=None,
            depthwise_initializer="he_normal",
        ),
        layers.BatchNormalization(),
        layers.ReLU(),
        layers.Conv2D(64, (1, 1), padding="same", kernel_initializer="he_normal"),
        layers.BatchNormalization(),
        layers.ReLU(),
        layers.MaxPooling2D(pool_size=(2, 2)),
        layers.Dropout(0.25),

        # Block 2: Depthwise Separable Conv — 128 filters
        layers.DepthwiseConv2D(
            kernel_size=(3, 3), padding="same", activation=None,
            depthwise_initializer="he_normal",
        ),
        layers.BatchNormalization(),
        layers.ReLU(),
        layers.Conv2D(128, (1, 1), padding="same", kernel_initializer="he_normal"),
        layers.BatchNormalization(),
        layers.ReLU(),
        layers.MaxPooling2D(pool_size=(2, 2)),
        layers.Dropout(0.25),

        # Block 3: Depthwise Separable Conv — 256 filters
        layers.DepthwiseConv2D(
            kernel_size=(3, 3), padding="same", activation=None,
            depthwise_initializer="he_normal",
        ),
        layers.BatchNormalization(),
        layers.ReLU(),
        layers.Conv2D(256, (1, 1), padding="same", kernel_initializer="he_normal"),
        layers.BatchNormalization(),
        layers.ReLU(),
        layers.MaxPooling2D(pool_size=(2, 2)),
        layers.Dropout(0.3),

        # Block 4: Depthwise Separable Conv — 256 filters
        layers.DepthwiseConv2D(
            kernel_size=(3, 3), padding="same", activation=None,
            depthwise_initializer="he_normal",
        ),
        layers.BatchNormalization(),
        layers.ReLU(),
        layers.Conv2D(256, (1, 1), padding="same", kernel_initializer="he_normal"),
        layers.BatchNormalization(),
        layers.ReLU(),
        layers.MaxPooling2D(pool_size=(2, 2)),
        layers.Dropout(0.3),

        # Block 5: Depthwise Separable Conv — 512 filters
        layers.DepthwiseConv2D(
            kernel_size=(3, 3), padding="same", activation=None,
            depthwise_initializer="he_normal",
        ),
        layers.BatchNormalization(),
        layers.ReLU(),
        layers.Conv2D(512, (1, 1), padding="same", kernel_initializer="he_normal"),
        layers.BatchNormalization(),
        layers.ReLU(),
        layers.MaxPooling2D(pool_size=(2, 2)),
        layers.Dropout(0.35),

        # Block 6: Depthwise Separable Conv — 512 filters (NEW)
        layers.DepthwiseConv2D(
            kernel_size=(3, 3), padding="same", activation=None,
            depthwise_initializer="he_normal",
        ),
        layers.BatchNormalization(),
        layers.ReLU(),
        layers.Conv2D(512, (1, 1), padding="same", kernel_initializer="he_normal"),
        layers.BatchNormalization(),
        layers.ReLU(),
        layers.MaxPooling2D(pool_size=(2, 2)),
        layers.Dropout(0.35),

        # Block 7: Depthwise Separable Conv — 512 filters (NEW)
        layers.DepthwiseConv2D(
            kernel_size=(3, 3), padding="same", activation=None,
            depthwise_initializer="he_normal",
        ),
        layers.BatchNormalization(),
        layers.ReLU(),
        layers.Conv2D(512, (1, 1), padding="same", kernel_initializer="he_normal"),
        layers.BatchNormalization(),
        layers.ReLU(),
        layers.GlobalAveragePooling2D(),
        layers.Dropout(0.4),

        # Classifier head (wider)
        layers.Dense(512, activation="relu", kernel_initializer="he_normal"),
        layers.Dropout(0.5),
        layers.Dense(256, activation="relu", kernel_initializer="he_normal"),
        layers.Dropout(0.4),
        layers.Dense(128, activation="relu", kernel_initializer="he_normal"),
        layers.Dropout(0.3),
        layers.Dense(num_classes, activation="softmax"),
    ])

    model.compile(
        optimizer=keras.optimizers.Adam(learning_rate=1e-3),
        loss="sparse_categorical_crossentropy",
        metrics=["accuracy"],
    )

    total_params = model.count_params()
    logger.info("Total parameters: %d", total_params)
    if total_params > 1_500_000:
        logger.warning("⚠️  Model exceeds 1.5M parameter target!")
    else:
        logger.info("✅  Model is within 1.5M parameter budget.")

    model.summary(print_fn=logger.info)
    return model


# ===================================================================
# 5. TRAINING
# ===================================================================

def compute_class_weights(labels: np.ndarray) -> dict:
    """Compute class weights to handle imbalanced datasets."""
    from sklearn.utils.class_weight import compute_class_weight
    classes = np.unique(labels)
    weights = compute_class_weight("balanced", classes=classes, y=labels)
    cw = {int(c): float(w) for c, w in zip(classes, weights)}
    logger.info("Class weights: %s", cw)
    return cw


def train(
    model: keras.Model,
    X: np.ndarray,
    y: np.ndarray,
    class_names: List[str],
    epochs: int = 100,
    batch_size: int = 32,
) -> keras.callbacks.History:
    """
    Train with 80/10/10 split, cosine decay LR, and class weights.

    Returns:
        Training History object.
    """
    # Split: 80% train, 10% val, 10% test
    X_train, X_temp, y_train, y_temp = train_test_split(
        X, y, test_size=0.2, random_state=42, stratify=y,
    )
    X_val, X_test, y_val, y_test = train_test_split(
        X_temp, y_temp, test_size=0.5, random_state=42, stratify=y_temp,
    )
    logger.info("Train: %d | Val: %d | Test: %d", len(X_train), len(X_val), len(X_test))

    # Compute class weights for the imbalanced dataset
    class_weights = compute_class_weights(y_train)

    # Cosine decay learning rate schedule
    steps_per_epoch = max(1, len(X_train) // batch_size)
    total_steps = steps_per_epoch * epochs
    cosine_lr = keras.optimizers.schedules.CosineDecay(
        initial_learning_rate=1e-3,
        decay_steps=total_steps,
        alpha=1e-6,  # minimum learning rate
    )
    model.optimizer.learning_rate = cosine_lr
    logger.info("Using cosine decay LR: 1e-3 → 1e-6 over %d steps", total_steps)

    callbacks = [
        keras.callbacks.EarlyStopping(
            monitor="val_loss",
            patience=15,
            restore_best_weights=True,
            verbose=1,
        ),
    ]

    history = model.fit(
        X_train, y_train,
        validation_data=(X_val, y_val),
        epochs=epochs,
        batch_size=batch_size,
        callbacks=callbacks,
        class_weight=class_weights,
        verbose=1,
    )

    # Evaluate on held-out test set
    test_loss, test_acc = model.evaluate(X_test, y_test, verbose=0)
    logger.info("🧪 Test accuracy: %.2f%% | Test loss: %.4f", test_acc * 100, test_loss)

    # Detailed classification report
    y_pred = np.argmax(model.predict(X_test, verbose=0), axis=1)
    report = classification_report(y_test, y_pred, target_names=class_names)
    logger.info("Classification Report:\n%s", report)

    cm = confusion_matrix(y_test, y_pred)
    logger.info("Confusion Matrix:\n%s", cm)

    return history


def plot_training_curves(history: keras.callbacks.History, save_path: Path) -> None:
    """Save accuracy and loss curves as a PNG image."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

    # Accuracy
    ax1.plot(history.history["accuracy"], label="Train Accuracy", linewidth=2)
    ax1.plot(history.history["val_accuracy"], label="Val Accuracy", linewidth=2)
    ax1.set_title("Model Accuracy", fontsize=14, fontweight="bold")
    ax1.set_xlabel("Epoch")
    ax1.set_ylabel("Accuracy")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Loss
    ax2.plot(history.history["loss"], label="Train Loss", linewidth=2)
    ax2.plot(history.history["val_loss"], label="Val Loss", linewidth=2)
    ax2.set_title("Model Loss", fontsize=14, fontweight="bold")
    ax2.set_xlabel("Epoch")
    ax2.set_ylabel("Loss")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(str(save_path), dpi=150)
    plt.close()
    logger.info("📊 Training curves saved to: %s", save_path)


# ===================================================================
# 6. QUANTIZATION & TFLITE EXPORT
# ===================================================================

def export_tflite(
    model: keras.Model,
    X_repr: np.ndarray,
    output_path: Path,
) -> None:
    """
    Convert Keras model to INT8 quantized TFLite format.

    Uses SavedModel as intermediate format — more reliable than
    converting directly from the in-memory Keras object.

    Args:
        model:       Trained Keras model.
        X_repr:      Representative dataset for calibration (subset of training data).
        output_path: Where to save the .tflite file.
    """
    logger.info("Converting to TFLite with INT8 quantization …")

    # Save to SavedModel first (avoids Keras 3 vs 2 TFLite bugs)
    saved_model_dir = output_path.parent / "_saved_model_tmp"
    model.export(str(saved_model_dir))
    logger.info("Intermediate SavedModel written to %s", saved_model_dir)

    def representative_dataset_gen():
        """Yield samples for calibration."""
        indices = np.random.choice(len(X_repr), size=min(200, len(X_repr)), replace=False)
        for idx in indices:
            sample = X_repr[idx : idx + 1].astype(np.float32)
            yield [sample]

    converter = tf.lite.TFLiteConverter.from_saved_model(str(saved_model_dir))
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    converter.representative_dataset = representative_dataset_gen
    converter.target_spec.supported_ops = [
        tf.lite.OpsSet.TFLITE_BUILTINS_INT8,
        tf.lite.OpsSet.TFLITE_BUILTINS,  # fallback for unsupported ops
    ]
    converter.inference_input_type = tf.uint8
    converter.inference_output_type = tf.uint8

    tflite_model = converter.convert()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(tflite_model)

    # Clean up temp dir
    import shutil
    shutil.rmtree(str(saved_model_dir), ignore_errors=True)

    size_kb = output_path.stat().st_size / 1024
    logger.info("✅  TFLite model saved: %s (%.1f KB)", output_path, size_kb)
    if size_kb > 500:
        logger.warning("⚠️  Model exceeds 500 KB target.")
    else:
        logger.info("✅  Model is within 500 KB budget.")


def save_class_names(class_names: List[str], output_path: Path) -> None:
    """Save class names to a text file for use during inference."""
    output_path.write_text("\n".join(class_names) + "\n")
    logger.info("📝 Class names saved to: %s", output_path)


# ===================================================================
# 7. MAIN
# ===================================================================

def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description="Train an emergency sound detection model.",
    )
    parser.add_argument("--epochs", type=int, default=100, help="Max training epochs.")
    parser.add_argument("--batch-size", type=int, default=32, help="Training batch size.")
    parser.add_argument("--augment-factor", type=int, default=5, help="Augmentation multiplier.")
    parser.add_argument("--dataset-dir", type=str, default=None, help="Override dataset directory.")
    parser.add_argument("--no-augment", action="store_true", help="Skip data augmentation.")
    parser.add_argument("--no-specaugment", action="store_true", help="Skip SpecAugment on features.")
    parser.add_argument("--no-mixup", action="store_true", help="Skip Mixup augmentation.")
    return parser.parse_args()


def main() -> None:
    """Run the complete training pipeline."""
    args = parse_args()

    dataset_dir = Path(args.dataset_dir) if args.dataset_dir else DATASET_DIR
    MODELS_DIR.mkdir(parents=True, exist_ok=True)

    print("\n" + "=" * 60)
    print("  🚨 Emergency Sound Detection — Training Pipeline")
    print("=" * 60 + "\n")

    # ---- Step 1: Load data ----
    logger.info("Step 1/6: Loading dataset …")
    audio_data, labels, class_names = load_dataset(dataset_dir)

    if len(audio_data) < 10:
        logger.error("Too few samples (%d). Collect more data first.", len(audio_data))
        logger.error("Download dataset from:")
        logger.error("  https://www.kaggle.com/datasets/vishnu0399/emergency-vehicle-siren-sounds")
        logger.error("Place files in: %s/emergency/ and %s/non_emergency/", dataset_dir, dataset_dir)
        sys.exit(1)

    # ---- Step 2: Augment ----
    if not args.no_augment:
        logger.info("Step 2/6: Augmenting data …")
        audio_data, labels = augment_dataset(
            audio_data, labels, augment_factor=args.augment_factor,
        )
    else:
        logger.info("Step 2/6: Augmentation skipped.")

    # ---- Step 3: Extract features ----
    logger.info("Step 3/8: Extracting Mel Spectrograms …")
    X = extract_all_features(audio_data)
    y = labels

    # ---- Step 4: SpecAugment on features ----
    if not args.no_specaugment:
        logger.info("Step 4/8: Applying SpecAugment …")
        X, y = spec_augment_features(X, y)
    else:
        logger.info("Step 4/8: SpecAugment skipped.")

    # ---- Step 5: Mixup augmentation ----
    if not args.no_mixup:
        logger.info("Step 5/8: Applying Mixup augmentation …")
        X_mix, y_mix = mixup_batch(X, y.astype(np.float32))
        # For Mixup we round labels back to int since we use sparse CE
        y_mix = np.round(y_mix).astype(np.intp)
        X = np.concatenate([X, X_mix], axis=0)
        y = np.concatenate([y, y_mix], axis=0)
        logger.info("Dataset after Mixup: %d samples.", len(y))
    else:
        logger.info("Step 5/8: Mixup skipped.")

    # ---- Step 6: Build model ----
    logger.info("Step 6/8: Building model …")
    input_shape = X.shape[1:]  # (N_MELS, T, 1)
    model = build_model(num_classes=len(class_names), input_shape=input_shape)

    # ---- Step 7: Train ----
    logger.info("Step 7/8: Training …")
    history = train(model, X, y, class_names=class_names, epochs=args.epochs, batch_size=args.batch_size)

    # Save Keras model
    keras_path = MODELS_DIR / "emergency_detector.keras"
    model.save(str(keras_path))
    logger.info("💾 Keras model saved: %s", keras_path)

    # Plot training curves
    plot_training_curves(history, MODELS_DIR / "training_curves.png")

    # ---- Step 8: Export TFLite ----
    logger.info("Step 8/8: Exporting quantized TFLite model …")
    tflite_path = MODELS_DIR / "emergency_detector.tflite"
    export_tflite(model, X, tflite_path)

    # Save class names
    save_class_names(class_names, MODELS_DIR / "class_names.txt")

    print("\n" + "=" * 60)
    print("  ✅  Training pipeline complete!")
    print(f"  📁  Keras model  : {keras_path}")
    print(f"  📁  Class names  : {MODELS_DIR / 'class_names.txt'}")
    print(f"  📁  TFLite model : {tflite_path}")
    print(f"  📊  Curves       : {MODELS_DIR / 'training_curves.png'}")
    print("=" * 60 + "\n")


if __name__ == "__main__":
    main()
