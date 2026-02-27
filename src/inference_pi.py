#!/usr/bin/env python3
"""
inference_pi.py — Real-Time Emergency Sound Detection for Raspberry Pi

Runs continuously, capturing 3-second audio windows from a USB microphone,
extracting Mel Spectrograms with pure NumPy/SciPy, and classifying them
using a quantized TFLite model.

Dependencies (Pi-only):
    tflite-runtime, sounddevice, numpy, scipy

Usage:
    python3 src/inference_pi.py
    python3 src/inference_pi.py --model models/emergency_detector.tflite --threshold 0.80

Author:  Emergency Sound Detection Project
License: MIT
"""

from __future__ import annotations

import argparse
import logging
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import sounddevice as sd
from scipy.fftpack import dct

# Attempt tflite_runtime first; fall back to tf.lite for dev machines
try:
    from tflite_runtime.interpreter import Interpreter  # type: ignore[import]
except ImportError:
    try:
        from tensorflow.lite.python.interpreter import Interpreter  # type: ignore[import]
    except ImportError:
        logging.error(
            "Neither tflite_runtime nor tensorflow is installed. "
            "Install tflite-runtime on Pi or tensorflow on PC."
        )
        sys.exit(1)


# ---------------------------------------------------------------------------
# Constants — MUST match train_model.py exactly
# ---------------------------------------------------------------------------
SAMPLE_RATE: int = 16_000
DURATION: float = 3.0                       # 3 seconds for siren context
NUM_SAMPLES: int = int(SAMPLE_RATE * DURATION)  # 48 000

N_MELS: int = 128           # Mel frequency bands
N_FFT: int = 1024           # FFT window size
HOP_LENGTH: int = 256       # Hop between STFT frames

# Inference parameters
DEFAULT_THRESHOLD: float = 0.80
CONSECUTIVE_HITS: int = 2           # consecutive frames above threshold
COOLDOWN_SECONDS: float = 3.0       # seconds between triggers

PROJECT_ROOT: Path = Path(__file__).resolve().parent.parent
DEFAULT_MODEL: Path = PROJECT_ROOT / "models" / "emergency_detector.tflite"
DEFAULT_LABELS: Path = PROJECT_ROOT / "models" / "class_names.txt"

# Logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)


# ===================================================================
# PURE NUMPY / SCIPY  MEL SPECTROGRAM EXTRACTION
# (No librosa, no tensorflow — keeps the Pi lean)
# ===================================================================

def _hz_to_mel(hz: float) -> float:
    """Convert frequency in Hz to Mel scale."""
    return 2595.0 * np.log10(1.0 + hz / 700.0)


def _mel_to_hz(mel: float) -> float:
    """Convert Mel scale to Hz."""
    return 700.0 * (10.0 ** (mel / 2595.0) - 1.0)


def _create_mel_filterbank(
    num_filters: int,
    fft_size: int,
    sample_rate: int,
) -> np.ndarray:
    """
    Create a Mel-spaced triangular filterbank.

    Returns:
        np.ndarray of shape (num_filters, fft_size // 2 + 1)
    """
    low_mel = _hz_to_mel(0)
    high_mel = _hz_to_mel(sample_rate / 2.0)
    mel_points = np.linspace(low_mel, high_mel, num_filters + 2)
    hz_points = np.array([_mel_to_hz(m) for m in mel_points])
    bin_points = np.floor((fft_size + 1) * hz_points / sample_rate).astype(int)

    num_bins = fft_size // 2 + 1
    filterbank = np.zeros((num_filters, num_bins), dtype=np.float32)

    for i in range(num_filters):
        left = bin_points[i]
        center = bin_points[i + 1]
        right = bin_points[i + 2]

        # Rising slope
        for j in range(left, center):
            if center != left:
                filterbank[i, j] = (j - left) / (center - left)
        # Falling slope
        for j in range(center, right):
            if right != center:
                filterbank[i, j] = (right - j) / (right - center)

    return filterbank


# Pre-compute filterbank (module-level, computed once)
_MEL_FILTERBANK: Optional[np.ndarray] = None


def _get_mel_filterbank() -> np.ndarray:
    """Lazy-initialise and cache the Mel filterbank."""
    global _MEL_FILTERBANK
    if _MEL_FILTERBANK is None:
        _MEL_FILTERBANK = _create_mel_filterbank(N_MELS, N_FFT, SAMPLE_RATE)
    return _MEL_FILTERBANK


def extract_mel_spectrogram_numpy(audio: np.ndarray) -> np.ndarray:
    """
    Extract log-Mel Spectrogram using pure NumPy and SciPy.

    Replicates the librosa logic used during training:
        1. STFT with Hann window
        2. Power spectrum
        3. Apply Mel filterbank (128 bands)
        4. Log compression (power_to_db equivalent)

    Args:
        audio: 1-D float32 array of length NUM_SAMPLES.

    Returns:
        np.ndarray of shape (N_MELS, T) where T ≈ 188.
    """
    # Ensure float32
    if audio.dtype != np.float32:
        audio = audio.astype(np.float32)

    # Frame the signal
    num_frames = 1 + (len(audio) - N_FFT) // HOP_LENGTH
    indices = (
        np.tile(np.arange(N_FFT), (num_frames, 1))
        + np.tile(np.arange(num_frames) * HOP_LENGTH, (N_FFT, 1)).T
    )
    frames = audio[indices]

    # Apply Hann window
    window = np.hanning(N_FFT).astype(np.float32)
    frames *= window

    # FFT → power spectrum
    fft_result = np.fft.rfft(frames, n=N_FFT)
    power_spectrum = (np.abs(fft_result) ** 2) / N_FFT

    # Apply Mel filterbank
    mel_fb = _get_mel_filterbank()
    mel_energies = np.dot(power_spectrum, mel_fb.T)

    # Log compression (power_to_db equivalent)
    mel_energies = np.where(mel_energies == 0, np.finfo(float).eps, mel_energies)
    log_mel = 10.0 * np.log10(mel_energies)

    # Normalize to match librosa.power_to_db(ref=np.max)
    log_mel -= np.max(log_mel)

    # Transpose to (N_MELS, T) to match librosa convention
    return log_mel.T.astype(np.float32)


# ===================================================================
# AUDIO INPUT STREAM (NON-BLOCKING ROLLING BUFFER)
# ===================================================================

class AudioRingBuffer:
    """Thread-safe rolling audio buffer fed by sounddevice.InputStream."""

    def __init__(self, buffer_samples: int = NUM_SAMPLES) -> None:
        self._buffer = np.zeros(buffer_samples, dtype=np.float32)
        self._lock = threading.Lock()
        self._ready = False

    def callback(self, indata: np.ndarray, frames: int, time_info: object, status: object) -> None:
        """Called by sounddevice on each audio block."""
        if status:
            logger.warning("Audio stream status: %s", status)
        mono = indata[:, 0] if indata.ndim > 1 else indata.flatten()
        with self._lock:
            shift = len(mono)
            self._buffer = np.roll(self._buffer, -shift)
            self._buffer[-shift:] = mono
            self._ready = True

    def get_audio(self) -> Optional[np.ndarray]:
        """Return a copy of the current 3-second buffer (or None if not ready)."""
        with self._lock:
            if not self._ready:
                return None
            return self._buffer.copy()


# ===================================================================
# TFLITE INTERPRETER WRAPPER
# ===================================================================

class EmergencyDetector:
    """Wraps a quantized TFLite model for emergency sound inference."""

    def __init__(self, model_path: str, class_names: List[str]) -> None:
        logger.info("Loading TFLite model: %s", model_path)
        self._interpreter = Interpreter(model_path=model_path)
        self._interpreter.allocate_tensors()

        self._input_details = self._interpreter.get_input_details()
        self._output_details = self._interpreter.get_output_details()
        self._class_names = class_names

        inp = self._input_details[0]
        out = self._output_details[0]
        logger.info("  Input  : shape=%s  dtype=%s", inp["shape"], inp["dtype"])
        logger.info("  Output : shape=%s  dtype=%s", out["shape"], out["dtype"])

        # Quantization parameters
        self._input_scale = inp["quantization"][0] if inp["quantization"][0] != 0 else 1.0
        self._input_zero_point = inp["quantization"][1]
        self._output_scale = out["quantization"][0] if out["quantization"][0] != 0 else 1.0
        self._output_zero_point = out["quantization"][1]

        logger.info("  Input  quant: scale=%.6f  zp=%d", self._input_scale, self._input_zero_point)
        logger.info("  Output quant: scale=%.6f  zp=%d", self._output_scale, self._output_zero_point)

    def predict(self, mel_spec: np.ndarray) -> Tuple[str, float, np.ndarray]:
        """
        Run inference on a Mel Spectrogram.

        Args:
            mel_spec: np.ndarray of shape (N_MELS, T).

        Returns:
            (predicted_class_name, confidence, all_probabilities)
        """
        inp_detail = self._input_details[0]
        expected_shape = inp_detail["shape"]  # e.g. [1, 128, 188, 1]

        # Reshape to match model input
        mel_input = mel_spec.copy()

        # Pad / truncate time axis to match expected shape
        target_t = expected_shape[2]
        current_t = mel_input.shape[1]
        if current_t < target_t:
            mel_input = np.pad(mel_input, ((0, 0), (0, target_t - current_t)))
        elif current_t > target_t:
            mel_input = mel_input[:, :target_t]

        mel_input = mel_input.reshape(expected_shape).astype(np.float32)

        # Quantize input if model expects uint8
        if inp_detail["dtype"] == np.uint8:
            mel_input = (mel_input / self._input_scale + self._input_zero_point)
            mel_input = np.clip(mel_input, 0, 255).astype(np.uint8)
        elif inp_detail["dtype"] == np.int8:
            mel_input = (mel_input / self._input_scale + self._input_zero_point)
            mel_input = np.clip(mel_input, -128, 127).astype(np.int8)

        self._interpreter.set_tensor(inp_detail["index"], mel_input)
        self._interpreter.invoke()

        output = self._interpreter.get_tensor(self._output_details[0]["index"])

        # Dequantize output if needed
        if self._output_details[0]["dtype"] in (np.uint8, np.int8):
            output = (output.astype(np.float32) - self._output_zero_point) * self._output_scale

        probabilities = output.flatten()
        # Apply softmax if raw logits
        if np.any(probabilities < 0) or np.sum(probabilities) < 0.5:
            exp_p = np.exp(probabilities - np.max(probabilities))
            probabilities = exp_p / exp_p.sum()

        predicted_idx = int(np.argmax(probabilities))
        confidence = float(probabilities[predicted_idx])
        class_name = self._class_names[predicted_idx] if predicted_idx < len(self._class_names) else f"class_{predicted_idx}"

        return class_name, confidence, probabilities


# ===================================================================
# MAIN INFERENCE LOOP
# ===================================================================

def load_class_names(path: Path) -> List[str]:
    """Load class names from a text file."""
    if not path.exists():
        logger.error("Class names file not found: %s", path)
        sys.exit(1)
    names = [line.strip() for line in path.read_text().splitlines() if line.strip()]
    logger.info("Loaded %d class names: %s", len(names), names)
    return names


def run_inference_loop(
    model_path: str,
    class_names: List[str],
    emergency_label: str,
    threshold: float = DEFAULT_THRESHOLD,
    cooldown: float = COOLDOWN_SECONDS,
    consecutive: int = CONSECUTIVE_HITS,
) -> None:
    """
    Infinite inference loop — records, extracts Mel Spectrograms, classifies.

    Args:
        model_path:      Path to the .tflite model.
        class_names:     List of class label strings.
        emergency_label: Which class label triggers the emergency event.
        threshold:       Minimum confidence to count as a detection.
        cooldown:        Seconds between trigger events.
        consecutive:     Number of consecutive detections required.
    """
    detector = EmergencyDetector(model_path, class_names)
    ring = AudioRingBuffer(NUM_SAMPLES)

    logger.info("Starting audio stream @ %d Hz …", SAMPLE_RATE)
    stream = sd.InputStream(
        samplerate=SAMPLE_RATE,
        channels=1,
        dtype="float32",
        blocksize=int(SAMPLE_RATE * 0.5),  # 500 ms blocks
        callback=ring.callback,
    )

    hit_count = 0
    last_trigger_time = 0.0
    inference_count = 0

    print("\n" + "=" * 60)
    print("  🚨  Emergency Sound Detector — LISTENING")
    print(f"  🎯  Target label : {emergency_label}")
    print(f"  📊  Threshold    : {threshold:.0%}")
    print(f"  🔁  Consecutive  : {consecutive} frames")
    print(f"  ⏱️   Cooldown     : {cooldown} s")
    print(f"  🎤  Audio window : {DURATION} s")
    print("  🛑  Press Ctrl+C to stop")
    print("=" * 60 + "\n")

    try:
        stream.start()
        # Wait for buffer to fill
        time.sleep(DURATION + 0.5)

        while True:
            audio = ring.get_audio()
            if audio is None:
                time.sleep(0.1)
                continue

            # Extract Mel Spectrogram (pure NumPy)
            try:
                mel_spec = extract_mel_spectrogram_numpy(audio)
            except Exception as exc:
                logger.warning("Feature extraction failed: %s", exc)
                time.sleep(0.2)
                continue

            # Classify
            label, confidence, probs = detector.predict(mel_spec)
            inference_count += 1

            # Status display (every 2nd inference to keep console calm)
            if inference_count % 2 == 0:
                probs_str = "  ".join(
                    f"{class_names[i]}:{probs[i]:.2f}"
                    for i in range(len(class_names))
                )
                sys.stdout.write(f"\r  [{probs_str}]   ")
                sys.stdout.flush()

            # Emergency detection logic
            if label == emergency_label and confidence >= threshold:
                hit_count += 1
            else:
                hit_count = max(0, hit_count - 1)  # decay slowly

            now = time.time()
            if hit_count >= consecutive and (now - last_trigger_time) > cooldown:
                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                print(
                    f"\n\n  🚨🚑 EMERGENCY SOUND DETECTED!  "
                    f"[{timestamp}]  confidence={confidence:.1%}\n"
                )
                hit_count = 0
                last_trigger_time = now

            # Sleep between inferences (3s window, check every 1s)
            time.sleep(1.0)

    except KeyboardInterrupt:
        logger.info("Stopping …")
    finally:
        stream.stop()
        stream.close()
        logger.info("Audio stream closed. Total inferences: %d", inference_count)


# ===================================================================
# CLI
# ===================================================================

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Real-time emergency sound detection on Raspberry Pi.",
    )
    parser.add_argument(
        "--model", type=str, default=str(DEFAULT_MODEL),
        help="Path to .tflite model file.",
    )
    parser.add_argument(
        "--labels", type=str, default=str(DEFAULT_LABELS),
        help="Path to class_names.txt.",
    )
    parser.add_argument(
        "--emergency-label", type=str, default="emergency",
        help="Class label for emergency sounds (default: 'emergency').",
    )
    parser.add_argument(
        "--threshold", type=float, default=DEFAULT_THRESHOLD,
        help="Detection confidence threshold (0-1).",
    )
    parser.add_argument(
        "--cooldown", type=float, default=COOLDOWN_SECONDS,
        help="Seconds between trigger events.",
    )
    parser.add_argument(
        "--consecutive", type=int, default=CONSECUTIVE_HITS,
        help="Consecutive detections required before triggering.",
    )
    parser.add_argument(
        "--list-devices", action="store_true",
        help="List audio devices and exit.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.list_devices:
        print(sd.query_devices())
        return

    class_names = load_class_names(Path(args.labels))

    emergency_label = args.emergency_label
    if emergency_label not in class_names:
        # Try to auto-detect
        candidates = [c for c in class_names if "emergency" in c.lower() or "siren" in c.lower()]
        if candidates:
            emergency_label = candidates[0]
            logger.info("Auto-detected emergency label: '%s'", emergency_label)
        else:
            logger.error(
                "Emergency label '%s' not in class names %s",
                emergency_label, class_names,
            )
            sys.exit(1)

    run_inference_loop(
        model_path=args.model,
        class_names=class_names,
        emergency_label=emergency_label,
        threshold=args.threshold,
        cooldown=args.cooldown,
        consecutive=args.consecutive,
    )


if __name__ == "__main__":
    main()
