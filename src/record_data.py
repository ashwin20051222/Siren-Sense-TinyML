#!/usr/bin/env python3
"""
record_data.py — CLI Data Collection Agent for Emergency Sound Detection

Records 3-second audio clips at 16 kHz mono and organizes them into
labelled directories under dataset/<label>/.

Usage:
    python3 src/record_data.py

Author:  Emergency Sound Detection Project
License: MIT
"""

from __future__ import annotations

import argparse
import logging
import os
import re
import sys
import tempfile
import time
from pathlib import Path
from typing import Optional

import numpy as np
import sounddevice as sd
from scipy.io import wavfile

# Optional: speech recognition for spoken label input
try:
    import speech_recognition as sr
    _HAS_SPEECH_RECOGNITION = True
except ImportError:
    _HAS_SPEECH_RECOGNITION = False

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
SAMPLE_RATE: int = 16_000          # 16 kHz — speech standard
DURATION: float = 3.0              # 3 seconds per clip (for siren context)
CHANNELS: int = 1                  # mono
DTYPE: str = "int16"               # 16-bit PCM
PROJECT_ROOT: Path = Path(__file__).resolve().parent.parent
DATASET_DIR: Path = PROJECT_ROOT / "dataset"

# Logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _ensure_directory(label: str) -> Path:
    """Create and return the dataset directory for the given label."""
    label_dir = DATASET_DIR / label
    label_dir.mkdir(parents=True, exist_ok=True)
    logger.info("Dataset directory: %s", label_dir)
    return label_dir


def _next_filename(label_dir: Path) -> Path:
    """Return the next zero-padded sequential filename (e.g. sample_0001.wav)."""
    existing = sorted(label_dir.glob("sample_*.wav"))
    next_index = len(existing) + 1
    return label_dir / f"sample_{next_index:04d}.wav"


def _print_progress_bar(
    current: float,
    total: float,
    bar_length: int = 40,
    label: str = "Recording",
) -> None:
    """Print a console progress bar."""
    fraction = min(current / total, 1.0)
    filled = int(bar_length * fraction)
    bar = "█" * filled + "░" * (bar_length - filled)
    pct = fraction * 100
    sys.stdout.write(f"\r  {label} |{bar}| {pct:5.1f}%")
    sys.stdout.flush()


# ---------------------------------------------------------------------------
# Speech-to-Text label input
# ---------------------------------------------------------------------------

SPEECH_LABEL_DURATION: float = 2.5  # seconds to record for label speech


def get_spoken_label(
    sample_rate: int = SAMPLE_RATE,
    duration: float = SPEECH_LABEL_DURATION,
) -> Optional[str]:
    """
    Record a short audio clip and transcribe it to text for use as a class label.

    Uses Google Web Speech API (free, no API key required) via the
    SpeechRecognition library.

    Args:
        sample_rate: Sampling rate in Hz.
        duration:    How long to record the spoken label.

    Returns:
        Sanitised label string, or None if transcription failed.
    """
    if not _HAS_SPEECH_RECOGNITION:
        logger.error(
            "SpeechRecognition is not installed. "
            "Install it with: pip install SpeechRecognition"
        )
        return None

    print("\n" + "─" * 50)
    print("  🗣️  SPEAK YOUR LABEL")
    print("  Say the label name clearly (e.g. 'kitchen lights')")
    print("─" * 50)

    logger.info("🎤  Recording for %.1f s — speak now!", duration)
    time.sleep(0.3)

    num_samples = int(sample_rate * duration)

    # Record audio
    print()
    audio_buffer = np.zeros(num_samples, dtype=np.float32)
    block_size = 1024
    blocks_recorded = 0
    total_blocks = int(np.ceil(num_samples / block_size))

    try:
        with sd.InputStream(
            samplerate=sample_rate,
            channels=CHANNELS,
            dtype="float32",
            blocksize=block_size,
        ) as stream:
            samples_left = num_samples
            while samples_left > 0:
                to_read = min(block_size, samples_left)
                data, _ = stream.read(to_read)
                start_idx = num_samples - samples_left
                audio_buffer[start_idx : start_idx + to_read] = data[:to_read, 0]
                samples_left -= to_read
                blocks_recorded += 1
                _print_progress_bar(
                    blocks_recorded, total_blocks, label="🗣️  LISTENING"
                )
        print()  # newline after progress bar
    except Exception as exc:
        logger.error("Failed to record speech: %s", exc)
        return None

    # Save to a temporary WAV file for SpeechRecognition
    audio_int16 = np.clip(audio_buffer * 32767, -32768, 32767).astype(np.int16)
    tmp_path: Optional[str] = None
    try:
        with tempfile.NamedTemporaryFile(
            suffix=".wav", delete=False
        ) as tmp_file:
            tmp_path = tmp_file.name
            wavfile.write(tmp_path, sample_rate, audio_int16)

        # Transcribe using SpeechRecognition
        recognizer = sr.Recognizer()
        with sr.AudioFile(tmp_path) as source:
            audio_data = recognizer.record(source)

        logger.info("🔄  Transcribing with Google Web Speech API …")
        raw_text: str = recognizer.recognize_google(audio_data)  # type: ignore[arg-type]
        logger.info("📝  Raw transcription: '%s'", raw_text)

        # Sanitise: lowercase, replace spaces/special chars with underscores
        sanitised = re.sub(r"[^a-z0-9]+", "_", raw_text.lower()).strip("_")
        if not sanitised:
            logger.warning("Transcription produced an empty label after sanitising.")
            return None

        # User confirmation
        print(f"\n  📝  Heard: \"{raw_text}\"")
        print(f"  🏷️  Label: \"{sanitised}\"")
        confirm = input("  ✅  Use this label? [Y/n]: ").strip().lower()
        if confirm in ("", "y", "yes"):
            logger.info("Label confirmed: '%s'", sanitised)
            return sanitised
        else:
            logger.info("Label rejected by user.")
            return None

    except sr.UnknownValueError:
        logger.warning("❌  Could not understand the speech. Please try again.")
        return None
    except sr.RequestError as exc:
        logger.error(
            "❌  Speech recognition API error: %s. "
            "Check your internet connection.", exc
        )
        return None
    except Exception as exc:
        logger.error("Speech-to-text failed: %s", exc)
        return None
    finally:
        # Clean up temporary file
        if tmp_path and os.path.exists(tmp_path):
            os.unlink(tmp_path)


# ---------------------------------------------------------------------------
# Core recording functions
# ---------------------------------------------------------------------------

def record_clip(
    sample_rate: int = SAMPLE_RATE,
    duration: float = DURATION,
    channels: int = CHANNELS,
) -> np.ndarray:
    """
    Record a single audio clip.

    Args:
        sample_rate: Sampling rate in Hz.
        duration:    Recording duration in seconds.
        channels:    Number of audio channels (1 = mono).

    Returns:
        Numpy array of shape (num_samples,) with dtype int16.
    """
    num_samples = int(sample_rate * duration)
    logger.info("🎤  GET READY — recording starts in 0.5 s …")
    time.sleep(0.5)

    # Allocate buffer
    audio_buffer = np.zeros(num_samples, dtype=np.float32)
    block_size = 1024
    blocks_recorded = 0
    total_blocks = int(np.ceil(num_samples / block_size))

    print()  # newline before progress bar
    with sd.InputStream(
        samplerate=sample_rate,
        channels=channels,
        dtype="float32",
        blocksize=block_size,
    ) as stream:
        samples_left = num_samples
        while samples_left > 0:
            to_read = min(block_size, samples_left)
            data, _ = stream.read(to_read)
            start_idx = num_samples - samples_left
            audio_buffer[start_idx : start_idx + to_read] = data[:to_read, 0]
            samples_left -= to_read
            blocks_recorded += 1
            _print_progress_bar(blocks_recorded, total_blocks, label="🔴 RECORDING")

    print()  # newline after progress bar

    # Convert float32 → int16
    audio_int16 = np.clip(audio_buffer * 32767, -32768, 32767).astype(np.int16)

    # Ensure exactly 1 second
    if len(audio_int16) < num_samples:
        audio_int16 = np.pad(audio_int16, (0, num_samples - len(audio_int16)))
    elif len(audio_int16) > num_samples:
        audio_int16 = audio_int16[:num_samples]

    peak_db = 20 * np.log10(max(np.max(np.abs(audio_int16)), 1) / 32767)
    logger.info("✅  Recording complete — peak level: %.1f dB", peak_db)
    return audio_int16


def save_clip(audio: np.ndarray, filepath: Path) -> None:
    """Save int16 audio as a WAV file."""
    wavfile.write(str(filepath), SAMPLE_RATE, audio)
    size_kb = filepath.stat().st_size / 1024
    logger.info("💾  Saved: %s (%.1f KB)", filepath.name, size_kb)


def playback_clip(filepath: Path) -> None:
    """Play back an audio file through the default output device."""
    try:
        sr, audio = wavfile.read(str(filepath))
        audio_float = audio.astype(np.float32) / 32767.0
        logger.info("🔊  Playing back: %s", filepath.name)
        sd.play(audio_float, samplerate=sr)
        sd.wait()
        logger.info("🔊  Playback complete.")
    except Exception as exc:
        logger.error("Playback failed: %s", exc)


# ---------------------------------------------------------------------------
# Interactive session
# ---------------------------------------------------------------------------

def interactive_session(label: str, num_clips: int = 0) -> None:
    """
    Run an interactive recording session.

    Args:
        label:     Class label for the recordings.
        num_clips: Number of clips to record (0 = unlimited, press Ctrl+C to stop).
    """
    label_dir = _ensure_directory(label)
    last_file: Optional[Path] = None
    clip_count = 0

    print("\n" + "=" * 60)
    print(f"  📁  Label        : {label}")
    print(f"  🎙️  Sample Rate  : {SAMPLE_RATE} Hz")
    print(f"  ⏱️   Duration     : {DURATION} s")
    target = num_clips if num_clips > 0 else "∞ (Ctrl+C to stop)"
    print(f"  🎯  Target Clips : {target}")
    print("=" * 60)

    try:
        while True:
            if 0 < num_clips <= clip_count:
                logger.info("Reached target of %d clips. Done!", num_clips)
                break

            filepath = _next_filename(label_dir)
            clip_count += 1

            print(f"\n--- Clip #{clip_count} ---")
            audio = record_clip()
            save_clip(audio, filepath)
            last_file = filepath

            # User menu
            while True:
                choice = input(
                    "\n  [Enter] Record next  |  [p] Play back  "
                    "|  [d] Delete & re-record  |  [q] Quit: "
                ).strip().lower()

                if choice == "":
                    break
                elif choice == "p" and last_file and last_file.exists():
                    playback_clip(last_file)
                elif choice == "d" and last_file and last_file.exists():
                    last_file.unlink()
                    logger.info("🗑️  Deleted: %s — re-recording …", last_file.name)
                    clip_count -= 1
                    audio = record_clip()
                    filepath = _next_filename(label_dir)
                    clip_count += 1
                    save_clip(audio, filepath)
                    last_file = filepath
                elif choice == "q":
                    raise KeyboardInterrupt
                else:
                    print("  ❓ Invalid option. Try again.")

    except KeyboardInterrupt:
        pass

    print(f"\n✅  Session complete — {clip_count} clip(s) saved to {label_dir}\n")


# ---------------------------------------------------------------------------
# CLI Entry point
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Record audio clips for emergency sound detection training.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  python3 src/record_data.py --label emergency --count 50\n"
            "  python3 src/record_data.py --label non_emergency\n"
            "  python3 src/record_data.py   (interactive label prompt)\n"
        ),
    )
    parser.add_argument(
        "--label", "-l",
        type=str,
        default=None,
        help="Class label (e.g. emergency, non_emergency).",
    )
    parser.add_argument(
        "--count", "-n",
        type=int,
        default=0,
        help="Number of clips to record (0 = unlimited, Ctrl+C to stop).",
    )
    parser.add_argument(
        "--list-devices",
        action="store_true",
        help="List available audio input devices and exit.",
    )
    return parser.parse_args()


def main() -> None:
    """Main entry point."""
    args = parse_args()

    # List devices mode
    if args.list_devices:
        print(sd.query_devices())
        return

    # Prompt for label if not provided
    label = args.label
    if label is None:
        print("\n🏷️  Choose how to set your class label:")
        print("     1. emergency        (sirens, ambulance, fire truck, police)")
        print("     2. non_emergency    (traffic, speech, birds, rain, etc.)")
        print("     3. background_noise (silence, ambient)")
        print("     4. Type a custom label")
        if _HAS_SPEECH_RECOGNITION:
            print("     5. 🗣️  Speak a custom label (Speech-to-Text)")
        choice = input("\n  Enter choice (1-5) or type a label: ").strip()

        shortcut_map = {
            "1": "emergency",
            "2": "non_emergency",
            "3": "background_noise",
        }

        if choice in shortcut_map:
            label = shortcut_map[choice]
        elif choice == "4":
            label = input("  Enter custom label: ").strip()
        elif choice == "5" and _HAS_SPEECH_RECOGNITION:
            # Speech-to-text loop — let user retry until successful
            while True:
                spoken = get_spoken_label()
                if spoken:
                    label = spoken
                    break
                retry = input("  🔄  Try again? [Y/n]: ").strip().lower()
                if retry in ("n", "no"):
                    label = input("  Enter label manually instead: ").strip()
                    break
        else:
            # Treat raw input as a custom label
            label = choice

        if not label:
            logger.error("Label cannot be empty.")
            sys.exit(1)

    # Sanitise label
    label = re.sub(r"[^a-z0-9]+", "_", label.lower()).strip("_")
    logger.info("Using label: '%s'", label)

    # Verify microphone access
    try:
        test = sd.rec(160, samplerate=SAMPLE_RATE, channels=CHANNELS, dtype="float32")
        sd.wait()
        del test
        logger.info("🎤  Microphone detected and working.")
    except Exception as exc:
        logger.error("Could not access microphone: %s", exc)
        logger.error("Check that a USB microphone is connected and accessible.")
        sys.exit(1)

    interactive_session(label, num_clips=args.count)


if __name__ == "__main__":
    main()
