#!/usr/bin/env python3
"""
download_extra_data.py — Download additional emergency/non-emergency audio.

Downloads free audio samples from online sources and processes them into
3-second clips for the training dataset.

Sources:
    - ESC-50: Environmental Sound Classification (GitHub)
    - Free emergency siren samples from various open repositories

Usage:
    python3 src/download_extra_data.py
    python3 src/download_extra_data.py --source esc50
    python3 src/download_extra_data.py --source freesound

Author:  Emergency Sound Detection Project
License: MIT
"""

from __future__ import annotations

import os
import shutil
import struct
import subprocess
import sys
import tempfile
import wave
import zipfile
from pathlib import Path
from urllib.request import urlretrieve
from urllib.error import URLError

SAMPLE_RATE = 16_000
DURATION = 3.0
NUM_SAMPLES = int(SAMPLE_RATE * DURATION)

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DATASET_DIR = PROJECT_ROOT / "dataset"

# ESC-50 dataset categories mapped to our classes
ESC50_EMERGENCY_CATEGORIES = [
    "siren",
]
ESC50_NON_EMERGENCY_CATEGORIES = [
    "dog", "rain", "sea_waves", "crackling_fire", "crickets",
    "chirping_birds", "water_drops", "wind", "pouring_water",
    "toilet_flush", "thunderstorm", "crying_baby", "sneezing",
    "clapping", "breathing", "coughing", "footsteps", "laughing",
    "brushing_teeth", "snoring", "drinking_sipping",
    "door_knock", "mouse_click", "keyboard_typing",
    "car_horn", "engine", "train", "church_bells",
    "clock_tick", "glass_breaking",
]

ESC50_URL = "https://github.com/karolpiczak/ESC-50/archive/refs/heads/master.zip"


def read_wav_file(filepath: str):
    """Read a WAV file and return (sample_rate, samples_list)."""
    with wave.open(filepath, "rb") as wf:
        sr = wf.getframerate()
        n_channels = wf.getnchannels()
        sample_width = wf.getsampwidth()
        n_frames = wf.getnframes()
        raw_data = wf.readframes(n_frames)

    if sample_width == 2:
        fmt = f"<{n_frames * n_channels}h"
        samples = list(struct.unpack(fmt, raw_data))
    elif sample_width == 4:
        fmt = f"<{n_frames * n_channels}i"
        samples = list(struct.unpack(fmt, raw_data))
    elif sample_width == 1:
        samples = [s - 128 for s in raw_data]
    else:
        raise ValueError(f"Unsupported sample width: {sample_width}")

    if n_channels > 1:
        samples = samples[::n_channels]

    return sr, samples


def resample_simple(samples, orig_sr, target_sr):
    """Simple linear interpolation resampling."""
    if orig_sr == target_sr:
        return samples
    ratio = target_sr / orig_sr
    new_length = int(len(samples) * ratio)
    resampled = []
    for i in range(new_length):
        orig_pos = i / ratio
        idx = int(orig_pos)
        frac = orig_pos - idx
        if idx + 1 < len(samples):
            val = samples[idx] * (1 - frac) + samples[idx + 1] * frac
        else:
            val = samples[idx] if idx < len(samples) else 0
        resampled.append(int(val))
    return resampled


def write_wav_file(filepath: str, samples, sample_rate=SAMPLE_RATE):
    """Write 16-bit mono WAV."""
    with wave.open(filepath, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        raw = struct.pack(
            f"<{len(samples)}h",
            *[max(-32768, min(32767, int(s))) for s in samples],
        )
        wf.writeframes(raw)


def split_into_clips(samples, sr, target_sr=SAMPLE_RATE, clip_duration=DURATION):
    """Split audio into fixed-length clips."""
    if sr != target_sr:
        samples = resample_simple(samples, sr, target_sr)
    clip_samples = int(target_sr * clip_duration)
    clips = []
    i = 0
    while i + clip_samples <= len(samples):
        clips.append(samples[i : i + clip_samples])
        i += clip_samples
    remainder = samples[i:]
    if len(remainder) >= target_sr:
        remainder.extend([0] * (clip_samples - len(remainder)))
        clips.append(remainder)
    return clips


def download_file(url: str, dest: str) -> bool:
    """Download a file with progress feedback."""
    try:
        print(f"  ⬇️  Downloading from {url[:80]}...")
        urlretrieve(url, dest)
        return True
    except (URLError, Exception) as e:
        print(f"  ❌ Download failed: {e}")
        return False


def process_esc50(tmp_dir: str):
    """Download and process ESC-50 dataset."""
    print("\n📦 Downloading ESC-50 dataset (~600 MB)...")
    print("   Source: https://github.com/karolpiczak/ESC-50")

    zip_path = os.path.join(tmp_dir, "esc50.zip")
    if not download_file(ESC50_URL, zip_path):
        print("  ❌ Could not download ESC-50. Try manually:")
        print(f"     wget {ESC50_URL} -O {zip_path}")
        return

    print("  📂 Extracting...")
    with zipfile.ZipFile(zip_path, "r") as zf:
        zf.extractall(tmp_dir)

    # Find the extracted directory
    esc_dir = Path(tmp_dir) / "ESC-50-master"
    audio_dir = esc_dir / "audio"
    meta_path = esc_dir / "meta" / "esc50.csv"

    if not audio_dir.exists() or not meta_path.exists():
        print("  ❌ ESC-50 extraction failed — unexpected structure.")
        return

    # Read metadata CSV
    categories = {}
    with open(meta_path, "r") as f:
        header = f.readline()  # skip header
        for line in f:
            parts = line.strip().split(",")
            if len(parts) >= 4:
                filename = parts[0]
                category = parts[3]
                categories[filename] = category

    emergency_dir = DATASET_DIR / "emergency"
    non_emergency_dir = DATASET_DIR / "non_emergency"
    emergency_dir.mkdir(parents=True, exist_ok=True)
    non_emergency_dir.mkdir(parents=True, exist_ok=True)

    # Get existing file count to avoid overwriting
    e_idx = len(list(emergency_dir.glob("*.wav"))) + 1
    ne_idx = len(list(non_emergency_dir.glob("*.wav"))) + 1

    e_count = 0
    ne_count = 0

    for wav_file in sorted(audio_dir.glob("*.wav")):
        cat = categories.get(wav_file.name, "")

        if cat in ESC50_EMERGENCY_CATEGORIES:
            label = "emergency"
        elif cat in ESC50_NON_EMERGENCY_CATEGORIES:
            label = "non_emergency"
        else:
            continue

        try:
            sr, samples = read_wav_file(str(wav_file))
            clips = split_into_clips(samples, sr)

            for clip in clips:
                if label == "emergency":
                    out_path = emergency_dir / f"esc50_{e_idx:04d}.wav"
                    e_idx += 1
                    e_count += 1
                else:
                    out_path = non_emergency_dir / f"esc50_{ne_idx:04d}.wav"
                    ne_idx += 1
                    ne_count += 1
                write_wav_file(str(out_path), clip)
        except Exception as e:
            print(f"  ⚠️  Skipping {wav_file.name}: {e}")

    print(f"  ✅ Added {e_count} emergency + {ne_count} non-emergency clips from ESC-50")


def show_manual_instructions():
    """Show instructions for manually downloading data."""
    print("\n" + "=" * 60)
    print("  📥 Manual Data Download Instructions")
    print("=" * 60)
    print()
    print("  For more data, download from these free sources:")
    print()
    print("  1. Kaggle — Emergency Vehicle Siren Sounds:")
    print("     https://www.kaggle.com/datasets/vishnu0399/emergency-vehicle-siren-sounds")
    print(f"     → Place emergency WAVs in: {DATASET_DIR}/emergency/")
    print(f"     → Place non-emergency WAVs in: {DATASET_DIR}/non_emergency/")
    print()
    print("  2. UrbanSound8K:")
    print("     https://urbansounddataset.weebly.com/urbansound8k.html")
    print("     → 'siren' category → emergency/")
    print("     → Other categories → non_emergency/")
    print()
    print("  3. ESC-50 (manual download):")
    print("     https://github.com/karolpiczak/ESC-50")
    print("     → Run: python3 src/download_extra_data.py --source esc50")
    print()
    print("=" * 60)


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Download extra training data.")
    parser.add_argument(
        "--source",
        choices=["esc50", "manual"],
        default="esc50",
        help="Data source to download from.",
    )
    args = parser.parse_args()

    print("\n" + "=" * 60)
    print("  📥 Emergency Sound Detection — Extra Data Download")
    print("=" * 60)

    # Count existing dataset
    e_before = len(list((DATASET_DIR / "emergency").glob("*.wav"))) if (DATASET_DIR / "emergency").exists() else 0
    ne_before = len(list((DATASET_DIR / "non_emergency").glob("*.wav"))) if (DATASET_DIR / "non_emergency").exists() else 0
    print(f"\n  Current dataset: {e_before} emergency + {ne_before} non-emergency = {e_before + ne_before} total")

    if args.source == "esc50":
        with tempfile.TemporaryDirectory() as tmp_dir:
            process_esc50(tmp_dir)
    elif args.source == "manual":
        show_manual_instructions()
        return

    # Count after
    e_after = len(list((DATASET_DIR / "emergency").glob("*.wav"))) if (DATASET_DIR / "emergency").exists() else 0
    ne_after = len(list((DATASET_DIR / "non_emergency").glob("*.wav"))) if (DATASET_DIR / "non_emergency").exists() else 0

    print(f"\n{'=' * 60}")
    print(f"  ✅ Dataset updated!")
    print(f"  📁 Emergency clips     : {e_before} → {e_after} (+{e_after - e_before})")
    print(f"  📁 Non-emergency clips : {ne_before} → {ne_after} (+{ne_after - ne_before})")
    print(f"  📁 Total               : {e_after + ne_after}")
    print(f"{'=' * 60}\n")

    show_manual_instructions()


if __name__ == "__main__":
    main()
