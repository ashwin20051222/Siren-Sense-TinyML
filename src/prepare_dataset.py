#!/usr/bin/env python3
"""
prepare_dataset.py — Prepare training dataset from available audio sources.

Splits longer WAV files into 3-second clips and organises them into the
dataset/emergency/ and dataset/non_emergency/ folders for training.

Usage:
    python3 src/prepare_dataset.py

This script uses only Python standard library modules (no pip packages needed).
"""

import os
import struct
import wave
from pathlib import Path

SAMPLE_RATE = 16_000
DURATION = 3.0  # seconds
NUM_SAMPLES = int(SAMPLE_RATE * DURATION)

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DATASET_DIR = PROJECT_ROOT / "dataset"
REFERENCE_DIR = PROJECT_ROOT / "reference_repo" / "test"


def read_wav_file(filepath: str):
    """Read a WAV file and return (sample_rate, samples_as_list, num_channels, sample_width)."""
    with wave.open(filepath, "rb") as wf:
        sr = wf.getframerate()
        n_channels = wf.getnchannels()
        sample_width = wf.getsampwidth()
        n_frames = wf.getnframes()
        raw_data = wf.readframes(n_frames)
    
    # Convert raw bytes to list of integer samples
    if sample_width == 2:  # 16-bit
        fmt = f"<{n_frames * n_channels}h"
        samples = list(struct.unpack(fmt, raw_data))
    elif sample_width == 4:  # 32-bit
        fmt = f"<{n_frames * n_channels}i"
        samples = list(struct.unpack(fmt, raw_data))
    elif sample_width == 1:  # 8-bit unsigned
        samples = [s - 128 for s in raw_data]
    else:
        raise ValueError(f"Unsupported sample width: {sample_width}")
    
    # If stereo, take only left channel
    if n_channels > 1:
        samples = samples[::n_channels]
    
    return sr, samples, sample_width


def resample_simple(samples, orig_sr, target_sr):
    """Very simple resampling by linear interpolation."""
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


def write_wav_file(filepath: str, samples, sample_rate=SAMPLE_RATE, sample_width=2):
    """Write samples to a WAV file."""
    with wave.open(filepath, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(sample_width)
        wf.setframerate(sample_rate)
        if sample_width == 2:
            raw = struct.pack(f"<{len(samples)}h", *[max(-32768, min(32767, int(s))) for s in samples])
        elif sample_width == 4:
            raw = struct.pack(f"<{len(samples)}i", *samples)
        else:
            raw = bytes([max(0, min(255, int(s) + 128)) for s in samples])
        wf.writeframes(raw)


def split_into_clips(samples, sr, target_sr=SAMPLE_RATE, clip_duration=DURATION):
    """Split a long audio into fixed-length clips. Returns list of sample lists."""
    # Resample if needed
    if sr != target_sr:
        print(f"    Resampling from {sr} Hz to {target_sr} Hz...")
        samples = resample_simple(samples, sr, target_sr)
    
    clip_samples = int(target_sr * clip_duration)
    clips = []
    
    i = 0
    while i + clip_samples <= len(samples):
        clip = samples[i:i + clip_samples]
        clips.append(clip)
        i += clip_samples
    
    # Handle remainder: pad if it's at least 1 second long
    remainder = samples[i:]
    if len(remainder) >= target_sr:  # at least 1 second
        # Pad to full clip length
        remainder.extend([0] * (clip_samples - len(remainder)))
        clips.append(remainder)
    
    return clips


def process_directory(src_dir, label, start_idx=1):
    """Process all WAV files in a directory, split into clips, save to dataset."""
    dest_dir = DATASET_DIR / label
    dest_dir.mkdir(parents=True, exist_ok=True)
    
    wav_files = sorted(Path(src_dir).glob("*.wav"))
    if not wav_files:
        print(f"  No WAV files found in {src_dir}")
        return start_idx
    
    clip_idx = start_idx
    for wav_path in wav_files:
        print(f"  Processing: {wav_path.name}")
        try:
            sr, samples, sw = read_wav_file(str(wav_path))
            duration = len(samples) / sr
            print(f"    Duration: {duration:.1f}s, Sample rate: {sr} Hz")
            
            clips = split_into_clips(samples, sr)
            print(f"    Generated {len(clips)} clips")
            
            for clip in clips:
                out_path = dest_dir / f"sample_{clip_idx:04d}.wav"
                write_wav_file(str(out_path), clip)
                clip_idx += 1
                
        except Exception as e:
            print(f"    ERROR: {e}")
    
    return clip_idx


def main():
    print("=" * 60)
    print("  🚨 Emergency Sound Detection — Dataset Preparation")
    print("=" * 60)
    
    # Process emergency sounds from local dataset directory
    print(f"\n📁 Processing EMERGENCY sounds from dataset directory...")
    emergency_dir = Path("dataset") / "emergency"
    if not emergency_dir.exists():
        print(f"    ❌ Expected directory not found: {emergency_dir}")
    else:
        next_idx = process_directory(emergency_dir, "emergency")
        print(f"   Total emergency clips: {next_idx - 1}")
    
    # Process non-emergency sounds
    print(f"\n📁 Processing NON-EMERGENCY sounds from dataset directory...")
    non_emergency_dir = Path("dataset") / "non_emergency"
    if not non_emergency_dir.exists():
        print(f"    ❌ Expected directory not found: {non_emergency_dir}")
    else:
        next_idx = process_directory(non_emergency_dir, "non_emergency")
        print(f"   Total non-emergency clips: {next_idx - 1}")
    
    # Summary
    emergency_count = len(list((DATASET_DIR / "emergency").glob("*.wav"))) if (DATASET_DIR / "emergency").exists() else 0
    non_emergency_count = len(list((DATASET_DIR / "non_emergency").glob("*.wav"))) if (DATASET_DIR / "non_emergency").exists() else 0
    
    print(f"\n{'=' * 60}")
    print(f"  ✅ Dataset prepared!")
    print(f"  📁 Emergency clips     : {emergency_count}")
    print(f"  📁 Non-emergency clips : {non_emergency_count}")
    print(f"  📁 Total               : {emergency_count + non_emergency_count}")
    print(f"  📂 Location            : {DATASET_DIR}")
    
    if emergency_count + non_emergency_count < 50:
        print(f"\n  ⚠️  This is a small starter dataset ({emergency_count + non_emergency_count} clips).")
        print(f"  For better accuracy, download more data from Kaggle:")
        print(f"  https://www.kaggle.com/datasets/vishnu0399/emergency-vehicle-siren-sounds")
        print(f"  Place emergency WAVs in: {DATASET_DIR}/emergency/")
        print(f"  Place non-emergency WAVs in: {DATASET_DIR}/non_emergency/")
    
    print(f"{'=' * 60}")


if __name__ == "__main__":
    main()
