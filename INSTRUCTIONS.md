# Instructions

This document explains how to install dependencies and train the emergency sound detector on both Linux and Windows, and how to deploy the model to a Raspberry Pi. Follow each section step‑by‑step.

---

## 1. Environment Setup

### 1.1 Training PC (Linux / Windows)

1. **Python version**: 3.10, 3.11 or 3.12 (TensorFlow does not support 3.13+).
2. Create and activate a virtual environment:

   **Linux / macOS:**
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   ```

   **Windows (PowerShell):**
   ```powershell
   python -m venv venv
   .\venv\Scripts\Activate.ps1
   ```

3. Install the training dependencies:

   ```bash
   pip install -r requirements_pc.txt
   ```

### 1.2 Raspberry Pi (Linux)

1. Use Raspberry Pi OS with Python 3.9–3.11 installed.
2. Create a virtual environment on the Pi:

   ```bash
   python3 -m venv venv
   source venv/bin/activate
   ```

3. Install runtime packages:

   ```bash
   pip install -r requirements_pi.txt
   ```

> **Note:** `requirements_pi.txt` includes `tflite-runtime`, `sounddevice`, `numpy`, `scipy`, etc.

## 2. Dataset Preparation

1. **Download emergency samples** from Kaggle:
   - [Emergency Vehicle Siren Sounds](https://www.kaggle.com/datasets/vishnu0399/emergency-vehicle-siren-sounds)
2. **(Optional)** Download non‑emergency sounds for diversity:
   - [ESC-50](https://www.kaggle.com/datasets/mmoreaux/environmental-sound-classification-50)
3. Arrange files:

   ```text
   dataset/
     emergency/
       ambulance_001.wav
       firetruck_001.wav
       ...
     non_emergency/
       traffic_001.wav
       birds_001.wav
       ...
   ```

4. **Recording your own clips** (microphone required):

   ```bash
   python3 src/record_data.py --label emergency --count 100
   python3 src/record_data.py --label non_emergency --count 100
   ```

   Use `--list-devices` to show available audio inputs. Each clip is 3 s long; vary distance and environment.

## 3. Training the Model

Run the training script from the project root:

```bash
python3 src/train_model.py --epochs 50 --batch-size 32
```

The script will:

- Load and augment audio data (noise, time-shift, pitch, stretch).
- Convert clips to 128‑band Mel spectrograms.
- Train a small depthwise‐separable CNN (~200 k parameters).
- Save artifacts under `models/`:
  - `emergency_detector.h5` (Keras)
  - `emergency_detector.tflite` (INT8 quantized)
  - `class_names.txt`
  - `training_curves.png`

## 4. Deployment to Raspberry Pi

1. Copy the model and support files:

   ```bash
   scp models/emergency_detector.tflite pi@<IP>:~/emergency_detector/models/
   scp models/class_names.txt           pi@<IP>:~/emergency_detector/models/
   scp src/inference_pi.py              pi@<IP>:~/emergency_detector/src/
   scp requirements_pi.txt              pi@<IP>:~/emergency_detector/
   ```

2. On the Pi, activate the venv and install packages (see section 1.2).
3. Run inference with a USB microphone attached:

   ```bash
   python3 src/inference_pi.py --threshold 0.80
   ```

   Adjust `--threshold`, `--consecutive`, and `--cooldown` as needed.

---

## 5. Essential Commands Summary

| Purpose | Linux | Windows (PowerShell) |
|---------|-------|----------------------|
| Create venv | `python3 -m venv venv` | `python -m venv venv` |
| Activate venv | `source venv/bin/activate` | `.\venv\Scripts\Activate.ps1` |
| Install deps | `pip install -r requirements_pc.txt` | same |
| Train model | `python3 src/train_model.py …` | `python src/train_model.py …` |
| Record audio | `python3 src/record_data.py …` | `python src/record_data.py …` |
| Inference (Pi) | `python3 src/inference_pi.py …` | n/a |


Feel free to adjust paths and parameters for your environment.
