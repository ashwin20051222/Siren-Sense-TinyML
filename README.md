# 🚨 TinyML Emergency Sound Detection

> **Detailed setup & training instructions are available in** [INSTRUCTIONS.md](INSTRUCTIONS.md)


A complete **Emergency Sound Detection** system using TinyML.

Record audio → Train a lightweight CNN on Mel Spectrograms → Deploy INT8-quantized TFLite model → Real-time detection on the edge.



---

## 📁 Project Structure

```
.
├── dataset/                      # Audio samples organised by label
│   ├── emergency/                # Ambulance, fire truck, police sirens
│   └── non_emergency/            # Traffic, speech, birds, rain, etc.
├── models/                       # Trained model outputs
│   ├── emergency_detector.h5     # Keras model
│   ├── emergency_detector.tflite # Quantized TFLite model
│   ├── class_names.txt           # Label mapping
│   └── training_curves.png       # Accuracy/loss plots
├── src/
│   ├── record_data.py            # Data collection CLI
│   ├── train_model.py            # Training pipeline
│   └── inference_pi.py           # Real-time Pi inference
├── requirements_pc.txt           # PC/training dependencies
├── requirements_pi.txt           # Raspberry Pi dependencies
└── README.md
```

---

## 🚀 Quick Start

### 1. Setup (Training PC)

> **Note:** Requires Python 3.10–3.12 (TensorFlow does not support 3.13+)

```bash
python3.12 -m venv venv
source venv/bin/activate
pip install -r requirements_pc.txt
```

### 2. Download Dataset

Download from Kaggle and extract into `dataset/`:

- **Emergency sounds**: [Emergency Vehicle Siren Sounds](https://www.kaggle.com/datasets/vishnu0399/emergency-vehicle-siren-sounds) (600 WAV files)
- **Non-emergency sounds** (optional diversity): [ESC-50](https://www.kaggle.com/datasets/mmoreaux/environmental-sound-classification-50)

```bash
# Place files like this:
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

### 3. (Optional) Record Your Own Data

Record additional clips with your microphone:

```bash
# Record emergency sounds (sirens, alarms)
python3 src/record_data.py --label emergency --count 100

# Record non-emergency sounds (traffic, speech)
python3 src/record_data.py --label non_emergency --count 100
```

**Tips:**
- Each clip is **3 seconds** long
- Vary your distance, volume, and environment
- Use `--list-devices` to see available microphones

### 4. Train the Model

```bash
python3 src/train_model.py --epochs 50 --batch-size 32
```

This will:
- Load all audio files from `dataset/`
- Apply data augmentation (noise, time-shift, pitch-shift, time-stretch)
- Extract 128-band Mel Spectrograms
- Train a 3-block DepthwiseSeparable CNN (< 200k params)
- Save `models/emergency_detector.h5` and `models/emergency_detector.tflite`
- Generate `models/training_curves.png`

### 5. Deploy to Raspberry Pi

Copy these to your Pi:

```bash
scp models/emergency_detector.tflite pi@<IP>:~/emergency_detector/models/
scp models/class_names.txt           pi@<IP>:~/emergency_detector/models/
scp src/inference_pi.py              pi@<IP>:~/emergency_detector/src/
scp requirements_pi.txt              pi@<IP>:~/emergency_detector/
```

On the Pi:

```bash
pip install -r requirements_pi.txt
python3 src/inference_pi.py --threshold 0.80
```

---

## 🧠 Model Architecture

| Layer | Type | Output Shape |
|-------|------|-------------|
| Input | Mel Spectrogram | (128, 188, 1) |
| Block 1 | DepthwiseSep Conv2D + BN + ReLU + Pool | (64, 94, 32) |
| Block 2 | DepthwiseSep Conv2D + BN + ReLU + Pool | (32, 47, 64) |
| Block 3 | DepthwiseSep Conv2D + BN + ReLU + Pool | (16, 23, 128) |
| Head | GlobalAvgPool → Dense(128) → Dense(64) → Dense(N) | (N,) |

- **Parameters**: < 200k
- **Quantized size**: < 200 KB (INT8)

---

## ⚙️ Configuration

### Feature Parameters (shared across training & inference)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `SAMPLE_RATE` | 16,000 Hz | Standard for audio |
| `DURATION` | 3.0 s | Audio clip length |
| `N_MELS` | 128 | Mel frequency bands |
| `N_FFT` | 1024 | FFT window size |
| `HOP_LENGTH` | 256 | Frame hop |

### Inference Tuning

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--threshold` | 0.80 | Min confidence for detection |
| `--consecutive` | 2 | Consecutive hits required |
| `--cooldown` | 3.0 s | Cooldown between triggers |

---

## 📋 Requirements

### Training PC
- Python 3.10–3.12
- TensorFlow ≥ 2.12
- USB microphone (for recording)

### Raspberry Pi
- Pi 3/4/5 or Zero 2 W
- USB microphone
- `tflite-runtime`, `sounddevice`, `numpy`, `scipy`

---

## 🙏 Credits

This project and its documentation were created independently without direct reference to any external repositories.

Datasets used for training are publicly available and listed in the instructions.

---

## 📄 License

MIT

---

## 📘 Detailed Installation & Training Instructions

The following content reproduces the step-by-step instructions found in `INSTRUCTIONS.md`.

### 1. Environment Setup

**Training PC (Linux / Windows)**

1. Python 3.10–3.12 (TensorFlow does not support 3.13+).
2. Create and activate a virtual environment:

   **Linux / macOS:**
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   ```

   **Windows (PowerShell):**
   ```powershell
   python -m venv venv
   .\\venv\\Scripts\\Activate.ps1
   ```

3. Install dependencies:

   ```bash
   pip install -r requirements_pc.txt
   ```

**Raspberry Pi (Linux)**

1. Use Raspberry Pi OS with Python 3.9–3.11.
2. Create a venv and install runtime packages:

   ```bash
   python3 -m venv venv
   source venv/bin/activate
   pip install -r requirements_pi.txt
   ```

### 2. Dataset Preparation

1. Download emergency samples from Kaggle:
   - Emergency Vehicle Siren Sounds
2. (Optional) Download non-emergency sounds (ESC-50).
3. Arrange files under `dataset/emergency/` and `dataset/non_emergency/`.
4. Record your own clips:

   ```bash
   python3 src/record_data.py --label emergency --count 100
   python3 src/record_data.py --label non_emergency --count 100
   ```

### 3. Training the Model

```bash
python3 src/train_model.py --epochs 50 --batch-size 32
```

Saves artifacts to `models/` including `.h5` and `.tflite` files and training curves.

### 4. Deployment to Raspberry Pi

```bash
scp models/emergency_detector.tflite pi@<IP>:~/emergency_detector/models/
scp models/class_names.txt           pi@<IP>:~/emergency_detector/models/
scp src/inference_pi.py              pi@<IP>:~/emergency_detector/src/
scp requirements_pi.txt              pi@<IP>:~/emergency_detector/
```

On the Pi:

```bash
pip install -r requirements_pi.txt
python3 src/inference_pi.py --threshold 0.80
```

### 5. Essential Commands Summary

| Purpose | Linux | Windows (PowerShell) |
|---------|-------|----------------------|
| Create venv | `python3 -m venv venv` | `python -m venv venv` |
| Activate venv | `source venv/bin/activate` | `.\\venv\\Scripts\\Activate.ps1` |
| Install deps | `pip install -r requirements_pc.txt` | same |
| Train model | `python3 src/train_model.py …` | `python src/train_model.py …` |
| Record audio | `python3 src/record_data.py …` | `python src/record_data.py …` |
| Inference (Pi) | `python3 src/inference_pi.py …` | n/a |

Feel free to adjust paths and parameters as needed.
