# Instructions

This document covers everything needed to set up, train, and deploy the Siren Sense emergency sound detection system — including ESP32-CAM integration with Roboflow and LoRa.

---

## 1. Environment Setup

### 1.1 Training / Development PC (Linux or Windows)

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

3. Install dependencies:

   ```bash
   pip install -r requirements_pc.txt
   ```

### 1.2 Raspberry Pi

1. Use Raspberry Pi OS with Python 3.9–3.11.
2. Create venv and install runtime packages:

   ```bash
   python3 -m venv venv && source venv/bin/activate
   pip install -r requirements_pi.txt
   ```

> **Note:** `requirements_pi.txt` includes `tflite-runtime`, `sounddevice`, `numpy`, `scipy`.

---

## 2. Dataset Preparation

1. **Download emergency samples** from Kaggle:
   - [Emergency Vehicle Siren Sounds](https://www.kaggle.com/datasets/vishnu0399/emergency-vehicle-siren-sounds)
2. **(Optional)** Download non‑emergency sounds:
   - [ESC-50](https://www.kaggle.com/datasets/mmoreaux/environmental-sound-classification-50)
3. Arrange files:

   ```
   dataset/
     emergency/
       ambulance_001.wav
       firetruck_001.wav
     non_emergency/
       traffic_001.wav
       birds_001.wav
   ```

4. **Record your own clips** (microphone required):

   ```bash
   python3 src/record_data.py --label emergency --count 100
   python3 src/record_data.py --label non_emergency --count 100
   ```

   Use `--list-devices` to show available audio inputs. Each clip is 3 seconds.

---

## 3. Training the Model

```bash
python3 src/train_model.py --epochs 50 --batch-size 32
```

The script will:
- Load and augment audio (noise, time-shift, pitch, stretch, SpecAugment, Mixup)
- Convert clips to 128-band Mel Spectrograms
- Train a DepthwiseSeparable CNN (~1.5M parameters)
- Save under `models/`:
  - `emergency_detector.keras` and `.h5` (Keras)
  - `emergency_detector.tflite` (INT8 quantized)
  - `class_names.txt`
  - `training_curves.png`

---

## 4. Running Inference (Standalone — No ESP32)

### 4.1 Laptop

```bash
python3 src/inference_pi.py --model models/emergency_detector.tflite \
    --labels models/class_names.txt --threshold 0.8
```

### 4.2 Raspberry Pi

```bash
scp models/emergency_detector.tflite pi@<IP>:~/siren_sense/models/
scp models/class_names.txt pi@<IP>:~/siren_sense/models/
scp src/inference_pi.py pi@<IP>:~/siren_sense/src/

# On Pi:
python3 src/inference_pi.py --threshold 0.8
```

---

## 5. ESP32-CAM Integration (Ambulance Detector)

This section sets up the full pipeline: **Laptop ML → ESP32-CAM → Roboflow → LoRa → STM32**.

### 5.1 Hardware Required

| Component | Purpose |
|-----------|---------|
| ESP32-CAM (AI-Thinker) | Camera + WiFi + controller |
| SX1278 LoRa module | 433 MHz long-range alert to STM32 |
| Sound sensor (analog) | Local fallback siren detection |
| FTDI USB-UART adapter | For flashing firmware |
| USB microphone (laptop) | Primary audio input for ML |

### 5.2 Wiring

**LoRa SX1278 → ESP32-CAM (HSPI):**

| SX1278 | ESP32-CAM |
|--------|-----------|
| VCC | 3.3V |
| GND | GND |
| SCK | GPIO 14 |
| MISO | GPIO 12 |
| MOSI | GPIO 13 |
| NSS/CS | GPIO 15 |
| RST | GPIO 2 |
| DIO0 | GPIO 4 |

**Sound Sensor → ESP32-CAM:**

| Sensor | ESP32-CAM |
|--------|-----------|
| VCC | 3.3V |
| GND | GND |
| OUT/AO | GPIO 33 |

**FTDI Programmer → ESP32-CAM (for flashing only):**

| FTDI | ESP32-CAM |
|------|-----------|
| 3.3V | 3.3V |
| GND | GND |
| TX | GPIO 3 (RX0) |
| RX | GPIO 1 (TX0) |
| — | IO0 → GND (during flash) |

### 5.3 Configure Firmware

Open `esp32_firmware/ambulance_detector/ambulance_detector.ino` and edit the defines at the top:

```cpp
/* WiFi */
#define WIFI_SSID           "YourWiFiName"
#define WIFI_PASSWORD       "YourWiFiPassword"

/* Roboflow — get from https://app.roboflow.com → Deploy → Hosted API */
#define ROBOFLOW_API_URL    "https://detect.roboflow.com/YOUR_PROJECT/YOUR_VERSION"
#define ROBOFLOW_API_KEY    "YOUR_API_KEY"
#define ROBOFLOW_TARGET_CLASS "ambulance"
#define ROBOFLOW_CONF_MIN   0.60

/* Lane this ESP32 monitors */
#define MONITORED_LANE_ID   0       // 0=North, 1=East, 2=South, 3=West

/* How laptop triggers this ESP32 */
#define TRIGGER_MODE        0       // 0=WiFi HTTP, 1=UART, 2=Both
```

### 5.4 Flash to ESP32-CAM

1. Open `ambulance_detector.ino` in Arduino IDE
2. Install required libraries via Library Manager:
   - **LoRa** by Sandeep Mistry (v0.8.0+)
   - **ArduinoJson** by Benoit Blanchon (v6.x or v7.x)
3. Board Settings:
   - Board: **AI Thinker ESP32-CAM**
   - Partition Scheme: **Huge APP (3MB No OTA/1MB SPIFFS)**
   - Upload Speed: 115200
4. Connect FTDI adapter, hold IO0 to GND, press Reset, then Upload
5. After upload: remove IO0 jumper, press Reset
6. Open Serial Monitor at 115200 baud — note the IP address:
   ```
   [WiFi] Connected! IP: 192.168.1.50
   [HTTP] Trigger URL: http://192.168.1.50/trigger
   ```

### 5.5 Run Laptop ML → ESP32 Trigger

**WiFi mode (recommended):**
```bash
python3 src/siren_to_esp32.py --mode wifi --esp32-ip 192.168.1.50 --threshold 0.80
```

**UART / wired mode:**
```bash
python3 src/siren_to_esp32.py --mode uart --uart-port /dev/ttyUSB0
```

**Both (WiFi + UART fallback):**
```bash
python3 src/siren_to_esp32.py --mode both --esp32-ip 192.168.1.50 --uart-port /dev/ttyUSB0
```

**Check ESP32 status:**
```bash
python3 src/siren_to_esp32.py --check-esp32 --esp32-ip 192.168.1.50
```

### 5.6 How It Works End-to-End

1. Laptop mic captures 3-second audio windows at 16 kHz
2. TFLite model classifies: `emergency` vs `non_emergency`
3. If siren detected (≥ threshold, consecutive hits) → HTTP GET to ESP32
4. ESP32 captures JPEG frame with OV2640 camera
5. Image base64-encoded and POSTed to Roboflow inference API
6. If `"ambulance"` detected with ≥ 60% confidence:
   - LoRa SX1278 transmits `"AMB:<lane_id>"` (3 retries)
   - STM32 traffic controller gives priority to that lane
7. 60-second cooldown before next detection

### 5.7 ESP32 HTTP Endpoints

| Endpoint | Response |
|----------|----------|
| `GET /trigger` | Starts camera → Roboflow → LoRa pipeline |
| `GET /status` | JSON with uptime, camera/LoRa health, trigger counts, lane |
| `GET /health` | `"OK"` — quick reachability check |

---

## 6. Essential Commands Summary

| Purpose | Command |
|---------|---------|
| Create venv | `python3 -m venv venv` |
| Activate venv | `source venv/bin/activate` |
| Install deps (PC) | `pip install -r requirements_pc.txt` |
| Install deps (Pi) | `pip install -r requirements_pi.txt` |
| Record samples | `python3 src/record_data.py --label emergency --count 100` |
| Train model | `python3 src/train_model.py --epochs 50 --batch-size 32` |
| Inference (standalone) | `python3 src/inference_pi.py --threshold 0.8` |
| Inference + ESP32 | `python3 src/siren_to_esp32.py --mode wifi --esp32-ip <IP>` |
| Check ESP32 | `python3 src/siren_to_esp32.py --check-esp32 --esp32-ip <IP>` |
| List audio devices | `python3 src/inference_pi.py --list-devices` |

---

## 7. Troubleshooting

| Problem | Solution |
|---------|----------|
| ESP32 WiFi won't connect | Double-check SSID/password, ensure 2.4 GHz network |
| Roboflow API error | Verify API key, project URL, and internet access |
| LoRa init fails | Check SX1278 wiring (SCK=14, MISO=12, MOSI=13, NSS=15) |
| Camera init fails | Ensure PSRAM present, check OV2640 ribbon cable |
| Detection too sensitive | Increase `--threshold` (e.g. 0.90) or `--consecutive` (e.g. 3) |
| No audio input | Run `--list-devices`, check mic is connected |
| Serial port permission | `sudo usermod -aG dialout $USER`, then re-login |
| TensorFlow import error | Use Python 3.10–3.12, not 3.13+ |

---

Feel free to adjust paths and parameters for your environment.
