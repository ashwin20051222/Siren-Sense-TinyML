# 🚨 Siren Sense — Emergency Sound Detection + Ambulance Verification

> **Detailed setup & training instructions →** [INSTRUCTIONS.md](INSTRUCTIONS.md)

A two-stage emergency vehicle detection system combining **ML audio detection** (laptop) with **visual verification** (ESP32-CAM + Roboflow) and **LoRa alerting** (STM32 traffic controller).

---

## System Architecture

```
                      STAGE 1: Audio ML (Laptop)          STAGE 2: Visual AI (ESP32-CAM)
                    ┌─────────────────────────┐       ┌──────────────────────────────────┐
  🎤 Laptop Mic ──►│ TFLite Siren Detection   │──WiFi─►│ OV2640 Camera Capture            │
                    │ Mel Spectrogram → CNN    │  or   │ Roboflow YOLO → ambulance?       │
                    │ emergency / non_emergency│ UART  │ LoRa SX1278 → "AMB:<lane_id>"    │
                    └─────────────────────────┘       └──────────┬───────────────────────┘
                                                                  │ LoRa 433 MHz
                                                                  ▼
                                                        🚦 STM32 Traffic Controller
```

**Fallback**: If the laptop is offline, the ESP32's on-board sound sensor (GPIO 33) provides basic siren detection locally.

---

## Contents

| Path | Description |
|------|-------------|
| `dataset/` | Labeled 3-second WAV clips (`emergency/`, `non_emergency/`) |
| `models/` | `emergency_detector.tflite`, `class_names.txt`, training curves |
| `src/train_model.py` | Training pipeline: augmentation → Mel Spectrograms → CNN → INT8 TFLite |
| `src/inference_pi.py` | Real-time inference on laptop/Pi mic (standalone mode) |
| `src/siren_to_esp32.py` | **ML inference → ESP32-CAM trigger** (integrated mode) |
| `src/record_data.py` | Record audio samples from microphone |
| `src/prepare_dataset.py` | Dataset preparation utilities |
| `src/download_extra_data.py` | Download additional training data |
| `esp32_firmware/ambulance_detector/` | **Single-file ESP32-CAM firmware** (camera + Roboflow + LoRa) |
| `requirements_pc.txt` | PC/laptop dependencies |
| `requirements_pi.txt` | Raspberry Pi runtime dependencies |

---

## Quick Start

### 1. Install & Train (Laptop)

```bash
python3 -m venv venv && source venv/bin/activate
pip install -r requirements_pc.txt
python3 src/train_model.py --epochs 50 --batch-size 32
```

### 2. Standalone Laptop Inference (No ESP32)

```bash
python3 src/inference_pi.py --threshold 0.8
```

### 3. Integrated Mode: Laptop ML → ESP32-CAM → Roboflow → LoRa

**a) Flash ESP32-CAM** — edit WiFi/Roboflow credentials at top of `ambulance_detector.ino`, then flash with Arduino IDE (Board: AI Thinker ESP32-CAM). Note the IP address from Serial Monitor.

**b) Run laptop ML:**

```bash
python3 src/siren_to_esp32.py --mode wifi --esp32-ip 192.168.1.50 --threshold 0.80
```

When the ML detects a siren → triggers ESP32 → camera captures image → Roboflow verifies ambulance → LoRa alerts STM32.

---

## ESP32-CAM Integration

### Hardware Required

- ESP32-CAM (AI-Thinker) with OV2640 camera
- SX1278 LoRa module (433 MHz)
- Sound sensor module (analog, for fallback)
- FTDI USB-UART adapter (for flashing)

### Wiring

| Component | Pin | ESP32-CAM GPIO |
|-----------|-----|----------------|
| **LoRa SX1278** | SCK | 14 |
| | MISO | 12 |
| | MOSI | 13 |
| | NSS/CS | 15 |
| | RST | 2 |
| | DIO0 | 4 |
| | VCC / GND | 3.3V / GND |
| **Sound Sensor** | OUT | 33 |
| | VCC / GND | 3.3V / GND |

### ESP32 Configuration (top of `.ino`)

```cpp
#define WIFI_SSID           "YourWiFi"
#define WIFI_PASSWORD       "YourPassword"
#define ROBOFLOW_API_URL    "https://detect.roboflow.com/YOUR_PROJECT/YOUR_VERSION"
#define ROBOFLOW_API_KEY    "YOUR_API_KEY"
#define ROBOFLOW_TARGET_CLASS "ambulance"
#define MONITORED_LANE_ID   0           // 0=North, 1=East, 2=South, 3=West
#define TRIGGER_MODE        0           // 0=WiFi, 1=UART, 2=Both
```

### ESP32 HTTP Endpoints

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/trigger` | GET | Start camera → Roboflow → LoRa pipeline |
| `/status` | GET | JSON system status (WiFi, camera, LoRa, trigger counts) |
| `/health` | GET | Simple `"OK"` health check |

### Connection Modes

| Mode | How | Command |
|------|-----|---------|
| **WiFi** (recommended) | HTTP GET `/trigger` over same WiFi | `--mode wifi --esp32-ip <IP>` |
| **UART** (wired) | `"SIREN\n"` over USB serial | `--mode uart --uart-port /dev/ttyUSB0` |
| **Both** | WiFi first, UART fallback | `--mode both --esp32-ip <IP> --uart-port /dev/ttyUSB0` |

### Check ESP32 Status from Laptop

```bash
python3 src/siren_to_esp32.py --check-esp32 --esp32-ip 192.168.1.50
```

---

## Detection Pipeline Detail

```
1. Laptop mic captures 3s audio window (16kHz mono)
2. Mel Spectrogram extraction (128 bands × 188 frames)
3. TFLite CNN classifies: emergency vs non_emergency
4. If emergency detected (≥80% confidence, 2 consecutive):
   └─► HTTP GET http://<ESP32_IP>/trigger
5. ESP32-CAM captures JPEG frame (640×480)
6. Base64-encoded image POSTed to Roboflow API
7. If "ambulance" class detected (≥60% confidence):
   └─► LoRa transmits "AMB:<lane_id>" to STM32
8. STM32 traffic controller gives priority to that lane
9. 60-second cooldown before next detection
```

---

## Raspberry Pi Deployment

```bash
scp models/emergency_detector.tflite pi@<IP>:~/siren_sense/models/
scp models/class_names.txt pi@<IP>:~/siren_sense/models/
scp src/inference_pi.py pi@<IP>:~/siren_sense/src/

# On Pi:
pip install -r requirements_pi.txt
python3 src/inference_pi.py --threshold 0.8
```

---

## CLI Options

### `siren_to_esp32.py` (Integrated Mode)

| Flag | Default | Description |
|------|---------|-------------|
| `--mode` | `wifi` | `wifi`, `uart`, or `both` |
| `--esp32-ip` | — | ESP32-CAM IP address |
| `--threshold` | `0.80` | Siren detection confidence |
| `--cooldown` | `3.0` | Seconds between triggers |
| `--consecutive` | `2` | Consecutive detections needed |
| `--check-esp32` | — | Check ESP32 status and exit |
| `--list-devices` | — | List audio devices and exit |

### `inference_pi.py` (Standalone Mode)

| Flag | Default | Description |
|------|---------|-------------|
| `--model` | `models/emergency_detector.tflite` | TFLite model path |
| `--labels` | `models/class_names.txt` | Class names file |
| `--threshold` | `0.80` | Detection confidence |
| `--cooldown` | `3.0` | Seconds between alerts |
| `--consecutive` | `2` | Consecutive detections needed |

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| ESP32 not reachable via WiFi | Check SSID/password, ensure laptop and ESP32 on same network |
| Roboflow API returns error | Verify API key and project URL in `.ino` |
| LoRa init fails | Check SX1278 wiring (GPIO 12-15) |
| ML too sensitive / not sensitive | Adjust `--threshold` and `--consecutive` |
| Audio device not found | Run `--list-devices` to see available inputs |
| `/dev/ttyUSB0` permission denied | `sudo usermod -aG dialout $USER` then re-login |

---

## License

MIT
