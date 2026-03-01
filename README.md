# 🚨 Siren Sense — Emergency Sound Detection + Ambulance Verification

> **Detailed setup & training instructions →** [INSTRUCTIONS.md](INSTRUCTIONS.md) | **Wiring →** [WIRING.md](WIRING.md)

A two-stage emergency vehicle detection system combining **ML audio detection** (laptop) with **visual verification** (ESP32-CAM + Roboflow) and **ESP-NOW WiFi alerting** (receiver ESP32 → STM32 traffic controller). Both ESP32s are paired via **MAC address** (Sender: `A8:42:E3:56:83:2C`, Receiver: `FC:E8:C0:7A:B7:A0`). The 16×2 LCD display is connected to the **STM32F103RB** only.

---

## System Architecture

```
                    STAGE 1: Audio ML (Laptop)           STAGE 2: Visual AI (ESP32-CAM)
                  ┌──────────────────────────┐       ┌────────────────────────────────┐
  🎤 Laptop Mic ─►│ TFLite Siren Detection   │─WiFi──►│ OV2640 Camera Capture          │
                  │ Mel Spectrogram → CNN    │  HTTP  │ Roboflow YOLO → ambulance?     │
                  │ emergency / non_emergency│       │ Sound Sensor (local fallback)   │
                  └──────────────────────────┘       └──────────┬─────────────────────┘
                                                                │ ESP-NOW WiFi (~200m)
                                                                │ (auto-paired via beacon)
                                                                ▼
                                                    ┌──────────────────────────┐
                                                    │ ESP32 DevKit (Receiver)  │
                                                    │ ESP-NOW RX → UART TX     │
                                                    └──────────┬───────────────┘
                                                               │ UART Wire
                                                               ▼
                                                    🚦 STM32 Traffic Controller
                                                       + 16×2 LCD (PC6/PC7)
```

**Fallback**: If the laptop is offline, the ESP32-CAM's on-board sound sensor (GPIO 33) provides basic siren detection locally.

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
| `esp32_firmware/esp32_cam_sender/` | **ESP32-CAM sender firmware** (camera + Roboflow + ESP-NOW TX) |
| `esp32_firmware/stm32_esp32_receiver/` | **Receiver ESP32 firmware** (ESP-NOW RX → UART → STM32) |
| `esp32_firmware/find_mac_address/` | **MAC address finder** utility (flash to any ESP32 to get its MAC) |
| `stm32_firmware/` | **STM32 traffic controller** (bare-metal C, traffic FSM + LCD) |
| `WIRING.md` | Complete wiring guide for all components |
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

### 3. Integrated Mode: Laptop ML → ESP32-CAM → Roboflow → ESP-NOW → STM32

**a) Find MAC Addresses** — Flash `esp32_firmware/find_mac_address/find_mac_address.ino` to each ESP32 board. Open Serial Monitor at 115200 baud to see the MAC address.

| Board | MAC Address |
|-------|-------------|
| ESP32-CAM (Sender) | `A8:42:E3:56:83:2C` |
| ESP32 DevKit (Receiver) | `FC:E8:C0:7A:B7:A0` |

**b) Flash Receiver ESP32** — WiFi credentials are pre-configured (`Vivo_V29_5G`), flash `stm32_esp32_receiver.ino` with Arduino IDE (Board: ESP32 Dev Module).

**c) Flash ESP32-CAM** — WiFi/Roboflow credentials pre-configured, flash `esp32_cam_sender.ino` (Board: AI Thinker ESP32-CAM). Note the IP address.

**c) Wire Receiver ESP32 → STM32** — Connect ESP32 GPIO 17 (TX2) → STM32 PB11 (USART3 RX) + common GND.

**d) Run laptop ML:**

```bash
python3 src/siren_to_esp32.py --mode wifi --esp32-ip 192.168.1.50 --threshold 0.80
```

When the ML detects a siren → triggers ESP32-CAM → camera captures image → Roboflow verifies ambulance → ESP-NOW alerts receiver → UART → STM32.

---

## Hardware Required

| Component | Quantity | Purpose |
|-----------|----------|---------|
| ESP32-CAM (AI-Thinker) | 1 | Camera + WiFi + ESP-NOW TX |
| ESP32 DevKit (any) | 1 | ESP-NOW RX → UART bridge to STM32 |
| STM32 NUCLEO-F103RB | 1 | Traffic light controller |
| Sound sensor (analog) | 1 | Local fallback siren detection |
| 16×2 I2C LCD (PCF8574) | 1 | Status display (on STM32 only, PC6/PC7) |
| FTDI USB-UART adapter | 1 | For flashing ESP32-CAM |
| USB microphone (laptop) | 1 | Primary audio input for ML |

> No LoRa modules needed! ESP-NOW uses built-in WiFi hardware.

---

## ESP32 Configuration

### ESP32-CAM Sender (`esp32_cam_sender.ino` Section 1)

```cpp
#define WIFI_SSID           "Vivo_V29_5G"
#define WIFI_PASSWORD       "123456789"
#define ROBOFLOW_API_URL    "https://detect.roboflow.com/YOUR_PROJECT/YOUR_VERSION"
#define ROBOFLOW_API_KEY    "YOUR_ROBOFLOW_API_KEY"
#define ROBOFLOW_TARGET_CLASS "ambulance"
#define MONITORED_LANE_ID   0           // 0=North, 1=East, 2=South, 3=West
// Sender MAC:   A8:42:E3:56:83:2C
// Receiver MAC: FC:E8:C0:7A:B7:A0 (hardcoded fallback + auto-pair)
```

### Receiver ESP32 (`stm32_esp32_receiver.ino` Section 1)

```cpp
#define WIFI_SSID           "Vivo_V29_5G"    // SAME as sender!
#define WIFI_PASSWORD       "123456789"
#define STM32_TX_PIN        17               // UART TX to STM32
// Receiver MAC: FC:E8:C0:7A:B7:A0
// Known Sender: A8:42:E3:56:83:2C
```

### MAC Address Finder (`find_mac_address.ino`)

Flash this utility to any ESP32 to discover its MAC address:
```cpp
// Uses esp_read_mac() from <esp_mac.h> — no WiFi needed!
// Open Serial Monitor at 115200 baud after flashing.
```

### ESP32-CAM HTTP Endpoints

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/trigger` | GET | Start camera → Roboflow → ESP-NOW pipeline |
| `/status` | GET | JSON system status (WiFi, camera, ESP-NOW, trigger counts) |
| `/health` | GET | Simple `"OK"` health check |

---

## Detection Pipeline Detail

```
1. Laptop mic captures 3s audio window (16kHz mono)
2. Mel Spectrogram extraction (128 bands × 188 frames)
3. TFLite CNN classifies: emergency vs non_emergency
4. If emergency detected (≥80% confidence, 2 consecutive):
   └─► HTTP GET http://<ESP32_CAM_IP>/trigger
5. ESP32-CAM captures JPEG frame (640×480)
6. Base64-encoded image POSTed to Roboflow API
7. If "ambulance" class detected (≥60% confidence):
   └─► ESP-NOW sends "AMB:<lane_id>" to receiver ESP32 (auto-paired)
8. Receiver forwards "AMB:<lane_id>" via UART to STM32
9. STM32 traffic controller gives priority to that lane
10. 60-second cooldown before next detection
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
| ESP32-CAM not reachable via WiFi | Check SSID/password (`Vivo_V29_5G`), ensure laptop and ESP32 on same network |
| Roboflow API returns error | Verify API key and project URL in sender `.ino` |
| ESP-NOW send fails | Ensure both ESP32s on same WiFi network (`Vivo_V29_5G`). Check Serial Monitor for pairing status |
| Receiver not getting messages | Both must be powered on — receiver broadcasts beacon, sender discovers it automatically. If auto-pair fails, hardcoded MAC (FC:E8:C0:7A:B7:A0) is used as fallback |
| Pairing timeout | Ensure both ESP32s are on and connected to same WiFi within 30 seconds of each other. Sender falls back to hardcoded receiver MAC after timeout |
| Need to find MAC address | Flash `esp32_firmware/find_mac_address/find_mac_address.ino` to the board, open Serial Monitor at 115200 |
| LCD blank / not working | LCD is on STM32 only (PC6/PC7). Try I2C address 0x3F, check wiring + 5V power |
| Audio device not found | Run `--list-devices` to see available inputs |
| `/dev/ttyUSB0` permission denied | `sudo usermod -aG dialout $USER` then re-login |

---

## License

MIT
