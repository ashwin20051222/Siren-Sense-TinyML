# Instructions

This document covers everything needed to set up, train, and deploy the Siren Sense emergency sound detection system — including ESP32-CAM integration with Roboflow and ESP-NOW WiFi alerting.

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

## 5. Two-ESP32 Integration (ESP-NOW WiFi Architecture)

This section sets up the full pipeline: **Laptop ML → ESP32-CAM → Roboflow → ESP-NOW → Receiver ESP32 → UART → STM32**.

### 5.1 Hardware Required

| Component | Purpose |
|-----------|---------|
| ESP32-CAM (AI-Thinker) | Camera + WiFi + ESP-NOW TX (sender) — MAC: `A8:42:E3:56:83:2C` |
| ESP32 DevKit (any standard) | ESP-NOW RX + UART bridge to STM32 (receiver) — MAC: `FC:E8:C0:7A:B7:A0` |
| Sound sensor (analog) | Local fallback siren detection (on ESP32-CAM) |
| 16×2 I2C LCD (PCF8574) | Status display (on STM32 only, PC6/PC7) |
| FTDI USB-UART adapter | For flashing ESP32-CAM firmware |
| USB microphone (laptop) | Primary audio input for ML |

> **No LoRa modules needed!** ESP-NOW uses the built-in WiFi hardware. Both ESP32s **auto-pair** via beacon, with hardcoded MAC fallback. The receiver **ACKs** every alert (`ACK:AMB`), and the sender retries up to 3× on failure.
>
> **Requires ESP32 Board Package v3.x+** (Espressif). The ESP-NOW callback API changed in v3.x — older v2.x code will not compile. Update via Arduino IDE: Tools → Board Manager → search "esp32".

### 5.2a Find Your Board's MAC Address (Optional)

If you have different ESP32 boards, flash the MAC finder utility first:

1. Open `esp32_firmware/find_mac_address/find_mac_address.ino` in Arduino IDE
2. Select your board (ESP32 Dev Module or AI Thinker ESP32-CAM)
3. Upload and open Serial Monitor at 115200 baud
4. The MAC address will be printed — copy it for firmware configuration

Current known MAC addresses:
| Board | MAC Address |
|-------|-------------|
| ESP32-CAM (Sender) | `A8:42:E3:56:83:2C` |
| ESP32 DevKit (Receiver) | `FC:E8:C0:7A:B7:A0` |

### 5.2 Wiring

See [WIRING.md](WIRING.md) for complete wiring diagrams and pin maps.

**Receiver ESP32 → STM32 (USART3):**

| ESP32 Pin | STM32 Pin | Function |
|-----------|-----------|----------|
| GPIO 17 (TX2) | PB11 (USART3 RX) | Data: ESP32 → STM32 |
| GPIO 16 (RX2) | PB10 (USART3 TX) | Data: STM32 → ESP32 (optional) |
| GND | GND | Common ground (mandatory!) |

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

> **Note:** No LCD is connected to the ESP32s. The 16×2 LCD display is on the STM32F103RB only (PC6/PC7 bit-banged I2C).

### 5.3 Flash Receiver ESP32 (Do This First!)

1. Open `esp32_firmware/stm32_esp32_receiver/stm32_esp32_receiver.ino` in Arduino IDE
2. WiFi credentials are pre-configured:
   ```cpp
   #define WIFI_SSID     "Vivo_V29_5G"
   #define WIFI_PASSWORD "123456789"
   ```
   Edit these if your WiFi network is different.
3. Board Settings:
   - Board: **ESP32 Dev Module** (or your specific DevKit)
   - Upload Speed: 115200
4. Upload and open Serial Monitor at 115200 baud
5. You'll see the receiver broadcasting auto-pairing beacons:
   ```
   [Info] This ESP32 MAC: FC:E8:C0:7A:B7:A0
   [PAIR] Beacon sent: "PAIR:RCV" (waiting for sender...)
   ```
   LED blinks fast while waiting for sender to connect.

### 5.4 Flash ESP32-CAM (Sender)

1. Open `esp32_firmware/esp32_cam_sender/esp32_cam_sender.ino` in Arduino IDE
2. Install required library via Library Manager:
   - **ArduinoJson** by Benoit Blanchon (v6.x or v7.x)
3. WiFi and MAC addresses are pre-configured (auto-pairs with receiver, hardcoded MAC fallback):
   ```cpp
   #define WIFI_SSID           "Vivo_V29_5G"
   #define WIFI_PASSWORD       "123456789"
   #define ROBOFLOW_API_URL    "https://detect.roboflow.com/YOUR_PROJECT/VERSION"
   #define ROBOFLOW_API_KEY    "YOUR_API_KEY"
   #define ROBOFLOW_TARGET_CLASS "Ambulance"
   #define MONITORED_LANE_ID   0
   // Sender MAC:   A8:42:E3:56:83:2C
   // Receiver MAC: FC:E8:C0:7A:B7:A0 (hardcoded fallback)
   ```
   Edit Roboflow API details for your project.
4. Board Settings:
   - Board: **AI Thinker ESP32-CAM**
   - Partition Scheme: **Huge APP (3MB No OTA/1MB SPIFFS)**
   - Upload Speed: 115200
5. Connect FTDI adapter, hold IO0 to GND, press Reset, then Upload
6. After upload: remove IO0 jumper, press Reset
7. Open Serial Monitor at 115200 baud — it auto-pairs with receiver:
   ```
   [PAIR] Searching for receiver...
   [PAIR] Beacon from FC:E8:C0:7A:B7:A0
   [PAIR] PAIRED with receiver: FC:E8:C0:7A:B7:A0
   [WiFi] Connected! IP: 192.168.1.50
   ```

### 5.5 Run Laptop ML → ESP32-CAM Trigger

**WiFi mode (recommended):**
```bash
python3 src/siren_to_esp32.py --mode wifi --esp32-ip 192.168.1.50 --threshold 0.80
```

**Check ESP32-CAM status:**
```bash
python3 src/siren_to_esp32.py --check-esp32 --esp32-ip 192.168.1.50
```

### 5.6 How It Works End-to-End

1. Laptop mic captures 3-second audio windows at 16 kHz
2. TFLite model classifies: `emergency` vs `non_emergency`
3. If siren detected (≥ threshold, consecutive hits) → HTTP GET to ESP32-CAM
4. Trigger queued to **FreeRTOS background task** (main loop stays responsive for HTTP server)
5. ESP32-CAM captures JPEG frame with OV2640 camera
6. Raw JPEG binary POSTed to Roboflow inference API (no Base64 — saves ~50KB RAM)
7. If `"Ambulance"` detected with ≥ 60% confidence:
   - ESP-NOW sends `"AMB:<lane_id>"` to receiver ESP32
   - Receiver sends `"ACK:AMB"` back to sender (application-layer ACK)
   - Sender retries up to 3× if no ACK within 500ms
   - Receiver forwards `"AMB:<lane_id>"` via UART to STM32
   - STM32 traffic controller gives priority to that lane
8. 60-second cooldown before next detection (applied immediately on trigger)

### 5.7 STM32 Traffic Controller

The STM32 firmware is in `stm32_firmware/stm32_traffic_controller.c`. It's a bare-metal C file (no HAL/CubeIDE required).

**To use in STM32CubeIDE:**
1. Create a new STM32CubeIDE project for NUCLEO-F103RB
2. Copy the relevant sections from `stm32_traffic_controller.c` into `main.c` (between `USER CODE BEGIN/END` blocks)
3. Build and flash

**The STM32 firmware handles:**
- 4-direction traffic light FSM (10s green / 3s yellow / 1s all-red)
- Pedestrian buttons (PB5-PB8) with walk phase
- Emergency vehicle override via USART3 (receives `AMB:<lane>\n`)
- 16x2 LCD display (on STM32 only, PC6/PC7 I2C)
- Debug output on USART2 (ST-Link VCP) at 115200 baud

### 5.8 ESP32-CAM HTTP Endpoints

| Endpoint | Response |
|----------|----------|
| `GET /trigger` | Starts camera → Roboflow → ESP-NOW pipeline |
| `GET /status` | JSON with uptime, camera/ESP-NOW health, trigger counts, lane |
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
| Check ESP32-CAM | `python3 src/siren_to_esp32.py --check-esp32 --esp32-ip <IP>` |
| List audio devices | `python3 src/inference_pi.py --list-devices` |

---

## 7. Troubleshooting

| Problem | Solution |
|---------|----------|
| ESP32-CAM WiFi won't connect | Double-check SSID/password (`Vivo_V29_5G` / `123456789`), ensure 2.4 GHz network |
| Roboflow API error | Verify API key, project URL, and internet access |
| ESP-NOW send fails | Both ESP32s must be on same WiFi network (`Vivo_V29_5G`, same channel) |
| Receiver not getting messages | Verify receiver MAC (`FC:E8:C0:7A:B7:A0`) matches. Use `find_mac_address.ino` to check |
| Camera init fails | Ensure PSRAM present, check OV2640 ribbon cable |
| LCD blank / not working | LCD is on STM32 only (PC6/PC7). Try `LCD_I2C_ADDR 0x3F`, check wiring, ensure 5V power |
| STM32 not responding | Check UART: ESP32 GPIO 17 → STM32 **PB11**, GND connected |
| Detection too sensitive | Increase `--threshold` (e.g. 0.90) or `--consecutive` (e.g. 3) |
| No audio input | Run `--list-devices`, check mic is connected |
| Serial port permission | `sudo usermod -aG dialout $USER`, then re-login |
| Need to find MAC address | Flash `esp32_firmware/find_mac_address/find_mac_address.ino` to the board |
| ESP-NOW ACK timeout | Sender retries 3× with 500ms wait. If all fail, check receiver is running, paired, and on same WiFi channel |
| Compile error on ESP32 | Requires **ESP32 Board Package v3.x+** (Espressif). Update via Arduino Board Manager |
| TensorFlow import error | Use Python 3.10–3.12, not 3.13+ |

---

Feel free to adjust paths and parameters for your environment.
