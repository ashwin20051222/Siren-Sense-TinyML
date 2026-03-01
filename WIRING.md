# 🔌 Complete System Wiring Guide

> **Smart Traffic Light System with Ambulance Detection**  
> STM32 NUCLEO-F103RB + ESP32-CAM (Sender) + ESP32 DevKit (Receiver)  
> Communication: ESP-NOW WiFi (ESP32-CAM ↔ ESP32) + UART (ESP32 → STM32)

---

## System Overview

```
┌──────────────────────────────────────────────────────────────────────┐
│                    REMOTE UNIT  (up to ~200m away)                   │
│                                                                      │
│   Laptop ML ──WiFi──→ ESP32-CAM ──→ Roboflow API (WiFi)            │
│   Sound Sensor ──→    OV2640 cam     ambulance? yes/no              │
│                           │                                          │
│                    ESP-NOW WiFi TX ── "AMB:0" ──┐                   │
└──────────────────────────────────────────────────┬──────────────────┘
                                        ESP-NOW RF │ (~100–200m range)
┌──────────────────────────────────────────────────┼──────────────────┐
│                    JUNCTION UNIT                  │                  │
│                                                   │                  │
│              ESP32 DevKit (Receiver)              │                  │
│                    ESP-NOW WiFi RX ←─────────────┘                  │
│                         │ UART (GPIO 17 TX)                         │
│                         ▼                                            │
│                 NUCLEO-F103RB (STM32)                                │
│                    │    │    │    │                                   │
│              ┌─────┘    │    │    └─────────┐                        │
│         4× Traffic   4× Ped  LCD     IR Sensors                     │
│          Light LEDs  Buttons  16×2   (optional)                     │
│          (R/Y/G)     + LEDs   I2C                                   │
└──────────────────────────────────────────────────────────────────────┘
```

---

## ⚠️ Three Golden Rules

1. **Common Ground** — All GNDs must be tied together (ESP32-CAM, ESP32 receiver, STM32)
2. **ESP32-CAM needs strong power** — Use 5V/2A supply, laptop USB may cause brownout resets
3. **Same WiFi Network** — Both ESP32s must connect to the same 2.4 GHz WiFi for ESP-NOW

---

# 🚦 PART 1: STM32 NUCLEO-F103RB (Junction Controller)

## 1.1 Traffic Light LEDs (4 lanes × 3 colors = 12 LEDs)

> Each LED: GPIO pin → 220Ω resistor → LED anode → LED cathode → GND

| Lane | Color  | STM32 Pin | Wire To           |
|------|--------|-----------|-------------------|
| **1 (North)** | 🔴 Red    | **PA0**  | TL1 Red LED    |
|                | 🟡 Yellow | **PA1**  | TL1 Yellow LED |
|                | 🟢 Green  | **PA8**  | TL1 Green LED  |
| **2 (East)**  | 🔴 Red    | **PA9**  | TL2 Red LED    |
|                | 🟡 Yellow | **PA10** | TL2 Yellow LED |
|                | 🟢 Green  | **PA11** | TL2 Green LED  |
| **3 (South)** | 🔴 Red    | **PB13** | TL3 Red LED    |
|                | 🟡 Yellow | **PB14** | TL3 Yellow LED |
|                | 🟢 Green  | **PB15** | TL3 Green LED  |
| **4 (West)**  | 🔴 Red    | **PC8**  | TL4 Red LED    |
|                | 🟡 Yellow | **PC9**  | TL4 Yellow LED |
|                | 🟢 Green  | **PC10** | TL4 Green LED  |

**Total: 12 GPIO pins, 12× 220Ω resistors, 12 LEDs**

---

## 1.2 Pedestrian Buttons & Indicator LEDs

> Buttons: GPIO → Push Button → GND (internal pull-up, no external resistor)  
> LEDs: GPIO → 220Ω resistor → LED → GND

| Function         | STM32 Pin | Wire To                       |
|------------------|-----------|-------------------------------|
| North Button     | **PB5**   | Push button → GND             |
| East Button      | **PB6**   | Push button → GND             |
| South Button     | **PB7**   | Push button → GND             |
| West Button      | **PB8**   | Push button → GND             |
| North Ped 🔴 Red   | **PB9**   | Red LED → 220Ω → GND       |
| North Ped 🟢 Green | **PB4**   | Green LED → 220Ω → GND     |
| East Ped 🔴 Red    | **PB3**   | Red LED → 220Ω → GND       |
| East Ped 🟢 Green  | **PB12**  | Green LED → 220Ω → GND     |
| South Ped 🔴 Red   | **PC0**   | Red LED → 220Ω → GND       |
| South Ped 🟢 Green | **PC1**   | Green LED → 220Ω → GND     |
| West Ped 🔴 Red    | **PC4**   | Red LED → 220Ω → GND       |
| West Ped 🟢 Green  | **PC5**   | Green LED → 220Ω → GND     |

> [!NOTE]
> PB10/PB11 are reserved for USART3 (ESP32 communication). North Ped Green moved to PB4, East Ped Red moved to PB3.

**Total: 4 push buttons + 8 LEDs + 8× 220Ω resistors**

---

## 1.3 ESP32 Receiver → STM32 (USART3)

> The receiver ESP32 connects to the STM32 via **USART3** (PB10/PB11) to forward ambulance alerts. PA2/PA3 (USART2) is reserved for ST-Link debug serial.

| ESP32 Receiver Pin | STM32 Pin | Function       |
|--------------------|-----------|----------------|
| **GPIO 17** (TX2)  | **PB11**  | USART3 RX (ESP32 TX → STM32 RX) |
| **GPIO 16** (RX2)  | **PB10**  | USART3 TX (STM32 TX → ESP32 RX, optional) |
| **GND**            | **GND**   | ⚠️ Common Ground — MUST connect! |

> [!IMPORTANT]
> The ESP32 uses 3.3V logic levels, which are compatible with STM32F103RB (also 3.3V). No level shifter needed.

> [!WARNING]
> Wire ESP32 GPIO 17 to STM32 **PB11** (not PA3). PA2/PA3 is used by ST-Link debug output.

---

## 1.4 I2C LCD Display (16×2) → STM32

| LCD Pin | STM32 Pin | Function     |
|---------|-----------|--------------|
| VCC     | **5V**    | Power        |
| GND     | **GND**   | Ground       |
| SDA     | **PC6**   | I2C Data     |
| SCL     | **PC7**   | I2C Clock    |

> [!NOTE]
> The LCD uses a PCF8574 I2C backpack (default address: 0x27). If the display is blank, try address 0x3F.

---

## 1.5 IR Sensors (Optional — Vehicle Detection)

> IR obstacle sensors output LOW when a vehicle is detected.  
> Connect: VCC → 3.3V, GND → GND, OUT → GPIO pin

| Sensor    | STM32 Pin | Position             |
|-----------|-----------|----------------------|
| IR North  | **PC11**  | North lane approach  |
| IR East   | **PC12**  | East lane approach   |
| IR South  | **PD2**   | South lane approach  |
| IR West   | **PA12**  | West lane approach   |

> [!TIP]
> IR sensors are optional for the basic traffic light operation. They can be added later for vehicle-density-aware green timing.

---

## 1.6 Debug Serial (Built-in — No Wiring Needed)

The NUCLEO board has built-in ST-Link with a Virtual COM Port:

| Function | STM32 Pin | Notes                     |
|----------|-----------|---------------------------|
| USART2 TX| **PA2**   | Auto-connected via ST-Link|
| USART2 RX| **PA3**   | Auto-connected via ST-Link|

> [!NOTE]
> USART2 (PA2/PA3) is dedicated to ST-Link debug output. ESP32 receiver uses USART3 (PB10/PB11) — no conflict.

**Baud: 115200** — Just plug in the USB cable and open a serial terminal.

---

# 🚑 PART 2: ESP32-CAM (Sender — Remote Ambulance Detector)

## 2.1 Camera OV2640 (On-Board — No Wiring)

The camera is fixed on the AI-Thinker board via ribbon cable. **Do not modify.**

---

## 2.2 Sound Sensor → ESP32-CAM

| Sensor Pin      | ESP32-CAM GPIO | Notes                  |
|-----------------|----------------|------------------------|
| AO (Analog Out) | **GPIO 33**    | ADC1_CH5, input-only   |
| VCC             | **3.3V or 5V** | Check sensor module    |
| GND             | **GND**        | Common ground          |

---

## 2.3 I2C LCD Display (16×2) → ESP32-CAM (Optional)

> Shares GPIO 26/27 with the camera's SCCB bus. Uses Wire1 (second I2C peripheral) so there's no conflict with the camera driver.

| LCD Pin | ESP32-CAM GPIO | Notes                              |
|---------|----------------|--------------------------------------|
| SDA     | **GPIO 26**    | Shared with camera SIOD            |
| SCL     | **GPIO 27**    | Shared with camera SIOC            |
| VCC     | **5V**         | Power (via backpack regulator)     |
| GND     | **GND**        | Ground                             |

> [!NOTE]
> The LCD uses a PCF8574 I2C backpack (default address: 0x27). If blank, try 0x3F. Set `LCD_I2C_ADDR` in the firmware. Set `LCD_ENABLED 0` to disable.

---

## 2.4 FTDI Programmer (For Uploading Code)

| FTDI Pin | ESP32-CAM Pin     | Notes                    |
|----------|-------------------|--------------------------|
| TX       | **U0R (GPIO 3)**  | FTDI TX → ESP32 RX      |
| RX       | **U0T (GPIO 1)**  | FTDI RX → ESP32 TX      |
| GND      | **GND**           | Common ground            |
| 5V       | **5V**            | Power (if no USB power)  |

### Upload Steps
1. Jumper **GPIO 0 → GND**
2. Press **RST** on ESP32-CAM
3. Click **Upload** in Arduino IDE (Board: AI Thinker ESP32-CAM)
4. Remove GPIO 0 jumper after upload
5. Press **RST** to run firmware

---

# 📡 PART 3: ESP32 DevKit (Receiver — Junction Side)

## 3.1 UART Connection → STM32 (USART3)

| ESP32 Pin        | STM32 Pin | Function                |
|------------------|-----------|-------------------------|
| **GPIO 17** (TX2)| **PB11**  | ESP32 TX → STM32 USART3 RX |
| **GPIO 16** (RX2)| **PB10**  | STM32 USART3 TX → ESP32 RX (optional) |
| **GND**          | **GND**   | ⚠️ Common Ground       |

---

## 3.2 I2C LCD Display (16×2) → ESP32 Receiver (Optional)

| LCD Pin | ESP32 GPIO   | Notes                     |
|---------|-------------|---------------------------|
| SDA     | **GPIO 21** | Default I2C SDA           |
| SCL     | **GPIO 22** | Default I2C SCL           |
| VCC     | **5V**      | Power                     |
| GND     | **GND**     | Ground                    |

---

## 3.3 Power

The ESP32 DevKit is powered via its USB port. No special power supply needed.

---

# 📡 PART 4: ESP-NOW WiFi Link (ESP32-CAM ↔ ESP32 Receiver)

The two ESP32 boards communicate wirelessly using **ESP-NOW** (built-in WiFi protocol). No extra hardware needed!

### How ESP-NOW Works

| Feature          | Value                           |
|------------------|---------------------------------|
| Protocol         | ESP-NOW (built into ESP-IDF)    |
| Range            | ~100–200m (line of sight)       |
| Latency          | < 5ms                           |
| Encryption       | Optional (disabled by default)  |
| Data size        | Up to 250 bytes per packet      |
| Requires router  | No (peer-to-peer), but both must be on same WiFi channel |

### Setup Steps

1. **Flash the receiver ESP32 first** (`stm32_esp32_receiver.ino`)
2. Open Serial Monitor at 115200 baud
3. Look for this line:
   ```
   ┌─────────────────────────────────────────────┐
   │  THIS ESP32 MAC: AA:BB:CC:DD:EE:FF          │
   │  ↑↑ Copy this to sender's RECEIVER_MAC! ↑↑  │
   └─────────────────────────────────────────────┘
   ```
4. Copy the MAC address
5. Edit `esp32_cam_sender.ino` — update `RECEIVER_MAC`:
   ```cpp
   static uint8_t RECEIVER_MAC[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
   ```
6. Flash the ESP32-CAM sender

### Message Protocol

```
ESP32-CAM sends:   "AMB:<lane>"     (e.g. "AMB:0" for North lane)
Receiver parses:   lane_id = 0
Receiver forwards: "AMB:0\n" via UART to STM32
STM32 action:      Makes North lane green for 15s, all others red
```

> [!IMPORTANT]
> Both ESP32 boards must connect to the **same WiFi network** (same 2.4 GHz SSID). ESP-NOW uses the WiFi channel, so they must be on the same channel. Connecting to the same network ensures this automatically.

---

# ⚡ Power Budget

| Component            | Voltage | Peak Current | Supply Source              |
|----------------------|---------|-------------|----------------------------|
| NUCLEO-F103RB        | 5V USB  | ~100 mA     | Laptop/USB charger         |
| 12 Traffic LEDs      | 3.3V    | ~240 mA     | NUCLEO GPIOs (20mA each)   |
| 8 Ped LEDs           | 3.3V    | ~160 mA     | NUCLEO GPIOs               |
| LCD 16×2 (STM32)     | 5V      | ~40 mA      | NUCLEO 5V pin              |
| IR Sensors (×4)      | 3.3V    | ~80 mA      | NUCLEO 3.3V                |
| **STM32 Total**      |         | **~620 mA** | **Use powered USB hub**    |
| | | | |
| ESP32-CAM (Sender)   | 5V      | ~310 mA     | External 5V/2A supply      |
| Sound Sensor         | 3.3V    | ~5 mA       | ESP32-CAM 3.3V             |
| LCD 16×2 (ESP32-CAM) | 5V      | ~40 mA      | ESP32-CAM 5V pin           |
| **ESP32-CAM Total**  |         | **~355 mA** | **Dedicated power supply** |
| | | | |
| ESP32 DevKit (Recv)  | 5V USB  | ~80 mA      | USB port on STM32/laptop   |
| LCD 16×2 (Receiver)  | 5V      | ~40 mA      | ESP32 DevKit 5V pin        |
| **Receiver Total**   |         | **~120 mA** | **USB power sufficient**   |

---

# 🗺️ Pin Usage Maps

## NUCLEO-F103RB

```
           PA Pins                        PB Pins
  ┌─────────────────────┐       ┌─────────────────────┐
  │ PA0  = TL1 Red      │       │ PB0  = (free)       │
  │ PA1  = TL1 Yellow   │       │ PB1  = (free)       │
  │ PA2  = Debug TX ⚡  │       │ PB3  = Ped E Red    │
  │ PA3  = Debug RX ⚡  │       │ PB4  = Ped N Green  │
  │ PA4  = (free)       │       │ PB5  = Ped Btn N    │
  │ PA5  = (free)       │       │ PB6  = Ped Btn E    │
  │ PA6  = (free)       │       │ PB7  = Ped Btn S    │
  │ PA7  = (free)       │       │ PB8  = Ped Btn W    │
  │ PA8  = TL1 Green    │       │ PB9  = Ped N Red    │
  │ PA9  = TL2 Red      │       │ PB10 = USART3 TX ⚡ │
  │ PA10 = TL2 Yellow   │       │ PB11 = USART3 RX ⚡ │
  │ PA11 = TL2 Green    │       │ PB12 = Ped E Green  │
  │ PA12 = IR West      │       │ PB13 = TL3 Red      │
  │ PA15 = (free)       │       │ PB14 = TL3 Yellow   │
  └─────────────────────┘       │ PB15 = TL3 Green    │
                                └─────────────────────┘
           PC Pins                        PD Pins
  ┌─────────────────────┐       ┌─────────────────────┐
  │ PC0  = Ped S Red    │       │ PD2  = IR South     │
  │ PC1  = Ped S Green  │       └─────────────────────┘
  │ PC2  = (free)       │
  │ PC3  = (free)       │
  │ PC4  = Ped W Red    │
  │ PC5  = Ped W Green  │
  │ PC6  = LCD SDA      │
  │ PC7  = LCD SCL      │
  │ PC8  = TL4 Red      │
  │ PC9  = TL4 Yellow   │
  │ PC10 = TL4 Green    │
  │ PC11 = IR North     │
  │ PC12 = IR East      │
  │ PC13 = User LED ⚡  │
  └─────────────────────┘
```

## ESP32-CAM (AI-Thinker) — Sender

```
             ┌──────────────────┐
             │  ○ ○  ANTENNA    │
             │                  │
       5V ───┤ 5V          GND ├─── GND
      GND ───┤ GND    GPIO  13 ├─── (free)
    (free) ──┤ GPIO 12  GPIO 15├─── (free)
    (free) ──┤ GPIO 13  GPIO 14├─── (free)
    (free) ──┤ GPIO 15  GPIO  2├─── (free)
    (free) ──┤ GPIO 14  GPIO  4├─── ⚠️ FLASH LED (DO NOT USE)
    (free) ──┤ GPIO 2   GPIO 33├─── Sound Sensor AO
      3.3V ──┤ 3V3      U0T   1├─── FTDI RX
             │           U0R   3├─── FTDI TX
             │                  │
             │   [OV2640 CAM]   │
             └──────────────────┘

  Note: GPIO 12-15 are now FREE (no LoRa module needed!)
  LCD uses GPIO 26/27 (camera SCCB pins, via Wire1)
```

## ESP32 DevKit (Receiver)

```
             ┌──────────────────┐
             │    ESP32 DevKit  │
             │                  │
       5V ───┤ VIN         GND ├─── GND (→ STM32 GND)
      GND ───┤ GND    GPIO  17 ├─── STM32 PB11 (USART3 RX)
             │ ...    GPIO  16 ├─── STM32 PB10 (USART3 TX, optional)
             │        GPIO  21 ├─── LCD SDA (optional)
             │        GPIO  22 ├─── LCD SCL (optional)
             │        GPIO   2 ├─── Built-in LED
             │                  │
             │    [USB PORT]    │
             └──────────────────┘
```

---

# 🧪 Testing Checklist

After wiring, verify each subsystem:

- [ ] **STM32 boots** — Blue user LED blinks, serial output at 115200 baud
- [ ] **Traffic LEDs cycle** — N→E→S→W round-robin (10s green, 3s yellow, 1s all-red)
- [ ] **Pedestrian buttons** — Press any button, walk phase activates after current green
- [ ] **LCD (STM32)** — Shows countdown timers for each lane
- [ ] **ESP32-CAM boots** — Serial shows WiFi ✅, Camera ✅, ESP-NOW ✅
- [ ] **ESP32-CAM MAC** — Serial prints `ESP32-CAM MAC: XX:XX:XX:XX:XX:XX`
- [ ] **Receiver ESP32 boots** — Serial shows WiFi ✅, ESP-NOW ✅, MAC address printed
- [ ] **Receiver MAC copied** — MAC from receiver's Serial Monitor → sender's `RECEIVER_MAC`
- [ ] **ESP-NOW link** — Trigger pipeline on ESP32-CAM → receiver Serial shows `AMB:0` received
- [ ] **UART bridge** — Receiver's `AMB:0` message appears on STM32 serial
- [ ] **End-to-end** — Play siren → laptop detects → ESP32 camera → Roboflow → ESP-NOW → receiver → STM32 → ambulance lane goes green

> [!TIP]
> Print this file and keep it at your workbench while wiring!
