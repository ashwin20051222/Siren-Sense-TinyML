# 🔌 Complete System Wiring Guide

> **Smart Traffic Light System with Ambulance Detection**  
> STM32 NUCLEO-F103RB + ESP32-CAM (AI-Thinker) + 2× SX1278 LoRa

---

## System Overview

```
┌──────────────────────────────────────────────────────────────────────┐
│                    REMOTE UNIT  (up to 1 km away)                   │
│                                                                      │
│   Sound Sensor ──→ ESP32-CAM ──→ Roboflow API (WiFi)               │
│                    OV2640 cam      ambulance? yes/no                │
│                         │                                            │
│                    SX1278 LoRa TX ─── "AMB:0" ───┐                  │
└──────────────────────────────────────────────────┬──────────────────┘
                                         433 MHz RF│ (~1 km range)
┌──────────────────────────────────────────────────┼──────────────────┐
│                    JUNCTION UNIT                  │                  │
│                                                   │                  │
│                    SX1278 LoRa RX ←──────────────┘                  │
│                         │                                            │
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

1. **LoRa = 3.3V ONLY!** — Powering SX1278 with 5V will instantly fry it
2. **Common Ground** — All GNDs must be tied together (ESP32, STM32, LoRa, sensors)
3. **ESP32-CAM needs strong power** — Use 5V/2A supply, laptop USB may cause brownout resets

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
| North Ped 🟢 Green | **PB10**  | Green LED → 220Ω → GND     |
| East Ped 🔴 Red    | **PB11**  | Red LED → 220Ω → GND       |
| East Ped 🟢 Green  | **PB12**  | Green LED → 220Ω → GND     |
| South Ped 🔴 Red   | **PC0**   | Red LED → 220Ω → GND       |
| South Ped 🟢 Green | **PC1**   | Green LED → 220Ω → GND     |
| West Ped 🔴 Red    | **PC4**   | Red LED → 220Ω → GND       |
| West Ped 🟢 Green  | **PC5**   | Green LED → 220Ω → GND     |

**Total: 4 push buttons + 8 LEDs + 8× 220Ω resistors**

---

## 1.3 LoRa SX1278 Receiver → STM32 (SPI1)

| SX1278 Pin | STM32 Pin | Function          |
|------------|-----------|-------------------|
| VCC        | **3V3**   | ⚠️ 3.3V ONLY!    |
| GND        | **GND**   | Ground            |
| NSS (CS)   | **PA4**   | SPI1 Chip Select  |
| SCK        | **PA5**   | SPI1 Clock        |
| MISO       | **PA6**   | SPI1 MISO         |
| MOSI       | **PA7**   | SPI1 MOSI         |
| RST        | **PC2**   | Module Reset      |
| DIO0       | **PC3**   | RX Done Interrupt |

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

**Baud: 115200** — Just plug in the USB cable and open a serial terminal.

---

# 🚑 PART 2: ESP32-CAM (Remote Ambulance Detector)

## 2.1 LoRa SX1278 Transmitter → ESP32-CAM (HSPI)

> Uses SD card pins since camera occupies most GPIOs

| SX1278 Pin | ESP32-CAM GPIO | Function         |
|------------|----------------|------------------|
| VCC        | **3.3V**       | ⚠️ 3.3V ONLY!   |
| GND        | **GND**        | Ground           |
| NSS (CS)   | **GPIO 15**    | HSPI CS          |
| SCK        | **GPIO 14**    | HSPI Clock       |
| MISO       | **GPIO 12**    | HSPI MISO        |
| MOSI       | **GPIO 13**    | HSPI MOSI        |
| RST        | **GPIO 2**     | Module Reset     |
| DIO0       | **NOT CONNECTED** | See warning below |

> [!CAUTION]
> **DIO0 is NOT connected on ESP32-CAM!** GPIO 4 is hard-wired to the flash LED. Since the ESP32 only transmits (never receives), DIO0 is not needed. Leave it disconnected.

---

## 2.2 Sound Sensor → ESP32-CAM

| Sensor Pin      | ESP32-CAM GPIO | Notes                  |
|-----------------|----------------|------------------------|
| AO (Analog Out) | **GPIO 33**    | ADC1_CH5, input-only   |
| VCC             | **3.3V or 5V** | Check sensor module    |
| GND             | **GND**        | Common ground          |

---

## 2.3 Camera OV2640 (On-Board — No Wiring)

The camera is fixed on the AI-Thinker board via ribbon cable. **Do not modify.**

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

# 📡 PART 3: LoRa Link (ESP32 ↔ STM32)

The two SX1278 modules communicate wirelessly at **433 MHz**. These settings **MUST be identical** on both sides:

| Parameter        | Value   | Set in (ESP32)            | Set in (STM32)         |
|------------------|---------|---------------------------|------------------------|
| Frequency        | 433 MHz | `config.h`                | `lora_receiver.c`      |
| Spreading Factor | SF7     | `LORA_SPREADING_FACTOR 7` | `LORA_SF_7`            |
| Bandwidth        | 125 kHz | `LORA_BANDWIDTH 125E3`   | `LORA_BW_125K`         |
| Coding Rate      | 4/5     | `LORA_CODING_RATE 5`     | `LORA_CR_4_5`          |
| Sync Word        | 0x12    | `LORA_SYNC_WORD 0x12`    | `LORA_SYNC_WORD 0x12`  |
| TX Power         | 17 dBm  | `LORA_TX_POWER 17`       | — (receiver)           |

### LoRa Message Protocol

```
ESP32 sends:   "AMB:<lane>"     (e.g. "AMB:0" for North lane)
STM32 parses:  lane_id = 0      → makes North lane green for 15s, all others red
```

> [!IMPORTANT]
> If LoRa fails on either side, first check that **both modules use 3.3V** and that **SPI wiring is correct** (SCK, MISO, MOSI, NSS).

---

# ⚡ Power Budget

| Component       | Voltage | Peak Current | Supply Source              |
|-----------------|---------|-------------|----------------------------|
| NUCLEO-F103RB   | 5V USB  | ~100 mA     | Laptop/USB charger         |
| SX1278 (STM32)  | 3.3V    | ~10 mA (RX) | NUCLEO 3.3V regulator      |
| 12 Traffic LEDs | 3.3V    | ~240 mA     | NUCLEO GPIOs (20mA each)   |
| 8 Ped LEDs      | 3.3V    | ~160 mA     | NUCLEO GPIOs               |
| LCD 16×2        | 5V      | ~40 mA      | NUCLEO 5V pin              |
| IR Sensors (×4) | 3.3V    | ~80 mA      | NUCLEO 3.3V                |
| **STM32 Total** |         | **~630 mA** | **Use powered USB hub**    |
| | | | |
| ESP32-CAM       | 5V      | ~310 mA     | External 5V/2A supply      |
| SX1278 (ESP32)  | 3.3V    | ~120 mA (TX)| ESP32 3.3V regulator       |
| Sound Sensor    | 3.3V    | ~5 mA       | ESP32 3.3V                 |
| **ESP32 Total** |         | **~435 mA** | **Dedicated power supply** |

---

# 🗺️ NUCLEO-F103RB Pin Usage Map

```
           PA Pins                        PB Pins
  ┌─────────────────────┐       ┌─────────────────────┐
  │ PA0  = TL1 Red      │       │ PB0  = (free)       │
  │ PA1  = TL1 Yellow   │       │ PB1  = (free)       │
  │ PA2  = USART2 TX ⚡ │       │ PB3  = (free)       │
  │ PA3  = USART2 RX ⚡ │       │ PB4  = (free)       │
  │ PA4  = LoRa NSS     │       │ PB5  = Ped Btn N    │
  │ PA5  = LoRa SCK     │       │ PB6  = Ped Btn E    │
  │ PA6  = LoRa MISO    │       │ PB7  = Ped Btn S    │
  │ PA7  = LoRa MOSI    │       │ PB8  = Ped Btn W    │
  │ PA8  = TL1 Green    │       │ PB9  = Ped N Red    │
  │ PA9  = TL2 Red      │       │ PB10 = Ped N Green  │
  │ PA10 = TL2 Yellow   │       │ PB11 = Ped E Red    │
  │ PA11 = TL2 Green    │       │ PB12 = Ped E Green  │
  │ PA12 = IR West      │       │ PB13 = TL3 Red      │
  │ PA15 = (free)       │       │ PB14 = TL3 Yellow   │
  └─────────────────────┘       │ PB15 = TL3 Green    │
                                └─────────────────────┘
           PC Pins                        PD Pins
  ┌─────────────────────┐       ┌─────────────────────┐
  │ PC0  = Ped S Red    │       │ PD2  = IR South     │
  │ PC1  = Ped S Green  │       └─────────────────────┘
  │ PC2  = LoRa RST     │
  │ PC3  = LoRa DIO0    │
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

---

# �️ ESP32-CAM GPIO Map (AI-Thinker)

```
             ┌──────────────────┐
             │  ○ ○  ANTENNA    │
             │                  │
       5V ───┤ 5V          GND ├─── GND
      GND ───┤ GND    GPIO  13 ├─── LoRa MOSI
  LoRa MISO──┤ GPIO 12  GPIO 15├─── LoRa NSS
  LoRa MOSI──┤ GPIO 13  GPIO 14├─── LoRa SCK
  LoRa NSS ──┤ GPIO 15  GPIO  2├─── LoRa RST
  LoRa SCK ──┤ GPIO 14  GPIO  4├─── ⚠️ FLASH LED (DO NOT USE)
  LoRa RST ──┤ GPIO 2   GPIO 33├─── Sound Sensor AO
      3.3V ──┤ 3V3      U0T   1├─── FTDI RX
             │           U0R   3├─── FTDI TX
             │                  │
             │   [OV2640 CAM]   │
             └──────────────────┘
```

---

# 🧪 Testing Checklist

After wiring, verify each subsystem:

- [ ] **STM32 boots** — Blue user LED blinks, serial output at 115200 baud
- [ ] **Traffic LEDs cycle** — N→E→S→W round-robin (10s green, 3s yellow, 1s all-red)
- [ ] **Pedestrian buttons** — Press any button, walk phase activates after current green
- [ ] **LCD** — Shows countdown timers for each lane
- [ ] **STM32 LoRa RX** — Serial shows `[LoRa] Initialized OK`
- [ ] **ESP32-CAM boots** — Serial shows WiFi ✅, Camera ✅, LoRa ✅
- [ ] **End-to-end** — Play siren → laptop detects → ESP32 camera → Roboflow → LoRa → STM32 → ambulance lane goes green

> [!TIP]
> Print this file and keep it at your workbench while wiring!
