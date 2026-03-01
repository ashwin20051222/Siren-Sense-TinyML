/**
 * ============================================================================
 *  config.h — ESP32-CAM Ambulance Detector Configuration
 *  Board: ESP32-CAM (AI-Thinker) + SX1278 LoRa + Sound Sensor
 * ============================================================================
 *
 *  Edit this file before flashing! All user-configurable settings are here.
 *
 *  Pin Reference (matching STM32 Traffic Light README wiring guide):
 *
 *    LoRa SX1278 (HSPI):
 *      SCK  → GPIO 14    MISO → GPIO 12    MOSI → GPIO 13
 *      NSS  → GPIO 15    RST  → GPIO 2     DIO0 → (not used, set -1)
 *
 *    Sound Sensor:
 *      AO   → GPIO 33 (ADC1_CH5, input-only)
 *
 *    Camera (OV2640 — fixed on AI-Thinker board, do not change):
 *      PWDN → GPIO 32    XCLK → GPIO 0     SIOD → GPIO 26
 *      SIOC → GPIO 27    D0–D7 → see below  VSYNC → GPIO 25
 *      HREF → GPIO 23    PCLK → GPIO 22
 *
 * ============================================================================
 */

#ifndef CONFIG_H
#define CONFIG_H

/* =====================================================================
 *  WiFi CREDENTIALS
 *  Set your 2.4 GHz WiFi network name and password.
 * ===================================================================== */
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

/* =====================================================================
 *  ROBOFLOW API — Object Detection
 *
 *  Go to https://app.roboflow.com → Your Project → Deploy → Hosted API
 *  Copy your model URL and API key.
 *
 *  URL format:
 *    https://detect.roboflow.com/<PROJECT>/<VERSION>
 *  Example:
 *    https://detect.roboflow.com/ambulance-detection/1
 *
 *  Response format (JSON):
 *  {
 *    "predictions": [
 *      { "class": "ambulance", "confidence": 0.92,
 *        "x": 320, "y": 240, "width": 200, "height": 150 }
 *    ],
 *    "image": { "width": 640, "height": 480 }
 *  }
 * ===================================================================== */
#define ROBOFLOW_API_URL "https://detect.roboflow.com/YOUR_PROJECT/YOUR_VERSION"
#define ROBOFLOW_API_KEY "YOUR_ROBOFLOW_API_KEY"
#define ROBOFLOW_TIMEOUT_MS 15000 /* 15s timeout (image upload is slow)   */
#define ROBOFLOW_CONF_MIN 0.60    /* Min confidence to confirm ambulance  */
#define ROBOFLOW_TARGET_CLASS                                                  \
  "ambulance" /* Class name trained in Roboflow       */

/* =====================================================================
 *  LANE ID — Which lane this detector unit monitors
 *  0 = North, 1 = East, 2 = South, 3 = West
 * ===================================================================== */
#define MONITORED_LANE_ID 0

/* =====================================================================
 *  LoRa SX1278 PINS (HSPI)
 *
 *  Wiring (from README):
 *    SX1278 SCK  → ESP32 GPIO 14 (HSPI Clock)
 *    SX1278 MISO → ESP32 GPIO 12 (HSPI MISO)
 *    SX1278 MOSI → ESP32 GPIO 13 (HSPI MOSI)
 *    SX1278 NSS  → ESP32 GPIO 15 (HSPI SS / Chip Select)
 *    SX1278 RST  → ESP32 GPIO 2  (Reset)
 *    SX1278 DIO0 → NOT CONNECTED (set -1, see warning below)
 *    SX1278 VCC  → 3.3V
 *    SX1278 GND  → GND
 * ===================================================================== */
#define LORA_SCK_PIN 14
#define LORA_MISO_PIN 12
#define LORA_MOSI_PIN 13
#define LORA_NSS_PIN 15
#define LORA_RST_PIN 2

/*
 * ⚠️ GPIO 4 WARNING: On AI-Thinker ESP32-CAM, GPIO 4 is hard-wired to the
 * ultra-bright white FLASH LED. Using it for LoRa DIO0 causes the LED to
 * strobe on every packet, and the current draw can cause brownout resets.
 *
 * Since we only TRANSMIT LoRa (never receive), DIO0 is not needed.
 * Set to -1 and leave the DIO0 wire disconnected on the SX1278.
 */
#define LORA_DIO0_PIN -1

/* =====================================================================
 *  LoRa RADIO PARAMETERS — MUST match STM32 receiver settings
 *
 *  These must be identical on both the ESP32-CAM TX and STM32 RX sides.
 *  Default: 433 MHz, SF7, BW 125 kHz, CR 4/5, Sync 0x12, 17 dBm
 * ===================================================================== */
#define LORA_FREQUENCY 433E6    /* 433 MHz band                      */
#define LORA_SPREADING_FACTOR 7 /* SF7: fastest data rate, ~1 km     */
#define LORA_BANDWIDTH 125E3    /* 125 kHz bandwidth                 */
#define LORA_CODING_RATE 5      /* CR 4/5                            */
#define LORA_SYNC_WORD 0x12     /* Private sync word                 */
#define LORA_TX_POWER 17        /* +17 dBm (max for SX1278)          */

/* =====================================================================
 *  SOUND SENSOR — Local Fallback Detection
 *
 *  Wiring (from README):
 *    Sensor AO → ESP32 GPIO 33 (ADC1_CH5, input-only pin)
 *    Sensor VCC → 3.3V or 5V (check your sensor module spec)
 *    Sensor GND → GND (common ground)
 *
 *  Tuning:
 *    SOUND_THRESHOLD   — ADC value (0–4095). Increase if too many false
 *                        triggers, decrease if not sensitive enough.
 *    SOUND_TRIGGER_COUNT — Number of consecutive loud readings required.
 *                        Higher = fewer false positives but slower response.
 * ===================================================================== */
#define SOUND_SENSOR_PIN 33         /* ADC1_CH5, safe on ESP32-CAM       */
#define SOUND_THRESHOLD 2500        /* ADC value 0–4095                  */
#define SOUND_TRIGGER_COUNT 10      /* Consecutive loud readings needed  */
#define SOUND_SAMPLE_INTERVAL_MS 50 /* ms between ADC samples            */

/* =====================================================================
 *  TRIGGER MODE — How the detection pipeline is started
 *
 *  0 = WiFi HTTP only   (laptop sends GET /trigger via WiFi)
 *  1 = UART only        (laptop sends "SIREN\n" over serial cable)
 *  2 = Both WiFi + UART (either can trigger the pipeline)
 * ===================================================================== */
#define TRIGGER_MODE 0

/* =====================================================================
 *  HTTP SERVER — Listens for laptop ML triggers
 * ===================================================================== */
#define HTTP_TRIGGER_PORT 80

/* =====================================================================
 *  mDNS HOSTNAME
 *  ESP32 will be discoverable as: http://<hostname>.local
 *  No need to know the IP address — laptop finds it automatically.
 * ===================================================================== */
#define MDNS_HOSTNAME "ambulance"

/* =====================================================================
 *  TIMING & COOLDOWN
 * ===================================================================== */
#define DETECTION_COOLDOWN_MS 60000 /* 60s between LoRa alerts            */
#define LORA_RETRY_COUNT 3          /* Number of LoRa TX retries          */
#define LORA_RETRY_DELAY_MS 500     /* ms between retries                 */
#define CAMERA_WARMUP_MS 500        /* ms to discard stale frame          */

/* =====================================================================
 *  DEBUG / SERIAL
 * ===================================================================== */
#define SERIAL_BAUD_RATE 115200
#define DEBUG_ENABLED 1

#if DEBUG_ENABLED
#define DBG(...) Serial.printf(__VA_ARGS__)
#define DBGLN(...) Serial.println(__VA_ARGS__)
#else
#define DBG(...)
#define DBGLN(...)
#endif

/* =====================================================================
 *  CAMERA PIN DEFINITIONS — AI-Thinker ESP32-CAM (OV2640)
 *
 *  These pins are fixed on the AI-Thinker board. Do NOT change them.
 *
 *    PWDN  → GPIO 32      RESET → -1 (not connected)
 *    XCLK  → GPIO 0       SIOD  → GPIO 26 (I2C SDA)
 *    SIOC  → GPIO 27 (I2C SCL)
 *    D7(Y9) → GPIO 35     D6(Y8) → GPIO 34
 *    D5(Y7) → GPIO 39     D4(Y6) → GPIO 36
 *    D3(Y5) → GPIO 21     D2(Y4) → GPIO 19
 *    D1(Y3) → GPIO 18     D0(Y2) → GPIO 5
 *    VSYNC → GPIO 25      HREF  → GPIO 23
 *    PCLK  → GPIO 22
 * ===================================================================== */
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#endif /* CONFIG_H */
