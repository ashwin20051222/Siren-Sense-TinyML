/**
 * ============================================================================
 *  ESP32-CAM Sender — Ambulance Detection + ESP-NOW WiFi Transmitter
 *  Board: ESP32-CAM (AI-Thinker) + Sound Sensor + I2C LCD
 * ============================================================================
 *
 *  SYSTEM ARCHITECTURE (Two-ESP32 WiFi):
 *    ┌─────────────────────────────────────────────────────────────┐
 *    │  ESP32-CAM (THIS BOARD)                                    │
 *    │  Laptop ML / Sound Sensor → Camera → Roboflow API          │
 *    │  → ESP-NOW TX → Receiver ESP32 → UART → STM32             │
 *    └─────────────────────────────────────────────────────────────┘
 *
 *  DETECTION PIPELINE:
 *    Laptop ML Audio / Sound Sensor → Camera → Roboflow API → ESP-NOW →
 * Receiver
 *
 *  ENDPOINTS (WiFi):
 *    GET /trigger  → start pipeline    GET /status → JSON status
 *    GET /health   → "OK"
 *
 *  LIBRARIES (Arduino Library Manager):
 *    - ArduinoJson by Benoit Blanchon (v6.x or v7.x)
 *    - ESP32 Board Package (esp32 by Espressif, v2.x+)
 *
 *  LAPTOP: python3 src/siren_to_esp32.py --mode wifi --esp32-ip <THIS_IP>
 * ============================================================================
 */

#include "base64.h"
#include "esp_camera.h"
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 1: USER CONFIGURATION — Edit these before flashing!    ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

/* ---- WiFi (2.4 GHz only) ---- */
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

/* ---- Roboflow API ----
 *  URL: https://detect.roboflow.com/<PROJECT>/<VERSION>
 *  Get API key from: https://app.roboflow.com → Deploy → Hosted API */
#define ROBOFLOW_API_URL "https://detect.roboflow.com/YOUR_PROJECT/YOUR_VERSION"
#define ROBOFLOW_API_KEY "YOUR_ROBOFLOW_API_KEY"
#define ROBOFLOW_TIMEOUT_MS 15000
#define ROBOFLOW_CONF_MIN 0.60
#define ROBOFLOW_TARGET_CLASS "ambulance"

/* ---- Lane ID: 0=North, 1=East, 2=South, 3=West ---- */
#define MONITORED_LANE_ID 0

/* ---- ESP-NOW Receiver MAC Address ----
 *  Flash stm32_esp32_receiver.ino first, then copy the MAC address
 *  from its Serial Monitor output. Format: {0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX}
 *  Example: {0x24, 0x6F, 0x28, 0xAB, 0xCD, 0xEF} */
static uint8_t RECEIVER_MAC[] = {0xFF, 0xFF, 0xFF,
                                 0xFF, 0xFF, 0xFF}; // ← CHANGE THIS!

/* ---- Sound Sensor (GPIO 33, ADC1_CH5) ---- */
#define SOUND_SENSOR_PIN 33
#define SOUND_THRESHOLD 2500
#define SOUND_TRIGGER_COUNT 10
#define SOUND_SAMPLE_INTERVAL_MS 50

/* ---- Trigger Mode: 0=WiFi, 1=UART, 2=Both ---- */
#define TRIGGER_MODE 0
#define HTTP_TRIGGER_PORT 80
#define MDNS_HOSTNAME "ambulance"

/* ---- Timing ---- */
#define DETECTION_COOLDOWN_MS 60000
#define ESPNOW_RETRY_COUNT 3
#define ESPNOW_RETRY_DELAY_MS 300
#define CAMERA_WARMUP_MS 500

/* ---- LCD Display (16×2 I2C, PCF8574 backpack) ----
 *  SDA/SCL share GPIO 26/27 with camera SCCB (uses Wire1, no conflict)
 *  Set LCD_ENABLED to 0 if no LCD connected */
#define LCD_ENABLED 1
#define LCD_I2C_ADDR 0x27
#define LCD_SDA_PIN 26
#define LCD_SCL_PIN 27
#define LCD_REFRESH_MS 500

/* ---- Traffic Timing (mirrors STM32 — keep in sync!) ---- */
#define GREEN_DURATION_MS 10000
#define YELLOW_DURATION_MS 3000
#define ALL_RED_GAP_MS 1000
#define AMBULANCE_OVERRIDE_MS 15000
#define LANE_CYCLE_MS (GREEN_DURATION_MS + YELLOW_DURATION_MS + ALL_RED_GAP_MS)

/* ---- Debug ---- */
#define SERIAL_BAUD_RATE 115200
#define DEBUG_ENABLED 1

#if DEBUG_ENABLED
#define DBG(...) Serial.printf(__VA_ARGS__)
#define DBGLN(...) Serial.println(__VA_ARGS__)
#else
#define DBG(...)
#define DBGLN(...)
#endif

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 2: CAMERA PINS — AI-Thinker ESP32-CAM (fixed, no edit) ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

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

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 3: LCD I2C DRIVER (self-contained, no library needed)  ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

/* PCF8574 backpack bits */
#define LCD_BL_BIT 0x08
#define LCD_EN_BIT 0x04
#define LCD_RW_BIT 0x02
#define LCD_RS_BIT 0x01

/* HD44780 commands */
#define LCD_CMD_CLEAR 0x01
#define LCD_CMD_HOME 0x02
#define LCD_CMD_ENTRY_MODE 0x06
#define LCD_CMD_DISPLAY_ON 0x0C
#define LCD_CMD_FUNC_4BIT 0x28
#define LCD_CMD_SET_DDRAM 0x80
#define LCD_LINE0_ADDR 0x00
#define LCD_LINE1_ADDR 0x40

static TwoWire LCDWire = TwoWire(1);
static bool lcdReady = false;

static void lcd_i2c_write(uint8_t data) {
  LCDWire.beginTransmission(LCD_I2C_ADDR);
  LCDWire.write(data);
  LCDWire.endTransmission();
}

static void lcd_pulse_enable(uint8_t data) {
  lcd_i2c_write(data | LCD_EN_BIT);
  delayMicroseconds(1);
  lcd_i2c_write(data & ~LCD_EN_BIT);
  delayMicroseconds(50);
}

static void lcd_send_nibble(uint8_t nibble, uint8_t mode) {
  lcd_pulse_enable((nibble & 0xF0) | mode | LCD_BL_BIT);
}

static void lcd_send_byte(uint8_t value, uint8_t mode) {
  lcd_send_nibble(value & 0xF0, mode);
  lcd_send_nibble((value << 4) & 0xF0, mode);
}

static void lcd_command(uint8_t cmd) {
  lcd_send_byte(cmd, 0);
  if (cmd <= 0x02)
    delay(2);
}

static void lcd_data(uint8_t ch) { lcd_send_byte(ch, LCD_RS_BIT); }

static bool LCD_Init(void) {
#if LCD_ENABLED == 0
  return false;
#endif
  LCDWire.begin(LCD_SDA_PIN, LCD_SCL_PIN, 100000);
  delay(50);
  lcd_send_nibble(0x30, 0);
  delay(5);
  lcd_send_nibble(0x30, 0);
  delay(5);
  lcd_send_nibble(0x30, 0);
  delay(1);
  lcd_send_nibble(0x20, 0);
  delay(1);
  lcd_command(LCD_CMD_FUNC_4BIT);
  lcd_command(LCD_CMD_DISPLAY_ON);
  lcd_command(LCD_CMD_CLEAR);
  lcd_command(LCD_CMD_ENTRY_MODE);
  lcdReady = true;
  return true;
}

static void LCD_Clear(void) {
  if (lcdReady)
    lcd_command(LCD_CMD_CLEAR);
}

static void LCD_SetCursor(uint8_t row, uint8_t col) {
  if (!lcdReady)
    return;
  lcd_command(LCD_CMD_SET_DDRAM |
              ((row ? LCD_LINE1_ADDR : LCD_LINE0_ADDR) + col));
}

static void LCD_Print(const char *str) {
  if (!lcdReady || !str)
    return;
  for (uint8_t i = 0; *str && i < 16; i++)
    lcd_data((uint8_t)*str++);
}

static void LCD_PrintPadded(const char *str) {
  if (!lcdReady || !str)
    return;
  uint8_t i = 0;
  while (*str && i < 16) {
    lcd_data((uint8_t)*str++);
    i++;
  }
  while (i < 16) {
    lcd_data(' ');
    i++;
  }
}

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 4: GLOBAL STATE                                        ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

static const char *LANE_NAMES[] = {"NORTH", "EAST", "SOUTH", "WEST"};

static bool wifiConnected = false;
static bool espNowInitialized = false;
static bool cameraInitialized = false;
static bool lcdInitialized = false;

/* ESP-NOW send status */
static volatile bool espNowSendSuccess = false;

/* Sound detection */
static int soundHighCount = 0;
static uint32_t lastSoundSample = 0;

/* Cooldown */
static uint32_t lastAlertTime = 0;
static bool inCooldown = false;

/* Stats */
static uint32_t totalTriggers = 0;
static uint32_t mlTriggers = 0;
static uint32_t soundTriggers = 0;
static uint32_t ambulancesFound = 0;
static String lastTriggerSrc = "none";

/* Traffic Simulation */
typedef enum {
  TSIM_GREEN,
  TSIM_YELLOW,
  TSIM_ALL_RED,
  TSIM_AMBULANCE
} TrafficSimPhase_t;
static TrafficSimPhase_t tsimPhase = TSIM_GREEN;
static uint8_t tsimActiveLane = 0;
static uint32_t tsimPhaseStartMs = 0;
static uint8_t tsimAmbulanceLane = 0;
static uint32_t lastLcdUpdate = 0;

/* HTTP trigger server */
#if TRIGGER_MODE == 0 || TRIGGER_MODE == 2
WebServer httpServer(HTTP_TRIGGER_PORT);
static volatile bool httpTriggerPending = false;
#endif

/* UART trigger buffer */
#if TRIGGER_MODE == 1 || TRIGGER_MODE == 2
static String uartBuffer = "";
#endif

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 5: WiFi                                                ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

bool connectWiFi(void) {
  DBG("[WiFi] Connecting to %s", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.setSleep(false);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    DBG(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    DBG("\n[WiFi] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
    return true;
  }
  DBGLN("\n[WiFi] FAILED to connect!");
  return false;
}

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 6: HTTP TRIGGER SERVER                                 ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

#if TRIGGER_MODE == 0 || TRIGGER_MODE == 2

void handleTrigger(void) {
  if (inCooldown) {
    uint32_t rem = (DETECTION_COOLDOWN_MS - (millis() - lastAlertTime)) / 1000;
    httpServer.send(
        200, "application/json",
        "{\"status\":\"cooldown\",\"remaining_seconds\":" + String(rem) + "}");
    return;
  }
  httpTriggerPending = true;
  httpServer.send(
      200, "application/json",
      "{\"status\":\"triggered\",\"message\":\"Pipeline starting...\"}");
  DBGLN("[HTTP] *** TRIGGER from laptop ML! ***");
}

void handleStatus(void) {
  JsonDocument doc;
  doc["uptime_s"] = millis() / 1000;
  doc["wifi"] = (WiFi.status() == WL_CONNECTED);
  doc["camera"] = cameraInitialized;
  doc["esp_now"] = espNowInitialized;
  doc["lcd"] = lcdInitialized;
  doc["in_cooldown"] = inCooldown;
  doc["total_triggers"] = totalTriggers;
  doc["ambulances_found"] = ambulancesFound;
  doc["lane_id"] = MONITORED_LANE_ID;
  doc["ip"] = WiFi.localIP().toString();
  String resp;
  serializeJsonPretty(doc, resp);
  httpServer.send(200, "application/json", resp);
}

void handleHealth(void) { httpServer.send(200, "text/plain", "OK"); }

void handleNotFound(void) {
  httpServer.send(404, "text/plain",
                  "Endpoints: GET /trigger, /status, /health");
}

void setupHTTPServer(void) {
  httpServer.on("/trigger", HTTP_GET, handleTrigger);
  httpServer.on("/status", HTTP_GET, handleStatus);
  httpServer.on("/health", HTTP_GET, handleHealth);
  httpServer.onNotFound(handleNotFound);
  httpServer.begin();
  DBG("[HTTP] Server on port %d\n", HTTP_TRIGGER_PORT);
  DBG("[HTTP] Trigger URL: http://%s/trigger\n",
      WiFi.localIP().toString().c_str());
}

#endif

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 7: UART TRIGGER                                       ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

#if TRIGGER_MODE == 1 || TRIGGER_MODE == 2

bool checkUARTTrigger(void) {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      uartBuffer.trim();
      if (uartBuffer == "SIREN") {
        uartBuffer = "";
        return true;
      }
      uartBuffer = "";
    } else {
      uartBuffer += c;
      if (uartBuffer.length() > 20)
        uartBuffer = "";
    }
  }
  return false;
}

#endif

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 8: CAMERA                                              ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

bool initCamera(void) {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
  }

  if (esp_camera_init(&config) != ESP_OK) {
    DBGLN("[Camera] Init FAILED!");
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_brightness(s, 1);
    s->set_contrast(s, 1);
    s->set_saturation(s, 0);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_wb_mode(s, 0);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);
    s->set_gain_ctrl(s, 1);
    s->set_agc_gain(s, 0);
    s->set_gainceiling(s, (gainceiling_t)6);
    s->set_bpc(s, 1);
    s->set_wpc(s, 1);
  }
  DBGLN("[Camera] Initialized OK.");
  return true;
}

camera_fb_t *captureFrame(void) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb)
    esp_camera_fb_return(fb);
  delay(CAMERA_WARMUP_MS);
  fb = esp_camera_fb_get();
  if (!fb) {
    DBGLN("[Camera] Capture FAILED!");
    return NULL;
  }
  DBG("[Camera] Captured %u bytes (%dx%d)\n", fb->len, fb->width, fb->height);
  return fb;
}

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 9: ESP-NOW SENDER                                     ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

/* ESP-NOW send callback */
void onEspNowSend(const uint8_t *mac_addr, esp_now_send_status_t status) {
  espNowSendSuccess = (status == ESP_NOW_SEND_SUCCESS);
  DBG("[ESP-NOW] Send %s to %02X:%02X:%02X:%02X:%02X:%02X\n",
      espNowSendSuccess ? "OK" : "FAIL", mac_addr[0], mac_addr[1], mac_addr[2],
      mac_addr[3], mac_addr[4], mac_addr[5]);
}

bool initEspNow(void) {
  if (esp_now_init() != ESP_OK) {
    DBGLN("[ESP-NOW] Init FAILED!");
    return false;
  }

  esp_now_register_send_cb(onEspNowSend);

  /* Add receiver as peer */
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, RECEIVER_MAC, 6);
  peerInfo.channel = 0; /* Use current WiFi channel */
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    DBGLN("[ESP-NOW] Failed to add peer!");
    return false;
  }

  DBG("[ESP-NOW] Initialized. Receiver: %02X:%02X:%02X:%02X:%02X:%02X\n",
      RECEIVER_MAC[0], RECEIVER_MAC[1], RECEIVER_MAC[2], RECEIVER_MAC[3],
      RECEIVER_MAC[4], RECEIVER_MAC[5]);
  return true;
}

bool sendAmbulanceAlert(uint8_t laneId) {
  char msg[16];
  snprintf(msg, sizeof(msg), "AMB:%d", laneId);
  DBG("[ESP-NOW] TX: %s\n", msg);

  for (int r = 0; r < ESPNOW_RETRY_COUNT; r++) {
    espNowSendSuccess = false;
    esp_err_t result = esp_now_send(RECEIVER_MAC, (uint8_t *)msg, strlen(msg));

    if (result == ESP_OK) {
      delay(50); /* Brief wait for callback */
      if (espNowSendSuccess) {
        DBG("[ESP-NOW] Alert sent on attempt %d/%d\n", r + 1,
            ESPNOW_RETRY_COUNT);
        return true;
      }
    }

    DBG("[ESP-NOW] Attempt %d/%d failed, retrying...\n", r + 1,
        ESPNOW_RETRY_COUNT);
    if (r < ESPNOW_RETRY_COUNT - 1)
      delay(ESPNOW_RETRY_DELAY_MS);
  }

  DBGLN("[ESP-NOW] All retries exhausted!");
  return false;
}

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 10: ROBOFLOW API                                      ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

bool callRoboflowAPI(camera_fb_t *fb) {
  if (!fb || WiFi.status() != WL_CONNECTED) {
    if (WiFi.status() != WL_CONNECTED) {
      DBGLN("[Roboflow] WiFi disconnected, reconnecting...");
      wifiConnected = connectWiFi();
      if (!wifiConnected)
        return false;
    }
    return false;
  }

  String url = String(ROBOFLOW_API_URL);
  url += "?api_key=" + String(ROBOFLOW_API_KEY);
  url += "&confidence=" + String(ROBOFLOW_CONF_MIN, 2);
  url += "&format=json";

  DBG("[Roboflow] Encoding %u bytes to base64...\n", fb->len);
  String imageBase64 = base64::encode(fb->buf, fb->len);
  DBG("[Roboflow] Base64 size: %u chars\n", imageBase64.length());

  HTTPClient http;
  http.begin(url);
  http.setTimeout(ROBOFLOW_TIMEOUT_MS);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  int httpCode = http.POST(imageBase64);

  if (httpCode != 200) {
    DBG("[Roboflow] API Error! HTTP %d\n", httpCode);
    if (httpCode > 0)
      DBG("[Roboflow] Response: %s\n", http.getString().c_str());
    http.end();
    return false;
  }

  String response = http.getString();
  http.end();
  DBG("[Roboflow] Response: %s\n", response.c_str());

  JsonDocument doc;
  if (deserializeJson(doc, response)) {
    DBGLN("[Roboflow] JSON parse error!");
    return false;
  }

  JsonArray predictions = doc["predictions"].as<JsonArray>();
  if (predictions.isNull() || predictions.size() == 0) {
    DBGLN("[Roboflow] No predictions returned.");
    return false;
  }

  for (JsonObject pred : predictions) {
    const char *cls = pred["class"] | "";
    float conf = pred["confidence"] | 0.0f;
    DBG("[Roboflow] Detection: class=\"%s\" conf=%.2f\n", cls, conf);
    if (strcmp(cls, ROBOFLOW_TARGET_CLASS) == 0 && conf >= ROBOFLOW_CONF_MIN) {
      DBG("[Roboflow] CONFIRMED! (%.1f%%)\n", conf * 100);
      return true;
    }
  }
  DBG("[Roboflow] No %s above %.0f%% threshold.\n", ROBOFLOW_TARGET_CLASS,
      ROBOFLOW_CONF_MIN * 100);
  return false;
}

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 11: SOUND DETECTION (Fallback)                         ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

bool detectSiren(void) {
  uint32_t now = millis();
  if ((now - lastSoundSample) < SOUND_SAMPLE_INTERVAL_MS)
    return false;
  lastSoundSample = now;
  int level = analogRead(SOUND_SENSOR_PIN);
  if (level > SOUND_THRESHOLD)
    soundHighCount++;
  else if (soundHighCount > 0)
    soundHighCount--;
  if (soundHighCount >= SOUND_TRIGGER_COUNT) {
    DBG("[Sound] Siren detected! Level=%d Count=%d\n", level, soundHighCount);
    soundHighCount = 0;
    return true;
  }
  return false;
}

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 12: TRAFFIC CYCLE SIMULATION + LCD DISPLAY             ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

void TrafficSim_Init(void) {
  tsimPhase = TSIM_GREEN;
  tsimActiveLane = 0;
  tsimPhaseStartMs = millis();
}

void TrafficSim_Update(void) {
  uint32_t elapsed = millis() - tsimPhaseStartMs;
  switch (tsimPhase) {
  case TSIM_GREEN:
    if (elapsed >= GREEN_DURATION_MS) {
      tsimPhase = TSIM_YELLOW;
      tsimPhaseStartMs = millis();
    }
    break;
  case TSIM_YELLOW:
    if (elapsed >= YELLOW_DURATION_MS) {
      tsimPhase = TSIM_ALL_RED;
      tsimPhaseStartMs = millis();
    }
    break;
  case TSIM_ALL_RED:
    if (elapsed >= ALL_RED_GAP_MS) {
      tsimActiveLane = (tsimActiveLane + 1) % 4;
      tsimPhase = TSIM_GREEN;
      tsimPhaseStartMs = millis();
    }
    break;
  case TSIM_AMBULANCE:
    if (elapsed >= AMBULANCE_OVERRIDE_MS) {
      tsimActiveLane = (tsimAmbulanceLane + 1) % 4;
      tsimPhase = TSIM_GREEN;
      tsimPhaseStartMs = millis();
      DBG("[TrafficSim] Override ended. Resuming lane %d (%s)\n",
          tsimActiveLane, LANE_NAMES[tsimActiveLane]);
    }
    break;
  }
}

void TrafficSim_AmbulanceOverride(uint8_t laneId) {
  tsimPhase = TSIM_AMBULANCE;
  tsimAmbulanceLane = laneId % 4;
  tsimPhaseStartMs = millis();
  DBG("[TrafficSim] AMBULANCE OVERRIDE -> lane %d (%s) for %ds\n", laneId,
      LANE_NAMES[laneId % 4], AMBULANCE_OVERRIDE_MS / 1000);
}

/*  LCD Display Modes:
 *    Normal:    N:10 E:-- S:-- W:--  |  Active: NORTH
 *    Ambulance: !! AMBULANCE !!      |  DIR:NORTH  14s
 *    Yellow:    N:03 E:-- S:-- W:--  |  YELLOW > NORTH    */

void LCD_DisplayUpdate(void) {
#if LCD_ENABLED == 0
  return;
#endif
  if (!lcdInitialized)
    return;
  uint32_t now = millis();
  if ((now - lastLcdUpdate) < LCD_REFRESH_MS)
    return;
  lastLcdUpdate = now;

  uint32_t elapsed = now - tsimPhaseStartMs;
  char row0[17], row1[17];

  if (tsimPhase == TSIM_AMBULANCE) {
    /* ---- AMBULANCE MODE ---- */
    uint32_t remMs =
        (elapsed < AMBULANCE_OVERRIDE_MS) ? AMBULANCE_OVERRIDE_MS - elapsed : 0;
    uint32_t remSec = (remMs + 999) / 1000;
    snprintf(row0, sizeof(row0), "!! AMBULANCE !!");
    snprintf(row1, sizeof(row1), "DIR:%-5s  %2lus",
             LANE_NAMES[tsimAmbulanceLane], (unsigned long)remSec);
  } else {
    /* ---- NORMAL / YELLOW / ALL_RED ---- */
    uint32_t dur = 0;
    switch (tsimPhase) {
    case TSIM_GREEN:
      dur = GREEN_DURATION_MS;
      break;
    case TSIM_YELLOW:
      dur = YELLOW_DURATION_MS;
      break;
    case TSIM_ALL_RED:
      dur = ALL_RED_GAP_MS;
      break;
    default:
      break;
    }
    uint32_t remMs = (elapsed < dur) ? dur - elapsed : 0;
    uint32_t remSec = (remMs + 999) / 1000;
    if (remSec > 99)
      remSec = 99;

    char ls[4][4];
    for (int i = 0; i < 4; i++) {
      if (i == tsimActiveLane && tsimPhase != TSIM_ALL_RED)
        snprintf(ls[i], sizeof(ls[i]), "%2lu", (unsigned long)remSec);
      else
        strcpy(ls[i], "--");
    }
    snprintf(row0, sizeof(row0), "N:%sE:%sS:%sW:%s", ls[0], ls[1], ls[2],
             ls[3]);

    if (tsimPhase == TSIM_GREEN)
      snprintf(row1, sizeof(row1), "Active: %-7s", LANE_NAMES[tsimActiveLane]);
    else if (tsimPhase == TSIM_YELLOW)
      snprintf(row1, sizeof(row1), "YELLOW > %-6s", LANE_NAMES[tsimActiveLane]);
    else
      snprintf(row1, sizeof(row1), "ALL RED  WAIT  ");
  }

  LCD_SetCursor(0, 0);
  LCD_PrintPadded(row0);
  LCD_SetCursor(1, 0);
  LCD_PrintPadded(row1);
}

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 13: DETECTION PIPELINE                                 ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

void runDetectionPipeline(const char *source) {
  DBG("\n=== AMBULANCE DETECTION PIPELINE | src: %s ===\n", source);
  totalTriggers++;
  lastTriggerSrc = String(source);
  if (strcmp(source, "sound") == 0)
    soundTriggers++;
  else
    mlTriggers++;

  /* Show on LCD */
  if (lcdInitialized) {
    LCD_SetCursor(0, 0);
    LCD_PrintPadded("DETECTING...");
    LCD_SetCursor(1, 0);
    LCD_PrintPadded("Camera+AI check");
  }

  /* Step 1: Capture */
  DBGLN("[1/3] Capturing camera frame...");
  camera_fb_t *fb = captureFrame();
  if (!fb) {
    DBGLN("[Pipeline] ABORT: Camera failed.");
    return;
  }

  /* Step 2: Roboflow */
  DBGLN("[2/3] Sending to Roboflow API...");
  bool confirmed = callRoboflowAPI(fb);
  esp_camera_fb_return(fb);

  if (!confirmed) {
    DBGLN("[Pipeline] No ambulance confirmed. False alarm.\n");
    return;
  }

  /* Step 3: ESP-NOW alert to receiver */
  DBGLN("[3/3] AMBULANCE CONFIRMED! Sending ESP-NOW alert...");
  ambulancesFound++;
  if (espNowInitialized)
    sendAmbulanceAlert(MONITORED_LANE_ID);
  else
    DBGLN("[Pipeline] WARNING: ESP-NOW not available!");

  TrafficSim_AmbulanceOverride(MONITORED_LANE_ID);
  lastAlertTime = millis();
  inCooldown = true;
  DBG("[Pipeline] Cooldown started (%ds)\n\n", DETECTION_COOLDOWN_MS / 1000);
}

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 14: SETUP                                              ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);

  DBGLN("\n================================================");
  DBGLN("  ESP32-CAM Sender v4.0 — ESP-NOW Architecture");
  DBGLN("  Camera + Roboflow + Sound + ESP-NOW TX");
  DBGLN("================================================");
  DBG("  Lane: %d | Trigger: %d", MONITORED_LANE_ID, TRIGGER_MODE);
#if TRIGGER_MODE == 0
  DBGLN(" (WiFi HTTP)");
#elif TRIGGER_MODE == 1
  DBGLN(" (UART serial)");
#else
  DBGLN(" (WiFi + UART)");
#endif

  /* Sound sensor */
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(SOUND_SENSOR_PIN, INPUT);

  /* WiFi (STA mode for both HTTP server and ESP-NOW) */
  wifiConnected = connectWiFi();

  /* Print own MAC address for reference */
  DBG("[Info] ESP32-CAM MAC: %s\n", WiFi.macAddress().c_str());

  /* mDNS */
  if (wifiConnected) {
    if (MDNS.begin(MDNS_HOSTNAME)) {
      MDNS.addService("http", "tcp", HTTP_TRIGGER_PORT);
      DBG("[mDNS] http://%s.local\n", MDNS_HOSTNAME);
    }
  }

  /* HTTP server */
#if TRIGGER_MODE == 0 || TRIGGER_MODE == 2
  if (wifiConnected)
    setupHTTPServer();
#endif

  /* Camera */
  cameraInitialized = initCamera();

  /* ESP-NOW (replaces LoRa) */
  espNowInitialized = initEspNow();

  /* LCD (AFTER camera — shares I2C pins) */
#if LCD_ENABLED
  lcdInitialized = LCD_Init();
  if (lcdInitialized) {
    LCD_SetCursor(0, 0);
    LCD_PrintPadded("Traffic Monitor");
    LCD_SetCursor(1, 0);
    LCD_PrintPadded("Booting...");
  }
#endif

  /* Traffic simulation */
  TrafficSim_Init();

  /* Status report */
  DBGLN("\n-------- SYSTEM STATUS --------");
  DBG("  WiFi:     %s\n", wifiConnected ? "OK" : "FAIL");
  DBG("  Camera:   %s\n", cameraInitialized ? "OK" : "FAIL");
  DBG("  ESP-NOW:  %s\n", espNowInitialized ? "OK" : "FAIL");
  DBG("  LCD:      %s\n", lcdInitialized ? "OK" : "OFF/FAIL");
#if TRIGGER_MODE == 0 || TRIGGER_MODE == 2
  if (wifiConnected)
    DBG("  Trigger: http://%s/trigger\n", WiFi.localIP().toString().c_str());
#endif
  DBGLN("-------------------------------\n");
  DBGLN("[Ready] Monitoring...");

  /* LCD ready screen */
  if (lcdInitialized) {
    LCD_SetCursor(0, 0);
    LCD_PrintPadded("System Ready!");
    char line[17];
    snprintf(line, sizeof(line), "Lane:%s WiFi:%s",
             LANE_NAMES[MONITORED_LANE_ID], wifiConnected ? "OK" : "NO");
    LCD_SetCursor(1, 0);
    LCD_PrintPadded(line);
    delay(2000);
  }
}

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 15: MAIN LOOP                                         ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

void loop() {
  uint32_t now = millis();

  /* Handle HTTP requests */
#if TRIGGER_MODE == 0 || TRIGGER_MODE == 2
  if (wifiConnected)
    httpServer.handleClient();
#endif

  /* Always update traffic simulation & LCD */
  TrafficSim_Update();
  LCD_DisplayUpdate();

  /* Cooldown check */
  if (inCooldown) {
    if ((now - lastAlertTime) >= DETECTION_COOLDOWN_MS) {
      inCooldown = false;
      DBGLN("[Cooldown] Ended. Resuming monitoring.");
    } else {
      delay(10);
      return;
    }
  }

  /* Priority 1: WiFi HTTP trigger */
#if TRIGGER_MODE == 0 || TRIGGER_MODE == 2
  if (httpTriggerPending) {
    httpTriggerPending = false;
    if (cameraInitialized && wifiConnected) {
      runDetectionPipeline("ml_wifi");
    } else if (espNowInitialized) {
      DBGLN("[Fallback] Camera/WiFi down -> direct ESP-NOW alert.");
      sendAmbulanceAlert(MONITORED_LANE_ID);
      TrafficSim_AmbulanceOverride(MONITORED_LANE_ID);
      lastAlertTime = millis();
      inCooldown = true;
    }
    return;
  }
#endif

  /* Priority 2: UART trigger */
#if TRIGGER_MODE == 1 || TRIGGER_MODE == 2
  if (checkUARTTrigger()) {
    if (cameraInitialized && wifiConnected) {
      runDetectionPipeline("ml_uart");
    } else if (espNowInitialized) {
      DBGLN("[Fallback] Camera/WiFi down -> direct ESP-NOW alert.");
      sendAmbulanceAlert(MONITORED_LANE_ID);
      TrafficSim_AmbulanceOverride(MONITORED_LANE_ID);
      lastAlertTime = millis();
      inCooldown = true;
    }
    return;
  }
#endif

  /* Priority 3: Local sound sensor */
  if (detectSiren()) {
    DBGLN("[Main] Sound sensor triggered (fallback).");
    if (cameraInitialized && wifiConnected) {
      runDetectionPipeline("sound");
    } else if (espNowInitialized) {
      DBGLN("[Fallback] Camera/WiFi down -> direct ESP-NOW alert.");
      sendAmbulanceAlert(MONITORED_LANE_ID);
      TrafficSim_AmbulanceOverride(MONITORED_LANE_ID);
      lastAlertTime = millis();
      inCooldown = true;
    }
  }

  delay(10);
}
