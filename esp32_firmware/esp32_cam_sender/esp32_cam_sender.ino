/**
 * ============================================================================
 * ESP32-CAM Sender — Ambulance Detection + ESP-NOW WiFi Transmitter
 * Board: ESP32-CAM (AI-Thinker) + Sound Sensor
 * ============================================================================
 *
 * SYSTEM ARCHITECTURE (Two-ESP32 WiFi):
 * ┌─────────────────────────────────────────────────────────────┐
 * │  ESP32-CAM (THIS BOARD) — MAC: A8:42:E3:56:83:2C        │
 * │  Laptop ML / Sound Sensor → Camera → Roboflow API          │
 * │  → ESP-NOW TX → Receiver ESP32 (FC:E8:C0:7A:B7:A0)       │
 * │  → UART → STM32                                           │
 * └─────────────────────────────────────────────────────────────┘
 *
 * MAC ADDRESSES:
 * This board (Sender):    A8:42:E3:56:83:2C
 * Receiver (ESP32 DevKit): FC:E8:C0:7A:B7:A0
 * Use esp32_firmware/find_mac_address/ sketch to find your board's MAC.
 *
 * DETECTION PIPELINE:
 * Laptop ML Audio / Sound Sensor → Camera → Roboflow API → ESP-NOW →
 * Receiver
 *
 * ENDPOINTS (WiFi):
 * GET /trigger  → start pipeline    GET /status → JSON status
 * GET /health   → "OK"
 *
 * LIBRARIES (Arduino Library Manager):
 * - ArduinoJson by Benoit Blanchon (v6.x or v7.x)
 * - ESP32 Board Package (esp32 by Espressif, v2.x+)
 *
 * WiFi: Vivo_V29_5G (2.4 GHz) — both boards must be on the SAME network.
 * LAPTOP: python3 src/siren_to_esp32.py --mode wifi --esp32-ip <THIS_IP>
 * ============================================================================
 */

/* NOTE: base64.h removed — Roboflow accepts raw binary JPEG via POST.
   Base64 encoding allocated 40-60KB in internal RAM, causing heap
   fragmentation crashes on the ESP32-CAM's limited internal SRAM. */
#include "esp_camera.h"
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 1: USER CONFIGURATION — Edit these before flashing!    ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

/* ---- WiFi (2.4 GHz only) ---- */
#define WIFI_SSID "Vivo_V29_5G"
#define WIFI_PASSWORD "123456789"

/* ---- Roboflow API ----
 * URL: https://detect.roboflow.com/<PROJECT>/<VERSION>
 * Get API key from: https://app.roboflow.com → Deploy → Hosted API */
#define ROBOFLOW_API_URL                                                       \
  "https://detect.roboflow.com/ambulance-detection-md0u4/3"
#define ROBOFLOW_API_KEY "YOUR_ROBOFLOW_API_KEY"
#define ROBOFLOW_TIMEOUT_MS 15000
#define ROBOFLOW_CONF_MIN 0.60
#define ROBOFLOW_TARGET_CLASS "Ambulance"

/* ---- Lane ID: 0=North, 1=East, 2=South, 3=West ---- */
#define MONITORED_LANE_ID 0

/* ---- ESP-NOW Auto-Pairing ----
 * The sender automatically discovers the receiver by listening for its
 * "PAIR:RCV" beacon broadcasts. Hardcoded MAC is used as a fallback.
 * Receiver MAC: FC:E8:C0:7A:B7:A0 */
#define PAIR_BEACON_MSG "PAIR:RCV" /* Beacon from receiver */
#define PAIR_ACK_MSG "PAIR:ACK"    /* ACK sent back to receiver */
#define PAIR_TIMEOUT_MS 30000      /* Max wait time for pairing (30s) */

/* ---- Hardcoded Receiver MAC (fallback if auto-pairing fails) ---- */
static const uint8_t HARDCODED_RECEIVER_MAC[] = {0xFC, 0xE8, 0xC0,
                                                 0x7A, 0xB7, 0xA0};

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

/* NOTE: LCD display is on the STM32F103RB (PC6/PC7), NOT this ESP32-CAM. */

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
 * ║  SECTION 4: GLOBAL STATE & FREERTOS                             ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

static const char *LANE_NAMES[] = {"NORTH", "EAST", "SOUTH", "WEST"};

static bool wifiConnected = false;
static bool espNowInitialized = false;
static bool cameraInitialized = false;

/* ESP-NOW auto-pairing state */
static bool receiverPaired = false;
static uint8_t receiverMAC[6] = {0};
static volatile bool pairBeaconReceived = false;
static uint8_t pendingPairMAC[6] = {0};

/* ESP-NOW send status */
static volatile bool espNowSendSuccess = false;

/* Application-layer ACK from receiver */
static volatile bool receiverAcked = false;

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

/* FreeRTOS: async detection pipeline */
typedef struct {
  char source[16];
} TriggerMsg_t;

QueueHandle_t pipelineQueue = NULL;
TaskHandle_t pipelineTaskHandle = NULL;

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
  doc["lcd"] = "on_stm32";
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

/* ESP-NOW send callback (Updated for ESP32 Core v3.x API) */
void onEspNowSend(const esp_now_send_info_t *info,
                  esp_now_send_status_t status) {
  espNowSendSuccess = (status == ESP_NOW_SEND_SUCCESS);
  DBG("[ESP-NOW] Send %s\n", espNowSendSuccess ? "OK" : "FAIL");
}

/* ESP-NOW receive callback — listens for pairing beacons from receiver (Updated
 * for v3.x API) */
void onEspNowReceive(const esp_now_recv_info_t *info, const uint8_t *data,
                     int len) {
  const uint8_t *mac_addr = info->src_addr;
  if (len <= 0 || len >= 32)
    return;

  char msg[32];
  memcpy(msg, data, len);
  msg[len] = '\0';

  /* Check for PAIR:RCV beacon from receiver */
  if (!receiverPaired && strcmp(msg, PAIR_BEACON_MSG) == 0) {
    memcpy(pendingPairMAC, mac_addr, 6);
    pairBeaconReceived = true;
    DBG("[PAIR] Beacon from %02X:%02X:%02X:%02X:%02X:%02X\n", mac_addr[0],
        mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  }

  /* Listen for application-layer ACK from receiver */
  if (receiverPaired && strcmp(msg, "ACK:AMB") == 0) {
    receiverAcked = true;
  }
}

bool initEspNow(void) {
  if (esp_now_init() != ESP_OK) {
    DBGLN("[ESP-NOW] Init FAILED!");
    return false;
  }

  esp_now_register_send_cb(onEspNowSend);
  esp_now_register_recv_cb(onEspNowReceive);

  /* Add broadcast peer (needed to send PAIR:ACK back) */
  esp_now_peer_info_t bcastPeer = {};
  memset(bcastPeer.peer_addr, 0xFF, 6);
  bcastPeer.channel = 0;
  bcastPeer.encrypt = false;
  bcastPeer.ifidx = WIFI_IF_STA;
  esp_now_add_peer(&bcastPeer);

  DBGLN("[ESP-NOW] Initialized. Listening for receiver beacon...");
  return true;
}

/* Auto-pairing: wait for receiver's PAIR:RCV beacon, then add it as peer */
bool autoDiscoverReceiver(void) {
  DBGLN("[PAIR] Searching for receiver...");
  uint32_t startMs = millis();

  while (!receiverPaired && (millis() - startMs) < PAIR_TIMEOUT_MS) {
    /* Fast-blink built-in LED (GPIO 33 on ESP32-CAM has no LED, use serial) */
    if ((millis() / 500) % 2)
      DBG(".");

    if (pairBeaconReceived) {
      pairBeaconReceived = false;

      /* Save receiver MAC */
      memcpy(receiverMAC, pendingPairMAC, 6);

      /* Add receiver as unicast peer */
      esp_now_peer_info_t peerInfo = {};
      memcpy(peerInfo.peer_addr, receiverMAC, 6);
      peerInfo.channel = 0;
      peerInfo.encrypt = false;
      peerInfo.ifidx = WIFI_IF_STA;

      if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        DBGLN("[PAIR] Failed to add receiver peer!");
        continue;
      }

      /* Send PAIR:ACK to confirm pairing */
      esp_now_send(receiverMAC, (uint8_t *)PAIR_ACK_MSG, strlen(PAIR_ACK_MSG));
      delay(100);

      receiverPaired = true;
      DBG("\n[PAIR] PAIRED with receiver: %02X:%02X:%02X:%02X:%02X:%02X\n",
          receiverMAC[0], receiverMAC[1], receiverMAC[2], receiverMAC[3],
          receiverMAC[4], receiverMAC[5]);
      return true;
    }

    delay(100);
  }

  DBGLN("\n[PAIR] Timeout! No receiver found within 30 seconds.");
  DBGLN("[PAIR] Falling back to hardcoded MAC: FC:E8:C0:7A:B7:A0");

  /* Use hardcoded receiver MAC as fallback */
  memcpy(receiverMAC, HARDCODED_RECEIVER_MAC, 6);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;

  if (esp_now_add_peer(&peerInfo) == ESP_OK) {
    receiverPaired = true;
    DBG("[PAIR] Fallback paired with: %02X:%02X:%02X:%02X:%02X:%02X\n",
        receiverMAC[0], receiverMAC[1], receiverMAC[2], receiverMAC[3],
        receiverMAC[4], receiverMAC[5]);
    return true;
  }

  DBGLN("[PAIR] Fallback pairing also failed!");
  return false;
}

bool sendAmbulanceAlert(uint8_t laneId) {
  if (!receiverPaired) {
    DBGLN("[ESP-NOW] Cannot send - not paired with receiver!");
    return false;
  }

  char msg[16];
  snprintf(msg, sizeof(msg), "AMB:%d", laneId);
  DBG("[ESP-NOW] TX: %s\n", msg);

  for (int r = 0; r < ESPNOW_RETRY_COUNT; r++) {
    receiverAcked = false;
    esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)msg, strlen(msg));

    if (result == ESP_OK) {
      /* Wait up to 500ms for application-layer ACK:AMB from receiver */
      uint32_t waitStart = millis();
      while ((millis() - waitStart) < 500) {
        if (receiverAcked) {
          DBG("[ESP-NOW] Alert confirmed by Receiver on attempt %d/%d\n", r + 1,
              ESPNOW_RETRY_COUNT);
          return true;
        }
        delay(10);
      }
    }

    DBG("[ESP-NOW] Attempt %d/%d failed (No ACK), retrying...\n", r + 1,
        ESPNOW_RETRY_COUNT);
    if (r < ESPNOW_RETRY_COUNT - 1)
      delay(ESPNOW_RETRY_DELAY_MS);
  }

  DBGLN("[ESP-NOW] All retries exhausted! Receiver did not ACK.");
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

  /* Send raw binary JPEG directly from PSRAM — no Base64 allocation needed.
     Roboflow accepts binary payloads via application/x-www-form-urlencoded. */
  DBG("[Roboflow] Uploading %u bytes directly to API...\n", fb->len);

  HTTPClient http;
  http.begin(url);
  http.setTimeout(ROBOFLOW_TIMEOUT_MS);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  int httpCode = http.POST(fb->buf, fb->len);

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
 * ║  SECTION 12: TRAFFIC CYCLE SIMULATION                           ║
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
  DBGLN("  ESP32-CAM Sender v5.0 — ESP-NOW Architecture");
  DBGLN("  Camera + Roboflow + Sound + ESP-NOW TX");
  DBGLN("  Sender MAC:   A8:42:E3:56:83:2C");
  DBGLN("  Receiver MAC: FC:E8:C0:7A:B7:A0");
  DBGLN("  WiFi: Vivo_V29_5G");
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

  /* ESP-NOW sender (auto-pairing — replaces manual MAC) */
  espNowInitialized = initEspNow();
  if (espNowInitialized) {
    autoDiscoverReceiver();
  }

  /* Traffic simulation */
  TrafficSim_Init();

  /* FreeRTOS: async detection pipeline (runs on Core 1) */
  pipelineQueue = xQueueCreate(5, sizeof(TriggerMsg_t));
  xTaskCreatePinnedToCore(
      [](void *) {
        TriggerMsg_t msg;
        while (1) {
          if (xQueueReceive(pipelineQueue, &msg, portMAX_DELAY) == pdTRUE) {
            runDetectionPipeline(msg.source);
          }
        }
      },
      "DetectTask", 8192, NULL, 1, &pipelineTaskHandle, 1);

  /* Status report */
  DBGLN("\n-------- SYSTEM STATUS --------");
  DBG("  WiFi:     %s\n", wifiConnected ? "OK" : "FAIL");
  DBG("  Camera:   %s\n", cameraInitialized ? "OK" : "FAIL");
  DBG("  ESP-NOW:  %s\n", espNowInitialized ? "OK" : "FAIL");
  DBG("  Paired:   %s\n", receiverPaired ? "YES" : "NO");
  DBGLN("  LCD:      On STM32 (not this board)");
#if TRIGGER_MODE == 0 || TRIGGER_MODE == 2
  if (wifiConnected)
    DBG("  Trigger: http://%s/trigger\n", WiFi.localIP().toString().c_str());
#endif
  DBGLN("-------------------------------\n");
  DBGLN("[Ready] Monitoring...");
}

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 15: MAIN LOOP                                         ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

void loop() {
  uint32_t now = millis();

  /* Handle HTTP requests (always responsive — pipeline runs in background) */
#if TRIGGER_MODE == 0 || TRIGGER_MODE == 2
  if (wifiConnected)
    httpServer.handleClient();
#endif

  /* Update traffic simulation */
  TrafficSim_Update();

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

  /* Helper: queue a trigger asynchronously + assert cooldown immediately */
  auto triggerAsync = [&](const char *source) {
    TriggerMsg_t msg;
    strncpy(msg.source, source, sizeof(msg.source) - 1);
    msg.source[sizeof(msg.source) - 1] = '\0';
    xQueueSend(pipelineQueue, &msg, 0);
    inCooldown = true;
    lastAlertTime = millis();
  };

  /* Priority 1: WiFi HTTP trigger */
#if TRIGGER_MODE == 0 || TRIGGER_MODE == 2
  if (httpTriggerPending) {
    httpTriggerPending = false;
    if (cameraInitialized && wifiConnected) {
      triggerAsync("ml_wifi");
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
      triggerAsync("ml_uart");
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
      triggerAsync("sound");
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