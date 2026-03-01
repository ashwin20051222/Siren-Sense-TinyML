/**
 * ============================================================================
 *  Ambulance Detector — Single-File Firmware (All-in-One)
 *  Board: ESP32-CAM (AI-Thinker) + SX1278 LoRa + Sound Sensor
 * ============================================================================
 *
 *  TWO DETECTION SYSTEMS IN ONE:
 *
 *    A) LAPTOP ML AUDIO DETECTION (Primary, via WiFi/UART):
 *       - Laptop runs Siren-Sense TFLite model on microphone
 *       - When siren detected, laptop sends HTTP GET /trigger to this ESP32
 *       - ESP32 captures image → Roboflow API → ambulance? → LoRa alert
 *
 *    B) LOCAL SOUND SENSOR (Fallback, on-board):
 *       - Sound sensor (GPIO 33) monitors ambient sound
 *       - If sustained loud sound detected → same image pipeline
 *
 *    ROBOFLOW OBJECT DETECTION:
 *       - Camera captures JPEG
 *       - Base64-encoded image POSTed to Roboflow Inference API
 *       - Roboflow returns predictions with class + confidence
 *       - If "ambulance" detected above threshold → LoRa alert to STM32
 *
 *  ENDPOINTS (WiFi HTTP Server):
 *    GET /trigger  → start camera → Roboflow → LoRa pipeline
 *    GET /status   → JSON system status
 *    GET /health   → "OK" health check
 *
 *  LIBRARIES REQUIRED (Arduino Library Manager):
 *    - LoRa by Sandeep Mistry (v0.8.0+)
 *    - ArduinoJson by Benoit Blanchon (v6.x or v7.x)
 *    - ESP32 Board Package (esp32 by Espressif, v2.x+)
 *    - WebServer (included with ESP32 board package)
 *
 *  LAPTOP SIDE:
 *    python3 src/siren_to_esp32.py --mode wifi --esp32-ip <THIS_ESP32_IP>
 *
 * ============================================================================
 **/

/* ---- Configuration (edit config.h, not this file) ---- */
#include "config.h"

/* ---- Libraries ---- */
#include "base64.h"
#include "esp_camera.h"
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <LoRa.h>
#include <WebServer.h>
#include <WiFi.h>

/* =====================================================================
 *  GLOBAL STATE
 * ===================================================================== */

static bool wifiConnected = false;
static bool loraInitialized = false;
static bool cameraInitialized = false;

/* Sound detection state (fallback) */
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

/* HTTP trigger server */
#if TRIGGER_MODE == 0 || TRIGGER_MODE == 2
WebServer httpServer(HTTP_TRIGGER_PORT);
static volatile bool httpTriggerPending = false;
#endif

/* UART trigger buffer */
#if TRIGGER_MODE == 1 || TRIGGER_MODE == 2
static String uartBuffer = "";
#endif

/* =====================================================================
 *  WiFi
 * ===================================================================== */

bool connectWiFi(void) {
  DBG("[WiFi] Connecting to %s", WIFI_SSID);
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

/* =====================================================================
 *  HTTP TRIGGER SERVER — Laptop ML sends GET /trigger here
 * ===================================================================== */

#if TRIGGER_MODE == 0 || TRIGGER_MODE == 2

void handleTrigger(void) {
  if (inCooldown) {
    uint32_t rem = (DETECTION_COOLDOWN_MS - (millis() - lastAlertTime)) / 1000;
    String msg =
        "{\"status\":\"cooldown\",\"remaining_seconds\":" + String(rem) + "}";
    httpServer.send(200, "application/json", msg);
    DBGLN("[HTTP] Trigger received — in cooldown.");
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
  doc["lora"] = loraInitialized;
  doc["in_cooldown"] = inCooldown;
  doc["total_triggers"] = totalTriggers;
  doc["ml_triggers"] = mlTriggers;
  doc["sound_triggers"] = soundTriggers;
  doc["ambulances_found"] = ambulancesFound;
  doc["last_trigger"] = lastTriggerSrc;
  doc["lane_id"] = MONITORED_LANE_ID;
  doc["ip"] = WiFi.localIP().toString();

  String resp;
  serializeJsonPretty(doc, resp);
  httpServer.send(200, "application/json", resp);
}

void handleHealth(void) { httpServer.send(200, "text/plain", "OK"); }

void handleNotFound(void) {
  httpServer.send(404, "text/plain",
                  "Endpoints: GET /trigger, GET /status, GET /health");
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

/* =====================================================================
 *  UART TRIGGER — Laptop sends "SIREN\n" over serial
 * ===================================================================== */

#if TRIGGER_MODE == 1 || TRIGGER_MODE == 2

bool checkUARTTrigger(void) {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      uartBuffer.trim();
      if (uartBuffer == "SIREN") {
        DBGLN("[UART] *** TRIGGER from laptop ML! ***");
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

/* =====================================================================
 *  CAMERA
 * ===================================================================== */

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
    config.frame_size = FRAMESIZE_VGA; /* 640×480 */
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA; /* 320×240 */
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
  /* Discard stale frame */
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

/* =====================================================================
 *  LoRa SX1278
 * ===================================================================== */

bool initLoRa(void) {
  LoRa.setPins(LORA_NSS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
  SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_NSS_PIN);

  if (!LoRa.begin((long)LORA_FREQUENCY)) {
    DBGLN("[LoRa] Init FAILED!");
    return false;
  }

  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth((long)LORA_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.setTxPower(LORA_TX_POWER);
  LoRa.enableCrc();

  DBGLN("[LoRa] Initialized OK @ 433 MHz.");
  return true;
}

bool sendAmbulanceAlert(uint8_t laneId) {
  char msg[8];
  snprintf(msg, sizeof(msg), "AMB:%d", laneId);
  DBG("[LoRa] TX: %s\n", msg);

  for (int r = 0; r < LORA_RETRY_COUNT; r++) {
    LoRa.beginPacket();
    LoRa.print(msg);
    LoRa.endPacket(true);
    DBG("[LoRa] Attempt %d/%d\n", r + 1, LORA_RETRY_COUNT);
    if (r < LORA_RETRY_COUNT - 1)
      delay(LORA_RETRY_DELAY_MS);
  }

  DBGLN("[LoRa] Alert sent!");
  return true;
}

/* =====================================================================
 *  ROBOFLOW API — Image Detection
 *
 *  Sends base64-encoded JPEG to Roboflow Hosted Inference API.
 *
 *  Request:
 *    POST https://detect.roboflow.com/<project>/<version>
 *         ?api_key=<key>&confidence=<min>
 *    Content-Type: application/x-www-form-urlencoded
 *    Body: <base64 image string>
 *
 *  Response JSON:
 *  {
 *    "predictions": [
 *      { "class":"ambulance", "confidence":0.92,
 *        "x":320, "y":240, "width":200, "height":150 }
 *    ],
 *    "image": { "width":640, "height":480 }
 *  }
 * ===================================================================== */

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

  /* Build URL with query parameters */
  String url = String(ROBOFLOW_API_URL);
  url += "?api_key=" + String(ROBOFLOW_API_KEY);
  url += "&confidence=" + String(ROBOFLOW_CONF_MIN, 2);
  url += "&format=json";

  DBG("[Roboflow] Encoding %u bytes to base64...\n", fb->len);

  /* Base64 encode the JPEG image */
  String imageBase64 = base64::encode(fb->buf, fb->len);

  DBG("[Roboflow] Base64 size: %u chars\n", imageBase64.length());
  DBG("[Roboflow] POST to: %s\n", ROBOFLOW_API_URL);

  HTTPClient http;
  http.begin(url);
  http.setTimeout(ROBOFLOW_TIMEOUT_MS);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  int httpCode = http.POST(imageBase64);

  if (httpCode != 200) {
    DBG("[Roboflow] API Error! HTTP %d\n", httpCode);
    if (httpCode > 0) {
      String errBody = http.getString();
      DBG("[Roboflow] Response: %s\n", errBody.c_str());
    }
    http.end();
    return false;
  }

  String response = http.getString();
  http.end();

  DBG("[Roboflow] Response: %s\n", response.c_str());

  /* Parse JSON */
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, response);

  if (error) {
    DBG("[Roboflow] JSON parse error: %s\n", error.c_str());
    return false;
  }

  /* Check predictions array */
  JsonArray predictions = doc["predictions"].as<JsonArray>();
  if (predictions.isNull() || predictions.size() == 0) {
    DBGLN("[Roboflow] No predictions returned.");
    return false;
  }

  /* Look for target class (ambulance) */
  for (JsonObject pred : predictions) {
    const char *cls = pred["class"] | "";
    float confidence = pred["confidence"] | 0.0f;
    float x = pred["x"] | 0.0f;
    float y = pred["y"] | 0.0f;
    float w = pred["width"] | 0.0f;
    float h = pred["height"] | 0.0f;

    DBG("[Roboflow] Detection: class=\"%s\"  conf=%.2f  "
        "bbox=(%.0f,%.0f,%.0f,%.0f)\n",
        cls, confidence, x - w / 2, y - h / 2, x + w / 2, y + h / 2);

    if (strcmp(cls, ROBOFLOW_TARGET_CLASS) == 0 &&
        confidence >= ROBOFLOW_CONF_MIN) {
      DBG("[Roboflow] ✅ %s CONFIRMED! (%.1f%% confidence)\n",
          ROBOFLOW_TARGET_CLASS, confidence * 100);
      return true;
    }
  }

  DBG("[Roboflow] No %s detected above %.0f%% threshold.\n",
      ROBOFLOW_TARGET_CLASS, ROBOFLOW_CONF_MIN * 100);
  return false;
}

/* =====================================================================
 *  SOUND DETECTION — Local Fallback
 * ===================================================================== */

bool detectSiren(void) {
  uint32_t now = millis();
  if ((now - lastSoundSample) < SOUND_SAMPLE_INTERVAL_MS)
    return false;
  lastSoundSample = now;

  int level = analogRead(SOUND_SENSOR_PIN);

  if (level > SOUND_THRESHOLD) {
    soundHighCount++;
  } else if (soundHighCount > 0) {
    soundHighCount--;
  }

  if (soundHighCount >= SOUND_TRIGGER_COUNT) {
    DBG("[Sound] Siren detected! Level=%d Count=%d\n", level, soundHighCount);
    soundHighCount = 0;
    return true;
  }
  return false;
}

/* =====================================================================
 *  DETECTION PIPELINE
 *  Trigger → Camera → Roboflow → LoRa
 * ===================================================================== */

void runDetectionPipeline(const char *source) {
  DBGLN("╔═══════════════════════════════════════════════╗");
  DBG("║  AMBULANCE DETECTION PIPELINE  |  src: %-8s║\n", source);
  DBGLN("╚═══════════════════════════════════════════════╝");

  totalTriggers++;
  lastTriggerSrc = String(source);
  if (strcmp(source, "sound") == 0)
    soundTriggers++;
  else
    mlTriggers++;

  /* Step 1: Capture image */
  DBGLN("[1/3] Capturing camera frame...");
  camera_fb_t *fb = captureFrame();
  if (!fb) {
    DBGLN("[Pipeline] ABORT: Camera failed.");
    return;
  }

  /* Step 2: Roboflow detection */
  DBGLN("[2/3] Sending to Roboflow API...");
  bool confirmed = callRoboflowAPI(fb);
  esp_camera_fb_return(fb);

  if (!confirmed) {
    DBGLN("[Pipeline] No ambulance confirmed. False alarm.");
    DBGLN("=== PIPELINE ENDED (no alert) ===\n");
    return;
  }

  /* Step 3: LoRa alert */
  DBGLN("[3/3] AMBULANCE CONFIRMED! Sending LoRa alert...");
  ambulancesFound++;

  if (loraInitialized) {
    sendAmbulanceAlert(MONITORED_LANE_ID);
  } else {
    DBGLN("[Pipeline] WARNING: LoRa not available!");
  }

  lastAlertTime = millis();
  inCooldown = true;
  DBG("[Pipeline] Cooldown started (%ds)\n", DETECTION_COOLDOWN_MS / 1000);
  DBGLN("=== PIPELINE ENDED (alert sent!) ===\n");
}

/* =====================================================================
 *  SETUP
 * ===================================================================== */

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);

  DBGLN("\n================================================");
  DBGLN("  Ambulance Detector v2.0 — All-in-One");
  DBGLN("  Laptop ML Audio + Roboflow Image Detection");
  DBGLN("  ESP32-CAM + SX1278 LoRa + Sound Sensor");
  DBGLN("================================================");
  DBG("  Lane: %d | Trigger: %d", MONITORED_LANE_ID, TRIGGER_MODE);
#if TRIGGER_MODE == 0
  DBGLN(" (WiFi HTTP)");
#elif TRIGGER_MODE == 1
  DBGLN(" (UART serial)");
#else
  DBGLN(" (WiFi + UART)");
#endif
  DBGLN("");

  /* Sound sensor (fallback) */
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(SOUND_SENSOR_PIN, INPUT);
  DBGLN("[Init] Sound sensor ready (GPIO 33, fallback).");

  /* WiFi */
  wifiConnected = connectWiFi();

  /* mDNS — makes ESP32 reachable as http://ambulance.local */
  if (wifiConnected) {
    if (MDNS.begin(MDNS_HOSTNAME)) {
      MDNS.addService("http", "tcp", HTTP_TRIGGER_PORT);
      DBG("[mDNS] Hostname: http://%s.local\n", MDNS_HOSTNAME);
    } else {
      DBGLN("[mDNS] Failed to start. Use IP address instead.");
    }
  }

  /* HTTP trigger server */
#if TRIGGER_MODE == 0 || TRIGGER_MODE == 2
  if (wifiConnected)
    setupHTTPServer();
#endif

  /* Camera */
  cameraInitialized = initCamera();

  /* LoRa */
  loraInitialized = initLoRa();

  /* Status */
  DBGLN("\n-------- SYSTEM STATUS --------");
  DBG("  WiFi:   %s\n", wifiConnected ? "✅ OK" : "❌ FAIL");
  DBG("  Camera: %s\n", cameraInitialized ? "✅ OK" : "❌ FAIL");
  DBG("  LoRa:   %s\n", loraInitialized ? "✅ OK" : "❌ FAIL");
#if TRIGGER_MODE == 0 || TRIGGER_MODE == 2
  if (wifiConnected) {
    DBG("  Trigger: http://%s/trigger\n", WiFi.localIP().toString().c_str());
  }
#endif
  DBGLN("-------------------------------\n");

  DBGLN("[Ready] Monitoring for triggers...");
  DBGLN("  Primary:  Laptop ML → HTTP /trigger");
  DBGLN("  Fallback: Local sound sensor (GPIO 33)");
  DBGLN("");
  DBGLN("  === ONE-CLICK LAUNCH (on laptop) ===");
  DBGLN("  ./start.sh");
  DBGLN("  OR: python3 src/siren_to_esp32.py --mode wifi");
  DBG("  mDNS: http://%s.local  |  IP: %s\n", MDNS_HOSTNAME,
      WiFi.localIP().toString().c_str());
  DBGLN("");
}

/* =====================================================================
 *  MAIN LOOP
 * ===================================================================== */

void loop() {
  uint32_t now = millis();

  /* Handle HTTP requests */
#if TRIGGER_MODE == 0 || TRIGGER_MODE == 2
  if (wifiConnected)
    httpServer.handleClient();
#endif

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

  /* Priority 1: WiFi HTTP trigger from laptop ML */
#if TRIGGER_MODE == 0 || TRIGGER_MODE == 2
  if (httpTriggerPending) {
    httpTriggerPending = false;
    if (cameraInitialized && wifiConnected) {
      runDetectionPipeline("ml_wifi");
    } else if (loraInitialized) {
      DBGLN("[Fallback] Camera/WiFi down → sound-only alert.");
      sendAmbulanceAlert(MONITORED_LANE_ID);
      lastAlertTime = millis();
      inCooldown = true;
    }
    return;
  }
#endif

  /* Priority 2: UART trigger from laptop ML */
#if TRIGGER_MODE == 1 || TRIGGER_MODE == 2
  if (checkUARTTrigger()) {
    if (cameraInitialized && wifiConnected) {
      runDetectionPipeline("ml_uart");
    } else if (loraInitialized) {
      DBGLN("[Fallback] Camera/WiFi down → sound-only alert.");
      sendAmbulanceAlert(MONITORED_LANE_ID);
      lastAlertTime = millis();
      inCooldown = true;
    }
    return;
  }
#endif

  /* Priority 3: Local sound sensor fallback */
  if (detectSiren()) {
    DBGLN("[Main] Sound sensor triggered (fallback).");
    if (cameraInitialized && wifiConnected) {
      runDetectionPipeline("sound");
    } else if (loraInitialized) {
      DBGLN("[Fallback] Camera/WiFi down → sound-only alert.");
      sendAmbulanceAlert(MONITORED_LANE_ID);
      lastAlertTime = millis();
      inCooldown = true;
    }
  }

  delay(10);
}
