/**
 * ============================================================================
 *  STM32-ESP32 Receiver — ESP-NOW WiFi Receiver + UART Bridge to STM32
 *  Board: ESP32 DevKit (any standard ESP32, NOT ESP32-CAM)
 * ============================================================================
 *
 *  SYSTEM ARCHITECTURE:
 *    ESP32-CAM (sender) ─── ESP-NOW WiFi ───► THIS ESP32 ─── UART ───► STM32
 *
 *  WHAT THIS DOES:
 *    1. Broadcasts "PAIR:RCV" beacon until sender auto-discovers this board
 *    2. Receives "AMB:<lane_id>" messages from ESP32-CAM via ESP-NOW
 *    3. Forwards them to STM32 via UART (Serial2, GPIO 17 TX)
 *    4. Blinks LED on alert receipt
 *
 *  MAC ADDRESSES:
 *    This board (Receiver): FC:E8:C0:7A:B7:A0
 *    Sender (ESP32-CAM):    A8:42:E3:56:83:2C
 *    Use esp32_firmware/find_mac_address/ sketch to find your board's MAC.
 *
 *  AUTO-PAIRING:
 *    On boot, this receiver broadcasts a "PAIR:RCV" beacon every 2 seconds.
 *    The sender (A8:42:E3:56:83:2C) auto-discovers this board, adds it as a
 *    peer, and sends "PAIR:ACK" to confirm. LED blinks fast during pairing,
 *    then shows brief solid to confirm paired.
 *
 *  LIBRARIES:
 *    - ESP32 Board Package (esp32 by Espressif, v2.x+)
 *
 *  WIRING (only 3 wires!):
 *    ESP32 GPIO 17 (TX2) ──► STM32 PB11 (USART3 RX)
 *    ESP32 GPIO 16 (RX2) ──► STM32 PB10 (USART3 TX) [optional]
 *    ESP32 GND            ──► STM32 GND (MUST be common ground!)
 *    ESP32 5V             ──► USB power
 *
 *  WiFi: Vivo_V29_5G (2.4 GHz) — both boards must be on the SAME network.
 *  NOTE: LCD display is connected to the STM32 (PC6/PC7), NOT this ESP32.
 * ============================================================================
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 1: USER CONFIGURATION — Edit these before flashing!    ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

/* ---- WiFi (2.4 GHz only) ----
 *  Must be on the SAME network/channel as ESP32-CAM sender
 *  for ESP-NOW to work reliably */
#define WIFI_SSID "Vivo_V29_5G"
#define WIFI_PASSWORD "123456789"

/* ---- UART to STM32 ---- */
#define STM32_UART_BAUD 115200
#define STM32_TX_PIN 17 /* ESP32 TX2 → STM32 PB11 (USART3 RX) */
#define STM32_RX_PIN 16 /* ESP32 RX2 ← STM32 PB10 (USART3 TX) [optional] */

/* ---- Status LED ---- */
#define LED_PIN 2 /* Built-in LED on most ESP32 DevKits */
#define LED_BLINK_MS 500

/* ---- Auto-Pairing ---- */
#define PAIR_BEACON_INTERVAL_MS 2000 /* Broadcast beacon every 2 seconds */
#define PAIR_BEACON_MSG "PAIR:RCV"   /* Beacon message sent to broadcast */
#define PAIR_ACK_MSG "PAIR:ACK"      /* ACK message from sender */

/* ---- Known Sender MAC (ESP32-CAM: A8:42:E3:56:83:2C) ---- */
static const uint8_t KNOWN_SENDER_MAC[] = {0xA8, 0x42, 0xE3, 0x56, 0x83, 0x2C};

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
 * ║  SECTION 2: GLOBAL STATE                                        ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

static const char *LANE_NAMES[] = {"NORTH", "EAST", "SOUTH", "WEST"};

static bool wifiConnected = false;
static bool espNowInitialized = false;

/* Auto-pairing state */
static bool paired = false;
static uint8_t senderMAC[6] = {0};
static uint32_t lastBeaconTime = 0;

/* Received message state */
static volatile bool newMessageReceived = false;
static char lastReceivedMsg[32] = "";
static uint8_t lastSenderMAC[6] = {0};
static uint32_t totalMessagesReceived = 0;
static uint32_t lastReceiveTime = 0;

/* LED blink */
static uint32_t ledOffTime = 0;
static bool ledOn = false;

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 3: WiFi                                                ║
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
 * ║  SECTION 4: ESP-NOW AUTO-PAIRING + RECEIVER                    ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

/* ESP-NOW receive callback — handles both pairing ACK and ambulance messages */
void onEspNowReceive(const uint8_t *mac_addr, const uint8_t *data, int len) {
  if (len <= 0 || len >= (int)sizeof(lastReceivedMsg))
    return;

  /* Temporary buffer to check message */
  char msg[32];
  memcpy(msg, data, len);
  msg[len] = '\0';

  /* Check for PAIR:ACK from sender */
  if (!paired && strcmp(msg, PAIR_ACK_MSG) == 0) {
    paired = true;
    memcpy(senderMAC, mac_addr, 6);
    DBG("\n[PAIR] ✅ PAIRED with sender: %02X:%02X:%02X:%02X:%02X:%02X\n",
        mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4],
        mac_addr[5]);

    /* Brief LED solid on to confirm pairing */
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    return;
  }

  /* Normal message (e.g., AMB:0) */
  memcpy(lastReceivedMsg, msg, len + 1);
  memcpy(lastSenderMAC, mac_addr, 6);
  newMessageReceived = true;
  totalMessagesReceived++;

  DBG("[ESP-NOW] RX from %02X:%02X:%02X:%02X:%02X:%02X: \"%s\"\n", mac_addr[0],
      mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5],
      lastReceivedMsg);
}

bool initEspNow(void) {
  if (esp_now_init() != ESP_OK) {
    DBGLN("[ESP-NOW] Init FAILED!");
    return false;
  }

  esp_now_register_recv_cb(onEspNowReceive);

  /* Add broadcast peer for pairing beacons */
  esp_now_peer_info_t bcastPeer = {};
  memset(bcastPeer.peer_addr, 0xFF, 6); /* FF:FF:FF:FF:FF:FF = broadcast */
  bcastPeer.channel = 0;
  bcastPeer.encrypt = false;
  esp_now_add_peer(&bcastPeer);

  DBGLN("[ESP-NOW] Receiver initialized with auto-pairing.");
  return true;
}

/* Send pairing beacon via broadcast */
void sendPairingBeacon(void) {
  uint32_t now = millis();
  if ((now - lastBeaconTime) < PAIR_BEACON_INTERVAL_MS)
    return;
  lastBeaconTime = now;

  uint8_t bcast[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_send(bcast, (uint8_t *)PAIR_BEACON_MSG, strlen(PAIR_BEACON_MSG));
  DBG("[PAIR] Beacon sent: \"%s\" (waiting for sender...)\n", PAIR_BEACON_MSG);
}

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 5: UART BRIDGE TO STM32                               ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

void initSTM32UART(void) {
  Serial2.begin(STM32_UART_BAUD, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);
  DBG("[UART] Serial2 initialized: TX=GPIO%d, RX=GPIO%d, Baud=%d\n",
      STM32_TX_PIN, STM32_RX_PIN, STM32_UART_BAUD);
}

void forwardToSTM32(const char *msg) {
  Serial2.println(msg);
  DBG("[UART] Forwarded to STM32: \"%s\"\n", msg);
}

/* Parse and validate AMB:<lane_id> message */
bool parseAmbulanceMessage(const char *msg, uint8_t *laneId) {
  if (strncmp(msg, "AMB:", 4) != 0)
    return false;

  int lane = atoi(msg + 4);
  if (lane < 0 || lane > 3)
    return false;

  *laneId = (uint8_t)lane;
  return true;
}

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 6: SETUP                                               ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);

  DBGLN("\n================================================");
  DBGLN("  STM32-ESP32 Receiver v7.0 — Auto-Pairing");
  DBGLN("  ESP-NOW → UART Bridge");
  DBGLN("  Receiver MAC: FC:E8:C0:7A:B7:A0");
  DBGLN("  Known Sender: A8:42:E3:56:83:2C");
  DBGLN("  WiFi: Vivo_V29_5G");
  DBGLN("  LCD is on STM32 only (PC6/PC7 I2C)");
  DBGLN("================================================");

  /* LED */
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  /* WiFi (STA mode — required for ESP-NOW) */
  wifiConnected = connectWiFi();

  /* Print own MAC address for reference */
  DBG("\n[Info] This ESP32 MAC: %s\n", WiFi.macAddress().c_str());

  /* UART to STM32 */
  initSTM32UART();

  /* ESP-NOW receiver with auto-pairing */
  espNowInitialized = initEspNow();

  /* Status report */
  DBGLN("\n-------- SYSTEM STATUS --------");
  DBG("  WiFi:     %s\n", wifiConnected ? "OK" : "FAIL");
  DBG("  ESP-NOW:  %s\n", espNowInitialized ? "OK" : "FAIL");
  DBG("  UART TX:  GPIO %d @ %d baud\n", STM32_TX_PIN, STM32_UART_BAUD);
  DBGLN("  LCD:      On STM32 (not this board)");
  DBGLN("  Pairing:  Broadcasting beacon...");
  DBGLN("-------------------------------\n");
}

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 7: MAIN LOOP                                          ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

void loop() {
  uint32_t now = millis();

  /* ---- AUTO-PAIRING: broadcast beacon until sender responds ---- */
  if (!paired && espNowInitialized) {
    sendPairingBeacon();

    /* Fast-blink LED while waiting for pairing */
    if ((now / 200) % 2)
      digitalWrite(LED_PIN, HIGH);
    else
      digitalWrite(LED_PIN, LOW);

    delay(10);
    return; /* Don't process ambulance messages until paired */
  }

  /* ---- NORMAL OPERATION (paired) ---- */

  /* Process received ESP-NOW message */
  if (newMessageReceived) {
    newMessageReceived = false;
    lastReceiveTime = now;

    /* Validate message */
    uint8_t laneId = 0;
    if (parseAmbulanceMessage(lastReceivedMsg, &laneId)) {
      DBG("\n╔══════════════════════════════════════╗\n");
      DBG("║  AMBULANCE ALERT! Lane %d (%s)\n", laneId,
          (laneId < 4) ? LANE_NAMES[laneId] : "???");
      DBG("║  From: %02X:%02X:%02X:%02X:%02X:%02X\n", lastSenderMAC[0],
          lastSenderMAC[1], lastSenderMAC[2], lastSenderMAC[3],
          lastSenderMAC[4], lastSenderMAC[5]);
      DBG("╚══════════════════════════════════════╝\n\n");

      /* Forward to STM32 via UART */
      forwardToSTM32(lastReceivedMsg);

      /* Blink LED */
      digitalWrite(LED_PIN, HIGH);
      ledOn = true;
      ledOffTime = now + LED_BLINK_MS;
    } else {
      DBG("[Warning] Unknown message: \"%s\"\n", lastReceivedMsg);
    }
  }

  /* LED auto-off */
  if (ledOn && now >= ledOffTime) {
    digitalWrite(LED_PIN, LOW);
    ledOn = false;
  }

  /* Also check if STM32 sends anything back (optional) */
  while (Serial2.available()) {
    char c = Serial2.read();
    Serial.write(c);
  }

  delay(10);
}
