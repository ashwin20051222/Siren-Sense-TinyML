/**
 * ============================================================================
 * STM32-ESP32 Receiver — ESP-NOW WiFi Receiver + UART Bridge to STM32
 * Board: ESP32 DevKit (any standard ESP32, NOT ESP32-CAM)
 * ============================================================================
 *
 * SYSTEM ARCHITECTURE:
 * ESP32-CAM (sender) ─── ESP-NOW WiFi ───► THIS ESP32 ─── UART ───► STM32
 *
 * WHAT THIS DOES:
 * 1. Broadcasts "PAIR:RCV" beacon until sender auto-discovers this board
 * 2. Receives "AMB:<lane_id>" messages from ESP32-CAM via ESP-NOW
 * 3. Forwards them to STM32 via UART (Serial2, GPIO 17 TX)
 * 4. Blinks LED on alert receipt
 *
 * MAC ADDRESSES:
 * This board (Receiver): FC:E8:C0:7A:B7:A0
 * Sender (ESP32-CAM):    A8:42:E3:56:83:2C
 * Use esp32_firmware/find_mac_address/ sketch to find your board's MAC.
 *
 * AUTO-PAIRING:
 * On boot, this receiver broadcasts a "PAIR:RCV" beacon every 2 seconds.
 * The sender (A8:42:E3:56:83:2C) auto-discovers this board, adds it as a
 * peer, and sends "PAIR:ACK" to confirm. LED blinks fast during pairing,
 * then shows brief solid to confirm paired.
 *
 * LIBRARIES:
 * - ESP32 Board Package (esp32 by Espressif, v2.x+)
 *
 * WIRING (only 3 wires!):
 * ESP32 GPIO 17 (TX2) ──► STM32 PB11 (USART3 RX)
 * ESP32 GPIO 16 (RX2) ──► STM32 PB10 (USART3 TX) [optional]
 * ESP32 GND            ──► STM32 GND (MUST be common ground!)
 * ESP32 5V             ──► USB power
 *
 * WiFi: Vivo_V29_5G (2.4 GHz) — both boards must be on the SAME network.
 * NOTE: LCD display is connected to the STM32 (PC6/PC7), NOT this ESP32.
 * ============================================================================
 **/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 1: USER CONFIGURATION — Edit these before flashing!    ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

/* ---- WiFi (2.4 GHz only) ----
 * Must be on the SAME network/channel as ESP32-CAM sender
 * for ESP-NOW to work reliably */
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
 * ║  SECTION 2: GLOBAL STATE & QUEUE                                ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

static const char *LANE_NAMES[] = {"NORTH", "EAST", "SOUTH", "WEST"};

/* FreeRTOS Queue: passes data from WiFi callback to main loop without
   volatile globals, priority inversion, or dropped messages. */
typedef struct {
  uint8_t mac[6];
  char msg[32];
} EspNowMsg_t;

QueueHandle_t espNowQueue = NULL;

static bool wifiConnected = false;
static bool espNowInitialized = false;

/* Auto-pairing state (no longer volatile — only accessed in loop) */
static bool paired = false;
static uint8_t senderMAC[6] = {0};
static uint32_t lastBeaconTime = 0;
static uint32_t totalMessagesReceived = 0;
static uint32_t lastReceiveTime = 0;

/* LED blink state (only accessed in loop) */
static uint32_t ledTurnedOnTime = 0;
static uint32_t ledDuration = 0;
static bool ledOn = false;

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 3: WiFi                                                ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

bool connectWiFi(void) {
  DBG("[WiFi] Connecting to %s", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.setSleep(false);
  /* Prevent channel hopping if the AP drops — ESP-NOW requires both
     radios locked to the same channel. Auto-reconnect scans ch1-13. */
  WiFi.setAutoReconnect(false);
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

/* ESP-NOW receive callback — runs in high-priority WiFi task.
   ZERO logic, ZERO Serial, ZERO blocking. Just enqueue and return. */
void onEspNowReceive(const esp_now_recv_info_t *info, const uint8_t *data,
                     int len) {
  if (len <= 0 || len >= 32 || espNowQueue == NULL)
    return;

  EspNowMsg_t rxData;
  memcpy(rxData.mac, info->src_addr, 6);
  memcpy(rxData.msg, data, len);
  rxData.msg[len] = '\0';

  /* Non-blocking enqueue (0 ticks wait — never stall the WiFi task) */
  xQueueSend(espNowQueue, &rxData, 0);
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
  bcastPeer.ifidx =
      WIFI_IF_STA; /* Explicitly bind to Station interface (ESP32 Core v3) */

  if (esp_now_add_peer(&bcastPeer) != ESP_OK) {
    DBGLN("[ESP-NOW] Failed to add broadcast peer!");
    return false;
  }

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

/* Send acknowledgment back to ESP32-CAM sender via ESP-NOW */
void sendAckToCamera(void) {
  if (!paired)
    return;

  const char *ackMsg = "ACK:AMB";
  esp_err_t result =
      esp_now_send(senderMAC, (const uint8_t *)ackMsg, strlen(ackMsg));
  if (result == ESP_OK) {
    DBG("[ESP-NOW] TX: Sent ACK to camera -> \"%s\"\n", ackMsg);
  } else {
    DBG("[ESP-NOW] TX: Failed to send ACK (Error: %d)\n", result);
  }
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

  /* FreeRTOS Queue: buffers up to 10 messages between callback and loop */
  espNowQueue = xQueueCreate(10, sizeof(EspNowMsg_t));
  if (espNowQueue == NULL) {
    DBGLN("[FATAL] Could not create FreeRTOS Queue!");
    while (1)
      ; /* halt — cannot operate without the queue */
  }

  /* ESP-NOW receiver with auto-pairing */
  espNowInitialized = initEspNow();

  /* Status report */
  DBGLN("\n-------- SYSTEM STATUS --------");
  DBG("  WiFi:     %s\n", wifiConnected ? "OK" : "FAIL");
  DBG("  ESP-NOW:  %s\n", espNowInitialized ? "OK" : "FAIL");
  DBG("  Queue:    OK (10 slots)\n");
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
    digitalWrite(LED_PIN, ((now / 200) % 2) ? HIGH : LOW);
  }

  /* ---- QUEUE PROCESSING ---- */
  EspNowMsg_t rxData;
  /* Process one message per loop tick to prevent task starvation.
     A while() loop here would starve the RTOS if the sender floods packets
     faster than Serial can print, triggering a TWDT crash. */
  if (xQueueReceive(espNowQueue, &rxData, 0) == pdTRUE) {

    /* 1. Handle Pairing Response */
    if (!paired && strcmp(rxData.msg, PAIR_ACK_MSG) == 0) {
      paired = true;
      memcpy(senderMAC, rxData.mac, 6);

      /* Register sender as ESP-NOW peer for two-way communication (ACK) */
      esp_now_peer_info_t senderPeer = {};
      memcpy(senderPeer.peer_addr, senderMAC, 6);
      senderPeer.channel = 0;
      senderPeer.encrypt = false;
      senderPeer.ifidx = WIFI_IF_STA;

      if (esp_now_add_peer(&senderPeer) == ESP_OK) {
        DBG("\n[PAIR] ✅ PAIRED and REGISTERED sender: "
            "%02X:%02X:%02X:%02X:%02X:%02X\n",
            rxData.mac[0], rxData.mac[1], rxData.mac[2], rxData.mac[3],
            rxData.mac[4], rxData.mac[5]);
      } else {
        DBG("\n[PAIR] ⚠️ PAIRED, but failed to register sender peer!\n");
      }

      digitalWrite(LED_PIN, HIGH);
      ledOn = true;
      ledTurnedOnTime = now;
      ledDuration = 1000;
    }

    /* 2. Handle Normal Messages (Only if paired) */
    else if (paired) {
      totalMessagesReceived++;
      lastReceiveTime = now;

      /* Self-heal if camera hardware is replaced (new MAC) */
      if (memcmp(senderMAC, rxData.mac, 6) != 0) {
        DBG("[PAIR] 🔄 Camera MAC changed! Updating peer...\n");
        esp_now_del_peer(senderMAC);
        memcpy(senderMAC, rxData.mac, 6);

        esp_now_peer_info_t newPeer = {};
        memcpy(newPeer.peer_addr, senderMAC, 6);
        newPeer.channel = 0;
        newPeer.encrypt = false;
        newPeer.ifidx = WIFI_IF_STA;
        esp_now_add_peer(&newPeer);
      }

      uint8_t laneId = 0;
      if (parseAmbulanceMessage(rxData.msg, &laneId)) {
        DBG("\n╔══════════════════════════════════════╗\n");
        DBG("║  AMBULANCE ALERT! Lane %d (%s)\n", laneId,
            (laneId < 4) ? LANE_NAMES[laneId] : "???");
        DBG("║  From: %02X:%02X:%02X:%02X:%02X:%02X\n", rxData.mac[0],
            rxData.mac[1], rxData.mac[2], rxData.mac[3], rxData.mac[4],
            rxData.mac[5]);
        DBG("╚══════════════════════════════════════╝\n\n");

        /* Forward to STM32 via UART */
        forwardToSTM32(rxData.msg);

        /* Send confirmation back to ESP32-CAM */
        sendAckToCamera();

        /* Blink LED */
        digitalWrite(LED_PIN, HIGH);
        ledOn = true;
        ledTurnedOnTime = now;
        ledDuration = LED_BLINK_MS;
      } else {
        DBG("[Warning] Unknown message: \"%s\"\n", rxData.msg);
      }
    }
  }

  /* ---- LED AUTO-OFF (rollover-safe) ---- */
  if (ledOn && (now - ledTurnedOnTime >= ledDuration)) {
    digitalWrite(LED_PIN, LOW);
    ledOn = false;
  }

  /* ---- UART BRIDGE READ ---- */
  while (Serial2.available()) {
    Serial.write(Serial2.read());
  }

  delay(10);
}