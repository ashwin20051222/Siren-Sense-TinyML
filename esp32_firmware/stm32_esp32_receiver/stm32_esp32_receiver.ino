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
 *    1. Receives "AMB:<lane_id>" messages from ESP32-CAM via ESP-NOW
 *    2. Forwards them to STM32 via UART (Serial2, GPIO 17 TX)
 *    3. Blinks LED and shows status on optional LCD
 *
 *  LIBRARIES:
 *    - ArduinoJson by Benoit Blanchon (v6.x or v7.x) — for status endpoint
 *    - ESP32 Board Package (esp32 by Espressif, v2.x+)
 *
 *  WIRING:
 *    ESP32 GPIO 17 (TX2) ──► STM32 USART RX (e.g. PA3 for USART2)
 *    ESP32 GPIO 16 (RX2) ──► STM32 USART TX (e.g. PA2 for USART2) [optional]
 *    ESP32 GND            ──► STM32 GND (MUST be common ground!)
 *    ESP32 5V             ──► USB power
 * ============================================================================
 */

#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 1: USER CONFIGURATION — Edit these before flashing!    ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

/* ---- WiFi (2.4 GHz only) ----
 *  Must be on the SAME network/channel as ESP32-CAM sender
 *  for ESP-NOW to work reliably */
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

/* ---- UART to STM32 ---- */
#define STM32_UART_BAUD 115200
#define STM32_TX_PIN 17 /* ESP32 TX2 → STM32 USART RX */
#define STM32_RX_PIN 16 /* ESP32 RX2 ← STM32 USART TX (optional) */

/* ---- Status LED ---- */
#define LED_PIN 2 /* Built-in LED on most ESP32 DevKits */
#define LED_BLINK_MS 500

/* ---- LCD Display (16×2 I2C, PCF8574 backpack) ----
 *  Set LCD_ENABLED to 0 if no LCD connected */
#define LCD_ENABLED 1
#define LCD_I2C_ADDR 0x27
#define LCD_SDA_PIN 21 /* Default I2C SDA on ESP32 DevKit */
#define LCD_SCL_PIN 22 /* Default I2C SCL on ESP32 DevKit */
#define LCD_REFRESH_MS 500

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
 * ║  SECTION 2: LCD I2C DRIVER (self-contained, no library needed)  ║
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

static bool lcdReady = false;

static void lcd_i2c_write(uint8_t data) {
  Wire.beginTransmission(LCD_I2C_ADDR);
  Wire.write(data);
  Wire.endTransmission();
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
  Wire.begin(LCD_SDA_PIN, LCD_SCL_PIN, 100000);
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
 * ║  SECTION 3: GLOBAL STATE                                        ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

static const char *LANE_NAMES[] = {"NORTH", "EAST", "SOUTH", "WEST"};

static bool wifiConnected = false;
static bool espNowInitialized = false;
static bool lcdInitialized = false;

/* Received message state */
static volatile bool newMessageReceived = false;
static char lastReceivedMsg[32] = "";
static uint8_t lastSenderMAC[6] = {0};
static uint32_t totalMessagesReceived = 0;
static uint32_t lastReceiveTime = 0;

/* LED blink */
static uint32_t ledOffTime = 0;
static bool ledOn = false;

/* LCD update */
static uint32_t lastLcdUpdate = 0;

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 4: WiFi                                                ║
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
 * ║  SECTION 5: ESP-NOW RECEIVER                                   ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

/* ESP-NOW receive callback — called from WiFi task context */
void onEspNowReceive(const uint8_t *mac_addr, const uint8_t *data, int len) {
  if (len <= 0 || len >= (int)sizeof(lastReceivedMsg))
    return;

  /* Copy message */
  memcpy(lastReceivedMsg, data, len);
  lastReceivedMsg[len] = '\0';
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

  DBGLN("[ESP-NOW] Receiver initialized, waiting for messages...");
  return true;
}

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 6: UART BRIDGE TO STM32                               ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

void initSTM32UART(void) {
  Serial2.begin(STM32_UART_BAUD, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);
  DBG("[UART] Serial2 initialized: TX=GPIO%d, RX=GPIO%d, Baud=%d\n",
      STM32_TX_PIN, STM32_RX_PIN, STM32_UART_BAUD);
}

void forwardToSTM32(const char *msg) {
  /* Send message + newline to STM32 via UART */
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
 * ║  SECTION 7: LCD STATUS DISPLAY                                  ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

void LCD_UpdateStatus(void) {
#if LCD_ENABLED == 0
  return;
#endif
  if (!lcdInitialized)
    return;
  uint32_t now = millis();
  if ((now - lastLcdUpdate) < LCD_REFRESH_MS)
    return;
  lastLcdUpdate = now;

  char row0[17], row1[17];

  if (lastReceiveTime > 0 && (now - lastReceiveTime) < 15000) {
    /* Recent alert — show ambulance info */
    uint8_t laneId = 0;
    parseAmbulanceMessage(lastReceivedMsg, &laneId);
    uint32_t agoSec = (now - lastReceiveTime) / 1000;
    snprintf(row0, sizeof(row0), "!! AMBULANCE !!");
    snprintf(row1, sizeof(row1), "Lane:%-5s %2lus",
             (laneId < 4) ? LANE_NAMES[laneId] : "???", (unsigned long)agoSec);
  } else {
    /* Idle — show uptime and message count */
    uint32_t uptimeMin = now / 60000;
    snprintf(row0, sizeof(row0), "Receiver Ready");
    snprintf(row1, sizeof(row1), "Up:%lum Rx:%lu", (unsigned long)uptimeMin,
             (unsigned long)totalMessagesReceived);
  }

  LCD_SetCursor(0, 0);
  LCD_PrintPadded(row0);
  LCD_SetCursor(1, 0);
  LCD_PrintPadded(row1);
}

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 8: SETUP                                               ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);

  DBGLN("\n================================================");
  DBGLN("  STM32-ESP32 Receiver v4.0 — ESP-NOW → UART");
  DBGLN("  Receives from ESP32-CAM, forwards to STM32");
  DBGLN("================================================");

  /* LED */
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  /* WiFi (STA mode — required for ESP-NOW) */
  wifiConnected = connectWiFi();

  /* Print MAC address — MUST be copied to sender's RECEIVER_MAC */
  DBGLN("\n┌─────────────────────────────────────────────┐");
  DBG("│  THIS ESP32 MAC: %s  │\n", WiFi.macAddress().c_str());
  DBGLN("│  ↑↑ Copy this to sender's RECEIVER_MAC! ↑↑  │");
  DBGLN("└─────────────────────────────────────────────┘\n");

  /* UART to STM32 */
  initSTM32UART();

  /* ESP-NOW receiver */
  espNowInitialized = initEspNow();

  /* LCD */
#if LCD_ENABLED
  lcdInitialized = LCD_Init();
  if (lcdInitialized) {
    LCD_SetCursor(0, 0);
    LCD_PrintPadded("ESP32 Receiver");
    LCD_SetCursor(1, 0);
    LCD_PrintPadded("Waiting...");
  }
#endif

  /* Status report */
  DBGLN("\n-------- SYSTEM STATUS --------");
  DBG("  WiFi:     %s\n", wifiConnected ? "OK" : "FAIL");
  DBG("  ESP-NOW:  %s\n", espNowInitialized ? "OK" : "FAIL");
  DBG("  UART TX:  GPIO %d @ %d baud\n", STM32_TX_PIN, STM32_UART_BAUD);
  DBG("  LCD:      %s\n", lcdInitialized ? "OK" : "OFF/FAIL");
  DBGLN("-------------------------------\n");
  DBGLN("[Ready] Waiting for ESP-NOW messages from ESP32-CAM...");
}

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 9: MAIN LOOP                                          ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

void loop() {
  uint32_t now = millis();

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

  /* LCD update */
  LCD_UpdateStatus();

  /* Also check if STM32 sends anything back (optional) */
  while (Serial2.available()) {
    char c = Serial2.read();
    Serial.write(c); /* Echo STM32 messages to debug serial */
  }

  delay(10);
}
