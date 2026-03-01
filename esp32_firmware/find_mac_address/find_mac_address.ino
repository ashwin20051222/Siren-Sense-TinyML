/**
 * ============================================================================
 *  Find MAC Address — ESP32 MAC Address Discovery Utility
 *  Board: Any ESP32 (ESP32 DevKit, ESP32-CAM, etc.)
 * ============================================================================
 *
 *  WHAT THIS DOES:
 *    Reads and prints the board's WiFi Station MAC address via Serial Monitor.
 *    Flash this sketch to any ESP32 board to find its unique MAC address.
 *    You need the MAC address to configure ESP-NOW peer communication.
 *
 *  HOW TO USE:
 *    1. Open this file in Arduino IDE
 *    2. Select your board (ESP32 Dev Module or AI Thinker ESP32-CAM)
 *    3. Upload to the board
 *    4. Open Serial Monitor at 115200 baud
 *    5. Press Reset — the MAC address will be printed
 *    6. Copy the MAC address for use in sender/receiver firmware
 *
 *  CURRENT BOARDS:
 *    Sender  (ESP32-CAM):   A8:42:E3:56:83:2C
 *    Receiver (ESP32 DevKit): FC:E8:C0:7A:B7:A0
 *
 *  LIBRARIES:
 *    - ESP32 Board Package (esp32 by Espressif, v2.x+)
 *    - No additional libraries required
 * ============================================================================
 */

#include <esp_mac.h>
#include <esp_system.h>

void setup() {
  // Start serial communication
  Serial.begin(115200);
  delay(1000);

  uint8_t mac[6];

  // Read the physical eFuse ROM directly without turning on Wi-Fi
  esp_read_mac(mac, ESP_MAC_WIFI_STA);

  Serial.println();
  Serial.println("================================================");
  Serial.println("  ESP32 MAC Address Finder");
  Serial.println("================================================");
  Serial.println();
  Serial.print("Board MAC Address: ");

  // Format and print the MAC address
  for (int i = 0; i < 5; i++) {
    Serial.printf("%02X:", mac[i]);
  }
  Serial.printf("%02X\n", mac[5]);

  Serial.println();
  Serial.println("Copy this MAC address and use it in your");
  Serial.println("ESP-NOW sender/receiver firmware configuration.");
  Serial.println();
  Serial.println("To use in code, format as:");
  Serial.printf(
      "  uint8_t mac[] = {0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X};\n",
      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.println("================================================");
}

void loop() {
  // Nothing needed in the loop
}
