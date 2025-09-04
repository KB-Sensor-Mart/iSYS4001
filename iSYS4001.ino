/********************************************************************************************
 * iSYS4001 Threshold Minimum Demo
 * ------------------------------------------------------------------------------------------
 * This example demonstrates how to set and read back the "threshold minimum" parameter
 * on an InnoSenT iSYS4001 radar sensor using a custom driver class.
 *
 * Hardware:
 *   - Tested with ESP32 (default Serial2 pins: RX=16, TX=17)
 *   - iSYS4001 radar sensor connected via UART (115200 baud, 8N1)
 *
 * Functionality:
 *   1. Initializes Serial2 for communication with the radar.
 *   2. Sends a command to set the threshold minimum to -26 dB.
 *   3. Reads back the threshold minimum from the sensor to verify.
 *   4. Periodically polls the threshold every 5 seconds and prints the value.
 *
 * Configuration:
 *   - DESTINATION_ADDRESS: The radar's address (default 0x80).
 *   - TIMEOUT_MS: Maximum wait time for responses (default 300 ms).
 *   - RX_PIN / TX_PIN: ESP32 pins used for Serial2.
 *
 * Notes:
 *   - Ensure your radar is wired correctly to Serial2 pins.
 *   - Change DESTINATION_ADDRESS if your radar uses a different address.
 *   - Uncomment "saveAllSettings()" block if you want to persist changes to EEPROM
 ********************************************************************************************/
#include "iSYS4001.h"

// Create radar object on Serial2 at 115200 baud
iSYS4001 radar(Serial2, 115200);

// Configuration
constexpr uint8_t DESTINATION_ADDRESS = 0x80; // Adjust to your sensor address
constexpr uint32_t TIMEOUT_MS = 300;          // Response timeout (ms)
constexpr int RX_PIN = 16;
constexpr int TX_PIN = 17;

static void printResult(const char *label, iSYSResult_t res)
{
  Serial.print(label);
  Serial.print(": ");
  Serial.println(res);
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    delay(10);
  }

  // Initialize Serial2 (ESP32 default: RX=16, TX=17)
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  // Clear any stale bytes
  while (Serial2.available())
  {
    Serial2.read();
  }
  delay(300);

  Serial.println("\n=== iSYS4001 Threshold Min Demo ===");

  // Set threshold min to -26 dB
  // int16_t setValue = -26;
  // iSYSResult_t r = radar.iSYS_setThresholdMin(setValue, DESTINATION_ADDRESS, TIMEOUT_MS);
  // if (r != ERR_OK) {
  // 	Serial.print("SET threshold min failed: ");
  // 	Serial.println(r);
  // } else {
  // 	Serial.print("SET threshold min OK (");
  // 	Serial.print(setValue);
  // 	Serial.println(" dB)");
  // }

  // Read back and print (value is returned in dB)
  int16_t readBack = 0;
  iSYSResult_t r = radar.iSYS_getThresholdMin(&readBack, DESTINATION_ADDRESS, TIMEOUT_MS);
  if (r != ERR_OK)
  {
    Serial.print("GET threshold min failed: ");
    Serial.println(r);
  }
  else
  {
    Serial.print("GET threshold min = ");
    Serial.print(readBack);
    Serial.println(" dB");
  }

  // Example (only if implemented in your class)
  Serial.println("Example: Save all settings");
  iSYSResult_t result = radar.saveAllSettings(DESTINATION_ADDRESS, TIMEOUT_MS);
  printResult("Save all settings", result);
  delay(2000);
}

void loop()
{
  // Optionally poll the threshold every 5 seconds
  static uint32_t last = 0;
  if (millis() - last > 5000)
  {
    last = millis();
    int16_t value = 0;
    iSYSResult_t r = radar.iSYS_getThresholdMin(&value, DESTINATION_ADDRESS, TIMEOUT_MS);
    if (r == ERR_OK)
    {
      Serial.print("Threshold min: ");
      Serial.print(value);
      Serial.println(" dB");
    }
    else
    {
      Serial.print("Read error: ");
      Serial.println(r);
    }
  }
}
