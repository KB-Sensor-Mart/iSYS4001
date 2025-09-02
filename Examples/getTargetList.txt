/*
  iSYS4001 Radar Sensor (Get Target List)
  
  --------------------------------------------------------
  This sketch demonstrates how to request and display the target list
  from the iSYS4001 radar module using an ESP32.

  Features:
    - Initializes communication with iSYS4001 over Serial2 (pins RX=16, TX=17).
    - Continuously requests the target list from radar.
    - Prints detected targets with details:
        * Signal strength
        * Velocity (m/s)
        * Range (m)
        * Angle (degrees)
    - Retries once if the radar does not respond within the timeout.
    - Cleans stale UART data before each request for stability.

  Hardware:
    - ESP32 board
    - iSYS4001 radar module connected via UART2
        ESP32 Pin 16 (RX) ←→ Radar TX
        ESP32 Pin 17 (TX) ←→ Radar RX
        GND shared between ESP32 and radar

  Notes:
    - DESTINATION_ADDRESS may need to be adjusted depending on radar setup.
    - TIMEOUT_MS defines how long to wait for a response.
    - By default, the library requests data from Output 1 (ISYS_OUTPUT_1).
      To use Output 2 or others, specify it in getTargetList32().

*/

#include "iSYS4001.h"

// Radar object using Serial2
iSYS4001 radar(Serial2, 115200);

// Storage for decoded targets
iSYSTargetList_t targetList;

// Configuration
const uint8_t DESTINATION_ADDRESS = 0x80;   // Adjust per your device config
const uint32_t TIMEOUT_MS = 300;            // Response timeout (ms)

// ---------------------- Helper Functions ---------------------- //
void flushSerial2() {
  while (Serial2.available()) { Serial2.read(); }
}

void printTargetList() {
  iSYSResult_t res = radar.getTargetList32(&targetList, DESTINATION_ADDRESS, TIMEOUT_MS);

  // Retry once if the first attempt fails
  if (res != ERR_OK) {
    Serial.print("First attempt failed (code ");
    Serial.print(res);
    Serial.println(") - retrying...");
    res = radar.getTargetList32(&targetList, DESTINATION_ADDRESS, TIMEOUT_MS);
  }

  if (res == ERR_OK) {
    if (targetList.error.iSYSTargetListError == TARGET_LIST_OK) {
      Serial.print("Targets: ");
      Serial.print(targetList.nrOfTargets);
      Serial.print(", Output: ");
      Serial.println(targetList.outputNumber);

      for (uint16_t i = 0; i < targetList.nrOfTargets && i < MAX_TARGETS; i++) {
        Serial.printf("Target #%u\n", i + 1);
        Serial.printf("  Signal: %.2f dB\n", targetList.targets[i].signal);
        Serial.printf("  Velocity: %.2f m/s\n", targetList.targets[i].velocity);
        Serial.printf("  Range: %.2f m\n", targetList.targets[i].range);
        Serial.printf("  Angle: %.2f deg\n", targetList.targets[i].angle);
      }
    } else {
      Serial.print("Target list error code: ");
      Serial.println(targetList.error.iSYSTargetListError);
    }
  } else {
    Serial.print("Failed to get target list - error code: ");
    Serial.println(res);
  }
}

// ---------------------- Arduino Setup & Loop ---------------------- //
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  // Initialize Serial2 for radar
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  Serial.println("\n--- Radar Initialization ---");
  Serial.printf("Dest Addr: 0x%X\n", DESTINATION_ADDRESS);
  Serial.printf("Timeout: %lu ms\n", TIMEOUT_MS);
  Serial.println("Output: 1 (default)");

  flushSerial2();
  delay(100);
}

void loop() {
  Serial.println("\n--- Requesting Target List ---");
  flushSerial2();
  printTargetList();
  delay(500);
}
