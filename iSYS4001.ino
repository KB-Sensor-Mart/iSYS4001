#include "iSYS4001.h"

// Use Serial2 (ESP32 default pins: RX=16, TX=17). Adjust if needed for your board.
iSYS4001 radar(Serial2, 115200);

// Storage for decoded targets
iSYSTargetList_t targetList;

// Configuration
const uint8_t DESTINATION_ADDRESS = 0x80;  // Adjust per your device config
const uint32_t TIMEOUT_MS = 3000;          // Response timeout
const iSYSOutputNumber_t OUTPUT_NUMBER = ISYS_OUTPUT_1;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("iSYS4001 Radar Target List Example");
  Serial.println("Serial2 (RX=16, TX=17) used for radar link");

  // Initialize Serial2 explicitly with pins on ESP32
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  // Print configuration
  Serial.print("Dest Addr: 0x"); Serial.println(DESTINATION_ADDRESS, HEX);
  Serial.print("Timeout: "); Serial.print(TIMEOUT_MS); Serial.println(" ms");
  Serial.print("Output: "); Serial.println(OUTPUT_NUMBER);

  // Flush any stale bytes from radar UART
  while (Serial2.available()) { Serial2.read(); }

  delay(500);
}

void loop() {
  Serial.println("\n--- Requesting Target List ---");

  // Flush any leftover bytes before each request
  while (Serial2.available()) { Serial2.read(); }

  iSYSResult_t res = radar.getTargetList32(
    &targetList,
    OUTPUT_NUMBER,
    DESTINATION_ADDRESS,
    TIMEOUT_MS
  );

  if (res != ERR_OK) {
    Serial.print("First attempt failed (code "); Serial.print(res); Serial.println(") - retrying once...");
    res = radar.getTargetList32(&targetList, OUTPUT_NUMBER, DESTINATION_ADDRESS, TIMEOUT_MS);
  }

  if (res == ERR_OK) {
    if (targetList.error.iSYSTargetListError == TARGET_LIST_OK) {
      Serial.print("Targets: ");
      Serial.print(targetList.nrOfTargets);
      Serial.print(", Output: ");
      Serial.println(targetList.outputNumber);

      for (uint16_t i = 0; i < targetList.nrOfTargets && i < MAX_TARGETS; i++) {
        Serial.print("#"); Serial.print(i + 1); Serial.println("");
        Serial.print("  Signal: "); Serial.println(targetList.targets[i].signal);
        Serial.print("  Velocity: "); Serial.print(targetList.targets[i].velocity); Serial.println(" m/s");
        Serial.print("  Range: "); Serial.print(targetList.targets[i].range); Serial.println(" m");
        Serial.print("  Angle: "); Serial.print(targetList.targets[i].angle); Serial.println(" deg");
      }
    } else {
      Serial.print("Target list error code: ");
      Serial.println(targetList.error.iSYSTargetListError);
    }
  } else {
    Serial.print("Failed to get target list - error code: ");
    Serial.println(res);
  }

  delay(1000);
}
