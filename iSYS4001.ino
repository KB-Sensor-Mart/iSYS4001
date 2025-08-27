#include "iSYS4001.h"

// Use Serial2 (ESP32 default pins: RX=16, TX=17). Adjust if needed for your board.
iSYS4001 radar(Serial2, 115200);

// Storage for decoded targets
iSYSTargetList_t targetList;

// Configuration
const uint8_t DESTINATION_ADDRESS = 0x81;  // Adjust per your device config
const uint32_t TIMEOUT_MS = 300;          // Response timeout
uint8_t SetminRangeValue = 0;
uint8_t SetmaxRangeValue = 150;



void setup() 
{
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  // Initialize Serial2 explicitly with pins on ESP32
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  // Print configuration
  Serial.print("Dest Addr: 0x"); Serial.println(DESTINATION_ADDRESS, HEX);
  Serial.print("Timeout: "); Serial.print(TIMEOUT_MS); Serial.println(" ms");
  Serial.println("Output: 1 (default)");

  // Flush any stale bytes from radar UART
  while (Serial2.available()) 
  { 
    Serial2.read(); 
  }

  delay(500);


 
  iSYSResult_t iSYS_setOutputRangeMin = radar.iSYS_setOutputRangeMin(ISYS_OUTPUT_1,SetminRangeValue, DESTINATION_ADDRESS, TIMEOUT_MS); 
  if (iSYS_setOutputRangeMin != ERR_OK) {
  Serial.print("iSYS_setOutputRangeMin failed: ");
  Serial.println(iSYS_setOutputRangeMin, HEX);
}

  iSYSResult_t iSYS_setOutputRangeMax = radar.iSYS_setOutputRangeMax(ISYS_OUTPUT_1,SetmaxRangeValue, DESTINATION_ADDRESS, TIMEOUT_MS); 
    if (iSYS_setOutputRangeMax != ERR_OK) {
  Serial.print("iSYS_setOutputRangeMax failed: ");
  Serial.println(iSYS_setOutputRangeMax, HEX);
}

  iSYSResult_t saveApplicationSettings = radar.saveApplicationSettings(DESTINATION_ADDRESS,TIMEOUT_MS);


}

void loop() {
  Serial.println("\n--- Requesting Target List ---");


  while (Serial2.available()) { Serial2.read(); }


  iSYSResult_t res = radar.getTargetList32(
    &targetList,
    DESTINATION_ADDRESS,
    TIMEOUT_MS
  );

  if (res != ERR_OK) {
    Serial.print("First attempt failed (code "); Serial.print(res); Serial.println(") - retrying once...");
    res = radar.getTargetList32(&targetList, DESTINATION_ADDRESS, TIMEOUT_MS);
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

  delay(300);
}
