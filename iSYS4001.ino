#include "iSYS4001.h"

// Use Serial2 (ESP32 default pins: RX=16, TX=17). Adjust if needed for your board.
iSYS4001 radar(Serial2, 115200);

// Storage for decoded targets
iSYSTargetList_t targetList;

// Configuration
const uint8_t deviceAddress = 0x81;  // Adjust per your device config
const uint32_t timeout = 300;          // Response timeout

void setup() 
{
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  // Initialize Serial2 explicitly with pins on ESP32
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  // Flush any stale bytes from radar UART
  Serial2.flush();
  
  Serial.println("iSYS4001 Radar Sensor Initialized");
  
  // Configure output direction for OUTPUT 1
  // Set to detect both approaching and receding targets
  // iSYSResult_t result = radar.iSYS_setOutputDirection(ISYS_OUTPUT_1, ISYS_TARGET_DIRECTION_BOTH, deviceAddress, timeout);
  
  // if (result == ERR_OK) {
  //   Serial.println("Output direction set successfully to BOTH directions");
  // } else {
  //   Serial.print("Failed to set output direction. Error code: ");
  //   Serial.println(result);
  // }
  
  // You can also set different directions for different outputs
  // // For OUTPUT 2: only approaching targets
  // iSYSResult_t result = radar.iSYS_setOutputDirection(ISYS_OUTPUT_2, ISYS_TARGET_DIRECTION_APPROACHING, deviceAddress, timeout);
  // if (result == ERR_OK) {
  //   Serial.println("OUTPUT 2 direction set to APPROACHING only");
  // }
  
  // For OUTPUT 3: only receding targets  
  iSYSResult_t result = radar.iSYS_setOutputDirection(ISYS_OUTPUT_1, ISYS_TARGET_DIRECTION_BOTH, deviceAddress, timeout);
  if (result == ERR_OK) {
    Serial.println("OUTPUT 1 direction set to RECEDING only");
  }
  
  // // Start acquisition
  // iSYSResult_t result = radar.iSYS_startAcquisition(deviceAddress, timeout);
  // if (result == ERR_OK) {
  //   Serial.println("Radar acquisition started");
  // } else {
  //   Serial.println("Failed to start acquisition");
  // }


    Serial.println("Saving all settings...");
 result = radar.saveAllSettings(deviceAddress,timeout);
  if (result == ERR_OK) {
    Serial.println(" All settings saved successfully");
  } else {
    Serial.print(" Failed to save all settings. Error code: 0x");
    Serial.println(result, HEX);
  }
}

void loop() 
{
  // Get target list from OUTPUT 1 (configured for both directions)
  iSYSResult_t result = radar.getTargetList32(&targetList, deviceAddress, timeout, ISYS_OUTPUT_1);
  
  if (result == ERR_OK) {
    Serial.print("Detected ");
    Serial.print(targetList.nrOfTargets);
    Serial.println(" targets:");
    
    for (int i = 0; i < targetList.nrOfTargets; i++) {
      Serial.print("Target ");
      Serial.print(i + 1);
      Serial.print(": Range=");
      Serial.print(targetList.targets[i].range);
      Serial.print("m, Velocity=");
      Serial.print(targetList.targets[i].velocity);
      Serial.print("m/s, Signal=");
      Serial.print(targetList.targets[i].signal);
      Serial.print(", Angle=");
      Serial.print(targetList.targets[i].angle);
      Serial.println("°");
    }
  } else if (result == TARGET_LIST_ACQUISITION_NOT_STARTED) {
    Serial.println("Acquisition not started");
  } else {
    Serial.print("Error getting target list: ");
    Serial.println(result);
  }
  
  delay(1000); // Update every second
}
  