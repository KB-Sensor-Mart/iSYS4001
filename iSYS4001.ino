/*
  iSYS4001 Radar Sensor - Velocity Configuration Example
  ------------------------------------------------------
  ESP32 interface with iSYS4001 radar sensor
  
  Features:
  - Configure velocity thresholds (18-217 km/h)
  - Real-time target detection with debug output
  - Efficient retry logic and error handling
  
  Wiring: Radar TX→GPIO16, Radar RX→GPIO17, GND shared
  Note: Velocity settings in km/h, target data in m/s
*/

#include "iSYS4001.h"

// Configuration
iSYS4001 radar(Serial2, 115200);
iSYSTargetList_t targetList;
const uint8_t DEVICE_ADDR = 0x80;
const uint32_t TIMEOUT_MS = 300;
const uint16_t MIN_VELOCITY_KMH = 18;     // Minimum velocity threshold (km/h)
const uint16_t MAX_VELOCITY_KMH = 217;    // Maximum velocity threshold (km/h)
const bool DEBUG_ENABLED = true;           // Set to false to disable debug output

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  
  Serial.println("=== iSYS4001 Radar Setup ===");
  Serial.printf("Velocity Range: %d - %d km/h\n", MIN_VELOCITY_KMH, MAX_VELOCITY_KMH);
  Serial.printf("Device Address: 0x%02X\n", DEVICE_ADDR);
  
  // Enable debug output
  if (DEBUG_ENABLED) {
    radar.setDebug(Serial, true);
    Serial.println("Debug enabled");
  }
  
  // Initialize radar communication
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  while (Serial2.available()) { Serial2.read(); } // Flush
  delay(500);
  
  // Configure velocity thresholds
  Serial.println("Configuring velocity thresholds...");
  
  iSYSResult_t res = radar.iSYS_setOutputVelocityMin(ISYS_OUTPUT_1, MIN_VELOCITY_KMH, DEVICE_ADDR, TIMEOUT_MS);
  if (res == ERR_OK) {
    Serial.printf("Min velocity: %d km/h\n", MIN_VELOCITY_KMH);
  } else {
    Serial.printf("Min velocity failed: 0x%02X\n", res);
  }
  
  res = radar.iSYS_setOutputVelocityMax(ISYS_OUTPUT_1, MAX_VELOCITY_KMH, DEVICE_ADDR, TIMEOUT_MS);
  if (res == ERR_OK) {
    Serial.printf("Max velocity: %d km/h\n", MAX_VELOCITY_KMH);
  } else {
    Serial.printf("Max velocity failed: 0x%02X\n", res);
  }
  
  // Save settings
  res = radar.saveApplicationSettings(DEVICE_ADDR, TIMEOUT_MS);
  if (res == ERR_OK) {
    Serial.println("Settings saved");
  } else {
    Serial.printf("Save failed: 0x%02X\n", res);
  }
  
  Serial.println("=== Ready - Starting Detection ===\n");
}

void loop() {
  // Clear stale data
  while (Serial2.available()) { Serial2.read(); }

  // Request target list with retry
  iSYSResult_t res = radar.getTargetList32(&targetList, DEVICE_ADDR, TIMEOUT_MS);
  if (res != ERR_OK) {
    res = radar.getTargetList32(&targetList, DEVICE_ADDR, TIMEOUT_MS); // Retry once
  }

  // Display results
  if (res == ERR_OK && targetList.error.iSYSTargetListError == TARGET_LIST_OK) {
    Serial.printf("%u targets detected\n", targetList.nrOfTargets);
    
    if (targetList.nrOfTargets > 0) {
      for (uint16_t i = 0; i < targetList.nrOfTargets && i < MAX_TARGETS; i++) {
        Serial.printf("  T%u: Sig=%.1fdB Vel=%.2fm/s Range=%.2fm Ang=%.1f°\n",
                     i + 1,
                     targetList.targets[i].signal,
                     targetList.targets[i].velocity,
                     targetList.targets[i].range,
                     targetList.targets[i].angle);
      }
    } else {
      Serial.println("No targets in velocity range");
    }
  } else {
    Serial.printf("Detection failed (0x%02X)\n", res);
  }

  delay(500); // 500ms between readings
}
