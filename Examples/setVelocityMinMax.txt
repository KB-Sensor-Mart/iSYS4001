/*
  iSYS4001 Radar Sensor — Set Velocity Min/Max and Read Targets (16/32-bit)
  -------------------------------------------------------------------------
  This example demonstrates how to:
  - Configure velocity thresholds (min/max in km/h) for Output 1
  - Persist settings to non-volatile memory
  - Enable the Multiple Target Filter so the radar can report multiple targets
  - Start acquisition before requesting target lists
  - Read and print targets using both 16-bit (compact) and 32-bit (high precision) lists

  Wiring (ESP32):
    - Radar TX → GPIO16 (ESP32 RX)
    - Radar RX → GPIO17 (ESP32 TX)
    - GND shared

  Notes:
    - Velocity thresholds are configured in km/h, but target velocity is reported in m/s.
    - Make sure to call iSYS_startAcquisition() before requesting targets; otherwise you might not see any data.
    - After changing configuration (velocity bounds, filters), call saveAllSettings() to persist across reboots.
*/

#include "iSYS4001.h"

// Configuration
iSYS4001 radar(Serial2, 115200);
// Shared buffer for target lists
iSYSTargetList_t targetList;
const uint8_t DEVICE_ADDR = 0x80;
const uint32_t TIMEOUT_MS = 300;
const uint16_t MIN_VELOCITY_KMH = 18;     // Minimum velocity threshold (km/h)
const uint16_t MAX_VELOCITY_KMH = 217;    // Maximum velocity threshold (km/h)
const bool DEBUG_ENABLED = true;           // Set to false to disable debug output

// Pretty-printer for the global target list buffer
void printTargetList(const char* label) {
  Serial.println(label);
  if (targetList.error.iSYSTargetListError != TARGET_LIST_OK) {
    Serial.print("  Error: ");
    Serial.println(targetList.error.iSYSTargetListError);
    return;
  }
  Serial.printf("  Targets: %u (Output %u)\n", targetList.nrOfTargets, targetList.outputNumber);
  for (uint16_t i = 0; i < targetList.nrOfTargets && i < MAX_TARGETS; i++) {
    Serial.printf("  #%u Sig=%.1fdB Vel=%.2fm/s Range=%.2fm Ang=%.1f°\n",
                  i + 1,
                  targetList.targets[i].signal,
                  targetList.targets[i].velocity,
                  targetList.targets[i].range,
                  targetList.targets[i].angle);
  }
}

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

  // Enable Multiple Target Filter on Output 1
  {
    iSYSResult_t fRes = radar.iSYS_setMultipleTargetFilter(ISYS_OUTPUT_1, DEVICE_ADDR, TIMEOUT_MS);
    if (fRes == ERR_OK) {
      Serial.println("MultipleTargetFilter set on Output 1");
    } else {
      Serial.print("[WARN] iSYS_setMultipleTargetFilter failed: ");
      Serial.println(fRes);
    }
  }

  // Persist all settings
  {
    iSYSResult_t sRes = radar.saveAllSettings(DEVICE_ADDR, TIMEOUT_MS);
    if (sRes == ERR_OK) {
      Serial.println("All settings saved (saveAllSettings)");
    } else {
      Serial.print("[WARN] saveAllSettings failed: ");
      Serial.println(sRes);
    }
  }

  // Start acquisition
  {
    iSYSResult_t aRes = radar.iSYS_startAcquisition(DEVICE_ADDR, TIMEOUT_MS);
    if (aRes == ERR_OK) {
      Serial.println("Acquisition started");
    } else {
      Serial.print("[WARN] iSYS_startAcquisition failed: ");
      Serial.println(aRes);
    }
  }
  
  Serial.println("=== Ready - Starting Detection ===\n");
}

void loop() {
  // Clear stale data
  while (Serial2.available()) { Serial2.read(); }

  // 16-bit list (compact)
  {
    iSYSResult_t r16 = radar.getTargetList16(&targetList, DEVICE_ADDR, TIMEOUT_MS, ISYS_OUTPUT_1);
    if (r16 == ERR_OK) {
      printTargetList("[16-bit] Target List:");
    } else {
      Serial.print("[16-bit] getTargetList16 failed: ");
      Serial.println(r16);
    }
  }

  // 32-bit list (high precision) with one retry
  {
    iSYSResult_t r32 = radar.getTargetList32(&targetList, DEVICE_ADDR, TIMEOUT_MS, ISYS_OUTPUT_1);
    if (r32 != ERR_OK) {
      r32 = radar.getTargetList32(&targetList, DEVICE_ADDR, TIMEOUT_MS, ISYS_OUTPUT_1);
    }
    if (r32 == ERR_OK) {
      printTargetList("[32-bit] Target List:");
    } else {
      Serial.print("[32-bit] getTargetList32 failed: ");
      Serial.println(r32);
    }
  }

  delay(500); // 500ms between readings
}
