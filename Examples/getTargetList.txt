/*
  iSYS4001 Radar Sensor — Get Target List (16-bit and 32-bit)
  -----------------------------------------------------------
  This example shows a complete, minimal flow to communicate with the iSYS4001 radar:

  What this sketch does:
  - Initializes UART (Serial2) to talk to the radar.
  - Enables library debug output on USB Serial for easier troubleshooting.
  - (Setup) Configures the Multiple Target Filter for Output 1, saves settings, then starts acquisition.
  - (Loop) Requests and prints the Target List twice: first with getTargetList16, then with getTargetList32.

  When to use 16-bit vs 32-bit:
  - getTargetList16: lower payload, faster, adequate precision for many applications.
  - getTargetList32: higher precision (recommended if you need fine range/velocity/angle resolution).

  Wiring (ESP32 example):
    - RX (GPIO16) ←→ Radar TX
    - TX (GPIO17) ←→ Radar RX
    - GND shared

  Notes:
    - Adjust DESTINATION_ADDRESS if your sensor uses a different address.
    - Make sure to call iSYS_startAcquisition() before requesting targets; otherwise you might not see any data.
    - After changing configuration (like filters), call saveAllSettings() to persist them.
*/

#include "iSYS4001.h"

// Radar object and configuration
iSYS4001 radar(Serial2, 115200);
constexpr uint8_t DESTINATION_ADDRESS = 0x80;
constexpr uint32_t TIMEOUT_MS = 300; // ms
static iSYSTargetList_t targetList;

// ---------------------- Helper Functions ---------------------- //
void flushSerial2() {
  while (Serial2.available()) { Serial2.read(); }
}

// Utility to print a target list already filled by getTargetList16/32
void printTargetList(const char* label) {
  Serial.println(label);
  if (targetList.error.iSYSTargetListError != TARGET_LIST_OK) {
    Serial.print("  Error: ");
    Serial.println(targetList.error.iSYSTargetListError);
    return;
  }
  Serial.printf("  Targets: %u (Out %u)\n", targetList.nrOfTargets, targetList.outputNumber);
  for (uint16_t i = 0; i < targetList.nrOfTargets && i < MAX_TARGETS; i++) {
    Serial.printf("  #%u Signal:%.2f dB Velocity:%.2f m/s Range:%.2f m Angle:%.2f deg\n",
                  i + 1,
                  targetList.targets[i].signal,
                  targetList.targets[i].velocity,
                  targetList.targets[i].range,
                  targetList.targets[i].angle);
  }
}

// ---------------------- Arduino Setup & Loop ---------------------- //
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  // Enable debug output from the iSYS4001 library to USB Serial
  radar.setDebug(Serial, true);

  // Initialize Serial2 for radar
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  flushSerial2();
  delay(100);

  // ---------------------- Configuration & Acquisition ---------------------- //
  // 1) Optionally configure Multiple Target Filter on Output 1
  //    This tells the radar to report multiple targets on the selected output.
  {
    iSYSResult_t res = radar.iSYS_setMultipleTargetFilter(ISYS_OUTPUT_1, DESTINATION_ADDRESS, TIMEOUT_MS);
    if (res == ERR_OK) {
      Serial.println("MultipleTargetFilter set on Output 1");
    } else {
      Serial.print("[WARN] iSYS_setMultipleTargetFilter failed: ");
      Serial.println(res);
    }
  }

  // 2) Save configuration to non-volatile memory (so it persists after power cycle)
  {
    iSYSResult_t res = radar.saveAllSettings(DESTINATION_ADDRESS, TIMEOUT_MS);
    if (res == ERR_OK) {
      Serial.println("Settings saved (saveAllSettings)");
    } else {
      Serial.print("[WARN] saveAllSettings failed: ");
      Serial.println(res);
    }
  }

  // 3) Start acquisition so target lists can be requested successfully
  {
    iSYSResult_t res = radar.iSYS_startAcquisition(DESTINATION_ADDRESS, TIMEOUT_MS);
    if (res == ERR_OK) {
      Serial.println("Acquisition started");
    } else {
      Serial.print("[WARN] iSYS_startAcquisition failed: ");
      Serial.println(res);
    }
  }
}

void loop() {
  flushSerial2();

  // First: getTargetList16 (compact)
  {
    iSYSResult_t res16 = radar.getTargetList16(&targetList, DESTINATION_ADDRESS, TIMEOUT_MS, ISYS_OUTPUT_1);
    if (res16 == ERR_OK) {
      printTargetList("[16-bit] Target List:");
    } else {
      Serial.print("[16-bit] getTargetList16 failed: ");
      Serial.println(res16);
    }
  }

  // Second: getTargetList32 (high precision)
  {
    iSYSResult_t res32 = radar.getTargetList32(&targetList, DESTINATION_ADDRESS, TIMEOUT_MS, ISYS_OUTPUT_1);
    if (res32 != ERR_OK) {
      // One quick retry to handle transient UART timing
      res32 = radar.getTargetList32(&targetList, DESTINATION_ADDRESS, TIMEOUT_MS, ISYS_OUTPUT_1);
    }
    if (res32 == ERR_OK) {
      printTargetList("[32-bit] Target List:");
    } else {
      Serial.print("[32-bit] getTargetList32 failed: ");
      Serial.println(res32);
    }
  }

  delay(400);
}
