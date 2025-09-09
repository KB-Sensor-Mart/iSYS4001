/*
  iSYS4001 Single-Target Filter Demo (32-bit target list)
 
  This example shows how to:
  - Initialize the iSYS4001 radar on a hardware serial port
  - Start acquisition
  - Configure OUTPUT SINGLE TARGET FILTER FUNCTIONS in setup:
      * iSYS_setOutputFilterType(...) to choose HIGHEST_SIGNAL/MEAN/MEDIAN/MIN/MAX
      (WHEN YOU NEED TO USE HIGHEST_SIGNAL, YOU DON'T NEED TO USE iSYS_setOutputSignalFilter(...))
      * iSYS_setOutputSignalFilter(...) to select OFF / VELOCITY_RADIAL / RANGE_RADIAL
    and then read back the applied settings using the corresponding getters
  - Read 32-bit target list using getTargetList32(...) in loop and print detected targets
 
  Changing to MULTIPLE TARGET mode:
  ---------------------------------
  By default, the example reads all reported targets when calling getTargetList32().
  If your sensor/application requires explicitly enabling multiple target output for a channel,
  call:
      iSYS_setMultipleTargetFilter(ISYS_OUTPUT_1, RADAR_ADDRESS, TIMEOUT_MS);
  (OutputSignalFilter controls signal component; it does NOT enable multi-target.)
 
  You can still use getTargetList16/getTargetList32 to retrieve multiple targets (up to MAX_TARGETS).
  Use the 16-bit or 32-bit variant depending on the precision/bitrate configured in your firmware.
 
  Notes:
  - Timeout recommendation: >= 100 ms (300–500 ms conservative)
  - Prefer hardware UART. The example uses ESP32 Serial2 pins GPIO16/17.
*/
 
#include <iSYS4001.h>
 
// Create radar object on a hardware serial port
// Adjust the serial instance and pins for your board
iSYS4001 radar(Serial2, 115200);
 
// Configuration constants
const uint8_t RADAR_ADDRESS = 0x80;
const uint32_t TIMEOUT_MS = 500; // increased for robustness on some links
 
// Variables to read back configured single target filter options
iSYSOutput_filter_t configuredFilterType;
iSYSFilter_signal_t configuredSignalFilter;
 
iSYSTargetList_t targetList32;
 
// Optional helpers to print readable error names
const char* resultToStr(iSYSResult_t r) {
  switch (r) {
    case ERR_OK: return "ERR_OK";
    case ERR_NULL_POINTER: return "ERR_NULL_POINTER";
    case ERR_PARAMETER_OUT_OF_RANGE: return "ERR_PARAMETER_OUT_OF_RANGE";
    case ERR_OUTPUT_OUT_OF_RANGE: return "ERR_OUTPUT_OUT_OF_RANGE";
    case ERR_TIMEOUT: return "ERR_TIMEOUT";
    case ERR_COMMAND_NO_DATA_RECEIVED: return "ERR_COMMAND_NO_DATA_RECEIVED";
    case ERR_COMMAND_NO_VALID_FRAME_FOUND: return "ERR_COMMAND_NO_VALID_FRAME_FOUND";
    case ERR_COMMAND_RX_FRAME_DAMAGED: return "ERR_COMMAND_RX_FRAME_DAMAGED";
    case ERR_COMMAND_RX_FRAME_LENGTH: return "ERR_COMMAND_RX_FRAME_LENGTH";
    case ERR_INVALID_CHECKSUM: return "ERR_INVALID_CHECKSUM";
    case ERR_COMMAND_MAX_DATA_OVERFLOW: return "ERR_COMMAND_MAX_DATA_OVERFLOW";
    case ERR_FRAME_INCOMPLETE: return "ERR_FRAME_INCOMPLETE";
    case ERR_COMMAND_FAILURE: return "ERR_COMMAND_FAILURE";
    default: return "ERR_UNKNOWN";
  }
}

const char* tlErrToStr(uint8_t e) {
  switch (e) {
    case TARGET_LIST_OK: return "TARGET_LIST_OK";
    case TARGET_LIST_FULL: return "TARGET_LIST_FULL";
    case TARGET_LIST_REFRESHED: return "TARGET_LIST_REFRESHED";
    case TARGET_LIST_ALREADY_REQUESTED: return "TARGET_LIST_ALREADY_REQUESTED";
    case TARGET_LIST_ACQUISITION_NOT_STARTED: return "TARGET_LIST_ACQUISITION_NOT_STARTED";
    default: return "TARGET_LIST_UNKNOWN";
  }
}
 
void setup() {
  Serial.begin(115200);
#if defined(ESP32)
  // ESP32 example pins for Serial2: RX=16, TX=17
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
#else
  // For other boards, initialize the chosen HardwareSerial accordingly
  Serial2.begin(115200);
#endif
 
  // Optional: route debug to Serial for verbose logs
  radar.setDebug(Serial, true);
 
  // Start acquisition
  iSYSResult_t result = radar.iSYS_startAcquisition(RADAR_ADDRESS, TIMEOUT_MS);
  if (result != ERR_OK) {
    Serial.println("[ERROR] Failed to start acquisition");
    while (true) { delay(1000); }
  }

  // Allow one sensor cycle before first reads (improves first responses)
  delay(150);
 
  // ----- OUTPUT SINGLE TARGET FILTER FUNCTIONS -----
  // Choose how a single target value is derived per output channel
  // Options: ISYS_OUTPUT_FILTER_HIGHEST_SIGNAL, ISYS_OUTPUT_FILTER_MEAN,
  //          ISYS_OUTPUT_FILTER_MEDIAN, ISYS_OUTPUT_FILTER_MIN, ISYS_OUTPUT_FILTER_MAX
  radar.iSYS_setOutputFilterType(ISYS_OUTPUT_1, ISYS_OUTPUT_FILTER_MEDIAN, RADAR_ADDRESS, TIMEOUT_MS);
 
  // Select which signal component the filter operates on (FOR HIGHEST_SIGNAL, YOU DON'T NEED TO USE iSYS_setOutputSignalFilter(...))
  // Options: ISYS_OFF, ISYS_VELOCITY_RADIAL, ISYS_RANGE_RADIAL
  radar.iSYS_setOutputSignalFilter(ISYS_OUTPUT_1, ISYS_VELOCITY_RADIAL, RADAR_ADDRESS, TIMEOUT_MS);
 
  // Read back filter configuration
  radar.iSYS_getOutputFilterType(ISYS_OUTPUT_1, &configuredFilterType, RADAR_ADDRESS, TIMEOUT_MS);
  radar.iSYS_getOutputSignalFilter(ISYS_OUTPUT_1, &configuredSignalFilter, RADAR_ADDRESS, TIMEOUT_MS);
 
  Serial.print("Configured Filter Type: ");
  switch (configuredFilterType) {
    case ISYS_OUTPUT_FILTER_HIGHEST_SIGNAL: Serial.println("HIGHEST_SIGNAL"); break;
    case ISYS_OUTPUT_FILTER_MEAN:           Serial.println("MEAN"); break;
    case ISYS_OUTPUT_FILTER_MEDIAN:         Serial.println("MEDIAN"); break;
    case ISYS_OUTPUT_FILTER_MIN:            Serial.println("MIN"); break;
    case ISYS_OUTPUT_FILTER_MAX:            Serial.println("MAX"); break;
    default:                                Serial.println("(unknown)"); break;
  }
 
  Serial.print("Configured Signal Filter: ");
  switch (configuredSignalFilter) {
    case ISYS_OFF:              Serial.println("OFF"); break;
    case ISYS_VELOCITY_RADIAL:  Serial.println("VELOCITY_RADIAL"); break;
    case ISYS_RANGE_RADIAL:     Serial.println("RANGE_RADIAL"); break;
    default:                    Serial.println("(unknown)"); break;
  }

  // Persist current configuration to EEPROM (sensor + application settings)
  result = radar.saveAllSettings(RADAR_ADDRESS, TIMEOUT_MS);
  if (result == ERR_OK) {
    Serial.println("Settings saved to EEPROM (saveAllSettings)");
  } else {
    Serial.print("[WARN] saveAllSettings failed: ");
    Serial.println(resultToStr(result));
  }
 
  // OPTIONAL: Enable multiple target output for this output channel
  // Uncomment this line if your application requires explicit multiple target filter enable.
  // radar.iSYS_setMultipleTargetFilter(ISYS_OUTPUT_1, RADAR_ADDRESS, TIMEOUT_MS);
 
  Serial.println("Setup complete. Reading 32-bit target list...");
}
 
void loop() {
  static unsigned long lastReadMs = 0;
  const unsigned long readIntervalMs = 1000; // 1 Hz polling
 
  if (millis() - lastReadMs >= readIntervalMs) {
    lastReadMs = millis();
 
    iSYSResult_t result = radar.getTargetList32(&targetList32, RADAR_ADDRESS, TIMEOUT_MS, ISYS_OUTPUT_1);
    if (result == ERR_OK && targetList32.error.iSYSTargetListError == TARGET_LIST_OK) {
      if (targetList32.nrOfTargets == 0) {
        Serial.println("No targets");
      } else {
        for (uint16_t i = 0; i < targetList32.nrOfTargets && i < MAX_TARGETS; i++) {
          Serial.print("[32-bit] Target "); Serial.print(i + 1);
          Serial.print(": R="); Serial.print(targetList32.targets[i].range, 3);
          Serial.print(" m, V="); Serial.print(targetList32.targets[i].velocity, 3);
          Serial.print(" m/s, S="); Serial.print(targetList32.targets[i].signal, 1);
          Serial.print(" dB, A="); Serial.print(targetList32.targets[i].angle, 1);
          Serial.println(" deg");
        }
      }
    } else {
      Serial.print("getTargetList32 error. API=");
      Serial.print(resultToStr(result));
      Serial.print(" ("); Serial.print(result); Serial.print(")");
      Serial.print(", TL error=");
      Serial.print(tlErrToStr(targetList32.error.iSYSTargetListError));
      Serial.print(" ("); Serial.print(targetList32.error.iSYSTargetListError); Serial.println(")");
    }
  }
 
  delay(10);
}
