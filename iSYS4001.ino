#include "iSYS4001.h"

// Create iSYS4001 radar object
iSYS4001 radar(Serial2, 115200);

// Configuration constants
const uint8_t RADAR_ADDRESS = 0x80;
const uint32_t TIMEOUT_MS = 300;

// Variables for storing retrieved values
float rangeMin, rangeMax;
float velocityMin, velocityMax;
float signalMin, signalMax;
uint8_t deviceAddress;
iSYSDirection_type_t direction;
iSYSOutput_filter_t filterType;
iSYSFilter_signal_t signalFilter;

// User configurable inputs (as integers)
int userRangeMin = 1;
int userRangeMax = 150;
int userVelocityMin = 1;
int userVelocityMax = 120;
int userSignalMin = 3;
int userSignalMax = 140;

// Helper function to get integer input via Serial
int getUserInputInt(String prompt, int defaultValue) {
    Serial.println(prompt + " (default: " + String(defaultValue) + "): ");
    while (!Serial.available()) {
        delay(10);
    }
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) {
        return defaultValue; // return default if nothing entered
    }
    return input.toInt();
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, 16, 17);
    radar.setDebug(Serial, true);

    while (!Serial) { delay(10); }

    Serial.println("iSYS4001 Radar Library Demo");
    Serial.println("==========================");

    // Optionally get user input as integers
    // userRangeMin = getUserInputInt("Enter Range Min (m)", userRangeMin);
    // userRangeMax = getUserInputInt("Enter Range Max (m)", userRangeMax);
    // userVelocityMin = getUserInputInt("Enter Velocity Min (km/h)", userVelocityMin);
    // userVelocityMax = getUserInputInt("Enter Velocity Max (km/h)", userVelocityMax);
    // userSignalMin = getUserInputInt("Enter Signal Min (dB)", userSignalMin);
    // userSignalMax = getUserInputInt("Enter Signal Max (dB)", userSignalMax);

    Serial.println("\n1. Starting radar acquisition...");
    iSYSResult_t result = radar.iSYS_startAcquisition(RADAR_ADDRESS, TIMEOUT_MS);
    if (result != ERR_OK) { while (1) { delay(1000); } }
    Serial.println("Acquisition started successfully");

    result = radar.iSYS_getDeviceAddress(&deviceAddress, RADAR_ADDRESS, TIMEOUT_MS);
    Serial.println("Device address: 0x" + String(deviceAddress, HEX));

    // Configure and read back settings (always use radar values for print)
    radar.iSYS_setOutputRangeMin(ISYS_OUTPUT_1, userRangeMin, RADAR_ADDRESS, TIMEOUT_MS);
    radar.iSYS_getOutputRangeMin(ISYS_OUTPUT_1, &rangeMin, RADAR_ADDRESS, TIMEOUT_MS);
    Serial.println("Range minimum set to " + String(rangeMin) + " m");

    radar.iSYS_setOutputRangeMax(ISYS_OUTPUT_1, userRangeMax, RADAR_ADDRESS, TIMEOUT_MS);
    radar.iSYS_getOutputRangeMax(ISYS_OUTPUT_1, &rangeMax, RADAR_ADDRESS, TIMEOUT_MS);
    Serial.println("Range maximum set to " + String(rangeMax) + " m");

    radar.iSYS_setOutputVelocityMin(ISYS_OUTPUT_1, userVelocityMin, RADAR_ADDRESS, TIMEOUT_MS);
    radar.iSYS_getOutputVelocityMin(ISYS_OUTPUT_1, &velocityMin, RADAR_ADDRESS, TIMEOUT_MS);
    Serial.println("Velocity minimum set to " + String(velocityMin) + " km/h");

    radar.iSYS_setOutputVelocityMax(ISYS_OUTPUT_1, userVelocityMax, RADAR_ADDRESS, TIMEOUT_MS);
    radar.iSYS_getOutputVelocityMax(ISYS_OUTPUT_1, &velocityMax, RADAR_ADDRESS, TIMEOUT_MS);
    Serial.println("Velocity maximum set to " + String(velocityMax) + " km/h");

    radar.iSYS_setOutputSignalMin(ISYS_OUTPUT_1, userSignalMin, RADAR_ADDRESS, TIMEOUT_MS);
    radar.iSYS_getOutputSignalMin(ISYS_OUTPUT_1, &signalMin, RADAR_ADDRESS, TIMEOUT_MS);
    Serial.println("Signal minimum set to " + String(signalMin) + " dB");

    radar.iSYS_setOutputSignalMax(ISYS_OUTPUT_1, userSignalMax, RADAR_ADDRESS, TIMEOUT_MS);
    radar.iSYS_getOutputSignalMax(ISYS_OUTPUT_1, &signalMax, RADAR_ADDRESS, TIMEOUT_MS);
    Serial.println("Signal maximum set to " + String(signalMax) + " dB");

    radar.iSYS_setOutputDirection(ISYS_OUTPUT_1, ISYS_TARGET_DIRECTION_BOTH, RADAR_ADDRESS, TIMEOUT_MS);
    radar.iSYS_getOutputDirection(ISYS_OUTPUT_1, &direction, RADAR_ADDRESS, TIMEOUT_MS);
    String dirStr = (direction == ISYS_TARGET_DIRECTION_APPROACHING) ? "APPROACHING" :
                    (direction == ISYS_TARGET_DIRECTION_RECEDING) ? "RECEDING" : "BOTH";
    Serial.println("Direction set to " + dirStr);

    radar.iSYS_setOutputFilterType(ISYS_OUTPUT_1, ISYS_OUTPUT_FILTER_MEDIAN, RADAR_ADDRESS, TIMEOUT_MS);
    radar.iSYS_getOutputFilterType(ISYS_OUTPUT_1, &filterType, RADAR_ADDRESS, TIMEOUT_MS);
    String filterStr = (filterType == ISYS_OUTPUT_FILTER_HIGHEST_SIGNAL) ? "HIGHEST_SIGNAL" :
                      (filterType == ISYS_OUTPUT_FILTER_MEAN) ? "MEAN" :
                      (filterType == ISYS_OUTPUT_FILTER_MEDIAN) ? "MEDIAN" :
                      (filterType == ISYS_OUTPUT_FILTER_MIN) ? "MIN" : "MAX";
    Serial.println("Filter type set to " + filterStr);

    radar.iSYS_setOutputSignalFilter(ISYS_OUTPUT_1, ISYS_VELOCITY_RADIAL, RADAR_ADDRESS, TIMEOUT_MS);
    radar.iSYS_getOutputSignalFilter(ISYS_OUTPUT_1, &signalFilter, RADAR_ADDRESS, TIMEOUT_MS);
    String signalStr = (signalFilter == ISYS_OFF) ? "OFF" :
                      (signalFilter == ISYS_VELOCITY_RADIAL) ? "VELOCITY_RADIAL" : "RANGE_RADIAL";
    Serial.println("Signal filter set to " + signalStr);

    radar.saveApplicationSettings(RADAR_ADDRESS, TIMEOUT_MS);
    radar.saveAllSettings(RADAR_ADDRESS, TIMEOUT_MS);

    Serial.println("Setup complete! Starting target detection...");
}

void loop() {
    static unsigned long lastTargetRead = 0;
    const unsigned long TARGET_READ_INTERVAL = 1000;

    if (millis() - lastTargetRead >= TARGET_READ_INTERVAL) {
        lastTargetRead = millis();

        iSYSTargetList_t targetList16;
        iSYSResult_t result = radar.getTargetList16(&targetList16, RADAR_ADDRESS, TIMEOUT_MS, ISYS_OUTPUT_1);
        if (result == ERR_OK && targetList16.error.iSYSTargetListError == TARGET_LIST_OK) {
            for (uint16_t i = 0; i < targetList16.nrOfTargets && i < MAX_TARGETS; i++) {
                Serial.println("16-bit Target " + String(i + 1) + ": R=" + String(targetList16.targets[i].range) +
                               " m, V=" + String(targetList16.targets[i].velocity) + " m/s, S=" +
                               String(targetList16.targets[i].signal) + " dB");
            }
        }

        iSYSTargetList_t targetList32;
        result = radar.getTargetList32(&targetList32, RADAR_ADDRESS, TIMEOUT_MS, ISYS_OUTPUT_1);
        if (result == ERR_OK && targetList32.error.iSYSTargetListError == TARGET_LIST_OK) {
            for (uint16_t i = 0; i < targetList32.nrOfTargets && i < MAX_TARGETS; i++) {
                Serial.println("32-bit Target " + String(i + 1) + ": R=" + String(targetList32.targets[i].range) +
                               " m, V=" + String(targetList32.targets[i].velocity) + " m/s, S=" +
                               String(targetList32.targets[i].signal) + " dB");
            }
        }
    }
    delay(10);
}
