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

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, 16, 17); // Radar on GPIO16/17
    
    // Wait for serial to be ready
    while (!Serial) {
        delay(10);
    }
    
    Serial.println("iSYS4001 Radar Library Demo");
    Serial.println("==========================");
    
    // Enable debug output
    //radar.setDebug(Serial, true);
    
    // Start acquisition
    Serial.println("\n1. Starting radar acquisition...");
    iSYSResult_t result = radar.iSYS_startAcquisition(RADAR_ADDRESS, TIMEOUT_MS);
    if (result != ERR_OK) {
        Serial.println("Failed to start acquisition. Error: " + String(result));
        Serial.println("Setup failed - stopping execution");
        while(1) { delay(1000); } // Stop execution
    }
    Serial.println("Acquisition started successfully");
    
    // Get device address
    Serial.println("\n2. Getting device address...");
    result = radar.iSYS_getDeviceAddress(&deviceAddress, RADAR_ADDRESS, TIMEOUT_MS);
    if (result != ERR_OK) {
        Serial.println("Failed to get device address. Error: " + String(result));
        Serial.println("Setup failed - stopping execution");
        while(1) { delay(1000); } // Stop execution
    }
    Serial.println("Device address: 0x" + String(deviceAddress, HEX));
    
    // Configure range settings
    Serial.println("\n3. Configuring range settings...");
    
    // Set range minimum
    result = radar.iSYS_setOutputRangeMin(ISYS_OUTPUT_1, 0, RADAR_ADDRESS, TIMEOUT_MS);
    if (result != ERR_OK) {
        Serial.println("Failed to set range minimum. Error: " + String(result));
        Serial.println("Setup failed - stopping execution");
        while(1) { delay(1000); } // Stop execution
    }
    Serial.println("Range minimum set to 0m");
    
    // Set range maximum
    result = radar.iSYS_setOutputRangeMax(ISYS_OUTPUT_1, 150, RADAR_ADDRESS, TIMEOUT_MS);
    if (result != ERR_OK) {
        Serial.println("Failed to set range maximum. Error: " + String(result));
        Serial.println("Setup failed - stopping execution");
        while(1) { delay(1000); } // Stop execution
    }
    Serial.println("Range maximum set to 150m");
    
    // Get range settings
    result = radar.iSYS_getOutputRangeMin(ISYS_OUTPUT_1, &rangeMin, RADAR_ADDRESS, TIMEOUT_MS);
    if (result != ERR_OK) {
        Serial.println("Failed to get range minimum. Error: " + String(result));
        Serial.println("Setup failed - stopping execution");
        while(1) { delay(1000); } // Stop execution
    }
    Serial.println("Current range minimum: " + String(rangeMin) + "m");
    
    result = radar.iSYS_getOutputRangeMax(ISYS_OUTPUT_1, &rangeMax, RADAR_ADDRESS, TIMEOUT_MS);
    if (result != ERR_OK) {
        Serial.println("Failed to get range maximum. Error: " + String(result));
        Serial.println("Setup failed - stopping execution");
        while(1) { delay(1000); } // Stop execution
    }
    Serial.println("Current range maximum: " + String(rangeMax) + "m");
    
    // Configure velocity settings
    Serial.println("\n4. Configuring velocity settings...");
    
    // Set velocity minimum
    result = radar.iSYS_setOutputVelocityMin(ISYS_OUTPUT_1, 0, RADAR_ADDRESS, TIMEOUT_MS);
    if (result != ERR_OK) {
        Serial.println("Failed to set velocity minimum. Error: " + String(result));
        Serial.println("Setup failed - stopping execution");
        while(1) { delay(1000); } // Stop execution
    }
    Serial.println("Velocity minimum set to 0 km/h");
    
    // Set velocity maximum
    result = radar.iSYS_setOutputVelocityMax(ISYS_OUTPUT_1, 120, RADAR_ADDRESS, TIMEOUT_MS);
    if (result != ERR_OK) {
        Serial.println("Failed to set velocity maximum. Error: " + String(result));
        Serial.println("Setup failed - stopping execution");
        while(1) { delay(1000); } // Stop execution
    }
    Serial.println("Velocity maximum set to 120 km/h");
    
    // Get velocity settings
    result = radar.iSYS_getOutputVelocityMin(ISYS_OUTPUT_1, &velocityMin, RADAR_ADDRESS, TIMEOUT_MS);
    if (result != ERR_OK) {
        Serial.println("Failed to get velocity minimum. Error: " + String(result));
        Serial.println("Setup failed - stopping execution");
        while(1) { delay(1000); } // Stop execution
    }
    Serial.println("Current velocity minimum: " + String(velocityMin) + " km/h");
    
    result = radar.iSYS_getOutputVelocityMax(ISYS_OUTPUT_1, &velocityMax, RADAR_ADDRESS, TIMEOUT_MS);
    if (result != ERR_OK) {
        Serial.println("Failed to get velocity maximum. Error: " + String(result));
        Serial.println("Setup failed - stopping execution");
        while(1) { delay(1000); } // Stop execution
    }
    Serial.println("Current velocity maximum: " + String(velocityMax) + " km/h");
    
    // Configure signal settings
    Serial.println("\n5. Configuring signal settings...");
    
    // Set signal minimum
    result = radar.iSYS_setOutputSignalMin(ISYS_OUTPUT_1, 0, RADAR_ADDRESS, TIMEOUT_MS);
    if (result != ERR_OK) {
        Serial.println("Failed to set signal minimum. Error: " + String(result));
        Serial.println("Setup failed - stopping execution");
        while(1) { delay(1000); } // Stop execution
    }
    Serial.println("Signal minimum set to 0 dB");
    
    // Set signal maximum
    result = radar.iSYS_setOutputSignalMax(ISYS_OUTPUT_1, 150, RADAR_ADDRESS, TIMEOUT_MS);
    if (result != ERR_OK) {
        Serial.println("Failed to set signal maximum. Error: " + String(result));
        Serial.println("Setup failed - stopping execution");
        while(1) { delay(1000); } // Stop execution
    }
    Serial.println("Signal maximum set to 150 dB");
    
    // Get signal settings
    result = radar.iSYS_getOutputSignalMin(ISYS_OUTPUT_1, &signalMin, RADAR_ADDRESS, TIMEOUT_MS);
    if (result != ERR_OK) {
        Serial.println("Failed to get signal minimum. Error: " + String(result));
        Serial.println("Setup failed - stopping execution");
        while(1) { delay(1000); } // Stop execution
    }
    Serial.println("Current signal minimum: " + String(signalMin) + " dB");
    
    result = radar.iSYS_getOutputSignalMax(ISYS_OUTPUT_1, &signalMax, RADAR_ADDRESS, TIMEOUT_MS);
    if (result != ERR_OK) {
        Serial.println("Failed to get signal maximum. Error: " + String(result));
        Serial.println("Setup failed - stopping execution");
        while(1) { delay(1000); } // Stop execution
    }
    Serial.println("Current signal maximum: " + String(signalMax) + " dB");
    
    // Configure direction settings
    Serial.println("\n6. Configuring direction settings...");
    
    // Set direction to both approaching and receding
    result = radar.iSYS_setOutputDirection(ISYS_OUTPUT_1, ISYS_TARGET_DIRECTION_BOTH, RADAR_ADDRESS, TIMEOUT_MS);
    if (result != ERR_OK) {
        Serial.println("Failed to set direction. Error: " + String(result));
        Serial.println("Setup failed - stopping execution");
        while(1) { delay(1000); } // Stop execution
    }
    Serial.println("Direction set to BOTH");
    
    // Get direction setting
    result = radar.iSYS_getOutputDirection(ISYS_OUTPUT_1, &direction, RADAR_ADDRESS, TIMEOUT_MS);
    if (result != ERR_OK) {
        Serial.println("Failed to get direction. Error: " + String(result));
        Serial.println("Setup failed - stopping execution");
        while(1) { delay(1000); } // Stop execution
    }
    String dirStr = (direction == ISYS_TARGET_DIRECTION_APPROACHING) ? "APPROACHING" :
                   (direction == ISYS_TARGET_DIRECTION_RECEDING) ? "RECEDING" : "BOTH";
    Serial.println("Current direction: " + dirStr);
    
    // Configure filter type settings
    Serial.println("\n7. Configuring filter type settings...");
    
    // Set filter type to median
    result = radar.iSYS_setOutputFilterType(ISYS_OUTPUT_1, ISYS_OUTPUT_FILTER_MEDIAN, RADAR_ADDRESS, TIMEOUT_MS);
    if (result != ERR_OK) {
        Serial.println("Failed to set filter type. Error: " + String(result));
        Serial.println("Setup failed - stopping execution");
        while(1) { delay(1000); } // Stop execution
    }
    Serial.println("Filter type set to MEDIAN");
    
    // Get filter type setting
    result = radar.iSYS_getOutputFilterType(ISYS_OUTPUT_1, &filterType, RADAR_ADDRESS, TIMEOUT_MS);
    if (result != ERR_OK) {
        Serial.println("Failed to get filter type. Error: " + String(result));
        Serial.println("Setup failed - stopping execution");
        while(1) { delay(1000); } // Stop execution
    }
    String filterStr = (filterType == ISYS_OUTPUT_FILTER_HIGHEST_SIGNAL) ? "HIGHEST_SIGNAL" :
                      (filterType == ISYS_OUTPUT_FILTER_MEAN) ? "MEAN" :
                      (filterType == ISYS_OUTPUT_FILTER_MEDIAN) ? "MEDIAN" :
                      (filterType == ISYS_OUTPUT_FILTER_MIN) ? "MIN" : "MAX";
    Serial.println("Current filter type: " + filterStr);
    
    // Configure signal filter settings
    Serial.println("\n8. Configuring signal filter settings...");
    
    // Set signal filter to velocity radial
    result = radar.iSYS_setOutputSignalFilter(ISYS_OUTPUT_1, ISYS_VELOCITY_RADIAL, RADAR_ADDRESS, TIMEOUT_MS);
    if (result != ERR_OK) {
        Serial.println("Failed to set signal filter. Error: " + String(result));
        Serial.println("Setup failed - stopping execution");
        while(1) { delay(1000); } // Stop execution
    }
    Serial.println("Signal filter set to VELOCITY_RADIAL");
    
    // Get signal filter setting
    result = radar.iSYS_getOutputSignalFilter(ISYS_OUTPUT_1, &signalFilter, RADAR_ADDRESS, TIMEOUT_MS);
    if (result != ERR_OK) {
        Serial.println("Failed to get signal filter. Error: " + String(result));
        Serial.println("Setup failed - stopping execution");
        while(1) { delay(1000); } // Stop execution
    }
    String signalStr = (signalFilter == ISYS_OFF) ? "OFF" :
                      (signalFilter == ISYS_VELOCITY_RADIAL) ? "VELOCITY_RADIAL" : "RANGE_RADIAL";
    Serial.println("Current signal filter: " + signalStr);
    
    // Save application settings
    Serial.println("\n9. Saving application settings...");
    result = radar.saveApplicationSettings(RADAR_ADDRESS, TIMEOUT_MS);
    if (result != ERR_OK) {
        Serial.println("Failed to save application settings. Error: " + String(result));
        Serial.println("Setup failed - stopping execution");
        while(1) { delay(1000); } // Stop execution
    }
    Serial.println("Application settings saved");
    
    // Save all settings
    Serial.println("\n10. Saving all settings...");
    result = radar.saveAllSettings(RADAR_ADDRESS, TIMEOUT_MS);
    if (result != ERR_OK) {
        Serial.println("Failed to save all settings. Error: " + String(result));
        Serial.println("Setup failed - stopping execution");
        while(1) { delay(1000); } // Stop execution
    }
    Serial.println("All settings saved");
    
    Serial.println("\n==========================");
    Serial.println("Setup complete! Starting target detection...");
    Serial.println("==========================");
}

void loop() {
    static unsigned long lastTargetRead = 0;
    const unsigned long TARGET_READ_INTERVAL = 1000; // Read targets every 1 second
    
    // Check if it's time to read targets
    if (millis() - lastTargetRead >= TARGET_READ_INTERVAL) {
        lastTargetRead = millis();
        
        // Read 16-bit target list
        Serial.println("\n--- Reading 16-bit Target List ---");
        iSYSTargetList_t targetList16;
        iSYSResult_t result = radar.getTargetList16(&targetList16, RADAR_ADDRESS, TIMEOUT_MS, ISYS_OUTPUT_1);
        
        if (result == ERR_OK && targetList16.error.iSYSTargetListError == TARGET_LIST_OK) {
            Serial.println("16-bit targets found: " + String(targetList16.nrOfTargets));
            for (uint16_t i = 0; i < targetList16.nrOfTargets && i < MAX_TARGETS; i++) {
                Serial.println("  Target " + String(i + 1) + ":");
                Serial.println("    Signal: " + String(targetList16.targets[i].signal) + " dB");
                Serial.println("    Velocity: " + String(targetList16.targets[i].velocity) + " m/s");
                Serial.println("    Range: " + String(targetList16.targets[i].range) + " m");
                Serial.println("    Angle: " + String(targetList16.targets[i].angle) + " deg");
            }
        } else {
            Serial.println("16-bit target read failed. Error: " + String(result));
        }
        
        // Read 32-bit target list
        Serial.println("\n--- Reading 32-bit Target List ---");
        iSYSTargetList_t targetList32;
        result = radar.getTargetList32(&targetList32, RADAR_ADDRESS, TIMEOUT_MS, ISYS_OUTPUT_1);
        
        if (result == ERR_OK && targetList32.error.iSYSTargetListError == TARGET_LIST_OK) {
            Serial.println("32-bit targets found: " + String(targetList32.nrOfTargets));
            for (uint16_t i = 0; i < targetList32.nrOfTargets && i < MAX_TARGETS; i++) {
                Serial.println("  Target " + String(i + 1) + ":");
                Serial.println("    Signal: " + String(targetList32.targets[i].signal) + " dB");
                Serial.println("    Velocity: " + String(targetList32.targets[i].velocity) + " m/s");
                Serial.println("    Range: " + String(targetList32.targets[i].range) + " m");
                Serial.println("    Angle: " + String(targetList32.targets[i].angle) + " deg");
            }
        } else {
            Serial.println("32-bit target read failed. Error: " + String(result));
        }
        
        Serial.println("--- End Target Reading ---");
    }
    
    // Small delay to prevent overwhelming the system
    delay(10);
}
