/*
 * iSYS4001 EEPROM Commands Example
 * 

 * This example demonstrates how to use the EEPROM command functions
 * to save settings to the iSYS4001 radar sensor's non-volatile memory.
 * 
 * Hardware Setup:
 * - Connect iSYS4001 radar sensor to Arduino via UART
 * - Default radar address is typically 0x80
 * - Ensure proper power supply and signal levels
 * 
 * Functions demonstrated:
 * - setFactorySettings() - Restore factory default settings
 * - saveSensorSettings() - Save sensor settings to EEPROM
 * - saveApplicationSettings() - Save application settings to EEPROM
 * - saveAllSettings() - Save both sensor and application settings to EEPROM
 */

#include "iSYS4001.h"

// Create iSYS4001 object using Serial1 (adjust pin numbers for your board)
// For ESP32: Serial1.begin(baud, SERIAL_8N1, RX_PIN, TX_PIN)
// For Arduino Mega: Serial1.begin(baud)
iSYS4001 radar(Serial2, 115200);

// Radar sensor address (typically 0x80, but may vary)
const uint8_t RADAR_ADDRESS = 0x80;

// Timeout for EEPROM operations (1000ms = 1 second)
const uint32_t EEPROM_TIMEOUT = 500;

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  Serial.println("iSYS4001 EEPROM Commands Example");
  Serial.println("=================================");
  
  // Initialize radar serial communication with specific pins
  // RX pin = 16, TX pin = 17
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  
  // Wait for serial to be ready
  delay(1000);
  
  Serial.println("Radar communication initialized");
  Serial.println();
}

void loop() {
  // Example 1: Set factory settings
  Serial.println("Example 1: Setting factory settings...");
  iSYSResult_t result = radar.setFactorySettings(RADAR_ADDRESS, EEPROM_TIMEOUT);
  if (result == ERR_OK) {
    Serial.println(" Factory settings restored successfully");
  } else {
    Serial.print(" Failed to set factory settings. Error code: 0x");
    Serial.println(result, HEX);
  }
  Serial.println();
  
  delay(2000);  // Wait 2 seconds between operations
  
  // Example 2: Save sensor settings
  Serial.println("Example 2: Saving sensor settings...");
  result = radar.saveSensorSettings(RADAR_ADDRESS, EEPROM_TIMEOUT);
  if (result == ERR_OK) {
    Serial.println(" Sensor settings saved successfully");
  } else {
    Serial.print(" Failed to save sensor settings. Error code: 0x");
    Serial.println(result, HEX);
  }
  Serial.println();
  
  delay(2000);
  
  // Example 3: Save application settings
  Serial.println("Example 3: Saving application settings...");
  result = radar.saveApplicationSettings(RADAR_ADDRESS, EEPROM_TIMEOUT);
  if (result == ERR_OK) {
    Serial.println(" Application settings saved successfully");
  } else {
    Serial.print(" Failed to save application settings. Error code: 0x");
    Serial.println(result, HEX);
  }
  Serial.println();
  
  delay(2000);
  
  // Example 4: Save all settings (sensor + application)
  Serial.println("Example 4: Saving all settings...");
  result = radar.saveAllSettings(RADAR_ADDRESS, EEPROM_TIMEOUT);
  if (result == ERR_OK) {
    Serial.println(" All settings saved successfully");
  } else {
    Serial.print(" Failed to save all settings. Error code: 0x");
    Serial.println(result, HEX);
  }
  Serial.println();
  
  delay(2000);
  
  // Example 5: Using the generic EEPROM command function
  Serial.println("Example 5: Using generic EEPROM command...");
  result = radar.sendEEPROMCommand(ISYS_EEPROM_SAVE_SENSOR_SETTINGS, RADAR_ADDRESS, EEPROM_TIMEOUT);
  if (result == ERR_OK) {
    Serial.println(" Generic EEPROM command executed successfully");
  } else {
    Serial.print(" Generic EEPROM command failed. Error code: 0x");
    Serial.println(result, HEX);
  }
  Serial.println();
  
  Serial.println("EEPROM command examples completed. Restarting in 10 seconds...");
  Serial.println("=============================================================");
  delay(10000);  // Wait 10 seconds before restarting
}
