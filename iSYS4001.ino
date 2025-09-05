/*
  ESP32 UART2 Multi-Command Test for iSYS Radar
  ---------------------------------------------
  - Sends multiple command frames to the radar via UART2 (pins RX=16, TX=17).
  - Prints the transmitted bytes (TX) and received response (RX) in HEX format.
  - Sends each command once, then loops again.
*/

#include <Arduino.h>

// Use UART2 (pins 16 = RX, 17 = TX on ESP32)
HardwareSerial RadarSerial(2);

// Define multiple commands
uint8_t cmd1[] = {0x68, 0x07, 0x07, 0x68, 0x80, 0x01, 0xD3, 0x00, 0x17, 0x00, 0x64, 0xCF, 0x16};
uint8_t cmd2[] = {0x68, 0x05, 0x05, 0x68, 0x80, 0x01, 0xD2, 0x00, 0x17, 0x6A, 0x16};
uint8_t cmd3[] = {0x68, 0x04, 0x04, 0x68, 0x80, 0x01, 0xDF, 0x04, 0x64, 0x16}; // SAVE ALL

// Array of pointers to commands
uint8_t* commands[] = {cmd1, cmd2, cmd3};
// Array with lengths of each command
size_t cmdLengths[] = {sizeof(cmd1), sizeof(cmd2), sizeof(cmd3)};

// Number of commands
const size_t numCommands = sizeof(commands) / sizeof(commands[0]);

void sendCommand(uint8_t* cmd, size_t len) {
  Serial.print("TX: ");
  for (size_t i = 0; i < len; i++) {
    if (cmd[i] < 0x10) Serial.print("0");
    Serial.print(cmd[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  RadarSerial.write(cmd, len);
  RadarSerial.flush();

  delay(50); // small delay after sending
}

void readResponse() {
  if (RadarSerial.available()) {
    Serial.print("RX: ");
    while (RadarSerial.available()) {
      uint8_t b = RadarSerial.read();
      if (b < 0x10) Serial.print("0");
      Serial.print(b, HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void setup() {
  Serial.begin(115200);
  RadarSerial.begin(115200, SERIAL_8N1, 16, 17);
  Serial.println("Starting multi-command UART test...");
}

void loop() {
  for (size_t i = 0; i < numCommands; i++) {
    sendCommand(commands[i], cmdLengths[i]);

    // Wait and try to read response
    delay(100);
    readResponse();
  }

  Serial.println("All commands sent. Looping again...");
  delay(1000); // wait before repeating
}
