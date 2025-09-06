# iSYS4001 Arduino Library


A comprehensive Arduino library for interfacing with the **InnoSenT iSYS4001 radar sensor**. This library provides an easy-to-use interface for all standard radar operations including target detection, configuration, and data processing.

## 📋 Table of Contents

- [Features](#features)
- [Installation](#installation)
- [Hardware Requirements](#hardware-requirements)
- [Quick Start](#quick-start)
- [API Reference](#api-reference)
- [Examples](#examples)
- [Configuration](#configuration)
- [Error Handling](#error-handling)
- [Contributing](#contributing)
- [License](#license)
- [Authors](#authors)

## ✨ Features

- **Complete API Coverage**: Full support for most of the iSYS4001 radar sensor functions
- **Dual Precision Support**: Both 16-bit and 32-bit target data formats
- **Comprehensive Configuration**: Range, velocity, signal, and direction settings
- **Multiple Output Support**: Configure up to 3 independent outputs
- **EEPROM Management**: Save/restore settings and factory reset
- **Debug Support**: Built-in debugging with hex frame output
- **Error Handling**: Comprehensive error codes and validation
- **Arduino Compatible**: Works with Arduino IDE and ESP32

## 📦 Installation

### Method 1: Manual Installation
1. Download the latest release from GitHub
2. Extract the ZIP file
3. Copy the `iSYS4001` folder to your Arduino `libraries` directory
4. Restart Arduino IDE

### Method 3: Git Clone
```bash
cd ~/Documents/Arduino/libraries
git clone https://github.com/yourusername/iSYS4001.git
```

## 🔌 Hardware Requirements

### Supported Platforms
- **ESP32** (recommended)

### Wiring (ESP32 Example)
```
ESP32          iSYS4001 Radar
------         --------------
GPIO16 (RX) ←→ TX
GPIO17 (TX) ←→ RX
GND          ←→ GND
3.3V/5V      ←→ VCC
```

### Power Requirements
- **Voltage**: 3.3V - 5V
- **Current**: ~100mA typical
- **Communication**: UART (115200 baud, 8N1)

## 🚀 Quick Start

```cpp
#include "iSYS4001.h"

// Create radar object
iSYS4001 radar(Serial2, 115200);

// Configuration
const uint8_t RADAR_ADDRESS = 0x80;
const uint32_t TIMEOUT_MS = 300;  //should be minimum 100ms

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, 16, 17);
    
    // Start acquisition
    radar.iSYS_startAcquisition(RADAR_ADDRESS, TIMEOUT_MS);
}

void loop() {
    // Get target list
    iSYSTargetList_t targets;
    iSYSResult_t result = radar.getTargetList32(&targets, RADAR_ADDRESS, TIMEOUT_MS);
    
    if (result == ERR_OK && targets.error.iSYSTargetListError == TARGET_LIST_OK) {
        Serial.println("Targets found: " + String(targets.nrOfTargets));
        
        for (uint16_t i = 0; i < targets.nrOfTargets; i++) {
            Serial.println("Target " + String(i + 1) + ":");
            Serial.println("  Range: " + String(targets.targets[i].range) + " m");
            Serial.println("  Velocity: " + String(targets.targets[i].velocity) + " m/s");
            Serial.println("  Signal: " + String(targets.targets[i].signal) + " dB");
            Serial.println("  Angle: " + String(targets.targets[i].angle) + " deg");
        }
    }
    
    delay(1000);
}
```

## 📚 API Reference

### Core Functions

#### Target Detection
```cpp
// Get target list (16-bit precision)
iSYSResult_t getTargetList16(iSYSTargetList_t *pTargetList, uint8_t destAddress, uint32_t timeout, iSYSOutputNumber_t outputnumber = ISYS_OUTPUT_1);

// Get target list (32-bit precision)
iSYSResult_t getTargetList32(iSYSTargetList_t *pTargetList, uint8_t destAddress, uint32_t timeout, iSYSOutputNumber_t outputnumber = ISYS_OUTPUT_1);
```

#### Range Configuration
```cpp
// Set range limits (in meters)
iSYSResult_t iSYS_setOutputRangeMin(iSYSOutputNumber_t outputnumber, uint16_t range, uint8_t destAddress, uint32_t timeout);
iSYSResult_t iSYS_setOutputRangeMax(iSYSOutputNumber_t outputnumber, uint16_t range, uint8_t destAddress, uint32_t timeout);

// Get range limits
iSYSResult_t iSYS_getOutputRangeMin(iSYSOutputNumber_t outputnumber, float *range, uint8_t destAddress, uint32_t timeout);
iSYSResult_t iSYS_getOutputRangeMax(iSYSOutputNumber_t outputnumber, float *range, uint8_t destAddress, uint32_t timeout);
```

#### Velocity Configuration
```cpp
// Set velocity limits (in km/h)
iSYSResult_t iSYS_setOutputVelocityMin(iSYSOutputNumber_t outputnumber, uint16_t velocity, uint8_t destAddress, uint32_t timeout);
iSYSResult_t iSYS_setOutputVelocityMax(iSYSOutputNumber_t outputnumber, uint16_t velocity, uint8_t destAddress, uint32_t timeout);

// Get velocity limits
iSYSResult_t iSYS_getOutputVelocityMin(iSYSOutputNumber_t outputnumber, float *velocity, uint8_t destAddress, uint32_t timeout);
iSYSResult_t iSYS_getOutputVelocityMax(iSYSOutputNumber_t outputnumber, float *velocity, uint8_t destAddress, uint32_t timeout);
```

#### Signal Configuration
```cpp
// Set signal limits (in dB)
iSYSResult_t iSYS_setOutputSignalMin(iSYSOutputNumber_t outputnumber, uint16_t signal, uint8_t destAddress, uint32_t timeout);
iSYSResult_t iSYS_setOutputSignalMax(iSYSOutputNumber_t outputnumber, uint16_t signal, uint8_t destAddress, uint32_t timeout);

// Get signal limits
iSYSResult_t iSYS_getOutputSignalMin(iSYSOutputNumber_t outputnumber, float *signal, uint8_t destAddress, uint32_t timeout);
iSYSResult_t iSYS_getOutputSignalMax(iSYSOutputNumber_t outputnumber, float *signal, uint8_t destAddress, uint32_t timeout);
```

#### Direction Control
```cpp
// Set target direction
iSYSResult_t iSYS_setOutputDirection(iSYSOutputNumber_t outputnumber, iSYSDirection_type_t direction, uint8_t destAddress, uint32_t timeout);

// Get target direction
iSYSResult_t iSYS_getOutputDirection(iSYSOutputNumber_t outputnumber, iSYSDirection_type_t *direction, uint8_t destAddress, uint32_t timeout);
```

#### Acquisition Control
```cpp
// Start/stop radar acquisition
iSYSResult_t iSYS_startAcquisition(uint8_t destAddress, uint32_t timeout);
iSYSResult_t iSYS_stopAcquisition(uint8_t destAddress, uint32_t timeout);
```

#### EEPROM Management
```cpp
// Save settings
iSYSResult_t saveApplicationSettings(uint8_t destAddress, uint32_t timeout);
iSYSResult_t saveSensorSettings(uint8_t destAddress, uint32_t timeout);
iSYSResult_t saveAllSettings(uint8_t destAddress, uint32_t timeout);

// Factory reset
iSYSResult_t setFactorySettings(uint8_t destAddress, uint32_t timeout);
```

#### Device Management
```cpp
// Set/get device address
iSYSResult_t iSYS_setDeviceAddress(uint8_t deviceaddress, uint8_t destAddress, uint32_t timeout);
iSYSResult_t iSYS_getDeviceAddress(uint8_t *deviceaddress, uint8_t destAddress, uint32_t timeout);
```

#### Debug Support
```cpp
// Enable/disable debug output
iSYSResult_t setDebug(Stream &stream, bool enabled);
```

### Data Structures

#### Target Information
```cpp
typedef struct iSYSTarget {
    float signal;   // Signal strength in dB
    float velocity; // Velocity in m/s
    float range;    // Range in meters
    float angle;    // Angle in degrees
} iSYSTarget_t;
```

#### Target List
```cpp
typedef struct iSYSTargetList {
    union iSYSTargetListError_u error;
    uint8_t outputNumber;
    uint16_t nrOfTargets;
    uint32_t clippingFlag;
    iSYSTarget_t targets[MAX_TARGETS]; // Up to 35 targets
} iSYSTargetList_t;
```

### Enumerations

#### Output Numbers
```cpp
typedef enum iSYSOutputNumber {
    ISYS_OUTPUT_1 = 1,
    ISYS_OUTPUT_2,
    ISYS_OUTPUT_3
} iSYSOutputNumber_t;
```

#### Direction Types
```cpp
typedef enum iSYSDirection_type {
    ISYS_TARGET_DIRECTION_APPROACHING = 1,
    ISYS_TARGET_DIRECTION_RECEDING = 2,
    ISYS_TARGET_DIRECTION_BOTH = 3
} iSYSDirection_type_t;
```

#### Filter Types
```cpp
typedef enum iSYSOutput_filter {
    ISYS_OUTPUT_FILTER_HIGHEST_SIGNAL = 0,
    ISYS_OUTPUT_FILTER_MEAN,
    ISYS_OUTPUT_FILTER_MEDIAN,
    ISYS_OUTPUT_FILTER_MIN,
    ISYS_OUTPUT_FILTER_MAX
} iSYSOutput_filter_t;
```

## 💡 Examples

The library includes several example sketches in the `Examples` folder:

### Basic Target Detection
```cpp
// See Examples/getTargetList.txt
// Minimal example for reading target data
```

### Range Configuration
```cpp
// See Examples/setRangeMinMax.txt
// Example for setting range limits
```

### Velocity Configuration
```cpp
// See Examples/setVelocityMinMax.txt
// Example for setting velocity limits
```

### Signal Filtering
```cpp
// See Examples/outputSignalFilter.txt
// Example for signal filtering configuration
```

### Multiple Target Handling
```cpp
// See Examples/multipleTarget16_32.txt
// Example comparing 16-bit vs 32-bit precision
```

## ⚙️ Configuration

### Timing Considerations
- **iSYS-4001 cycle time**: ~75ms
- **Recommended timeout**: ≥100ms (300ms used in examples)

### Parameter Ranges
- **Range**: 0-150 meters
- **Velocity**: 0-250 km/h
- **Signal**: 0-250 dB
- **Max Targets**: 35 per output

### Default Settings
- **Baud Rate**: 115200
- **Device Address**: 0x80
- **Output**: ISYS_OUTPUT_1
- **Direction**: ISYS_TARGET_DIRECTION_BOTH

## 🚨 Error Handling

The library provides comprehensive error codes:

```cpp
typedef enum {
    ERR_OK = 0,                           // Success
    ERR_NULL_POINTER = 1,                 // Null pointer passed
    ERR_PARAMETER_OUT_OF_RANGE = 2,       // Parameter out of valid range
    ERR_OUTPUT_OUT_OF_RANGE = 3,          // Invalid output number
    ERR_TIMEOUT = 4,                      // Operation timeout
    ERR_COMMAND_NO_DATA_RECEIVED = 5,     // No response from radar
    ERR_COMMAND_NO_VALID_FRAME_FOUND = 6, // Invalid frame format
    ERR_COMMAND_RX_FRAME_DAMAGED = 7,     // Corrupted frame received
    ERR_COMMAND_RX_FRAME_LENGTH = 8,      // Incorrect frame length
    ERR_INVALID_CHECKSUM = 9,             // Checksum mismatch
    ERR_COMMAND_MAX_DATA_OVERFLOW = 10,   // Buffer overflow
    ERR_FRAME_INCOMPLETE = 11,            // Incomplete frame
    ERR_COMMAND_FAILURE = 12              // General command failure
} iSYSResult_t;
```

### Error Handling Best Practices
```cpp
iSYSResult_t result = radar.getTargetList32(&targets, RADAR_ADDRESS, TIMEOUT_MS);

if (result != ERR_OK) {
    Serial.println("Error: " + String(result));
    // Handle error appropriately
    return;
}

if (targets.error.iSYSTargetListError != TARGET_LIST_OK) {
    Serial.println("Target list error: " + String(targets.error.iSYSTargetListError));
    // Handle target list error
    return;
}

// Process targets...
```

## 🔧 Debug Support

Enable debug output to troubleshoot communication issues:

```cpp
// Enable debug output to Serial
radar.setDebug(Serial, true);

// Disable debug output
radar.setDebug(Serial, false);
```

Debug output includes:
- Hex frame dumps of all communication
- Command/response timing
- Error details

## 🤝 Contributing

We welcome contributions! Please follow these steps:

1. **Fork** the repository
2. **Create** a feature branch (`git checkout -b feature/amazing-feature`)
3. **Commit** your changes (`git commit -m 'Add amazing feature'`)
4. **Push** to the branch (`git push origin feature/amazing-feature`)
5. **Open** a Pull Request

### Development Guidelines
- Follow Arduino coding standards
- Add comprehensive comments
- Include example code for new features
- Update documentation
- Test on multiple platforms

## 👥 Authors

- **Ankit Sharma** - *Initial development*
- **Uday Singh Gangola** - *Initial development*
- **Kuchhal Brothers** - *Maintainer*

## 🙏 Acknowledgments

- InnoSenT GmbH for the iSYS4001 radar sensor

---

**Made with ❤️ for the Arduino and radar sensing community**
