## iSYS4001 Library

An easy-to-use library for the InnoSenT iSYS4001 radar sensor. It provides a high-level C++ API to start/stop acquisition, configure output filters and thresholds, read single/multiple targets, and persist settings to EEPROM.

### Table of Contents
- [Features](#features)
- [Supported platforms](#supported-platforms)
- [Installation](#installation)
- [Wiring (example: ESP32)](#wiring-example-esp32)
- [Quick start](#quick-start)
- [Examples](#examples)
- [API overview](#api-overview)
- [Return codes](#return-codes)
- [Architecture and protocol (high-level)](#architecture-and-protocol-high-level)
- [Data model](#data-model)
- [Enumerations (selected)](#enumerations-selected)
- [Configuration semantics](#configuration-semantics)
- [EEPROM operations](#eeprom-operations)
- [Timing & performance guidance](#timing--performance-guidance)
- [Troubleshooting](#troubleshooting)
- [Versioning & compatibility](#versioning--compatibility)
- [Authors](#authors)

### Features
- **Target acquisition**: Read 16-bit and 32-bit target lists with range, velocity, angle, and signal
- **Configuration**: Set/get min/max for range, velocity, signal; set direction filters; set output filter type
- **EEPROM support**: Save factory, sensor, application, or all settings
- **Device address**: Get/set device address
- **Debugging**: Optional debug output to any `Stream`

### Supported platforms
- Arduino-compatible boards with a hardware UART (e.g., ESP32 uses `Serial2` in the example)

### Installation
1. Download or clone this repository into your Arduino `libraries` folder as `iSYS4001`.
2. Or, use Sketch → Include Library → Add .ZIP Library… and select the ZIP of this repo.
3. Include the header in your sketch:

```cpp
#include <iSYS4001.h>
```

### Wiring (example: ESP32)
- Connect iSYS4001 UART to your board’s hardware serial pins (example uses `Serial2` on GPIO16/17)
- Power the sensor according to its datasheet

### Quick start
The snippet below demonstrates initialization, starting acquisition, basic configuration, and reading target lists.

```cpp
#include <iSYS4001.h>

// Create radar object on a hardware serial port
iSYS4001 radar(Serial2, 115200);

const uint8_t RADAR_ADDRESS = 0x80;
const uint32_t TIMEOUT_MS = 300;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17); // ESP32 example pins
  radar.setDebug(Serial, true);

  // Start acquisition
  if (radar.iSYS_startAcquisition(RADAR_ADDRESS, TIMEOUT_MS) != ERR_OK) {
    Serial.println("Failed to start acquisition");
    while (true) {}
  }

  // Configure limits (output 1)
  radar.iSYS_setOutputRangeMin(ISYS_OUTPUT_1, 1,   RADAR_ADDRESS, TIMEOUT_MS);
  radar.iSYS_setOutputRangeMax(ISYS_OUTPUT_1, 150, RADAR_ADDRESS, TIMEOUT_MS);
  radar.iSYS_setOutputVelocityMin(ISYS_OUTPUT_1, 1,   RADAR_ADDRESS, TIMEOUT_MS);
  radar.iSYS_setOutputVelocityMax(ISYS_OUTPUT_1, 120, RADAR_ADDRESS, TIMEOUT_MS);
  radar.iSYS_setOutputSignalMin(ISYS_OUTPUT_1, 3,   RADAR_ADDRESS, TIMEOUT_MS);
  radar.iSYS_setOutputSignalMax(ISYS_OUTPUT_1, 140, RADAR_ADDRESS, TIMEOUT_MS);
  radar.iSYS_setOutputDirection(ISYS_OUTPUT_1, ISYS_TARGET_DIRECTION_BOTH, RADAR_ADDRESS, TIMEOUT_MS);
  radar.iSYS_setOutputFilterType(ISYS_OUTPUT_1, ISYS_OUTPUT_FILTER_MEDIAN, RADAR_ADDRESS, TIMEOUT_MS);
  radar.iSYS_setOutputSignalFilter(ISYS_OUTPUT_1, ISYS_VELOCITY_RADIAL, RADAR_ADDRESS, TIMEOUT_MS);
}

void loop() {
  static unsigned long lastRead = 0;
  if (millis() - lastRead >= 1000) {
    lastRead = millis();

    iSYSTargetList_t targets;
    if (radar.getTargetList16(&targets, RADAR_ADDRESS, TIMEOUT_MS, ISYS_OUTPUT_1) == ERR_OK &&
        targets.error.iSYSTargetListError == TARGET_LIST_OK) {
      for (uint16_t i = 0; i < targets.nrOfTargets && i < MAX_TARGETS; i++) {
        Serial.print("Target "); Serial.print(i + 1);
        Serial.print(": R="); Serial.print(targets.targets[i].range);
        Serial.print(" m, V="); Serial.print(targets.targets[i].velocity);
        Serial.print(" m/s, S="); Serial.print(targets.targets[i].signal);
        Serial.println(" dB");
      }
    }
  }
}
```

### Examples
- `iSYS4001.ino`: End-to-end demo (setup, configure, read 16/32-bit targets)
- `Examples/getTargetList.txt`: Notes on reading target lists
- `Examples/multipleTarget16_32.txt`: Multiple target decoding
- `Examples/outputSignalFilter.txt`: Output signal filter usage
- `Examples/setRangeMinMax.txt`: Range min/max
- `Examples/setVelocityMinMax.txt`: Velocity min/max

### API overview
Create an instance:

```cpp
iSYS4001(HardwareSerial &serial, uint32_t baud = 115200);
```

Debug:
```cpp
iSYSResult_t setDebug(Stream &stream, bool enabled);
```

Acquisition:
```cpp
iSYSResult_t iSYS_startAcquisition(uint8_t destAddress, uint32_t timeout);
iSYSResult_t iSYS_stopAcquisition(uint8_t destAddress, uint32_t timeout);
```

Target list:
```cpp
iSYSResult_t getTargetList16(iSYSTargetList_t* list, uint8_t dest, uint32_t timeout, iSYSOutputNumber_t out=ISYS_OUTPUT_1);
iSYSResult_t getTargetList32(iSYSTargetList_t* list, uint8_t dest, uint32_t timeout, iSYSOutputNumber_t out=ISYS_OUTPUT_1);
```

Configuration (range/velocity/signal min/max):
```cpp
iSYS_setOutputRangeMin/Max(...)
iSYS_getOutputRangeMin/Max(...)
iSYS_setOutputVelocityMin/Max(...)
iSYS_getOutputVelocityMin/Max(...)
iSYS_setOutputSignalMin/Max(...)
iSYS_getOutputSignalMin/Max(...)
```

Direction and filters:
```cpp
iSYS_setOutputDirection(...)
iSYS_getOutputDirection(...)
iSYS_setOutputFilterType(...)
iSYS_getOutputFilterType(...)
iSYS_setOutputSignalFilter(...)
iSYS_getOutputSignalFilter(...)
```

Device address and EEPROM:
```cpp
iSYS_setDeviceAddress(...)
iSYS_getDeviceAddress(...)
setFactorySettings(...)
saveSensorSettings(...)
saveApplicationSettings(...)
saveAllSettings(...)
```

### Return codes
All API calls return an `iSYSResult_t`:
- `ERR_OK` on success
- Communication/frame errors: `ERR_COMMAND_NO_DATA_RECEIVED`, `ERR_INVALID_CHECKSUM`, etc.
- Parameter/timeouts: `ERR_PARAMETER_OUT_OF_RANGE`, `ERR_TIMEOUT`, etc.

### Architecture and protocol (high-level)
- Physical layer: UART, default `115200 8N1`
- Addressing: 1-byte destination address (default `0x80`)
- Framing: Commands/acknowledgements and data frames with an 8-bit Frame Check Sequence (FCS)
- Reliability: All setters expect an acknowledgement within the configured timeout

### Data model
Structs exposed by the API and their units:

```cpp
// Signal in dB, velocity in m/s, range in meters, angle in degrees
typedef struct iSYSTarget {
  float signal;
  float velocity;
  float range;
  float angle;
} iSYSTarget_t;

// Holds up to MAX_TARGETS (0x23 = 35) targets
typedef struct iSYSTargetList {
  union iSYSTargetListError_u error; // TARGET_LIST_OK, ...
  uint8_t outputNumber;              // iSYS_OUTPUT_1..3
  uint16_t nrOfTargets;              // number of valid entries in targets[]
  uint32_t clippingFlag;             // sensor-reported clipping flags
  iSYSTarget_t targets[MAX_TARGETS]; // target array
} iSYSTargetList_t;
```

Key constants and limits:
- `MAX_TARGETS = 0x23` (35 targets max per response)
- Output channels: `ISYS_OUTPUT_1`, `ISYS_OUTPUT_2`, `ISYS_OUTPUT_3`

### Enumerations (selected)
- Output filter type (`iSYSOutput_filter_t`): `HIGHEST_SIGNAL`, `MEAN`, `MEDIAN`, `MIN`, `MAX`
- Signal filter selection (`iSYSFilter_signal_t`): `OFF`, `VELOCITY_RADIAL`, `RANGE_RADIAL`
- Direction (`iSYSDirection_type_t`): `APPROACHING`, `RECEDING`, `BOTH`
- Target list acquisition state (`iSYSTargetListError_t`): `TARGET_LIST_OK`, `TARGET_LIST_FULL`, `TARGET_LIST_REFRESHED`, `TARGET_LIST_ALREADY_REQUESTED`, `TARGET_LIST_ACQUISITION_NOT_STARTED`

### Configuration semantics
- Range/velocity/signal min/max apply per output channel. Use getters to read back actual values applied by the sensor (they may be quantized internally).
- Direction filtering limits which target motions are reported on the specified output.
- `iSYS_setOutputFilterType` selects how a single output value is derived when aggregating (e.g., median filtering).
- `iSYS_setOutputSignalFilter` selects which signal component the filter operates on.

### EEPROM operations
- `setFactorySettings` restores factory defaults (not persisted unless saved afterward)
- `saveSensorSettings` persists sensor-level configuration
- `saveApplicationSettings` persists application-level configuration
- `saveAllSettings` persists both sensor and application settings

### Timing & performance guidance
- Sensor cycle time ≈ 75 ms. Use `timeout >= 100 ms`; `300 ms` is conservative.
- Poll target lists at ≤10 Hz to avoid starving the sensor’s processing pipeline.
- Use a dedicated hardware UART. Shared serial prints at high rates can introduce jitter; enable debug only during setup or troubleshooting.

### Troubleshooting
- `ERR_TIMEOUT`: Check wiring, ground reference, baud rate, and ensure acquisition is started.
- `ERR_COMMAND_NO_VALID_FRAME_FOUND` / `ERR_INVALID_CHECKSUM`: Reduce noise, verify UART settings (`115200 8N1`), shorten cables, avoid software serial.
- `TARGET_LIST_ACQUISITION_NOT_STARTED`: Call `iSYS_startAcquisition` and wait one cycle before reading targets.
- Empty lists with `TARGET_LIST_OK`: Increase thresholds (range/velocity/signal) or reposition the sensor for better SNR.

### Versioning & compatibility
- Library version: see `library.properties` (currently `1.0.0`).
- The public API (headers) is considered stable; breaking changes will bump the minor/major version.

### Authors
- Author: Ankit Sharma, Uday Singh Gangola
- Maintainer: Kuchhal Brothers


