/*
  iSYS4001 Radar Sensor - Set Range Min/Max
  ---------------------------------------------------
    This program is used to configure the minimum and maximum detection range of the iSYS4001 radar.
    Before setting these range values, the user must first call stopAcquisition(), followed by setRangeBound().

Note:
    If the radar is configured with range bounds of 0–50 meters, any values specified for minimum or maximum range beyond 50 will be capped at 50.
    To configure RANGE values greater than 50 m, ensure the radar's range bound is set to 0–150 m.

  What setup() does (in order):
  1) Initializes USB Serial and the radar UART (HardwareSerial).
  2) Enables library debug to USB Serial for visibility.
  3) Stops acquisition (safe state before changing configuration).
  4) Sets the sensor Range Bound (0–50 m or 0–150 m).
  5) Reads back and prints the configured Range Bound to confirm.
  6) Sets Output 1 minimum and maximum detection range (in meters).
  7) Reads back and prints Output 1 min/max range to confirm.
  8) Saves all settings to non-volatile memory (persist across power cycles).
  9) Starts acquisition so target lists can be requested in loop().

  What loop() does:
  - Requests the 16-bit Target List using getTargetList16() and prints any
    detected targets with signal, velocity, range, and angle.

  Notes and Best Practices:
  - DESTINATION_ADDRESS is typically 0x80; change if your device differs.
  - TIMEOUT_MS >= 100 is recommended; 300–500 ms offers extra margin.
  - After changing configuration (range bound, min/max, etc.), saving with
    saveAllSettings() ensures persistence across reboots.
  - If you change UART pins or board, update the Serial2.begin(...) call.
*/

#include "iSYS4001.h"

// ---------------------- Configuration ----------------------
// Adjust these for your board and application
iSYS4001 radar(Serial2, 115200);
constexpr uint8_t DESTINATION_ADDRESS = 0x80;
constexpr uint32_t TIMEOUT_MS = 300; // ms

// Range Bound option: choose one of ISYS_RANGE_0_TO_50 or ISYS_RANGE_0_TO_150
constexpr iSYSRangeBound_t RANGE_BOUND = ISYS_RANGE_0_TO_50;

// Output 1 minimum/maximum detection range (meters)
constexpr uint16_t OUTPUT1_MIN_RANGE_M = 2;   // set as needed
constexpr uint16_t OUTPUT1_MAX_RANGE_M = 50; // set as needed

// Shared buffer for reading target lists
static iSYSTargetList_t gTargetList;

// ---------------------- Helpers ----------------------
static void flushRadarUart()
{
    while (Serial2.available())
    {
        Serial2.read();
    }
}

static const char *resultToStr(iSYSResult_t r)
{
    switch (r)
    {
    case ERR_OK:
        return "ERR_OK";
    case ERR_NULL_POINTER:
        return "ERR_NULL_POINTER";
    case ERR_PARAMETER_OUT_OF_RANGE:
        return "ERR_PARAMETER_OUT_OF_RANGE";
    case ERR_OUTPUT_OUT_OF_RANGE:
        return "ERR_OUTPUT_OUT_OF_RANGE";
    case ERR_TIMEOUT:
        return "ERR_TIMEOUT";
    case ERR_COMMAND_NO_DATA_RECEIVED:
        return "ERR_COMMAND_NO_DATA_RECEIVED";
    case ERR_COMMAND_NO_VALID_FRAME_FOUND:
        return "ERR_COMMAND_NO_VALID_FRAME_FOUND";
    case ERR_COMMAND_RX_FRAME_DAMAGED:
        return "ERR_COMMAND_RX_FRAME_DAMAGED";
    case ERR_COMMAND_RX_FRAME_LENGTH:
        return "ERR_COMMAND_RX_FRAME_LENGTH";
    case ERR_INVALID_CHECKSUM:
        return "ERR_INVALID_CHECKSUM";
    case ERR_COMMAND_MAX_DATA_OVERFLOW:
        return "ERR_COMMAND_MAX_DATA_OVERFLOW";
    case ERR_FRAME_INCOMPLETE:
        return "ERR_FRAME_INCOMPLETE";
    case ERR_COMMAND_FAILURE:
        return "ERR_COMMAND_FAILURE";
    default:
        return "ERR_UNKNOWN";
    }
}

static void printTargetList(const char *label)
{
    Serial.println(label);
    if (gTargetList.error.iSYSTargetListError != TARGET_LIST_OK)
    {
        Serial.print("  TargetList error: ");
        Serial.println(gTargetList.error.iSYSTargetListError);
        return;
    }
    Serial.printf("  Targets: %u (Output %u)\n", gTargetList.nrOfTargets, gTargetList.outputNumber);
    for (uint16_t i = 0; i < gTargetList.nrOfTargets && i < MAX_TARGETS; i++)
    {
        Serial.printf("  #%u Sig=%.1f dB Vel=%.2f m/s Range=%.2f m Angle=%.1f°\n",
                      i + 1,
                      gTargetList.targets[i].signal,
                      gTargetList.targets[i].velocity,
                      gTargetList.targets[i].range,
                      gTargetList.targets[i].angle);
    }
}

// ---------------------- Arduino Setup & Loop ----------------------
void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }

    // Initialize the radar UART. Update pins for your board if needed.
#if defined(ESP32)
    // ESP32 example pins for Serial2: RX=16, TX=17
    Serial2.begin(115200, SERIAL_8N1, 16, 17);
#else
    Serial2.begin(115200);
#endif

    // Route library debug to USB Serial for visibility
    radar.setDebug(Serial, true);

    // Clear any stale bytes before sending commands
    flushRadarUart();
    delay(100);

    Serial.println("=== iSYS4001 setup: configuring device ===");

    /*
     * 1) Stop acquisition before changing configuration.
     *    You must call iSYS_stopAcquisition() before using iSYS_setRangeBound().
     *    Without this step, the range limit will not be updated.
     */
    {
        iSYSResult_t r = radar.iSYS_stopAcquisition(DESTINATION_ADDRESS, TIMEOUT_MS);
        Serial.print("Stop acquisition: ");
        Serial.println(resultToStr(r));
    }

    // 2) Set Range Bound (overall sensor distance capability)
    {
        iSYSResult_t r = radar.iSYS_setRangeBound(RANGE_BOUND, DESTINATION_ADDRESS, TIMEOUT_MS);
        Serial.print("Set RangeBound: ");
        Serial.println(resultToStr(r));
    }

    // 3) Read back Range Bound to confirm
    {
        iSYSRangeBound_t rb = ISYS_RANGE_0_TO_50;
        iSYSResult_t r = radar.iSYS_getRangeBound(&rb, DESTINATION_ADDRESS, TIMEOUT_MS);
        Serial.print("Get RangeBound: ");
        Serial.println(resultToStr(r));
        if (r == ERR_OK)
        {
            Serial.print("  Current RangeBound: ");
            Serial.println(rb == ISYS_RANGE_0_TO_50 ? "0 TO 50 m" : "0 TO 150 m");
        }
    }

    // 4) Configure Output 1 minimum and maximum detection range (meters)
    {
        iSYSResult_t rmin = radar.iSYS_setOutputRangeMin(ISYS_OUTPUT_1, OUTPUT1_MIN_RANGE_M, DESTINATION_ADDRESS, TIMEOUT_MS);
        Serial.print("Set Output1 Min Range: ");
        Serial.println(resultToStr(rmin));

        iSYSResult_t rmax = radar.iSYS_setOutputRangeMax(ISYS_OUTPUT_1, OUTPUT1_MAX_RANGE_M, DESTINATION_ADDRESS, TIMEOUT_MS);
        Serial.print("Set Output1 Max Range: ");
        Serial.println(resultToStr(rmax));
    }

    // 5) Read back Output 1 min/max detection range
    {
        float readMin = 0.0f, readMax = 0.0f;
        iSYSResult_t gmin = radar.iSYS_getOutputRangeMin(ISYS_OUTPUT_1, &readMin, DESTINATION_ADDRESS, TIMEOUT_MS);
        iSYSResult_t gmax = radar.iSYS_getOutputRangeMax(ISYS_OUTPUT_1, &readMax, DESTINATION_ADDRESS, TIMEOUT_MS);
        Serial.print("Get Output1 Min Range: ");
        Serial.println(resultToStr(gmin));
        Serial.print("Get Output1 Max Range: ");
        Serial.println(resultToStr(gmax));
        if (gmin == ERR_OK || gmax == ERR_OK)
        {
            Serial.printf("  Output1 Range Window: %.2f m .. %.2f m\n", readMin, readMax);
        }
    }

    // 6) Persist configuration (sensor + application settings)
    {
        iSYSResult_t r = radar.saveAllSettings(DESTINATION_ADDRESS, TIMEOUT_MS);
        Serial.print("Save All Settings: ");
        Serial.println(resultToStr(r));
    }

    // 7) Start acquisition so we can request target lists in loop()
    {
        iSYSResult_t r = radar.iSYS_startAcquisition(DESTINATION_ADDRESS, TIMEOUT_MS);
        Serial.print("Start acquisition: ");
        Serial.println(resultToStr(r));
    }

    Serial.println("=== setup complete; entering loop() ===");
}

void loop()
{
    // Keep the UART clean between request/response cycles
    flushRadarUart();

    // Request compact 16-bit target list on Output 1
    iSYSResult_t res16 = radar.getTargetList16(&gTargetList, DESTINATION_ADDRESS, TIMEOUT_MS, ISYS_OUTPUT_1);
    if (res16 == ERR_OK)
    {
        printTargetList("[16-bit] Target List:");
    }
    else
    {
        Serial.print("getTargetList16 failed: ");
        Serial.println(resultToStr(res16));
    }

    // Polling interval — adjust for your application and processing needs
    delay(500);
}
