#ifndef ISYS4001_H
#define ISYS4001_H

#include <Arduino.h>

// Define sint16_t as a 16-bit signed integer for pointer compatibility
#if !defined(sint16_t)
typedef int16_t sint16_t;
#endif

// Define MAX_TARGETS constant
#define MAX_TARGETS (0x23)

/**
 * @brief Result/error codes for iSYS radar sensor API calls
 *
 * This enum provides comprehensive error codes for all iSYS radar sensor operations.
 * Error codes are organized into logical groups for better maintainability.
 *
 * @note Based on iSYS protocol specification with timeout recommendations:
 *       - iSYS-400x: 75ms cycle time, 100ms minimum timeout
 */
typedef enum
{
    // ===== SUCCESS =====
    ERR_OK = 0,

    // ===== GENERIC ERRORS =====
    ERR_NULL_POINTER = 1,
    ERR_PARAMETER_OUT_OF_RANGE = 2,
    ERR_OUTPUT_OUT_OF_RANGE = 3,
    ERR_TIMEOUT = 4,

    // ===== FRAME/COMMUNICATION ERRORS =====
    ERR_COMMAND_NO_DATA_RECEIVED = 5,
    ERR_COMMAND_NO_VALID_FRAME_FOUND = 6,
    ERR_COMMAND_RX_FRAME_DAMAGED = 7,
    ERR_COMMAND_RX_FRAME_LENGTH = 8,
    ERR_INVALID_CHECKSUM = 9,
    ERR_COMMAND_MAX_DATA_OVERFLOW = 10,
    ERR_FRAME_INCOMPLETE = 11,

    // ===== COMMAND ERRORS =====
    ERR_COMMAND_FAILURE = 12

} iSYSResult_t;

typedef enum iSYSTargetListError
{
    TARGET_LIST_OK = 0,
    TARGET_LIST_FULL,
    TARGET_LIST_REFRESHED,
    TARGET_LIST_ALREADY_REQUESTED,
    TARGET_LIST_ACQUISITION_NOT_STARTED
} iSYSTargetListError_t;

typedef enum iSYSOutput_filter
{
    ISYS_OUTPUT_FILTER_HIGHEST_SIGNAL = 0,
    ISYS_OUTPUT_FILTER_MEAN,
    ISYS_OUTPUT_FILTER_MEDIAN,
    ISYS_OUTPUT_FILTER_MIN,
    ISYS_OUTPUT_FILTER_MAX
} iSYSOutput_filter_t;

typedef enum iSYSFilter_signal
{
    ISYS_OFF = 0,
    ISYS_VELOCITY_RADIAL,
    ISYS_RANGE_RADIAL
} iSYSFilter_signal_t;

typedef enum iSYSOutputNumber
{
    ISYS_OUTPUT_1 = 1,
    ISYS_OUTPUT_2,
    ISYS_OUTPUT_3
} iSYSOutputNumber_t;

typedef enum iSYSEEPROMSubFunction
{
    ISYS_EEPROM_SET_FACTORY_SETTINGS = 0x01,      // Restore factory default settings
    ISYS_EEPROM_SAVE_SENSOR_SETTINGS = 0x02,      // Save sensor settings to EEPROM
    ISYS_EEPROM_SAVE_APPLICATION_SETTINGS = 0x03, // Save application settings to EEPROM
    ISYS_EEPROM_SAVE_ALL_SETTINGS = 0x04          // Save both sensor and application settings to EEPROM
} iSYSEEPROMSubFunction_t;

typedef struct iSYSTarget
{
    float signal;   /* signal indicator dB */
    float velocity; /* velocity in m/s */
    float range;    /* range in m */
    float angle;    /* angle of detected object [°/Deg] */
} iSYSTarget_t;

union iSYSTargetListError_u
{
    uint8_t iSYSTargetListError;
    uint32_t dummy;
};

typedef enum iSYSDirection_type
{
    ISYS_TARGET_DIRECTION_APPROACHING = 1,
    ISYS_TARGET_DIRECTION_RECEDING = 2,
    ISYS_TARGET_DIRECTION_BOTH = ISYS_TARGET_DIRECTION_APPROACHING | ISYS_TARGET_DIRECTION_RECEDING
} iSYSDirection_type_t;

typedef struct iSYSTargetList
{
    union iSYSTargetListError_u error;
    uint8_t outputNumber;
    uint16_t nrOfTargets;
    uint32_t clippingFlag;
    iSYSTarget_t targets[MAX_TARGETS];
} iSYSTargetList_t;


class iSYS4001
{

public:
    iSYS4001(HardwareSerial &serial, uint32_t baud = 115200);

    // Debug configuration
    void setDebugEnabled(bool enabled);
    void setDebugStream(Stream &stream);
    void setDebug(Stream &stream, bool enabled);

    /***************************************************************
     *  GET TARGET LIST FUNCTION
     ***************************************************************/
    iSYSResult_t getTargetList(iSYSTargetList_t *pTargetList, uint8_t destAddress, uint32_t timeout, iSYSOutputNumber_t outputnumber = ISYS_OUTPUT_1);
    iSYSResult_t getTargetList16(iSYSTargetList_t *pTargetList, uint8_t destAddress, uint32_t timeout, iSYSOutputNumber_t outputnumber = ISYS_OUTPUT_1);
    iSYSResult_t getTargetList32(iSYSTargetList_t *pTargetList, uint8_t destAddress, uint32_t timeout, iSYSOutputNumber_t outputnumber = ISYS_OUTPUT_1);

    /***************************************************************
     *  SET/GET RANGE MIN/MAX FUNCTIONS
     ***************************************************************/

    iSYSResult_t iSYS_setOutputRangeMin(iSYSOutputNumber_t outputnumber, uint16_t range, uint8_t destAddress, uint32_t timeout);
    iSYSResult_t iSYS_setOutputRangeMax(iSYSOutputNumber_t outputnumber, uint16_t range, uint8_t destAddress, uint32_t timeout);
    iSYSResult_t iSYS_getOutputRangeMin(iSYSOutputNumber_t outputnumber, float *range, uint8_t destAddress, uint32_t timeout);
    iSYSResult_t iSYS_getOutputRangeMax(iSYSOutputNumber_t outputnumber, float *range, uint8_t destAddress, uint32_t timeout);

    /***************************************************************
     *  SET/GET VELOCITY MIN/MAX FUNCTIONS
     ***************************************************************/

    iSYSResult_t iSYS_setOutputVelocityMin(iSYSOutputNumber_t outputnumber, uint16_t velocity, uint8_t destAddress, uint32_t timeout);
    iSYSResult_t iSYS_setOutputVelocityMax(iSYSOutputNumber_t outputnumber, uint16_t velocity, uint8_t destAddress, uint32_t timeout);
    iSYSResult_t iSYS_getOutputVelocityMin(iSYSOutputNumber_t outputnumber, float *velocity, uint8_t destAddress, uint32_t timeout);
    iSYSResult_t iSYS_getOutputVelocityMax(iSYSOutputNumber_t outputnumber, float *velocity, uint8_t destAddress, uint32_t timeout);

    /***************************************************************
     *  SET/GET SIGNAL MIN/MAX FUNCTIONS
     ***************************************************************/

    iSYSResult_t iSYS_setOutputSignalMin(iSYSOutputNumber_t outputnumber, uint16_t signal, uint8_t destAddress, uint32_t timeout);
    iSYSResult_t iSYS_setOutputSignalMax(iSYSOutputNumber_t outputnumber, uint16_t signal, uint8_t destAddress, uint32_t timeout);
    iSYSResult_t iSYS_getOutputSignalMin(iSYSOutputNumber_t outputnumber, float *signal, uint8_t destAddress, uint32_t timeout);
    iSYSResult_t iSYS_getOutputSignalMax(iSYSOutputNumber_t outputnumber, float *signal, uint8_t destAddress, uint32_t timeout);

    /***************************************************************
     *  SET/GET VELOCITY DIRECTION FUNCTION
     ***************************************************************/

    iSYSResult_t iSYS_setOutputDirection(iSYSOutputNumber_t outputnumber, iSYSDirection_type_t direction, uint8_t destAddress, uint32_t timeout);
    iSYSResult_t iSYS_getOutputDirection(iSYSOutputNumber_t outputnumber, iSYSDirection_type_t *direction, uint8_t destAddress, uint32_t timeout);

    /***************************************************************
     *  EEPROM COMMAND FUNCTIONS
     ***************************************************************/

    iSYSResult_t setFactorySettings(uint8_t destAddress, uint32_t timeout);
    iSYSResult_t saveSensorSettings(uint8_t destAddress, uint32_t timeout);
    iSYSResult_t saveApplicationSettings(uint8_t destAddress, uint32_t timeout);
    iSYSResult_t saveAllSettings(uint8_t destAddress, uint32_t timeout);

    /***************************************************************
     *  DEVICE ADDRESS FUNCTIONS
     ***************************************************************/

    iSYSResult_t iSYS_setDeviceAddress(uint8_t deviceaddress, uint8_t destAddress, uint32_t timeout);
    iSYSResult_t iSYS_getDeviceAddress(uint8_t *deviceaddress, uint8_t destAddress, uint32_t timeout);

    /***************************************************************
     *  ACQUISITION CONTROL FUNCTIONS
     ***************************************************************/

    iSYSResult_t iSYS_startAcquisition(uint8_t destAddress, uint32_t timeout);
    iSYSResult_t iSYS_stopAcquisition(uint8_t destAddress, uint32_t timeout);

    /***************************************************************
     *  OUTPUT SINGLE TARGET FILTER FUNCTIONS
     ***************************************************************/

    iSYSResult_t iSYS_setOutputFilter(iSYSOutputNumber_t outputnumber, iSYSOutput_filter_t filter, uint8_t destAddress, uint32_t timeout);
    iSYSResult_t iSYS_getOutputFilter(iSYSOutputNumber_t outputnumber, iSYSOutput_filter_t *filter, uint8_t destAddress, uint32_t timeout);
    iSYSResult_t iSYS_setOutputSignalFilter(iSYSOutputNumber_t outputnumber, iSYSFilter_signal_t signal, uint8_t destAddress, uint32_t timeout);
    iSYSResult_t iSYS_getOutputSignalFilter(iSYSOutputNumber_t outputnumber, iSYSFilter_signal_t *signal, uint8_t destAddress, uint32_t timeout);



private:
    uint32_t _baud;
    HardwareSerial &_serial;
    bool _debugEnabled;
    Stream *_debugStream; // e.g., &Serial, &Serial1

    /***************************************************************
     *  HELPER FUNCTIONS FOR COMMUNICATION AND DECODING
     ***************************************************************/

    iSYSResult_t decodeTargetFrame(uint8_t *frame_array, uint16_t nrOfElements, uint8_t bitrate, iSYSTargetList_t *targetList);
    iSYSResult_t sendTargetListRequest(iSYSOutputNumber_t outputnumber, uint8_t destAddress, uint8_t bitrate);
    iSYSResult_t receiveTargetListResponse(iSYSTargetList_t *pTargetList, uint32_t timeout, uint8_t bitrate);
    iSYSResult_t decodeTargetList(const uint8_t *data, uint16_t length, iSYSTargetList_t *pTargetList);

    /***************************************************************
     *  EEPROM COMMAND HELPER FUNCTIONS
     ***************************************************************/
    iSYSResult_t sendEEPROMCommand(iSYSEEPROMSubFunction_t subFunction, uint8_t destAddress, uint32_t timeout);
    iSYSResult_t sendEEPROMCommandFrame(iSYSEEPROMSubFunction_t subFunction, uint8_t destAddress);
    iSYSResult_t receiveEEPROMAcknowledgement(uint8_t destAddress, uint32_t timeout);
    uint8_t calculateFCS(const uint8_t *data, uint8_t startIndex, uint8_t endIndex);

    /***************************************************************
     *  ACQUISITION CONTROL HELPER FUNCTIONS
     ***************************************************************/

    iSYSResult_t sendAcquisitionCommand(uint8_t destAddress, bool start);
    iSYSResult_t receiveAcquisitionAcknowledgement(uint8_t destAddress, uint32_t timeout);

    /***************************************************************
     *  OUTPUT SINGLE TARGET FILTER HELPER FUNCTIONS
     ***************************************************************/

    iSYSResult_t sendSetOutputFilterRequest(iSYSOutputNumber_t outputnumber, iSYSOutput_filter_t filter, uint8_t destAddress);
    iSYSResult_t receiveSetOutputFilterAcknowledgement(uint8_t destAddress, uint32_t timeout);
    iSYSResult_t sendGetOutputFilterRequest(iSYSOutputNumber_t outputnumber, uint8_t destAddress);
    iSYSResult_t receiveGetOutputFilterResponse(iSYSOutput_filter_t *filter, uint8_t destAddress, uint32_t timeout);
    iSYSResult_t sendSetOutputSignalFilterRequest(iSYSOutputNumber_t outputnumber, iSYSFilter_signal_t signal, uint8_t destAddress);
    iSYSResult_t receiveSetOutputSignalFilterAcknowledgement(uint8_t destAddress, uint32_t timeout);
    iSYSResult_t sendGetOutputSignalFilterRequest(iSYSOutputNumber_t outputnumber, uint8_t destAddress);
    iSYSResult_t receiveGetOutputSignalFilterResponse(iSYSFilter_signal_t *signal, uint8_t destAddress, uint32_t timeout);

    // Debug helpers (no-ops when debug disabled or stream not set)
    void debugPrint(const char *msg);
    void debugPrintln(const char *msg);
    void debugPrintHexFrame(const char *prefix, const uint8_t *data, size_t length);
};

#endif