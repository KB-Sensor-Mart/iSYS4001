#ifndef ISYS4001_H
#define ISYS4001_H

#include <Arduino.h>

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
typedef enum iSYSResult
{
    // ===== SUCCESS CODES =====
    ERR_OK                                  = 0x0000,   // (0)
    
    // ===== SYSTEM INITIALIZATION ERRORS =====
    ERR_FUNCTION_DEPRECATED                 = 0x0001,   // (1)
    ERR_DLL_NOT_FINISHED                    = 0x0002,   // (2)
    ERR_HANDLE_NOT_INITIALIZED              = 0x0003,   // (3)
    ERR_COMPORT_SYSTEM_INIT_FAILED          = 0x0004,   // (4)
    ERR_COMPORT_SYSTEM_ALREADY_INITIALIZED  = 0x0005,   // (5)
    
    // ===== COM PORT CONFIGURATION ERRORS =====
    ERR_COMPORT_DOESNT_EXIST                = 0x0010,   // (16)
    ERR_COMPORT_CANT_INITIALIZE             = 0x0011,   // (17)
    ERR_COMPORT_ACCESS_DENIED               = 0x0012,   // (18)
    ERR_COMPORT_BAUDRATE_NOT_VALID          = 0x0013,   // (19)
    ERR_COMPORT_CANT_OPEN                   = 0x0014,   // (20)
    ERR_COMPORT_ALREADY_INITIALIZED         = 0x0015,   // (21)
    ERR_COMPORT_EQUALS_NULL                 = 0x0016,   // (22)
    
    // ===== COM PORT PARAMETER SETTING ERRORS =====
    ERR_COMPORT_CANT_SET_FLOW_CONTROL       = 0x0020,   // (32)
    ERR_COMPORT_CANT_SET_PARITY             = 0x0021,   // (33)
    ERR_COMPORT_CANT_SET_STOP_BITS          = 0x0022,   // (34)
    ERR_COMPORT_CANT_SET_DATA_BITS          = 0x0023,   // (35)
    ERR_COMPORT_CANT_SET_BAUDRATE           = 0x0024,   // (36)
    
    // ===== COM PORT I/O OPERATION ERRORS =====
    ERR_COMPORT_NOT_OPEN                    = 0x0030,   // (48)
    ERR_COMPORT_NOT_READABLE                = 0x0031,   // (49)
    ERR_COMPORT_NOT_WRITEABLE               = 0x0032,   // (50)
    ERR_COMPORT_CANT_WRITE                  = 0x0033,   // (51)
    ERR_COMPORT_CANT_READ                   = 0x0034,   // (52)
    ERR_COMPORT_LESS_DATA_READ              = 0x0035,   // (53)
    
    // ===== COMMAND TRANSMISSION ERRORS =====
    ERR_COMMAND_NOT_WRITTEN                 = 0x0040,   // (64)
    ERR_COMMAND_NOT_READ                    = 0x0041,   // (65)
    ERR_COMMAND_NOT_ACCEPTED                = 0x0042,   // (66)
    ERR_COMMAND_UNEXPECTED_FRAMETYPE        = 0x0043,   // (67)
    
    // ===== RESPONSE RECEPTION ERRORS =====
    ERR_COMMAND_NO_DATA_RECEIVED            = 0x0050,   // (80)
    ERR_COMMAND_NO_VALID_FRAME_FOUND        = 0x0051,   // (81)
    ERR_COMMAND_RX_FRAME_DAMAGED            = 0x0052,   // (82)
    ERR_COMMAND_RX_FRAME_LENGTH             = 0x0053,   // (83)
    ERR_COMMAND_MAX_DATA_OVERFLOW           = 0x0054,   // (84)
    ERR_COMMAND_MAX_IQPAIRS_OVERFLOW        = 0x0055,   // (85)
    ERR_UNDEFINED_READ                      = 0x0056,   // (86)
    ERR_INVALID_CHECKSUM                    = 0x0057,   // (87)
    
    // ===== GENERAL OPERATION ERRORS =====
    ERR_COMMAND_FAILURE                     = 0x0060,   // (96)
    ERR_NULL_POINTER                        = 0x0061,   // (97)
    ERR_CALC_CORRECTION_PARAMS              = 0x0062,   // (98)
    ERR_PARAMETER_OUT_OF_RANGE              = 0x0063,   // (99)
    ERR_OUTPUT_OUT_OF_RANGE                 = 0x0064,   // (99)
    ERR_TIMEOUT                             = 0X0065,
    
} iSYSResult_t;

typedef enum iSYSTargetListError
{
    TARGET_LIST_OK                          = 0x00,
    TARGET_LIST_FULL                        = 0x01,
    TARGET_LIST_REFRESHED                   = 0x02,
    TARGET_LIST_ALREADY_REQUESTED           = 0x03,
    TARGET_LIST_ACQUISITION_NOT_STARTED     = 0x04
}iSYSTargetListError_t;

typedef enum iSYSOutputNumber
{
    ISYS_OUTPUT_1     = 1,
    ISYS_OUTPUT_2        ,
    ISYS_OUTPUT_3
} iSYSOutputNumber_t;

// Output single target filter type (sub-function 0x15)
typedef enum iSYSOutput_filter
{
    ISYS_SIGNAL = 0,
    ISYS_MEAN           ,
    ISYS_MEDIAN         ,
    ISYS_MIN            ,
    ISYS_MAX            
} iSYSOutput_filter_t;

// Output single target filter signal (sub-function 0x16)
typedef enum iSYSFilter_signal
{
    ISYS_OFF = 0,
    ISYS_VELOCITY_RADIAL,
    ISYS_RANGE_RADIAL
} iSYSFilter_signal_t;

// EEPROM sub-function codes for saving settings
typedef enum iSYSEEPROMSubFunction
{
    ISYS_EEPROM_SET_FACTORY_SETTINGS        = 0x01,    // Restore factory default settings
    ISYS_EEPROM_SAVE_SENSOR_SETTINGS        = 0x02,    // Save sensor settings to EEPROM
    ISYS_EEPROM_SAVE_APPLICATION_SETTINGS   = 0x03,    // Save application settings to EEPROM
    ISYS_EEPROM_SAVE_ALL_SETTINGS           = 0x04     // Save both sensor and application settings to EEPROM
} iSYSEEPROMSubFunction_t;

typedef struct iSYSTarget {
    float signal;           /* signal indicator */
    float velocity;         /* radial velocity in m/s */
    float range;            /* range in m */
    float angle;            /* angle of detected object [°] */
} iSYSTarget_t;

union iSYSTargetListError_u
{
    iSYSTargetListError_t iSYSTargetListError;
    uint32_t dummy;
};

typedef enum iSYSDirection_type
{
    ISYS_TARGET_DIRECTION_APPROACHING = 1,
    ISYS_TARGET_DIRECTION_RECEDING = 2,
    //ISYS_TARGET_DIRECTION_BOTH = 3
    
    ISYS_TARGET_DIRECTION_BOTH = ISYS_TARGET_DIRECTION_APPROACHING | ISYS_TARGET_DIRECTION_RECEDING,
    
} iSYSDirection_type_t;


typedef struct iSYSTargetList {
    union iSYSTargetListError_u error;
    uint8_t outputNumber;
    uint16_t nrOfTargets;
    uint32_t clippingFlag;
    iSYSTarget_t targets[MAX_TARGETS];
} iSYSTargetList_t;

class iSYS4001 {
    
public:
    iSYS4001(HardwareSerial& serial, uint32_t baud = 115200);
    


/***************************************************************  
 *  GET TARGET LIST FUNCTION  
 ***************************************************************/

    iSYSResult_t getTargetList32(iSYSTargetList_t *pTargetList,uint8_t destAddress,uint32_t timeout,iSYSOutputNumber_t outputnumber = ISYS_OUTPUT_1);


/***************************************************************  
 *  SET RANGE MIN/MAX FUNCTIONS  
 ***************************************************************/

    iSYSResult_t iSYS_setOutputRangeMin(iSYSOutputNumber_t outputnumber, uint16_t range, uint8_t destAddress, uint32_t timeout);
    iSYSResult_t iSYS_setOutputRangeMax( iSYSOutputNumber_t outputnumber, uint16_t range, uint8_t destAddress, uint32_t timeout);


/***************************************************************  
 *  SET VELOCITY MIN/MAX FUNCTIONS  
 ***************************************************************/

    iSYSResult_t iSYS_setOutputVelocityMin(iSYSOutputNumber_t outputnumber, uint16_t velocity, uint8_t destAddress, uint32_t timeout);
    iSYSResult_t iSYS_setOutputVelocityMax(iSYSOutputNumber_t outputnumber, uint16_t velocity, uint8_t destAddress, uint32_t timeout);

/***************************************************************  
 *  SET SIGNAL MIN/MAX FUNCTIONS  
 ***************************************************************/

    iSYSResult_t iSYS_setOutputSignalMin(iSYSOutputNumber_t outputnumber, uint16_t signal, uint8_t destAddress, uint32_t timeout);
    iSYSResult_t iSYS_setOutputSignalMax(iSYSOutputNumber_t outputnumber, uint16_t signal, uint8_t destAddress, uint32_t timeout);


/***************************************************************  
 *  SET VELOCITY DIRECTION FUNCTION  
 ***************************************************************/

    iSYSResult_t iSYS_setOutputDirection(iSYSOutputNumber_t outputnumber, iSYSDirection_type_t direction, uint8_t destAddress, uint32_t timeout);
    iSYSResult_t iSYS_getOutputDirection(iSYSOutputNumber_t outputnumber, iSYSDirection_type_t *direction, uint8_t destAddress, uint32_t timeout);






/***************************************************************  
 *  EEPROM COMMAND FUNCTIONS 
 ***************************************************************/

    iSYSResult_t sendEEPROMCommand(iSYSEEPROMSubFunction_t subFunction,uint8_t destAddress,uint32_t timeout);
    iSYSResult_t setFactorySettings(uint8_t destAddress, uint32_t timeout);
    iSYSResult_t saveSensorSettings(uint8_t destAddress,uint32_t timeout);
    iSYSResult_t saveApplicationSettings(uint8_t destAddress,uint32_t timeout);
    iSYSResult_t saveAllSettings(uint8_t destAddress,uint32_t timeout);

/***************************************************************  
 *  DEVICE ADDRESS FUNCTIONS 
 ***************************************************************/

    iSYSResult_t iSYS_setDeviceAddress(uint8_t deviceaddress, uint8_t destAddress , uint32_t timeout);
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
    HardwareSerial& _serial;
    
/***************************************************************  
 *  HELPER FUNCTIONS FOR COMMUNICATION AND DECODING  
 ***************************************************************/
   
    iSYSResult_t decodeTargetFrame(uint8_t *frame_array, uint16_t nrOfElements,uint16_t productcode, uint8_t bitrate,iSYSTargetList_t *targetList);
    iSYSResult_t sendTargetListRequest(iSYSOutputNumber_t outputnumber, uint8_t destAddress);
    iSYSResult_t receiveTargetListResponse(iSYSTargetList_t *pTargetList, uint32_t timeout);
    iSYSResult_t decodeTargetList(const uint8_t* data, uint16_t length, iSYSTargetList_t *pTargetList);


/***************************************************************  
 *  EEPROM COMMAND HELPER FUNCTIONS
 ***************************************************************/    
 
    iSYSResult_t sendEEPROMCommandFrame(iSYSEEPROMSubFunction_t subFunction, uint8_t destAddress);
    iSYSResult_t receiveEEPROMAcknowledgement(uint8_t destAddress,uint32_t timeout);
    uint8_t calculateFCS(const uint8_t* data, uint8_t startIndex, uint8_t endIndex);

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
   
};

#endif