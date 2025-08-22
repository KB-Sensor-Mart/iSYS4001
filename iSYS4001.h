#ifndef ISYS4001_H
#define ISYS4001_H

#include <Arduino.h>

// Define MAX_TARGETS constant
#define MAX_TARGETS (0x23)

// Result/error codes for API calls
typedef enum iSYSResult
{
    ERR_OK                                  = 0x0000,
    ERR_FUNCTION_DEPRECATED                 ,
    ERR_DLL_NOT_FINISHED                    ,
    ERR_HANDLE_NOT_INITIALIZED              ,
    ERR_COMPORT_DOESNT_EXIST                ,
    ERR_COMPORT_CANT_INITIALIZE             ,
    ERR_COMPORT_ACCESS_DENIED               ,
    ERR_COMPORT_BAUDRATE_NOT_VALID          ,
    ERR_COMPORT_CANT_OPEN                   ,
    ERR_COMPORT_CANT_SET_FLOW_CONTROL       ,
    ERR_COMPORT_CANT_SET_PARITY             ,
    ERR_COMPORT_CANT_SET_STOP_BITS          ,
    ERR_COMPORT_CANT_SET_DATA_BITS          ,
    ERR_COMPORT_CANT_SET_BAUDRATE           ,
    ERR_COMPORT_ALREADY_INITIALIZED         ,
    ERR_COMPORT_EQUALS_NULL                 ,
    ERR_COMPORT_NOT_OPEN                    ,
    ERR_COMPORT_NOT_READABLE                ,
    ERR_COMPORT_NOT_WRITEABLE               ,
    ERR_COMPORT_CANT_WRITE                  ,
    ERR_COMPORT_CANT_READ                   ,
    ERR_COMMAND_NOT_WRITTEN                 ,
    ERR_COMMAND_NOT_READ                    ,
    ERR_COMMAND_NO_DATA_RECEIVED            ,
    ERR_COMMAND_NO_VALID_FRAME_FOUND        ,
    ERR_COMMAND_RX_FRAME_DAMAGED            ,
    ERR_COMMAND_FAILURE                     ,
    ERR_UNDEFINED_READ                      ,
    ERR_COMPORT_LESS_DATA_READ              ,
    ERR_COMPORT_SYSTEM_INIT_FAILED          ,
    ERR_COMPORT_SYSTEM_ALREADY_INITIALIZED  ,
    ERR_COMMAND_RX_FRAME_LENGTH             ,
    ERR_COMMAND_MAX_DATA_OVERFLOW           ,
    ERR_COMMAND_MAX_IQPAIRS_OVERFLOW        ,
    ERR_COMMAND_NOT_ACCEPTED                ,
    ERR_NULL_POINTER                        ,
    ERR_CALC_CORRECTION_PARAMS              ,
    ERR_PARAMETER_OUT_OF_RANGE              ,
    ERR_COMMAND_UNEXPECTED_FRAMETYPE
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
    
    // Function to send command, receive response, and decode target list
    iSYSResult_t getTargetList32(
        iSYSTargetList_t *pTargetList,
        iSYSOutputNumber_t outputnumber,
        uint8_t destAddress,
        uint32_t timeout
    );

private:
    uint32_t _baud;
    HardwareSerial& _serial;
    
    // Helper functions for communication and decoding
    iSYSResult_t decodeTargetFrame(uint8_t *frame_array, uint16_t nrOfElements,
                                   uint16_t productcode, uint8_t bitrate,
                                   iSYSTargetList_t *targetList);
    iSYSResult_t sendTargetListRequest(iSYSOutputNumber_t outputnumber, uint8_t destAddress);
    iSYSResult_t receiveTargetListResponse(iSYSTargetList_t *pTargetList, uint32_t timeout);
    iSYSResult_t decodeTargetList(const uint8_t* data, uint16_t length, iSYSTargetList_t *pTargetList);
   
};

#endif