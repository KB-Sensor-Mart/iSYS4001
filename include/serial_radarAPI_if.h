/**************************************************************************************

      II    N     N     N     N      OOOO      SSSSS     EEEEE    N     N    TTTTTTT
     II    NNN   N     NNN   N    OO    OO    S         E        NNN   N       T
    II    N NN  N     N NN  N    OO    OO    SSSSS     EEE      N NN  N       T
   II    N  NN N     N  NN N    OO    OO        S     E        N  NN N       T
  II    N    NN     N    NN      OOOO      SSSSS     EEEEE    N    NN       T
                         copyright (c) 2016, InnoSenT GmbH
                                 all rights reserved

***************************************************************************************

    filename:			serial_radarAPI_if.h
    brief:				API for communication between iSYS-4XXX/iSYS-6XXX and PC over serialport
    creation:			28.01.2016
    author:				Sebastian Weidmann

version:			v1.0
    last edit:          28.01.2016
    last editor:        Sebastian Weidmann
    change:             -first release
    compile switches:

version:			v2.0
    last edit:          18.02.2016
    last editor:        Sebastian Weidmann
    change:             -bug fixed: cmdSetCalibrationDataRangeOffset
                        -fixed typing error (Linefilter)
                        -changed some enums and set all iSYS enum to ISYS enum
    compile switches:

version:            v3.0
    last edit:
    last editor:
    change:             -fixed bug in iSYS_getDigitalOutputState function
    compile switches:

version:            v4.0
    last edit:          15.04.2016
    last editor:        Benjamin Grünewald
    change:             -added commands from IDS-1000
                        -changed "sensivity" to "sensitivity"
						-bug fixed: factor for angle changed
						-added support for iSYS-6203
                        -added new command Get/SetMeasurementMode and ENUM iSYSMeasurementMode
	compile switches:

version:            v4.1
    last edit:          18.05.2016
    last editor:        Thomas Popp
    change:             -bug fixed set/get calibration values
    compile switches:

version:            v5.1
    last edit:          25.11.2016
    last editor:        Benjamin Gruenewald
    change:             -added support for iSYS-5010

version:            v6.0
    last edit:          07.12.2016
    last editor:        Benjamin Gruenewald
    change:             -removed all enumerator and included it in a single file

version:            v6.1
    last edit:          16.03.2017
    last editor:        Philipp Goetz
    change:             -added support for iSYS-5110

version:            v6.2
    last edit:          24.04.2017
    last editor:        Philipp Goetz
    change:             -optimize EOL functions

version:            v6.4
    last edit:          09.05.2017
    last editor:        Johannes Witzgall
    change:             -added baud rate 230400
                        -added support for iSYS-6005

version:            v6.5
    last edit:          09.06.2017
    last editor:        Johannes Witzgall
    change:             -added support for iSYS-6004 and iSYS-6006 target list


version:            v6.6
    last edit:          28.10.2017
    last editor:        Cyrus Nabavinegad
    change:             -added new functions for iSYS-6203

version:            v6.7
    last edit:          30.11.2017
    last editor:        Sebastian Weidmann
    change:             -added sensitivity functions for iSYS-5010

version:            v6.8
    last edit:          04.12.2017
    last editor:        Johannes Witzgall
    change:             - fixed not working iSYS_getRawData for iSYS-6xxx and some 4xxx with newer firmwares

version:            v6.9
    last edit:          02.01.2018
    last editor:        Johannes Witzgall
    change:             - added EOL commands

version:            v7.000
    last edit:          10.01.2018
    last editor:        Claus Boehm
    change:             -added stop bar zone functions for iSYS-5110


version:            v7.001
    last edit:          07.05.2018
    last editor:        Johannes Witzgall
    change:             -added target list interface function for iSYS-5020

version:            v7.010
    last edit:          07.01.2019
    last editor:        Johannes Witzgall
    change:             - added EOL commands
                        - added LED configuration function for iSYS-400x


***************************************************************************************/

#ifndef ISYS_API_IF_H
#define ISYS_API_IF_H

#include "serial_radarAPI_basicTypes.h"
#include "serial_radarAPI_enums.h"

/**************************************************************************************
 defines
**************************************************************************************/
#if defined(_WIN32) && !defined(ISYS_API_AS_SOURCE)
    #ifdef ISYS_API_LIBRARY
        #define ISYS_API_EXPORT __declspec(dllexport)
    #else
        #define ISYS_API_EXPORT __declspec(dllimport)
    #endif
#else
    #define ISYS_API_EXPORT
#endif

#define MAX_TARGETS     (0x23)
#define MAX_IQ_PAIRS    (0x04)
#define MAX_DATA_SIZE   (2048)
#define MAX_ERROR_COUNT (100)

#ifdef __cplusplus
extern "C" {
#endif


/**************************************************************************************
 typedefs
**************************************************************************************/
typedef struct iSYSHandle *iSYSHandle_t;


union iSYSTargetListError_u
{
    iSYSTargetListError_t iSYSTargetListError;
    uint32_t dummy;
};

typedef struct iSYSTarget {
    float32_t velocity;         /* radial velocity in m/s */
    float32_t range;            /* range in m */
    float32_t signal;           /* signal indicator */
    float32_t angle;            /* angle of detected object [°] */
} iSYSTarget_t;

typedef struct iSYSTargetList {
    union iSYSTargetListError_u error;
    uint8_t outputNumber;
    uint16_t nrOfTargets;
    uint32_t clippingFlag;
    iSYSTarget_t targets[MAX_TARGETS];
} iSYSTargetList_t;

typedef struct iSYSComplex{
    float32_t real;
    float32_t imag;
}iSYSComplex_t;

typedef struct iSYSIQSignal{
  iSYSComplex_t IQ1A[MAX_DATA_SIZE];
  iSYSComplex_t IQ1B[MAX_DATA_SIZE];
  iSYSComplex_t IQ1C[MAX_DATA_SIZE];
  iSYSComplex_t IQ1D[MAX_DATA_SIZE];
}iSYSIQSignal_t;

typedef struct iSYSRawData{
  iSYSIQSignal_t signal;
  uint32_t signalNrOfSamples;
  uint32_t signalNrOfIQPairs;
}iSYSRawData_t;

typedef struct iSYSDetectionData{
  uint32_t clippingFlag;
  sint16_t fftMagnitude[MAX_DATA_SIZE];
  sint16_t threshold[MAX_DATA_SIZE];
}iSYSDetectionData_t;

typedef struct iSYSDetection{
  iSYSDetectionData_t detection;
  uint32_t signalNrOfSamples;
  uint32_t nrOfDetections;
}iSYSDetection_t;

typedef struct iSYSRangeList{
    sint32_t range;
    sint16_t amplitude;
    sint16_t sensor_temperature;
    sint32_t standard_deviation;
    sint32_t variance;
    sint32_t statistical_min;
    sint32_t statistical_max;
} iSYSRangeList_t;

typedef struct iSYSIPConfig{
    uint8_t ipAddressByte1;
    uint8_t ipAddressByte2;
    uint8_t ipAddressByte3;
    uint8_t ipAddressByte4;
    uint8_t subnetMaskByte1;
    uint8_t subnetMaskByte2;
    uint8_t subnetMaskByte3;
    uint8_t subnetMaskByte4;
    uint8_t gatewayByte1;
    uint8_t gatewayByte2;
    uint8_t gatewayByte3;
    uint8_t gatewayByte4;
} iSYSIPConfig_t;

typedef struct iSYSIPDestination{
    uint8_t ipAddressByte1;
    uint8_t ipAddressByte2;
    uint8_t ipAddressByte3;
    uint8_t ipAddressByte4;
    uint16_t targeListPort;
    uint16_t recordPort;
} iSYSIPDestination_t;


/**************************************************************************************
 api functions
**************************************************************************************/
#ifndef __linux__
ISYS_API_EXPORT iSYSResult_t iSYS_initComPort(iSYSHandle_t *pHandle, uint8_t comportNr, iSYSBaudrate_t baudrate);
#else
ISYS_API_EXPORT iSYSResult_t iSYS_initComPort(iSYSHandle_t *pHandle, char *comport, int length, iSYSBaudrate_t baudrate);
#endif

ISYS_API_EXPORT iSYSResult_t iSYS_initSystem(iSYSHandle_t pHandle, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_exitComPort(iSYSHandle_t pHandle);
ISYS_API_EXPORT iSYSResult_t iSYS_exitSystem(iSYSHandle_t pHandle, uint8_t destAddress);
ISYS_API_EXPORT iSYSResult_t iSYS_getApiVersion(float32_t *version);

/*SD2 - D0 */
ISYS_API_EXPORT iSYSResult_t iSYS_ReadDeviceName(iSYSHandle_t pHandle, char *devicename_array, uint16_t array_length, uint8_t destAddress, uint32_t timeout);


/*SD2 - D1 */
ISYS_API_EXPORT iSYSResult_t iSYS_StartAcquisition(iSYSHandle_t pHandle, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_StopAcquisition(iSYSHandle_t pHandle, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_SetDefaultTemperatureThreshold(iSYSHandle_t pHandle, float32_t *tempThldOut1, float32_t *tempThldOut2, float32_t *tempThldOut3, uint8_t destAddress, uint32_t timeout); /*iSYS-6203 only*/


/*SD2 - D2 */
ISYS_API_EXPORT iSYSResult_t iSYS_setDeviceAddress(iSYSHandle_t pHandle, uint8_t deviceaddress, uint8_t destAddress , uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getDeviceAddress(iSYSHandle_t pHandle,uint8_t *deviceaddress, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getFrequencyChannel(iSYSHandle_t pHandle, iSYSFrequencyChannel_t *channel, uint8_t destAddress, uint32_t timeout);   /* Measurement mode must be stopped before (cmdStopAcquisition) */
ISYS_API_EXPORT iSYSResult_t iSYS_getThresholdMin(iSYSHandle_t pHandle, sint16_t *sensitivity, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getThresholdSensitivityLeft(iSYSHandle_t pHandle, uint16_t *threshold, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getThresholdSensitivityRight(iSYSHandle_t pHandle, uint16_t *threshold, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getSerialNumber(iSYSHandle_t pHandle,uint32_t *serialNumber, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getFirmwareVersion(iSYSHandle_t pHandle, uint16_t *major, uint16_t *fix, uint16_t *minor, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getMeasurementMode(iSYSHandle_t pHandle, iSYSMeasurementMode_t *mode, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getIPConfig(iSYSHandle_t pHandle, iSYSSaveLocation_t place, iSYSIPConfig_t *ipConfig, uint8_t destAddress , uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getIPDestination(iSYSHandle_t pHandle, iSYSSaveLocation_t place, iSYSIPDestination_t *ipDestination, uint8_t destAddress , uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getTargetListInterface(iSYSHandle_t pHandle, iSYSSaveLocation_t place, iSYSTargetListInterface_t *pInterface, uint8_t destAddress , uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getLedConfiguration(iSYSHandle_t pHandle, iSYSLedConfiguration_t *config, uint8_t destAddress , uint32_t timeout);


/*SD2 - D3 */
ISYS_API_EXPORT iSYSResult_t iSYS_setDeviceAddress(iSYSHandle_t pHandle, uint8_t deviceaddress, uint8_t destAddress , uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setFrequencyChannel(iSYSHandle_t pHandle, iSYSFrequencyChannel_t channel, uint8_t destAddress , uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setThresholdMin(iSYSHandle_t pHandle, sint16_t sensitivity, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setThresholdSensitivityLeft(iSYSHandle_t pHandle, uint16_t threshold, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setThresholdSensitivityRight(iSYSHandle_t pHandle, uint16_t threshold, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setMeasurementMode(iSYSHandle_t pHandle, iSYSMeasurementMode_t mode, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setIPConfig(iSYSHandle_t pHandle, iSYSSaveLocation_t place, iSYSIPConfig_t ipConfig, uint8_t destAddress , uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setIPDestination(iSYSHandle_t pHandle, iSYSSaveLocation_t place, iSYSIPDestination_t ipDestination, uint8_t destAddress , uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setTargetListInterface(iSYSHandle_t pHandle, iSYSSaveLocation_t place, iSYSTargetListInterface_t interf, uint8_t destAddress , uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setLedConfiguration(iSYSHandle_t pHandle, iSYSLedConfiguration_t config, uint8_t destAddress , uint32_t timeout);


/*SD2 - D4 */
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputAngleMin(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *angle, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputAngleMax(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *angle, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputRangeMin(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *range, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputRangeMax(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *range, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputSignalMin(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *signal, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputSignalMax(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *signal, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputVelocityMin(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *velocity, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputVelocityMax(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *velocity, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputDirection(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, iSYSDirection_type_t *direction, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputFilter(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, iSYSOutput_filter_t *filter, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputSignalFilter(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, iSYSFilter_signal_t *signal, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputAlphaFilterVelocity(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *filter, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputAlphaFilterRange(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *filter, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputRangeMinExt(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *range, uint8_t destAddress, uint32_t timeout);  /*iSYS-6203 only*/
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputRangeMaxExt(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *range, uint8_t destAddress, uint32_t timeout);  /*iSYS-6203 only*/
ISYS_API_EXPORT iSYSResult_t iSYS_getLineFilter1(iSYSHandle_t pHandle, uint16_t *enable, uint16_t *frequency, uint16_t *offset, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getLineFilter2(iSYSHandle_t pHandle, uint16_t *enable, uint16_t *frequency, uint16_t *offset, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputEnable(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, iSYSOutput_t *enable, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputRisingDelay(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *delay, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputFallingDelay(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *delay, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputPlausibility(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *plausibility, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputSetting1(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *setting, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getOutputSetting2(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t *setting, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getPotis(iSYSHandle_t pHandle, uint16_t *value, uint8_t destAddress , uint32_t timeout); /* iSYS-4001,4002,4003 only */
ISYS_API_EXPORT iSYSResult_t iSYS_getTemperatureThreshold(iSYSHandle_t pHandle, iSYSSaveLocation_t location, float32_t *tempThldOut1, float32_t *tempThldOut2, float32_t *tempThldOut3, uint8_t destAddress, uint32_t timeout); /*iSYS-6203 only*/
ISYS_API_EXPORT iSYSResult_t iSYS_getTargetClusteringEnable(iSYSHandle_t pHandle, iSYSSaveLocation_t location, uint8_t *enable, uint8_t destAddress , uint32_t timeout); /* iSYS-5010 only */
ISYS_API_EXPORT iSYSResult_t iSYS_getRcsOutputEnable(iSYSHandle_t pHandle, iSYSSaveLocation_t location, uint8_t *enable, uint8_t destAddress , uint32_t timeout); /* iSYS-5010 only */
ISYS_API_EXPORT iSYSResult_t iSYS_getApplicationMode(iSYSHandle_t pHandle, iSYSSaveLocation_t location, iSYSApplicationMode_t *mode, uint8_t destAddress , uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getMountingOffsetValue(iSYSHandle_t pHandle, uint32_t *mountingOffset, uint8_t destAddress , uint32_t timeout);  /*iSYS-6203 only*/
ISYS_API_EXPORT iSYSResult_t iSYS_getRangeTemperatureWarningThresholdSwitch(iSYSHandle_t pHandle, iSYSSaveLocation_t location, iSYSOutputNumber_t selectedOutput, iSYSWarningMode_t *warningMode, uint8_t destAddress, uint32_t timeout);   /*iSYS-6203 only*/
ISYS_API_EXPORT iSYSResult_t iSYS_getRangeWarningThreshold(iSYSHandle_t pHandle, iSYSSaveLocation_t location, iSYSOutputNumber_t selectedOutput, uint32_t *rangeWarningThreshold, uint8_t destAddress, uint32_t timeout);   /*iSYS-6203 only*/
ISYS_API_EXPORT iSYSResult_t iSYS_getTemperatureWarningThreshold(iSYSHandle_t pHandle, iSYSSaveLocation_t location, iSYSOutputNumber_t selectedOutput, float32_t *temperatureWarningThreshold, uint8_t destAddress, uint32_t timeout);  /*iSYS-6203 only*/
ISYS_API_EXPORT iSYSResult_t iSYS_getThresholdMovingTargetsNearRangeMargin(iSYSHandle_t pHandle, iSYSSaveLocation_t location, sint16_t *margin, uint8_t destAddress , uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getThresholdMovingTargetsMainRangeMargin(iSYSHandle_t pHandle, iSYSSaveLocation_t location, sint16_t *margin, uint8_t destAddress , uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getThresholdMovingTargetsLongRangeMargin(iSYSHandle_t pHandle, iSYSSaveLocation_t location, sint16_t *margin, uint8_t destAddress , uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getStopBarEnable(iSYSHandle_t pHandle, iSYSSaveLocation_t location, uint8_t zoneNumber, uint8_t *enable, uint8_t destAddress , uint32_t timeout); /* iSYS-5110 only */
ISYS_API_EXPORT iSYSResult_t iSYS_getStopBarRange(iSYSHandle_t pHandle, iSYSSaveLocation_t location, uint8_t zoneNumber, uint16_t *minRange, uint16_t *maxRange, uint8_t destAddress , uint32_t timeout); /* iSYS-5110 only */
ISYS_API_EXPORT iSYSResult_t iSYS_getStopBarAngle(iSYSHandle_t pHandle, iSYSSaveLocation_t location, uint8_t zoneNumber, float32_t *minAngle, float32_t *maxAngle, uint8_t destAddress , uint32_t timeout); /* iSYS-5110 only */

/*SD2 - D5 */
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputAngleMin(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t angle, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputAngleMax(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t angle, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputRangeMin(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t range, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputRangeMax(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t range, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputSignalMin(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t signal, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputSignalMax(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t signal, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputVelocityMin(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t velocity, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputVelocityMax(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t velocity, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputDirection(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, iSYSDirection_type_t direction, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputFilter(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, iSYSOutput_filter_t filter, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputSignalFilter(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, iSYSFilter_signal_t signal, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputAlphaFilterVelocity(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t filter, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputAlphaFilterRange(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t filter, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputRangeMinExt(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t range, uint8_t destAddress, uint32_t timeout);   /*iSYS-6203 only*/
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputRangeMaxExt(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t range, uint8_t destAddress, uint32_t timeout);   /*iSYS-6203 only*/
ISYS_API_EXPORT iSYSResult_t iSYS_setLineFilter1(iSYSHandle_t pHandle, uint16_t enable, uint16_t frequency, uint16_t offset, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setLineFilter2(iSYSHandle_t pHandle, uint16_t enable, uint16_t frequency, uint16_t offset, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputEnable(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, iSYSOutput_t enable, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputRisingDelay(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t delay, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputFallingDelay(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t delay, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputPlausibility(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t plausibility, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputSetting1(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t setting, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setOutputSetting2(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, uint16_t setting, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setPotis(iSYSHandle_t pHandle, uint16_t value, uint8_t destAddress , uint32_t timeout);    /* iSYS-4001,4002,4003 only */
ISYS_API_EXPORT iSYSResult_t iSYS_setTemperatureThreshold(iSYSHandle_t pHandle, iSYSSaveLocation_t location, float32_t tempThldOut1, float32_t tempThldOut2, float32_t tempThldOut3, uint8_t destAddress, uint32_t timeout);  /*iSYS-6203 only*/
ISYS_API_EXPORT iSYSResult_t iSYS_setTargetClusteringEnable(iSYSHandle_t pHandle, iSYSSaveLocation_t location, uint8_t enable, uint8_t destAddress , uint32_t timeout); /* iSYS-5010 only */
ISYS_API_EXPORT iSYSResult_t iSYS_setRcsOutputEnable(iSYSHandle_t pHandle, iSYSSaveLocation_t location, uint8_t enable, uint8_t destAddress , uint32_t timeout); /* iSYS-5010 only */
ISYS_API_EXPORT iSYSResult_t iSYS_setApplicationMode(iSYSHandle_t pHandle, iSYSSaveLocation_t location, iSYSApplicationMode_t mode, uint8_t destAddress , uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setMountingOffsetValue(iSYSHandle_t pHandle, uint32_t mountingOffset, uint8_t destAddress, uint32_t timeout);   /*iSYS-6203 only*/
ISYS_API_EXPORT iSYSResult_t iSYS_setRangeTemperatureWarningThresholdSwitch(iSYSHandle_t pHandle, iSYSSaveLocation_t location, iSYSOutputNumber_t selectedOutput, iSYSWarningMode_t warningMode, uint8_t destAddress, uint32_t timeout);    /*iSYS-6203 only*/
ISYS_API_EXPORT iSYSResult_t iSYS_setRangeWarningThreshold(iSYSHandle_t pHandle, iSYSSaveLocation_t location, iSYSOutputNumber_t selectedOutput, uint32_t rangeWarningThreshold, uint8_t destAddress, uint32_t timeout);    /*iSYS-6203 only*/
ISYS_API_EXPORT iSYSResult_t iSYS_setTemperatureWarningThreshold(iSYSHandle_t pHandle, iSYSSaveLocation_t location, iSYSOutputNumber_t selectedOutput, float32_t temperatureWarningThreshold, uint8_t destAddress, uint32_t timeout);   /*iSYS-6203 only*/
ISYS_API_EXPORT iSYSResult_t iSYS_setThresholdMovingTargetsNearRangeMargin(iSYSHandle_t pHandle, iSYSSaveLocation_t location, sint16_t margin, uint8_t destAddress , uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setThresholdMovingTargetsMainRangeMargin(iSYSHandle_t pHandle, iSYSSaveLocation_t location, sint16_t margin, uint8_t destAddress , uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setThresholdMovingTargetsLongRangeMargin(iSYSHandle_t pHandle, iSYSSaveLocation_t location, sint16_t margin, uint8_t destAddress , uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_setStopBarEnable(iSYSHandle_t pHandle, iSYSSaveLocation_t location, uint8_t zoneNumber, uint8_t enable, uint8_t destAddress , uint32_t timeout); /* iSYS-5110 only */
ISYS_API_EXPORT iSYSResult_t iSYS_setStopBarRange(iSYSHandle_t pHandle, iSYSSaveLocation_t location, uint8_t zoneNumber, uint16_t minRange, uint16_t maxRange, uint8_t destAddress , uint32_t timeout); /* iSYS-5110 only */
ISYS_API_EXPORT iSYSResult_t iSYS_setStopBarAngle(iSYSHandle_t pHandle, iSYSSaveLocation_t location, uint8_t zoneNumber, float32_t minAngle, float32_t maxAngle, uint8_t destAddress , uint32_t timeout); /* iSYS-5110 only */


/*SD2 - D6 */
ISYS_API_EXPORT iSYSResult_t iSYS_getDspHardwareVersion(iSYSHandle_t pHandle, uint16_t *major, uint16_t *fix, uint16_t *minor, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getRfeHardwareVersion(iSYSHandle_t pHandle, uint16_t *major, uint16_t *fix, uint16_t *minor, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getProductInfo(iSYSHandle_t pHandle, uint16_t *productInfo, uint8_t destAddress, uint32_t timeout);


/*SD2 - DA Sensor must be in measurement mode (iSYS_StartAcquisition) */
ISYS_API_EXPORT iSYSResult_t iSYS_getTargetList(iSYSHandle_t pHandle, iSYSTargetList_t *pTargetList, iSYSOutputNumber_t outputnumber, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getTargetList16(iSYSHandle_t pHandle, iSYSTargetList_t *pTargetList, iSYSOutputNumber_t outputnumber, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_getTargetList32(iSYSHandle_t pHandle, iSYSTargetList_t *pTargetList, iSYSOutputNumber_t outputnumber, uint8_t destAddress, uint32_t timeout);


/*SD2 - DB Sensor must be in measurement mode (iSYS_StartAcquisition) */
ISYS_API_EXPORT iSYSResult_t iSYS_getDigitalOutputState(iSYSHandle_t pHandle, iSYSOutputNumber_t outputnumber, sint16_t *value, uint8_t destAddress, uint32_t timeout);


/*SD2 - DC Sensor must be in measurement mode (iSYS_StartAcquisition) */
ISYS_API_EXPORT iSYSResult_t iSYS_getRawData(iSYSHandle_t pHandle, iSYSRawData_t *data, uint8_t destAddress, uint32_t timeout);


/*SD2 - DD Sensor must be in measurement mode (iSYS_StartAcquisition) */
ISYS_API_EXPORT iSYSResult_t iSYS_getFftData(iSYSHandle_t pHandle, iSYSDetection_t *data, uint8_t destAddress, uint32_t timeout);


/*SD2 - DE */


/*SD2 - DF */
ISYS_API_EXPORT iSYSResult_t iSYS_setFactorySetting(iSYSHandle_t pHandle, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_saveSensorSettings(iSYSHandle_t pHandle, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_saveApplicationSettings(iSYSHandle_t pHandle, uint8_t destAddress, uint32_t timeout);
ISYS_API_EXPORT iSYSResult_t iSYS_saveSensorAndApplicationSettings(iSYSHandle_t pHandle, uint8_t destAddress, uint32_t timeout);


/* iSYS-6003 only */
/*SD2 - E1 Sensor must be in measurement mode (iSYS_StartAcquisition) */
ISYS_API_EXPORT iSYSResult_t iSYS_getRangeList(iSYSHandle_t pHandle, iSYSRangeList_t *pRangeList, uint8_t destAddress, uint32_t timeout);


/* IDS-1000 only (default destAddress = 128) */
/* set functions */
ISYS_API_EXPORT iSYSResult_t IDS_setOutputEnable(iSYSHandle_t pHandle, bool_t enable, uint8_t destAddress); /* enable = 0 -> OFF ; enable = 1 -> ON */
ISYS_API_EXPORT iSYSResult_t IDS_setFrequency(iSYSHandle_t pHandle, uint32_t frequency, uint8_t destAddress); /* set frequency in Hz */
ISYS_API_EXPORT iSYSResult_t IDS_setDirection(iSYSHandle_t pHandle, IDSDirection_type_t direction, uint8_t destAddress);


/* get functions */
ISYS_API_EXPORT iSYSResult_t IDS_getOutputEnable(iSYSHandle_t pHandle, bool_t *enable, uint8_t destAddress); /* enable = 0 -> OFF ; enable = 1 -> ON */
ISYS_API_EXPORT iSYSResult_t IDS_getFrequency(iSYSHandle_t pHandle, uint32_t *frequency, uint8_t destAddress); /* get frequency in Hz */
ISYS_API_EXPORT iSYSResult_t IDS_getDirection(iSYSHandle_t pHandle, IDSDirection_type_t *direction, uint8_t destAddress);


/* save settings to eeprom */
ISYS_API_EXPORT iSYSResult_t IDS_saveSettings(iSYSHandle_t pHandle, uint8_t destAddress);


#ifdef __cplusplus
}
#endif

#endif // ISYS_API_IF_H
