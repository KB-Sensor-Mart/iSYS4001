#include "iSYS4001.h"

iSYS4001::iSYS4001(HardwareSerial &serial, uint32_t baud)
    : _serial(serial), _baud(baud), _debugEnabled(false), _debugStream(nullptr)
{
}


/***************************************************************
 *  DEBUG CONFIGURATION FUNCTIONS
 ***************************************************************/

/**
 * Overview
 *  - Configure library-level debug logging and helper printing.
 *  - You can set the output Stream (e.g., Serial) and enable/disable logging.
 *  - Helper methods (`debugPrint`, `debugPrintln`, `debugPrintHexFrame`) respect
 *    the enabled flag and return a status so callers can check for issues.
 *
 * Functions
 *  - setDebugEnabled(bool enabled)
 *  - setDebug(Stream &stream, bool enabled)
 *  - setDebugStream(Stream &stream)
 *  - debugPrint(const char*), debugPrintln(const char*), debugPrintHexFrame(const char* prefix, const uint8_t* data, size_t length)
 *
 * Parameters
 *  - enabled:   turn library debug on/off
 *  - stream:    any Arduino Stream (Serial, Serial1, SoftwareSerial, etc.)
 *  - prefix:    optional text before hex frame output (can be nullptr)
 *  - data:      pointer to bytes to print as hex (can be nullptr if length == 0)
 *  - length:    number of bytes in data
 *
 * Return (iSYSResult_t)
 *  - setDebug, setDebugStream: ERR_OK, or ERR_NULL_POINTER if stream is invalid
 *  - debugPrint, debugPrintln, debugPrintHexFrame:
 *      - ERR_OK on successful print
 *      - ERR_COMMAND_NO_DATA_RECEIVED if debug is disabled or stream not set
 *      - ERR_NULL_POINTER if message/data pointer is null (and length > 0 for hex)
 *
 * Notes
 *  - Debug output is a no-op when disabled or when no stream is configured; helpers
 *    return an error code so you can detect and react during testing.
 *
 * Usage example
 *  radar.setDebug(Serial, true);
 *  radar.debugPrintln("Radar debug enabled");
 */
iSYSResult_t iSYS4001::setDebugEnabled(bool enabled)
{
    _debugEnabled = enabled;
    return ERR_OK;
}

iSYSResult_t iSYS4001::setDebugStream(Stream &stream)
{
    _debugStream = &stream;
    return (_debugStream != nullptr) ? ERR_OK : ERR_NULL_POINTER;
}

iSYSResult_t iSYS4001::setDebug(Stream &stream, bool enabled)
{
    _debugStream = &stream;
    _debugEnabled = enabled;
    return (_debugStream != nullptr) ? ERR_OK : ERR_NULL_POINTER;
}

// Internal debug helpers
iSYSResult_t iSYS4001::debugPrint(const char *msg)
{
    if (!(_debugEnabled && _debugStream))
    {
        return ERR_COMMAND_NO_DATA_RECEIVED; // treat as no-op due to disabled or missing stream
    }
    if (msg == nullptr)
    {
        return ERR_NULL_POINTER;
    }
    _debugStream->print(msg);
    return ERR_OK;
}

iSYSResult_t iSYS4001::debugPrintln(const char *msg)
{
    if (!(_debugEnabled && _debugStream))
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }
    if (msg == nullptr)
    {
        return ERR_NULL_POINTER;
    }
    _debugStream->println(msg);
    return ERR_OK;
}

iSYSResult_t iSYS4001::debugPrintHexFrame(const char *prefix, const uint8_t *data, size_t length)
{
    if (!(_debugEnabled && _debugStream))
        return ERR_COMMAND_NO_DATA_RECEIVED;
    if (prefix && *prefix)
    {
        _debugStream->print(prefix);
    }
    if (data == nullptr && length > 0)
    {
        return ERR_NULL_POINTER;
    }
    for (size_t i = 0; i < length; i++)
    {
        _debugStream->print("0x");
        if (data[i] < 0x10)
        {
            _debugStream->print("0");
        }
        _debugStream->print(data[i], HEX);
        _debugStream->print(" ");
    }
    _debugStream->println();
    return ERR_OK;
}

/***************************************************************
 *  GET TARGET LIST FUNCTIONS
 ***************************************************************/
/**
 * Overview
 *  - These APIs request the current detected targets from the iSYS-4001 radar
 *    and decode the response into a strongly-typed structure (`iSYSTargetList_t`).
 *  - Two payload formats are supported by the device:
 *      getTargetList16(...) -> 16-bit  (smaller frames)
 *      getTargetList32(...) -> 32-bit  (higher precision)
 *
 * Data model (iSYSTargetList_t)
 *  - error.iSYSTargetListError: high-level target-list condition (OK, FULL, ...)
 *  - outputNumber: which output the list belongs to (1..3)
 *  - nrOfTargets: number of valid entries populated in `targets[]`
 *  - clippingFlag: non-zero if data is clipped on the device
 *  - targets[i]: per-target values
 *      - signal (dB), velocity (m/s), range (m), angle (deg)
 *
 * Parameters
 *  - pTargetList: OUT. Pointer to structure that will receive decoded targets
 *  - destAddress: Device address on the UART bus (e.g. 0x80)
 *  - timeout:     Max time in ms to wait for a complete response frame
 *  - outputnumber: Optional output channel selector (ISYS_OUTPUT_1.._3)
 *
 * Return (iSYSResult_t)
 *  - ERR_OK: success, `pTargetList` filled
 *  - ERR_COMMAND_NO_DATA_RECEIVED: no bytes arrived before timeout
 *  - ERR_FRAME_INCOMPLETE: response ended before full frame was received
 *  - ERR_COMMAND_RX_FRAME_DAMAGED: framing did not match expected protocol
 *  - ERR_INVALID_CHECKSUM: checksum mismatch
 *  - ERR_COMMAND_MAX_DATA_OVERFLOW: response exceeded internal buffer limits
 *  - ERR_COMMAND_NO_VALID_FRAME_FOUND / ERR_COMMAND_FAILURE: protocol-level errors
 *  - ERR_TIMEOUT: invalid timeout parameter (0)
 *
 * Timing and reliability notes
 *  - The iSYS-400x cycle time is ~75 ms. Recommended minimum timeout is >=100 ms;
 *    examples here use 300 ms for robustness.
 *  - For stability, flush stale UART bytes before issuing a new request.
 *
 * Usage example
 *  iSYSTargetList_t list;
 *  iSYSResult_t res = radar.getTargetList32(&list, 0x80, 300, ISYS_OUTPUT_1);
 *  if (res == ERR_OK && list.error.iSYSTargetListError == TARGET_LIST_OK) {
 *      for (uint16_t i = 0; i < list.nrOfTargets && i < MAX_TARGETS; i++) {
 *          // use list.targets[i].signal / velocity / range / angle
 *      }
 *  }
 */

//``````````````````````````````````````````````````````````` 16 BIT ```````````````````````````````````````````````````````````//
iSYSResult_t iSYS4001::getTargetList16(iSYSTargetList_t *pTargetList, uint8_t destAddress, uint32_t timeout, iSYSOutputNumber_t outputnumber)
{
    memset(pTargetList, 0, sizeof(iSYSTargetList_t));
    iSYSResult_t res = sendTargetListRequest(outputnumber, destAddress, 16);
    if (res != ERR_OK)
        return res;
    res = receiveTargetListResponse(pTargetList, timeout, 16);
    if (res != ERR_OK)
        return res;
    return ERR_OK;
}

//``````````````````````````````````````````````````````````` 32 BIT```````````````````````````````````````````````````````````//
iSYSResult_t iSYS4001::getTargetList32(iSYSTargetList_t *pTargetList, uint8_t destAddress, uint32_t timeout, iSYSOutputNumber_t outputnumber)
{
    memset(pTargetList, 0, sizeof(iSYSTargetList_t));
    iSYSResult_t res = sendTargetListRequest(outputnumber, destAddress, 32);
    if (res != ERR_OK)
        return res;
    res = receiveTargetListResponse(pTargetList, timeout, 32);
    if (res != ERR_OK)
        return res;
    return ERR_OK;
}

//``````````````````````````````````````````````````````````` SEND TARGET LIST REQUEST```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::sendTargetListRequest(iSYSOutputNumber_t outputnumber, uint8_t destAddress, uint8_t bitrate)
{

    uint8_t command[11];
    uint8_t index = 0;

    command[index++] = 0x68;
    command[index++] = 0x05;
    command[index++] = 0x05;
    command[index++] = 0x68;
    command[index++] = destAddress;
    command[index++] = 0x01;
    command[index++] = 0xDA;
    command[index++] = outputnumber;
    command[index++] = (bitrate == 32) ? 0x20 : 0x10; // 0x20 for 32-bit, 0x10 for 16-bit

    uint8_t fcs = calculateFCS(command, 4, index - 1);
    command[index++] = fcs;
    command[index++] = 0x16;

    debugPrintHexFrame("Sending command to radar: ", command, 11);

    _serial.write(command, 11);
    _serial.flush();
    return ERR_OK;
}

//``````````````````````````````````````````````````````````` RECEIVE AND PROCESS TARGET LIST ```````````````````````````````````````````````````````````//
iSYSResult_t iSYS4001::receiveTargetListResponse(iSYSTargetList_t *pTargetList, uint32_t timeout, uint8_t bitrate)
{
    uint32_t startTime = millis();
    std::vector<uint8_t> buffer;

    // Determine header length and index where number-of-targets lives
    const uint8_t headerLen = (bitrate == 32) ? 6 : 9;  // 0-based: need 6 bytes to read index 5, 9 for index 8
    const uint8_t countIndex = (bitrate == 32) ? 5 : 8; // 6th index for 32-bit, 9th index for 16-bit
    const uint8_t bytesPerTgt = (bitrate == 32) ? 14 : 7;

    // Read header first
    while ((millis() - startTime) < timeout && buffer.size() < headerLen)
    {
        if (_serial.available())
        {
            buffer.push_back(_serial.read());
        }
    }
    if (buffer.size() < headerLen)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    uint8_t nrOfTargets = buffer[countIndex];
    if (nrOfTargets > MAX_TARGETS && nrOfTargets != 0xFF)
    {
        return ERR_COMMAND_MAX_DATA_OVERFLOW;
    }

    const uint16_t expectedLength = headerLen + (bytesPerTgt * nrOfTargets) + 2;
    buffer.reserve(expectedLength);

    while ((millis() - startTime) < timeout && buffer.size() < expectedLength)
    {
        if (_serial.available())
        {
            buffer.push_back(_serial.read());
        }
    }

    if (buffer.size() != expectedLength)
    {
        return ERR_FRAME_INCOMPLETE;
    }

    if (buffer.back() != 0x16)
    {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }

    debugPrintHexFrame("Received response from radar: ", buffer.data(), buffer.size());

    return decodeTargetFrame(buffer.data(), buffer.size(), bitrate, pTargetList);
}

//``````````````````````````````````````````````````````````` DECODE TARGET FRAME ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::decodeTargetFrame(uint8_t *frame_array, uint16_t nrOfElements, uint8_t bitrate, iSYSTargetList_t *targetList)
{
    uint16_t ui16_fc;
    uint8_t output_number;
    uint8_t nrOfTargets;
    uint8_t *pData;
    int16_t tmp;
    uint8_t i;

    if (frame_array[0] == 0x68)
    {
        ui16_fc = 6;
    }
    else
    {
        ui16_fc = 3;
    }

    output_number = (uint16_t)(frame_array[ui16_fc + 1] & 0x00ff);
    nrOfTargets = (uint16_t)(frame_array[ui16_fc + 2] & 0x00ff);
    pData = &frame_array[ui16_fc + 3];

    if (frame_array[nrOfElements - 1] != 0x16)
    {
        return ERR_COMMAND_NO_VALID_FRAME_FOUND;
    }

    if ((nrOfTargets > MAX_TARGETS) && (nrOfTargets != 0xff))
    {
        return ERR_COMMAND_FAILURE;
    }

    if (nrOfTargets != 0xff)
    { // 0xff  clipping
        for (i = 0; i < MAX_TARGETS; i++)
        {
            targetList->targets[i].angle = 0;
            targetList->targets[i].range = 0;
            targetList->targets[i].signal = 0;
            targetList->targets[i].velocity = 0;
        }

        targetList->nrOfTargets = nrOfTargets;
        targetList->clippingFlag = 0;
        targetList->outputNumber = output_number;

        if (bitrate == 32)
        {
            int tmp32;

            for (i = 0; i < nrOfTargets; i++)
            {
                tmp = (((*pData++) & 0x00ff) << 8);
                tmp |= ((*pData++) & 0x00ff);
                targetList->targets[i].signal = (float)(tmp * 0.01f);
                tmp32 = (((*pData++) & 0x000000ff) << 24);
                tmp32 |= (((*pData++) & 0x000000ff) << 16);
                tmp32 |= (((*pData++) & 0x000000ff) << 8);
                tmp32 |= ((*pData++) & 0x000000ff);
                targetList->targets[i].velocity = (float)tmp32 * 0.001f;
                tmp32 = (((*pData++) & 0x000000ff) << 24);
                tmp32 |= (((*pData++) & 0x000000ff) << 16);
                tmp32 |= (((*pData++) & 0x000000ff) << 8);
                tmp32 |= ((*pData++) & 0x000000ff);
                targetList->targets[i].range = (float)tmp32 * 1E-6f;
                tmp32 = (((*pData++) & 0x000000ff) << 24);
                tmp32 |= (((*pData++) & 0x000000ff) << 16);
                tmp32 |= (((*pData++) & 0x000000ff) << 8);
                tmp32 |= ((*pData++) & 0x000000ff);
                targetList->targets[i].angle = (float)tmp32 * 0.01f;
            }
        }

        if (bitrate == 16)
        {
            for (i = 0; i < nrOfTargets; i++)
            {
                targetList->targets[i].signal = (float)((*pData++) & 0x00ff);
                tmp = (((*pData++) & 0x00ff) << 8);
                tmp |= ((*pData++) & 0x00ff);
                targetList->targets[i].velocity = (float)tmp * 0.01f;
                tmp = (((*pData++) & 0x00ff) << 8);
                tmp |= ((*pData++) & 0x00ff);
                targetList->targets[i].range = (float)tmp * 0.01f;
                tmp = (((*pData++) & 0x00ff) << 8);
                tmp |= ((*pData++) & 0x00ff);
                targetList->targets[i].angle = (float)tmp * 0.01f;
            }
        }
    }
    else
    {
        targetList->clippingFlag = 1;
    }
    if (nrOfTargets == MAX_TARGETS)
    {
        targetList->error.iSYSTargetListError = TARGET_LIST_FULL;
    }
    else
    {
        targetList->error.iSYSTargetListError = TARGET_LIST_OK;
    }
    return ERR_OK;
}

/***************************************************************
 *  SET/GET RANGE MIN/MAX FUNCTIONS
 ***************************************************************/
/**
 * Overview
 *  - Configure the minimum and maximum detection range for a selected output.
 *  - Values are provided in meters. The device expects fixed-point values
 *    scaled by 0.1 m internally (handled by the library).
 *
 * Functions
 *  - iSYS_setOutputRangeMin(output, rangeMeters, destAddress, timeoutMs)
 *  - iSYS_setOutputRangeMax(output, rangeMeters, destAddress, timeoutMs)
 *  - iSYS_getOutputRangeMin(output, float* outRangeMeters, destAddress, timeoutMs)
 *  - iSYS_getOutputRangeMax(output, float* outRangeMeters, destAddress, timeoutMs)
 *
 * Parameters
 *  - output:        ISYS_OUTPUT_1..ISYS_OUTPUT_3
 *  - rangeMeters:   0..150 (min) / 0.1..150 (max) depending on function
 *  - destAddress:   device UART address (e.g. 0x80)
 *  - timeoutMs:     max wait for response, recommended >= 100 ms (300 ms used in examples)
 *  - outRangeMeters: pointer to receive the decoded range (for getters)
 *
 * Return (iSYSResult_t)
 *  - ERR_OK on success
 *  - ERR_OUTPUT_OUT_OF_RANGE if output not in [1..3]
 *  - ERR_PARAMETER_OUT_OF_RANGE if range outside permitted limits
 *  - ERR_TIMEOUT if timeoutMs == 0
 *  - Frame/communication errors: ERR_COMMAND_NO_DATA_RECEIVED, ERR_COMMAND_RX_FRAME_LENGTH,
 *    ERR_COMMAND_RX_FRAME_DAMAGED, ERR_INVALID_CHECKSUM
 *  - ERR_NULL_POINTER if getter output pointer is null
 *
 * Notes
 *  - Internally the library converts meters to 0.1 m fixed point by multiplying by 10
 *    for the SET commands and converts back for the GET commands.
 *  - It is recommended to save application settings after changing ranges using
 *    saveApplicationSettings(destAddress, timeoutMs).
 *
 * Usage example
 *  iSYSResult_t r;
 *  r = radar.iSYS_setOutputRangeMin(ISYS_OUTPUT_1, 0,    0x80, 300);
 *  r = radar.iSYS_setOutputRangeMax(ISYS_OUTPUT_1, 150,  0x80, 300);
 *  if (r == ERR_OK) {
 *      float minM = 0, maxM = 0;
 *      radar.iSYS_getOutputRangeMin(ISYS_OUTPUT_1, &minM, 0x80, 300);
 *      radar.iSYS_getOutputRangeMax(ISYS_OUTPUT_1, &maxM, 0x80, 300);
 *  }
 */

//``````````````````````````````````````````````````````````` SET RANGE MIN FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::iSYS_setOutputRangeMin(iSYSOutputNumber_t outputnumber, uint16_t range, uint8_t destAddress, uint32_t timeout)
{

    if (outputnumber < ISYS_OUTPUT_1 || outputnumber > ISYS_OUTPUT_3)
    {
        return ERR_OUTPUT_OUT_OF_RANGE;
    }

    if (range < 0.0 || range >= 150)
    {
        return ERR_PARAMETER_OUT_OF_RANGE;
    }

    if (timeout == 0)
    {
        return ERR_TIMEOUT;
    }

    uint8_t command[13];
    uint8_t index = 0;
    uint16_t scaledRange = range * 10;
    uint8_t minHighByte = (scaledRange >> 8) & 0xFF;
    uint8_t minLowByte = scaledRange & 0xFF;

    // Build command frame
    command[index++] = 0x68;
    command[index++] = 0x07;
    command[index++] = 0x07;
    command[index++] = 0x68;
    command[index++] = destAddress;
    command[index++] = 0x01;
    command[index++] = 0xD5;
    command[index++] = outputnumber;
    command[index++] = 0x08;
    command[index++] = minHighByte;
    command[index++] = minLowByte;

    uint8_t fcs = calculateFCS(command, 4, 10);

    command[11] = fcs;
    command[12] = 0x16;

    debugPrintHexFrame("", command, 13);

    size_t bytesWritten = _serial.write(command, 13);
    _serial.flush();

    // Response Buffer
    uint8_t response[9];
    size_t minIndex = 0;
    uint32_t startTime = millis();

    while ((millis() - startTime) < timeout && minIndex < sizeof(response))
    {
        if (_serial.available())
        {
            response[minIndex++] = _serial.read();

            if (response[minIndex - 1] == 0x16)
                break;
        }
    }

    debugPrintHexFrame("", response, minIndex);

    if (minIndex == 0)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    if (minIndex < 9)
    {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }

    if (response[0] != 0x68 || response[1] != 0x03 || response[2] != 0x03 ||
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress ||
        response[6] != 0xD5 || response[8] != 0x16)
    {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }

    if (minIndex == 9)
    {
        uint8_t expectedFCS = calculateFCS(response, 4, 6);
        if (response[7] != expectedFCS)
        {
            return ERR_INVALID_CHECKSUM;
        }
    }
    return ERR_OK;
}

//``````````````````````````````````````````````````````````` SET RANGE MAX FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::iSYS_setOutputRangeMax(iSYSOutputNumber_t outputnumber, uint16_t range, uint8_t destAddress, uint32_t timeout)
{
    if (outputnumber < ISYS_OUTPUT_1 || outputnumber > ISYS_OUTPUT_3)
    {
        return ERR_OUTPUT_OUT_OF_RANGE;
    }

    if (range < 0.1 || range > 150.0)
    {
        return ERR_PARAMETER_OUT_OF_RANGE;
    }

    if (timeout == 0)
    {
        return ERR_TIMEOUT;
    }

    uint8_t command[13];
    uint8_t index = 0;
    uint16_t scaledRange = range * 10;
    uint8_t maxHighByte = (scaledRange >> 8) & 0xFF;
    uint8_t maxLowByte = scaledRange & 0xFF;

    command[index++] = 0x68;
    command[index++] = 0x07;
    command[index++] = 0x07;
    command[index++] = 0x68;
    command[index++] = destAddress;
    command[index++] = 0x01;
    command[index++] = 0xD5;
    command[index++] = outputnumber;
    command[index++] = 0x09;
    command[index++] = maxHighByte;
    command[index++] = maxLowByte;

    uint8_t fcs = calculateFCS(command, 4, 10);

    command[index++] = fcs;
    command[index++] = 0x16;

    debugPrintHexFrame("", command, 13);

    size_t bytesWritten = _serial.write(command, 13);
    _serial.flush();

    uint8_t response[9];
    uint32_t startTime = millis();
    size_t maxIndex = 0;

    while ((millis() - startTime) < timeout && maxIndex < 9)
    {
        if (_serial.available())
        {
            response[maxIndex++] = _serial.read();
            if (response[maxIndex - 1] == 0x16)
                break;
        }
    }

    debugPrintHexFrame("", response, maxIndex);

    if (maxIndex == 0)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    if (maxIndex < 9)
    {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }

    if (response[0] != 0x68 || response[1] != 0x03 || response[2] != 0x03 ||
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress ||
        response[6] != 0xD5 || response[8] != 0x16)
    {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }

    if (maxIndex >= 9)
    {
        uint8_t expectedFCS = calculateFCS(response, 4, 6);
        if (response[7] != expectedFCS)
        {
            return ERR_INVALID_CHECKSUM;
        }
    }

    return ERR_OK;
}

//``````````````````````````````````````````````````````````` GET RANGE MIN FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::iSYS_getOutputRangeMin(iSYSOutputNumber_t outputnumber, float *range, uint8_t destAddress, uint32_t timeout)
{
    if (range == NULL)
    {
        return ERR_NULL_POINTER;
    }

    if (outputnumber < ISYS_OUTPUT_1 || outputnumber > ISYS_OUTPUT_3)
    {
        return ERR_OUTPUT_OUT_OF_RANGE;
    }

    if (timeout == 0)
    {
        return ERR_TIMEOUT;
    }

    uint8_t command[11];
    uint8_t index = 0;

    command[index++] = 0x68;
    command[index++] = 0x05;
    command[index++] = 0x05;
    command[index++] = 0x68;
    command[index++] = destAddress;
    command[index++] = 0x01;
    command[index++] = 0xD4;
    command[index++] = outputnumber;
    command[index++] = 0x08;

    uint8_t fcs = calculateFCS(command, 4, 8);
    command[index++] = fcs;
    command[index++] = 0x16;

    debugPrintHexFrame("Sending GET Range Min command: ", command, 11);

    _serial.write(command, 11);
    _serial.flush();

    uint8_t response[11];
    uint32_t startTime = millis();
    uint8_t rIdx = 0;
    while ((millis() - startTime) < timeout && rIdx < 11)
    {
        if (_serial.available())
        {
            response[rIdx++] = _serial.read();
            if (response[rIdx - 1] == 0x16)
                break;
        }
    }

    debugPrintHexFrame("Received Range Min response: ", response, rIdx);

    if (rIdx == 0)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    if (rIdx < 11)
    {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }

    if (response[0] != 0x68 || response[1] != 0x05 || response[2] != 0x05 ||
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress ||
        response[6] != 0xD4 || response[10] != 0x16)
    {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }

    uint8_t expectedFCS = calculateFCS(response, 4, 8);
    if (response[9] != expectedFCS)
    {
        return ERR_INVALID_CHECKSUM;
    }

    uint16_t rawRange = (response[7] << 8) | response[8];
    *range = (float)rawRange / 10.0f; // return in m(meters)

    return ERR_OK;
}

//``````````````````````````````````````````````````````````` GET RANGE MAX FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::iSYS_getOutputRangeMax(iSYSOutputNumber_t outputnumber, float *range, uint8_t destAddress, uint32_t timeout)
{
    if (range == NULL)
    {
        return ERR_NULL_POINTER;
    }

    if (outputnumber < ISYS_OUTPUT_1 || outputnumber > ISYS_OUTPUT_3)
    {
        return ERR_OUTPUT_OUT_OF_RANGE;
    }

    if (timeout == 0)
    {
        return ERR_TIMEOUT;
    }

    uint8_t command[11];
    uint8_t index = 0;

    command[index++] = 0x68;
    command[index++] = 0x05;
    command[index++] = 0x05;
    command[index++] = 0x68;
    command[index++] = destAddress;
    command[index++] = 0x01;
    command[index++] = 0xD4;
    command[index++] = outputnumber;
    command[index++] = 0x09;

    uint8_t fcs = calculateFCS(command, 4, 8);
    command[index++] = fcs;
    command[index++] = 0x16;

    debugPrintHexFrame("Sending GET Range Max command: ", command, 11);

    _serial.write(command, 11);
    _serial.flush();

    uint8_t response[11];
    uint32_t startTime = millis();
    uint8_t rIdx = 0;
    while ((millis() - startTime) < timeout && rIdx < 11)
    {
        if (_serial.available())
        {
            response[rIdx++] = _serial.read();
            if (response[rIdx - 1] == 0x16)
                break;
        }
    }

    debugPrintHexFrame("Received Range Max response: ", response, rIdx);

    if (rIdx == 0)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    if (rIdx < 11)
    {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }

    if (response[0] != 0x68 || response[1] != 0x05 || response[2] != 0x05 ||
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress ||
        response[6] != 0xD4 || response[10] != 0x16)
    {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }

    uint8_t expectedFCS = calculateFCS(response, 4, 8);
    if (response[9] != expectedFCS)
    {
        return ERR_INVALID_CHECKSUM;
    }

    uint16_t rawRange = (response[7] << 8) | response[8];
    *range = (float)rawRange / 10.0f; // return in m(meters)

    return ERR_OK;
}

/***************************************************************
 *  SET/GET VELOCITY MIN/MAX FUNCTIONS
 ***************************************************************/
/**
 * Overview
 *  - Configure velocity thresholds for a selected output.
 *  - Setters accept velocity in km/h. The device protocol uses m/s in 0.1 steps;
 *    the library converts km/h -> m/s and applies 0.1 scaling internally.
 *  - Getters return velocity in km/h.
 *
 * Functions
 *  - iSYS_setOutputVelocityMin(output, velocityKmh, destAddress, timeoutMs)
 *  - iSYS_setOutputVelocityMax(output, velocityKmh, destAddress, timeoutMs)
 *  - iSYS_getOutputVelocityMin(output, float* outVelocityKmh, destAddress, timeoutMs)
 *  - iSYS_getOutputVelocityMax(output, float* outVelocityKmh, destAddress, timeoutMs)
 *
 * Parameters
 *  - output:           ISYS_OUTPUT_1..ISYS_OUTPUT_3
 *  - velocityKmh:      Min: 0..250,  Max: 0.5..250 (km/h)
 *  - destAddress:      device UART address (e.g. 0x80)
 *  - timeoutMs:        max wait for response; recommend >=100 ms (300 ms typical)
 *  - outVelocityKmh:   pointer to receive decoded value (for getters)
 *
 * Return (iSYSResult_t)
 *  - ERR_OK on success
 *  - ERR_OUTPUT_OUT_OF_RANGE if output not in [1..3]
 *  - ERR_PARAMETER_OUT_OF_RANGE if velocity outside permitted limits
 *  - ERR_TIMEOUT if timeoutMs == 0
 *  - Frame/communication errors: ERR_COMMAND_NO_DATA_RECEIVED, ERR_COMMAND_RX_FRAME_LENGTH,
 *    ERR_COMMAND_RX_FRAME_DAMAGED, ERR_INVALID_CHECKSUM
 *  - ERR_NULL_POINTER if getter output pointer is null
 *
 * Notes
 *  - Scaling (handled by library):
 *      setters: scaled = round((kmh / 3.6) * 10) → protocol units (0.1 m/s)
 *      getters: kmh = (raw / 10) * 3.6
 *  - Save application settings after changes with saveApplicationSettings().
 *
 * Usage example
 *  iSYSResult_t r;
 *  r = radar.iSYS_setOutputVelocityMin(ISYS_OUTPUT_1, 0,    0x80, 300);
 *  r = radar.iSYS_setOutputVelocityMax(ISYS_OUTPUT_1, 120,  0x80, 300);
 *  if (r == ERR_OK) {
 *      float vmin = 0, vmax = 0;
 *      radar.iSYS_getOutputVelocityMin(ISYS_OUTPUT_1, &vmin, 0x80, 300);
 *      radar.iSYS_getOutputVelocityMax(ISYS_OUTPUT_1, &vmax, 0x80, 300);
 *  }
 */

//``````````````````````````````````````````````````````````` SET VELOCITY MIN FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::iSYS_setOutputVelocityMin(iSYSOutputNumber_t outputnumber, uint16_t velocity, uint8_t destAddress, uint32_t timeout)
{

    if (outputnumber < ISYS_OUTPUT_1 || outputnumber > ISYS_OUTPUT_3)
    {
        return ERR_OUTPUT_OUT_OF_RANGE;
    }

    if (velocity < 0.0 || velocity >= 250.0)
    {
        return ERR_PARAMETER_OUT_OF_RANGE;
    }

    if (timeout == 0)
    {
        return ERR_TIMEOUT;
    }

    uint8_t command[13];
    uint8_t index = 0;
    uint16_t scaledVelocity = (uint16_t)round((velocity / 3.6f) * 10.0f);
    uint8_t minHighByte = (scaledVelocity >> 8) & 0xFF;
    uint8_t minLowByte = scaledVelocity & 0xFF;

    command[index++] = 0x68;
    command[index++] = 0x07;
    command[index++] = 0x07;
    command[index++] = 0x68;
    command[index++] = destAddress;
    command[index++] = 0x01;
    command[index++] = 0xD5;
    command[index++] = outputnumber;
    command[index++] = 0x0C;
    command[index++] = minHighByte;
    command[index++] = minLowByte;

    uint8_t fcs = calculateFCS(command, 4, 10);

    command[11] = fcs;
    command[12] = 0x16;

    debugPrintHexFrame("", command, 13);

    size_t bytesWritten = _serial.write(command, 13);
    _serial.flush();

    uint8_t response[9];
    size_t minIndex = 0;
    uint32_t startTime = millis();

    while ((millis() - startTime) < timeout && minIndex < sizeof(response))
    {
        if (_serial.available())
        {
            response[minIndex++] = _serial.read();

            if (response[minIndex - 1] == 0x16)
                break;
        }
    }

    debugPrintHexFrame("", response, minIndex);

    if (minIndex == 0)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    if (minIndex < 9)
    {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }

    if (response[0] != 0x68 || response[1] != 0x03 || response[2] != 0x03 ||
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress ||
        response[6] != 0xD5 || response[8] != 0x16)
    {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }

    if (minIndex >= 9)
    {
        uint8_t expectedFCS = calculateFCS(response, 4, 6);
        if (response[7] != expectedFCS)
        {
            return ERR_INVALID_CHECKSUM;
        }
    }
    return ERR_OK;
}

//``````````````````````````````````````````````````````````` SET VELOCITY MAX FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::iSYS_setOutputVelocityMax(iSYSOutputNumber_t outputnumber, uint16_t velocity, uint8_t destAddress, uint32_t timeout)
{

    if (outputnumber < ISYS_OUTPUT_1 || outputnumber > ISYS_OUTPUT_3)
    {
        return ERR_OUTPUT_OUT_OF_RANGE;
    }

    if (velocity < 0.5 || velocity > 250.0)
    {
        return ERR_PARAMETER_OUT_OF_RANGE;
    }

    if (timeout == 0)
    {
        return ERR_TIMEOUT;
    }

    uint8_t command[13];
    uint8_t index = 0;
    uint16_t scaledVelocity = (uint16_t)round((velocity / 3.6f) * 10.0f);
    uint8_t maxHighByte = (scaledVelocity >> 8) & 0xFF;
    uint8_t maxLowByte = scaledVelocity & 0xFF;

    command[index++] = 0x68;
    command[index++] = 0x07;
    command[index++] = 0x07;
    command[index++] = 0x68;
    command[index++] = destAddress;
    command[index++] = 0x01;
    command[index++] = 0xD5;
    command[index++] = outputnumber;
    command[index++] = 0x0D;
    command[index++] = maxHighByte;
    command[index++] = maxLowByte;

    // Calculate checksum (sum of bytes 4 to 10)
    uint8_t fcs = calculateFCS(command, 4, 10);

    command[index++] = fcs;
    command[index++] = 0x16;

    debugPrintHexFrame("", command, 13);

    size_t bytesWritten = _serial.write(command, 13);
    _serial.flush();

    uint8_t response[9];
    uint32_t startTime = millis();
    size_t maxIndex = 0;

    while ((millis() - startTime) < timeout && maxIndex < 9)
    {
        if (_serial.available())
        {
            response[maxIndex++] = _serial.read();
            if (response[maxIndex - 1] == 0x16)
                break;
        }
    }

    debugPrintHexFrame("", response, maxIndex);

    if (maxIndex == 0)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    if (maxIndex < 9)
    {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }

    if (response[0] != 0x68 || response[1] != 0x03 || response[2] != 0x03 ||
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress ||
        response[6] != 0xD5 || response[8] != 0x16)
    {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }

    if (maxIndex == 9)
    {
        uint8_t expectedFCS = calculateFCS(response, 4, 6);
        if (response[7] != expectedFCS)
        {
            return ERR_INVALID_CHECKSUM;
        }
    }

    return ERR_OK;
}

//``````````````````````````````````````````````````````````` GET VELOCITY MIN FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::iSYS_getOutputVelocityMin(iSYSOutputNumber_t outputnumber, float *velocity, uint8_t destAddress, uint32_t timeout)
{
    if (velocity == NULL)
        return ERR_NULL_POINTER;

    uint8_t command[11];
    uint8_t index = 0;
    command[index++] = 0x68;
    command[index++] = 0x05;
    command[index++] = 0x05;
    command[index++] = 0x68;
    command[index++] = destAddress;
    command[index++] = 0x01;
    command[index++] = 0xD4;
    command[index++] = outputnumber;
    command[index++] = 0x0C;
    uint8_t fcs = calculateFCS(command, 4, 8);
    command[index++] = fcs;
    command[index++] = 0x16;

    debugPrintHexFrame("Sending GET Velocity min command: ", command, sizeof(command));

    _serial.write(command, sizeof(command));
    _serial.flush();

    uint8_t response[11];
    uint32_t startTime = millis();
    uint8_t rIdx = 0;
    while ((millis() - startTime) < timeout && rIdx < 11)
    {
        if (_serial.available())
        {
            response[rIdx++] = _serial.read();
            if (response[rIdx - 1] == 0x16)
                break;
        }
    }

    debugPrintHexFrame("Received Velocity min response: ", response, rIdx);

    if (rIdx == 0)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    if (rIdx < 11)
    {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }

    if (response[0] != 0x68 || response[1] != 0x05 || response[2] != 0x05 ||
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress ||
        response[6] != 0xD4 || response[10] != 0x16)
    {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }

    if (rIdx == 11)
    {
        uint8_t expectedFCS = calculateFCS(response, 4, 8);
        if (response[9] != expectedFCS)
        {
            return ERR_INVALID_CHECKSUM;
        }
    }

    uint16_t rawvelocity = ((uint16_t)response[7] << 8) | (uint16_t)response[8];
    *velocity = ((float)rawvelocity / 10.0f) * 3.6f; // return in km/h

    return ERR_OK;
}

//``````````````````````````````````````````````````````````` GET VELOCITY MAX FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::iSYS_getOutputVelocityMax(iSYSOutputNumber_t outputnumber, float *velocity, uint8_t destAddress, uint32_t timeout)
{
    if (velocity == NULL)
        return ERR_NULL_POINTER;

    uint8_t command[11];
    uint8_t index = 0;
    command[index++] = 0x68;
    command[index++] = 0x05;
    command[index++] = 0x05;
    command[index++] = 0x68;
    command[index++] = destAddress;
    command[index++] = 0x01;
    command[index++] = 0xD4;
    command[index++] = outputnumber;
    command[index++] = 0x0D;
    uint8_t fcs = calculateFCS(command, 4, 8);
    command[index++] = fcs;
    command[index++] = 0x16;

    debugPrintHexFrame("Sending GET Velocity max command: ", command, sizeof(command));

    _serial.write(command, sizeof(command));
    _serial.flush();

    uint8_t response[11];
    uint32_t startTime = millis();
    uint8_t rIdx = 0;
    while ((millis() - startTime) < timeout && rIdx < 11)
    {
        if (_serial.available())
        {
            response[rIdx++] = _serial.read();
            if (response[rIdx - 1] == 0x16)
                break;
        }
    }

    debugPrintHexFrame("Received Velocity max response: ", response, rIdx);

    if (rIdx == 0)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    if (rIdx < 11)
    {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }

    if (response[0] != 0x68 || response[1] != 0x05 || response[2] != 0x05 ||
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress ||
        response[6] != 0xD4 || response[10] != 0x16)
    {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }

    if (rIdx == 11)
    {
        uint8_t expectedFCS = calculateFCS(response, 4, 8);
        if (response[9] != expectedFCS)
        {
            return ERR_INVALID_CHECKSUM;
        }
    }

    uint16_t rawvelocity = ((uint16_t)response[7] << 8) | (uint16_t)response[8];
    *velocity = ((float)rawvelocity / 10.0f) * 3.6f; // return in km/h

    return ERR_OK;
}

/***************************************************************
 *  SET/GET SIGNAL MIN/MAX FUNCTIONS
 ***************************************************************/
/**
 * Overview
 *  - Configure signal strength thresholds (in dB) for a selected output.
 *  - Setters accept dB values. The protocol uses fixed-point with 0.1 dB steps;
 *    the library multiplies by 10 for transmission and divides by 10 when reading.
 *
 * Functions
 *  - iSYS_setOutputSignalMin(output, signalDb, destAddress, timeoutMs)
 *  - iSYS_setOutputSignalMax(output, signalDb, destAddress, timeoutMs)
 *  - iSYS_getOutputSignalMin(output, float* outSignalDb, destAddress, timeoutMs)
 *  - iSYS_getOutputSignalMax(output, float* outSignalDb, destAddress, timeoutMs)
 *
 * Parameters
 *  - output:        ISYS_OUTPUT_1..ISYS_OUTPUT_3
 *  - signalDb:      Min: 0.0..249.9 dB,  Max: 0.1..250.0 dB
 *  - destAddress:   device UART address (e.g. 0x80)
 *  - timeoutMs:     max wait for response; recommend >=100 ms (300 ms typical)
 *  - outSignalDb:   pointer to receive decoded value (for getters)
 *
 * Return (iSYSResult_t)
 *  - ERR_OK on success
 *  - ERR_OUTPUT_OUT_OF_RANGE if output not in [1..3]
 *  - ERR_PARAMETER_OUT_OF_RANGE if signal outside permitted limits
 *  - ERR_TIMEOUT if timeoutMs == 0
 *  - Frame/communication errors: ERR_COMMAND_NO_DATA_RECEIVED, ERR_COMMAND_RX_FRAME_LENGTH,
 *    ERR_COMMAND_RX_FRAME_DAMAGED, ERR_INVALID_CHECKSUM
 *  - ERR_NULL_POINTER if getter output pointer is null
 *
 * Notes
 *  - Scaling (handled by library): protocol units = dB * 10 (0.1 dB steps).
 *  - Persist changes with saveApplicationSettings(destAddress, timeoutMs) if desired.
 *
 * Usage example
 *  iSYSResult_t r;
 *  r = radar.iSYS_setOutputSignalMin(ISYS_OUTPUT_1, 0.0f,  0x80, 300);
 *  r = radar.iSYS_setOutputSignalMax(ISYS_OUTPUT_1, 50.0f, 0x80, 300);
 *  if (r == ERR_OK) {
 *      float smin = 0, smax = 0;
 *      radar.iSYS_getOutputSignalMin(ISYS_OUTPUT_1, &smin, 0x80, 300);
 *      radar.iSYS_getOutputSignalMax(ISYS_OUTPUT_1, &smax, 0x80, 300);
 *  }
 */

//``````````````````````````````````````````````````````````` SET SIGNAL MIN FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::iSYS_setOutputSignalMin(iSYSOutputNumber_t outputnumber, uint16_t signal, uint8_t destAddress, uint32_t timeout)
{

    if (outputnumber < ISYS_OUTPUT_1 || outputnumber > ISYS_OUTPUT_3)
    {
        return ERR_OUTPUT_OUT_OF_RANGE;
    }

    if (signal < 0.0 || signal > 249.9)
    {
        return ERR_PARAMETER_OUT_OF_RANGE;
    }

    if (timeout == 0)
    {
        return ERR_TIMEOUT;
    }

    uint8_t command[13];
    uint8_t index = 0;
    uint16_t scaledSignal = signal * 10;
    uint8_t minHighByte = (scaledSignal >> 8) & 0xFF;
    uint8_t minLowByte = scaledSignal & 0xFF;

    // Build command frame
    command[index++] = 0x68;
    command[index++] = 0x07;
    command[index++] = 0x07;
    command[index++] = 0x68;
    command[index++] = destAddress;
    command[index++] = 0x01;
    command[index++] = 0xD5;
    command[index++] = outputnumber;
    command[index++] = 0x0A;
    command[index++] = minHighByte;
    command[index++] = minLowByte;

    uint8_t fcs = calculateFCS(command, 4, 10);

    command[11] = fcs;
    command[12] = 0x16;

    for (int i = 0; i < 13; i++)
    {
        Serial.print("0x");
        if (command[i] < 0x10)
            Serial.print("0");
        Serial.print(command[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    size_t bytesWritten = _serial.write(command, 13);
    _serial.flush();

    uint8_t response[9];
    size_t minIndex = 0;
    uint32_t startTime = millis();

    while ((millis() - startTime) < timeout && minIndex < sizeof(response))
    {
        if (_serial.available())
        {
            response[minIndex++] = _serial.read();

            if (response[minIndex - 1] == 0x16)
                break;
        }
    }

    for (int i = 0; i < minIndex; i++)
    {
        Serial.print("0x");
        if (response[i] < 0x10)
            Serial.print("0");
        Serial.print(response[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    if (minIndex == 0)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    if (minIndex < 9)
    {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }

    if (response[0] != 0x68 || response[1] != 0x03 || response[2] != 0x03 ||
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress ||
        response[6] != 0xD5 || response[8] != 0x16)
    {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }

    if (minIndex >= 9)
    {
        uint8_t expectedFCS = calculateFCS(response, 4, 6);
        if (response[7] != expectedFCS)
        {
            return ERR_INVALID_CHECKSUM;
        }
    }
    return ERR_OK;
}

//``````````````````````````````````````````````````````````` SET SIGNAL MAX FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::iSYS_setOutputSignalMax(iSYSOutputNumber_t outputnumber, uint16_t signal, uint8_t destAddress, uint32_t timeout)
{

    if (outputnumber < ISYS_OUTPUT_1 || outputnumber > ISYS_OUTPUT_3)
    {
        return ERR_OUTPUT_OUT_OF_RANGE;
    }

    if (signal < 0.1 || signal > 250.0)
    {
        return ERR_PARAMETER_OUT_OF_RANGE;
    }

    if (timeout == 0)
    {
        return ERR_TIMEOUT;
    }

    uint8_t command[13];
    uint8_t index = 0;
    uint16_t scaledRange = signal * 10;
    uint8_t maxHighByte = (scaledRange >> 8) & 0xFF;
    uint8_t maxLowByte = scaledRange & 0xFF;

    command[index++] = 0x68;
    command[index++] = 0x07;
    command[index++] = 0x07;
    command[index++] = 0x68;
    command[index++] = destAddress;
    command[index++] = 0x01;
    command[index++] = 0xD5;
    command[index++] = outputnumber;
    command[index++] = 0x0B;
    command[index++] = maxHighByte;
    command[index++] = maxLowByte;

    uint8_t fcs = calculateFCS(command, 4, 10);

    command[index++] = fcs;
    command[index++] = 0x16;

    for (int i = 0; i < 13; i++)
    {
        Serial.print("0x");
        if (command[i] < 0x10)
            Serial.print("0");
        Serial.print(command[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    size_t bytesWritten = _serial.write(command, 13);
    _serial.flush();

    uint8_t response[9];
    uint32_t startTime = millis();
    size_t maxIndex = 0;

    while ((millis() - startTime) < timeout && maxIndex < 9)
    {
        if (_serial.available())
        {
            response[maxIndex++] = _serial.read();
            if (response[maxIndex - 1] == 0x16)
                break;
        }
    }

    debugPrintHexFrame("", response, maxIndex);

    if (maxIndex == 0)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    if (maxIndex < 9)
    {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }

    if (response[0] != 0x68 || response[1] != 0x03 || response[2] != 0x03 ||
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress ||
        response[6] != 0xD5 || response[8] != 0x16)
    {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }

    if (maxIndex >= 9)
    {
        uint8_t expectedFCS = calculateFCS(response, 4, 6);
        if (response[7] != expectedFCS)
        {
            return ERR_INVALID_CHECKSUM;
        }
    }

    return ERR_OK;
}

//``````````````````````````````````````````````````````````` GET SIGNAL MIN FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::iSYS_getOutputSignalMin(iSYSOutputNumber_t outputnumber, float *signal, uint8_t destAddress, uint32_t timeout)
{

    if (signal == NULL)
        return ERR_NULL_POINTER;

    uint8_t command[11];
    uint8_t index = 0;
    command[index++] = 0x68;
    command[index++] = 0x05;
    command[index++] = 0x05;
    command[index++] = 0x68;
    command[index++] = destAddress;
    command[index++] = 0x01;
    command[index++] = 0xD4;
    command[index++] = outputnumber;
    command[index++] = 0x0A;
    uint8_t fcs = calculateFCS(command, 4, 8);
    command[index++] = fcs;
    command[index++] = 0x16;

    Serial.print("Sending GET Signal Min command: ");
    for (int i = 0; i < (int)sizeof(command); i++)
    {
        Serial.print("0x");
        if (command[i] < 0x10)
            Serial.print("0");
        Serial.print(command[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    _serial.write(command, sizeof(command));
    _serial.flush();

    uint8_t response[11];
    uint32_t startTime = millis();
    uint8_t rIdx = 0;
    while ((millis() - startTime) < timeout && rIdx < 11)
    {
        if (_serial.available())
        {
            response[rIdx++] = _serial.read();
            if (response[rIdx - 1] == 0x16)
                break;
        }
    }

    Serial.print("Received Signal Min response: ");
    for (int i = 0; i < (int)rIdx; i++)
    {
        Serial.print("0x");
        if (response[i] < 0x10)
            Serial.print("0");
        Serial.print(response[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    if (rIdx == 0)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    if (rIdx < 11)
    {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }

    if (response[0] != 0x68 || response[1] != 0x05 || response[2] != 0x05 ||
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress ||
        response[6] != 0xD4 || response[10] != 0x16)
    {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }

    if (rIdx == 11)
    {
        uint8_t expectedFCS = calculateFCS(response, 4, 8);
        if (response[9] != expectedFCS)
        {
            return ERR_INVALID_CHECKSUM;
        }
    }

    uint16_t rawSignal = ((uint16_t)response[7] << 8) | (uint16_t)response[8];

    *signal = (float)rawSignal / 10.0f; // result in dB

    return ERR_OK;
}

//``````````````````````````````````````````````````````````` GET SIGNAL MAX FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::iSYS_getOutputSignalMax(iSYSOutputNumber_t outputnumber, float *signal, uint8_t destAddress, uint32_t timeout)
{
    if (signal == NULL)
        return ERR_NULL_POINTER;

    uint8_t command[11];
    uint8_t index = 0;
    command[index++] = 0x68;
    command[index++] = 0x05;
    command[index++] = 0x05;
    command[index++] = 0x68;
    command[index++] = destAddress;
    command[index++] = 0x01;
    command[index++] = 0xD4;
    command[index++] = outputnumber;
    command[index++] = 0x0B;
    uint8_t fcs = calculateFCS(command, 4, 8);
    command[index++] = fcs;
    command[index++] = 0x16;

    Serial.print("Sending GET Signal Max command: ");
    for (int i = 0; i < (int)sizeof(command); i++)
    {
        Serial.print("0x");
        if (command[i] < 0x10)
            Serial.print("0");
        Serial.print(command[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    _serial.write(command, sizeof(command));
    _serial.flush();

    uint8_t response[11];
    uint32_t startTime = millis();
    uint8_t rIdx = 0;
    while ((millis() - startTime) < timeout && rIdx < 11)
    {
        if (_serial.available())
        {
            response[rIdx++] = _serial.read();
            if (response[rIdx - 1] == 0x16)
                break;
        }
    }

    Serial.print("Received Signal Max response: ");
    for (int i = 0; i < (int)rIdx; i++)
    {
        Serial.print("0x");
        if (response[i] < 0x10)
            Serial.print("0");
        Serial.print(response[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    if (rIdx == 0)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    if (rIdx < 11)
    {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }

    if (response[0] != 0x68 || response[1] != 0x05 || response[2] != 0x05 ||
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress ||
        response[6] != 0xD4 || response[10] != 0x16)
    {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }

    if (rIdx == 11)
    {
        uint8_t expectedFCS = calculateFCS(response, 4, 8);
        if (response[9] != expectedFCS)
        {
            return ERR_INVALID_CHECKSUM;
        }
    }

    uint16_t rawSignal = ((uint16_t)response[7] << 8) | (uint16_t)response[8];
    *signal = (float)rawSignal / 10.0f; // result in dB

    return ERR_OK;
}

/***************************************************************
 *  SET VELOCITY DIRECTION FUNCTION
 ***************************************************************/
/**
 * Overview
 *  - Configure which target directions are considered for a selected output:
 *      ISYS_TARGET_DIRECTION_APPROACHING, ISYS_TARGET_DIRECTION_RECEDING,
 *      or ISYS_TARGET_DIRECTION_BOTH.
 *  - You can also query the current direction setting.
 *
 * Functions
 *  - iSYS_setOutputDirection(output, direction, destAddress, timeoutMs)
 *  - iSYS_getOutputDirection(output, iSYSDirection_type_t* outDirection, destAddress, timeoutMs)
 *
 * Parameters
 *  - output:       ISYS_OUTPUT_1..ISYS_OUTPUT_3
 *  - direction:    one of iSYSDirection_type_t (APPROACHING, RECEDING, BOTH)
 *  - destAddress:  device UART address (e.g. 0x80)
 *  - timeoutMs:    max wait for response; recommend >=100 ms (300 ms typical)
 *  - outDirection: pointer to receive decoded value (for getter)
 *
 * Return (iSYSResult_t)
 *  - ERR_OK on success
 *  - ERR_OUTPUT_OUT_OF_RANGE if output not in [1..3]
 *  - ERR_TIMEOUT if timeoutMs == 0
 *  - Frame/communication errors: ERR_COMMAND_NO_DATA_RECEIVED, ERR_COMMAND_RX_FRAME_LENGTH,
 *    ERR_COMMAND_RX_FRAME_DAMAGED, ERR_INVALID_CHECKSUM
 *  - ERR_NULL_POINTER if getter output pointer is null
 *
 * Notes
 *  - Setting direction filters which targets are reported by the device for the selected output.
 *  - Persist changes with saveApplicationSettings(destAddress, timeoutMs) if desired.
 *
 * Usage example
 *  iSYSResult_t r;
 *  r = radar.iSYS_setOutputDirection(ISYS_OUTPUT_1, ISYS_TARGET_DIRECTION_BOTH, 0x80, 300);
 *  if (r == ERR_OK) {
 *      iSYSDirection_type_t dir;
 *      radar.iSYS_getOutputDirection(ISYS_OUTPUT_1, &dir, 0x80, 300);
 *  }
 */

//``````````````````````````````````````````````````````````` SET OUTPUT DIRECTION FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::iSYS_setOutputDirection(iSYSOutputNumber_t outputnumber, iSYSDirection_type_t direction, uint8_t destAddress, uint32_t timeout)
{

    if (outputnumber < ISYS_OUTPUT_1 || outputnumber > ISYS_OUTPUT_3)
    {
        return ERR_OUTPUT_OUT_OF_RANGE;
    }

    if (timeout == 0)
    {
        return ERR_TIMEOUT;
    }

    uint8_t command[13];
    uint8_t index = 0;

    command[index++] = 0x68;
    command[index++] = 0x07;
    command[index++] = 0x07;
    command[index++] = 0x68;
    command[index++] = destAddress;
    command[index++] = 0x01;
    command[index++] = 0xD5;
    command[index++] = outputnumber;
    command[index++] = 0x0E;
    command[index++] = 0X00;
    command[index++] = (uint8_t)direction;

    uint8_t fcs = calculateFCS(command, 4, 10);

    command[index++] = fcs;
    command[index++] = 0x16;

    for (int i = 0; i < 13; i++)
    {
        Serial.print("0x");
        if (command[i] < 0x10)
            Serial.print("0");
        Serial.print(command[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    size_t bytesWritten = _serial.write(command, 13);
    _serial.flush();

    uint8_t response[9];
    uint32_t startTime = millis();
    size_t maxIndex = 0;

    while ((millis() - startTime) < timeout && maxIndex < 9)
    {
        if (_serial.available())
        {
            response[maxIndex++] = _serial.read();
            if (response[maxIndex - 1] == 0x16)
                break;
        }
    }

    for (int i = 0; i < maxIndex; i++)
    {
        Serial.print("0x");
        if (response[i] < 0x10)
            Serial.print("0");
        Serial.print(response[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    if (maxIndex == 0)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    if (maxIndex < 9)
    {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }

    if (response[0] != 0x68 || response[1] != 0x03 || response[2] != 0x03 ||
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress ||
        response[6] != 0xD5 || response[8] != 0x16)
    {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }

    if (maxIndex >= 9)
    {
        uint8_t expectedFCS = calculateFCS(response, 4, 6);
        if (response[7] != expectedFCS)
        {
            return ERR_INVALID_CHECKSUM;
        }
    }

    return ERR_OK;
}

//``````````````````````````````````````````````````````````` GET OUTPUT DIRECTION FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::iSYS_getOutputDirection(iSYSOutputNumber_t outputnumber, iSYSDirection_type_t *direction, uint8_t destAddress, uint32_t timeout)

{
    if (direction == NULL)
        return ERR_NULL_POINTER;

    uint8_t command[11];
    uint8_t index = 0;
    command[index++] = 0x68;
    command[index++] = 0x05;
    command[index++] = 0x05;
    command[index++] = 0x68;
    command[index++] = destAddress;
    command[index++] = 0x01;
    command[index++] = 0xD4;
    command[index++] = outputnumber;
    command[index++] = 0x0E;
    uint8_t fcs = calculateFCS(command, 4, 8);
    command[index++] = fcs;
    command[index++] = 0x16;

    debugPrintHexFrame("Sending GET Direction command: ", command, sizeof(command));

    _serial.write(command, sizeof(command));
    _serial.flush();

    uint8_t response[11];
    uint32_t startTime = millis();
    uint8_t rIdx = 0;
    while ((millis() - startTime) < timeout && rIdx < 11)
    {
        if (_serial.available())
        {
            response[rIdx++] = _serial.read();
            if (response[rIdx - 1] == 0x16)
                break;
        }
    }

    debugPrintHexFrame("Received Direction response: ", response, rIdx);

    if (rIdx == 0)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    if (rIdx < 11)
    {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }

    if (response[0] != 0x68 || response[1] != 0x05 || response[2] != 0x05 ||
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress ||
        response[6] != 0xD4 || response[7] != 0x00 || response[10] != 0x16)
    {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }

    *direction = (iSYSDirection_type_t)response[8];

    if (rIdx == 11)
    {
        uint8_t expectedFCS = calculateFCS(response, 4, 8);
        if (response[9] != expectedFCS)
        {
            return ERR_INVALID_CHECKSUM;
        }
    }

    return ERR_OK;
}

/***************************************************************
 *  EEPROM COMMAND FUNCTIONS
 ***************************************************************/
/**
 * Overview
 *  - Persist and manage device configuration via EEPROM sub-functions.
 *  - Supported operations:
 *      ISYS_EEPROM_SET_FACTORY_SETTINGS       (restore factory defaults)
 *      ISYS_EEPROM_SAVE_SENSOR_SETTINGS       (persist sensor-side parameters)
 *      ISYS_EEPROM_SAVE_APPLICATION_SETTINGS  (persist application parameters)
 *      ISYS_EEPROM_SAVE_ALL_SETTINGS          (persist both sensor + application)
 *
 * Public convenience
 *  - setFactorySettings(dest, timeout)
 *  - saveSensorSettings(dest, timeout)
 *  - saveApplicationSettings(dest, timeout)
 *  - saveAllSettings(dest, timeout)
 *    Each calls the internal 2-step flow:
 *      1) sendEEPROMCommandFrame(subFunction, dest)
 *      2) receiveEEPROMAcknowledgement(dest, timeout)
 *
 * Parameters
 *  - dest:       device UART address (e.g. 0x80)
 *  - timeout:    max wait for ack; recommend >=100 ms (300 ms typical)
 *
 * Return (iSYSResult_t)
 *  - ERR_OK on success
 *  - ERR_TIMEOUT if timeout == 0
 *  - ERR_COMMAND_NO_DATA_RECEIVED if no ack before timeout
 *  - ERR_COMMAND_RX_FRAME_LENGTH if ack shorter than expected
 *  - ERR_COMMAND_RX_FRAME_DAMAGED if framing bytes do not match protocol
 *  - ERR_INVALID_CHECKSUM if checksum mismatch in ack
 *
 * Notes
 *  - EEPROM writes take effect immediately after the acknowledgement.
 *  - Use factory reset with care; it reverts device configuration.
 *
 * Usage example
 *  // Save current application parameters
 *  iSYSResult_t r = radar.saveApplicationSettings(0x80, 300);
 *  if (r != ERR_OK) {
 *      Serial.print("EEPROM save failed: ");
 *      Serial.println(r);
 *  }
 */

iSYSResult_t iSYS4001::sendEEPROMCommand(iSYSEEPROMSubFunction_t subFunction, uint8_t destAddress, uint32_t timeout)
{

    iSYSResult_t res = sendEEPROMCommandFrame(subFunction, destAddress);
    if (res != ERR_OK)
    {
        return res;
    }

    res = receiveEEPROMAcknowledgement(destAddress, timeout);
    if (res != ERR_OK)
    {
        return res;
    }

    return ERR_OK;
}

// Convenience function to set factory settings
iSYSResult_t iSYS4001::setFactorySettings(uint8_t destAddress, uint32_t timeout)
{
    return sendEEPROMCommand(ISYS_EEPROM_SET_FACTORY_SETTINGS, destAddress, timeout);
}

// Convenience function to save sensor settings
iSYSResult_t iSYS4001::saveSensorSettings(uint8_t destAddress, uint32_t timeout)
{
    return sendEEPROMCommand(ISYS_EEPROM_SAVE_SENSOR_SETTINGS, destAddress, timeout);
}

// Convenience function to save application settings
iSYSResult_t iSYS4001::saveApplicationSettings(uint8_t destAddress, uint32_t timeout)
{
    return sendEEPROMCommand(ISYS_EEPROM_SAVE_APPLICATION_SETTINGS, destAddress, timeout);
}

// Convenience function to save all settings (sensor + application)
iSYSResult_t iSYS4001::saveAllSettings(uint8_t destAddress, uint32_t timeout)
{
    return sendEEPROMCommand(ISYS_EEPROM_SAVE_ALL_SETTINGS, destAddress, timeout);
}

//``````````````````````````````````````````````````````````` SEND EEPROM COMMAND FRAME FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::sendEEPROMCommandFrame(iSYSEEPROMSubFunction_t subFunction, uint8_t destAddress)
{

    uint8_t command[10];
    uint8_t index = 0;

    command[index++] = 0x68;
    command[index++] = 0x04;
    command[index++] = 0x04;
    command[index++] = 0x68;
    command[index++] = destAddress;
    command[index++] = 0x01;
    command[index++] = 0xDF;
    command[index++] = subFunction;

    uint8_t fcs = calculateFCS(command, 4, 7);
    command[index++] = fcs;
    command[index++] = 0x16;

    debugPrintHexFrame("Sending EEPROM command to radar: ", command, 10);

    _serial.write(command, 10);
    _serial.flush();
    return ERR_OK;
}
//``````````````````````````````````````````````````````````` RECEIVE EEPROM ACKNOWLEDGEMENT FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::receiveEEPROMAcknowledgement(uint8_t destAddress, uint32_t timeout)
{
    uint32_t startTime = millis();
    uint8_t response[9];
    uint16_t maxIndex = 0;

    while ((millis() - startTime) < timeout && maxIndex < 9)
    {
        if (_serial.available())
        {
            response[maxIndex++] = _serial.read();
            if (response[maxIndex - 1] == 0x16)
                break;
        }
    }

    for (int i = 0; i < maxIndex; i++)
    {
        Serial.print("0x");
        if (response[i] < 0x10)
            Serial.print("0");
        Serial.print(response[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    if (maxIndex == 0)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    if (maxIndex < 9)
    {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }

    if (response[0] != 0x68 || response[1] != 0x03 || response[2] != 0x03 ||
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress ||
        response[6] != 0xDF || response[8] != 0x16)
    {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }

    if (maxIndex >= 9)
    {
        uint8_t expectedFCS = calculateFCS(response, 4, 6);
        if (response[7] != expectedFCS)
        {
            return ERR_INVALID_CHECKSUM;
        }
    }

    return ERR_OK;
}

/***************************************************************
 *  CALCULATE CHECKSUM FUNCTION
 ***************************************************************/
/**
 * Overview
 *  - Compute the Frame Check Sequence (FCS) used by the iSYS-4001 protocol.
 *  - The FCS is the unsigned 8-bit sum of bytes in an inclusive range.
 *
 * Prototype
 *  - uint8_t calculateFCS(const uint8_t* data, uint8_t startIndex, uint8_t endIndex)
 *
 * Parameters
 *  - data:       pointer to the frame bytes
 *  - startIndex: first index to include in the sum (inclusive)
 *  - endIndex:   last index to include in the sum (inclusive)
 *
 * Return
 *  - 8-bit sum of all bytes in [startIndex, endIndex]
 *
 * Notes
 *  - Callers must ensure indices are within bounds of the provided buffer.
 *  - This helper is used when building frames (to append FCS) and when
 *    validating responses (to compare against device-provided FCS).
 *
 * Usage example
 *  uint8_t fcs = calculateFCS(cmd, 4, 10);
 *  cmd[11] = fcs; // append before 0x16 end-of-frame
 */

uint8_t iSYS4001::calculateFCS(const uint8_t *data, uint8_t startIndex, uint8_t endIndex)
{
    uint8_t fcs = 0;
    for (uint8_t i = startIndex; i <= endIndex; i++)
    {
        fcs = (uint8_t)(fcs + data[i]);
    }
    return fcs;
}

/***************************************************************
 *  DEVICE ADDRESS FUNCTIONS
 ***************************************************************/
/**
 * Overview
 *  - Manage the radar's device address on the UART bus.
 *  - You can set a new address and query the current address.
 *
 * Functions
 *  - iSYS_setDeviceAddress(newAddr, destAddress, timeoutMs)
 *  - iSYS_getDeviceAddress(uint8_t* outAddr, destAddress, timeoutMs)
 *
 * Parameters
 *  - newAddr:     the address to set on the device (for setter)
 *  - destAddress: current known address of the device (0x00 can be used if unknown)
 *  - timeoutMs:   max wait for acknowledgement/response; recommend >=100 ms (300 ms typical)
 *  - outAddr:     pointer to receive current address (for getter)
 *
 * Return (iSYSResult_t)
 *  - ERR_OK on success
 *  - ERR_NULL_POINTER if outAddr is null (getter)
 *  - ERR_COMMAND_NO_DATA_RECEIVED if no response before timeout
 *  - ERR_COMMAND_RX_FRAME_LENGTH if response length is shorter than expected
 *  - ERR_COMMAND_RX_FRAME_DAMAGED if framing does not match protocol
 *  - ERR_INVALID_CHECKSUM if checksum mismatch
 *  - ERR_COMMAND_MAX_DATA_OVERFLOW if internal buffer would overflow
 *
 * Notes
 *  - After setting a new address, subsequent communications must use the new address.
 *  - Consider persisting configuration with EEPROM commands if required by your workflow.
 *
 * Usage example
 *  // Set address to 0x81, communicating to current device at 0x80
 *  iSYSResult_t r = radar.iSYS_setDeviceAddress(0x81, 0x80, 300);
 *  if (r == ERR_OK) {
 *      uint8_t addr = 0;
 *      radar.iSYS_getDeviceAddress(&addr, 0x81, 300);
 *  }
 */
//``````````````````````````````````````````````````````````` SET DEVICE ADDRESS FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::iSYS_setDeviceAddress(uint8_t deviceaddress, uint8_t destAddress, uint32_t timeout)
{

    uint8_t command[13];
    uint8_t index = 0;

    command[index++] = 0x68;
    command[index++] = 0x07;
    command[index++] = 0x07;
    command[index++] = 0x68;
    command[index++] = destAddress;
    command[index++] = 0x01;
    command[index++] = 0xD3;
    command[index++] = 0x00;
    command[index++] = 0x01;
    command[index++] = 0x00;
    command[index++] = deviceaddress;

    uint8_t fcs = calculateFCS(command, 4, 10);
    command[index++] = fcs;
    command[index++] = 0x16;

    debugPrintHexFrame("Sending SET address command: ", command, sizeof(command));

    _serial.write(command, sizeof(command));
    _serial.flush();

    uint32_t startTime = millis();
    uint8_t buffer[32];
    uint8_t count = 0;
    while ((millis() - startTime) < timeout)
    {
        if (_serial.available())
        {
            uint8_t b = _serial.read();
            if (count < sizeof(buffer))
                buffer[count++] = b;
            else
                return ERR_COMMAND_MAX_DATA_OVERFLOW;
            if (b == 0x16 && count >= 9)
            {
                debugPrintHexFrame("Received SET address ack: ", buffer, count);

                if (count == 9 && buffer[0] == 0x68 && buffer[1] == 0x03 && buffer[2] == 0x03 &&
                    buffer[3] == 0x68 && buffer[4] == 0x01 && buffer[5] == deviceaddress &&
                    buffer[6] == 0xD3 && buffer[8] == 0x16)
                {
                    uint8_t calc = calculateFCS(buffer, 4, 6);
                    if (calc == buffer[7])
                        return ERR_OK;
                    return ERR_COMMAND_RX_FRAME_DAMAGED;
                }
                return ERR_COMMAND_RX_FRAME_DAMAGED;
            }
        }
    }

    return ERR_COMMAND_NO_DATA_RECEIVED;
}

//``````````````````````````````````````````````````````````` GET DEVICE ADDRESS FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::iSYS_getDeviceAddress(uint8_t *deviceaddress, uint8_t destAddress, uint32_t timeout)
{
    if (deviceaddress == NULL)
        return ERR_NULL_POINTER;

    uint8_t command[11];
    uint8_t index = 0;
    command[index++] = 0x68;
    command[index++] = 0x05;
    command[index++] = 0x05;
    command[index++] = 0x68;
    command[index++] = 0x00;
    command[index++] = 0x01;
    command[index++] = 0xD2;
    command[index++] = 0x00;
    command[index++] = 0x01;
    uint8_t fcs = calculateFCS(command, 4, 8);
    command[index++] = fcs;
    command[index++] = 0x16;

    debugPrintHexFrame("Sending GET address command: ", command, sizeof(command));

    _serial.write(command, sizeof(command));
    _serial.flush();

    uint32_t startTime = millis();
    uint8_t buffer[32];
    uint8_t count = 0;
    while ((millis() - startTime) < timeout)
    {
        if (_serial.available())
        {
            uint8_t b = _serial.read();
            if (count < sizeof(buffer))
                buffer[count++] = b;
            else
                return ERR_COMMAND_MAX_DATA_OVERFLOW;
            if (b == 0x16 && count >= 11)
            {

                debugPrintHexFrame("Received GET address response: ", buffer, count);

                if (count == 11 && buffer[0] == 0x68 && buffer[1] == 0x05 && buffer[2] == 0x05 &&
                    buffer[3] == 0x68 && buffer[6] == 0xD2 && buffer[10] == 0x16)
                {
                    uint8_t calc = calculateFCS(buffer, 4, 8);
                    if (calc == buffer[9])
                    {
                        *deviceaddress = buffer[8];
                        return ERR_OK;
                    }
                    return ERR_COMMAND_RX_FRAME_DAMAGED;
                }
                return ERR_COMMAND_RX_FRAME_DAMAGED;
            }
        }
    }

    return ERR_COMMAND_NO_DATA_RECEIVED;
}

/***************************************************************
 *  ACQUISITION CONTROL FUNCTIONS
 ***************************************************************/
/**
 * Overview
 *  - Control the measurement cycle of the radar (start/stop acquisition).
 *  - Public functions:
 *      iSYS_startAcquisition(destAddress, timeoutMs)
 *      iSYS_stopAcquisition(destAddress, timeoutMs)
 *    Internally these call:
 *      sendAcquisitionCommand(destAddress, true|false)
 *      receiveAcquisitionAcknowledgement(destAddress, timeoutMs)
 *
 * Parameters
 *  - destAddress: device UART address (e.g. 0x80)
 *  - timeoutMs:   max wait for acknowledgement; recommend >=100 ms (300 ms typical)
 *
 * Return (iSYSResult_t)
 *  - ERR_OK on success
 *  - ERR_TIMEOUT if timeoutMs == 0
 *  - ERR_COMMAND_NO_DATA_RECEIVED if no ack before timeout
 *  - ERR_COMMAND_RX_FRAME_LENGTH if ack shorter than expected
 *  - ERR_COMMAND_RX_FRAME_DAMAGED if framing does not match protocol
 *  - ERR_INVALID_CHECKSUM if checksum mismatch
 *  - ERR_COMMAND_MAX_DATA_OVERFLOW if response exceeds internal buffer
 *
 * Notes
 *  - Start acquisition before attempting to stream or repeatedly request data.
 *  - Stopping acquisition may be required before changing certain settings.
 *  - Flush stale UART bytes before issuing commands for improved reliability.
 *
 * Usage example
 *  iSYSResult_t r = radar.iSYS_startAcquisition(0x80, 300);
 *  if (r != ERR_OK) {
 *      Serial.print("Start acquisition failed: ");
 *      Serial.println(r);
 *  }
 *  // ... perform reads ...
 *  r = radar.iSYS_stopAcquisition(0x80, 300);
 */

//``````````````````````````````````````````````````````````` START ACQUISITION FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::iSYS_startAcquisition(uint8_t destAddress, uint32_t timeout)
{
    iSYSResult_t res = sendAcquisitionCommand(destAddress, true);
    if (res != ERR_OK)
    {
        return res;
    }

    res = receiveAcquisitionAcknowledgement(destAddress, timeout);
    if (res != ERR_OK)
    {
        return res;
    }

    return ERR_OK;
}

//``````````````````````````````````````````````````````````` STOP ACQUISITION FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::iSYS_stopAcquisition(uint8_t destAddress, uint32_t timeout)
{

    iSYSResult_t res = sendAcquisitionCommand(destAddress, false);
    if (res != ERR_OK)
    {
        return res;
    }

    res = receiveAcquisitionAcknowledgement(destAddress, timeout);
    if (res != ERR_OK)
    {
        return res;
    }

    return ERR_OK;
}

iSYSResult_t iSYS4001::sendAcquisitionCommand(uint8_t destAddress, bool start)
{
    uint8_t command[11];
    uint8_t index = 0;

    command[index++] = 0x68;
    command[index++] = 0x05;
    command[index++] = 0x05;
    command[index++] = 0x68;
    command[index++] = destAddress;
    command[index++] = 0x01;
    command[index++] = 0xD1;
    command[index++] = 0x00;
    command[index++] = start ? 0x00 : 0x01;

    uint8_t fcs = calculateFCS(command, 4, 8);
    command[index++] = fcs;
    command[index++] = 0x16;

    if (_debugEnabled && _debugStream)
    {
        _debugStream->print(start ? "Starting" : "Stopping");
        _debugStream->print(" acquisition command to radar: ");
    }
    debugPrintHexFrame("", command, 11);

    _serial.write(command, 11);
    _serial.flush();
    return ERR_OK;
}

iSYSResult_t iSYS4001::receiveAcquisitionAcknowledgement(uint8_t destAddress, uint32_t timeout)
{
    if (timeout == 0)
    {
        return ERR_TIMEOUT;
    }

    uint32_t startTime = millis();
    uint8_t buffer[9];
    uint16_t index = 0;
    uint8_t byte;

    while ((millis() - startTime) < timeout)
    {
        if (_serial.available())
        {
            byte = _serial.read();
            buffer[index++] = byte;

            if (byte == 0x16 && index >= 9)
            {

                debugPrintHexFrame("", buffer, index);

                if (index == 9 &&
                    buffer[0] == 0x68 && buffer[1] == 0x03 && buffer[2] == 0x03 &&
                    buffer[3] == 0x68 && buffer[4] == 0x01 && buffer[5] == destAddress &&
                    buffer[6] == 0xD1 && buffer[8] == 0x16)
                {

                    uint8_t expectedFCS = calculateFCS(buffer, 4, 6);
                    if (buffer[7] == expectedFCS)
                    {
                        return ERR_OK;
                    }
                    else
                    {
                        return ERR_INVALID_CHECKSUM;
                    }
                }
                else
                {
                    return ERR_COMMAND_RX_FRAME_DAMAGED;
                }
            }

            if (index > 9)
            {
                return ERR_COMMAND_MAX_DATA_OVERFLOW;
            }
        }
    }

    return ERR_COMMAND_NO_DATA_RECEIVED;
}

/***************************************************************
 *  OUTPUT SINGLE TARGET FILTER FUNCTIONS
 ***************************************************************/
/**
 * Overview
 *  - Configure and query per-output filtering for the single-target output.
 *  - Two dimensions are supported:
 *      1) Output filter type: highest signal, mean, median, min, max
 *      2) Output signal selection: off, velocity radial, range radial
 *
 * Functions
 *  - Set/get filter type:
 *      iSYS_setOutputFilter(...)
 *      iSYS_getOutputFilter(...)
 *  - Set/get signal selection:
 *      iSYS_setOutputSignalFilter(...)
 *      iSYS_getOutputSignalFilter(...)
 *
 * Parameters
 *  - output:     ISYS_OUTPUT_1..ISYS_OUTPUT_3
 *  - filter:     one of iSYSOutput_filter_t (HIGHEST_SIGNAL, MEAN, MEDIAN, MIN, MAX)
 *  - signal:     one of iSYSFilter_signal_t (ISYS_OFF, ISYS_VELOCITY_RADIAL, ISYS_RANGE_RADIAL)
 *  - dest:       device UART address (e.g. 0x80)
 *  - timeout:    max wait for acknowledgement/response; recommend >=100 ms (300 ms typical)
 *  - outFilter/outSignal: pointers to receive decoded values (for getters)
 *
 * Return (iSYSResult_t)
 *  - ERR_OK on success
 *  - ERR_TIMEOUT if timeout == 0
 *  - ERR_NULL_POINTER if output pointer is null (getters)
 *  - Frame/communication errors: ERR_COMMAND_NO_DATA_RECEIVED,
 *    ERR_COMMAND_RX_FRAME_LENGTH, ERR_COMMAND_RX_FRAME_DAMAGED, ERR_INVALID_CHECKSUM,
 *    ERR_COMMAND_MAX_DATA_OVERFLOW
 *
 * Notes
 *  - Set operations are a two-step process: send request frame, then validate ack.
 *  - Persist desired configuration with saveAllSettings(dest, timeout).
 *  - When using HIGHEST_SIGNAL as the filter type, you do not need to call
 *    iSYS_setOutputSignalFilter(); the signal selection is not required.
 *
 * Usage example
 *  iSYSResult_t r;
 *  r = radar.iSYS_setOutputFilter(ISYS_OUTPUT_1, ISYS_OUTPUT_FILTER_MEDIAN, 0x80, 300);
 *  if (r == ERR_OK) {
 *      iSYSOutput_filter_t current;
 *      radar.iSYS_getOutputFilter(ISYS_OUTPUT_1, &current, 0x80, 300);
 *  }
 *  r = radar.iSYS_setOutputSignalFilter(ISYS_OUTPUT_1, ISYS_VELOCITY_RADIAL, 0x80, 300);
 *  if (r == ERR_OK) {
 *      iSYSFilter_signal_t sig;
 *      radar.iSYS_getOutputSignalFilter(ISYS_OUTPUT_1, &sig, 0x80, 300);
 *  }
 */

//``````````````````````````````````````````````````````````` SET OUTPUT FILTER TYPE FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::iSYS_setOutputFilterType(iSYSOutputNumber_t outputnumber, iSYSOutput_filter_t filter, uint8_t destAddress, uint32_t timeout)
{

    iSYSResult_t res = sendSetOutputFilterRequest(outputnumber, filter, destAddress);
    if (res != ERR_OK)
    {
        return res;
    }

    res = receiveSetOutputFilterAcknowledgement(destAddress, timeout);
    if (res != ERR_OK)
    {
        return res;
    }

    return ERR_OK;
}

//``````````````````````````````````````````````````````````` SEND OUTPUT FILTER TYPE REQUEST FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::sendSetOutputFilterRequest(iSYSOutputNumber_t outputnumber, iSYSOutput_filter_t filter, uint8_t destAddress)
{
    uint8_t command[13];
    uint8_t index = 0;

    command[index++] = 0x68;
    command[index++] = 0x07;
    command[index++] = 0x07;
    command[index++] = 0x68;
    command[index++] = destAddress;
    command[index++] = 0x01;
    command[index++] = 0xD5;
    command[index++] = outputnumber;
    command[index++] = 0x15;
    command[index++] = 0x00;
    command[index++] = (uint8_t)filter;

    uint8_t fcs = calculateFCS(command, 4, index - 1);
    command[index++] = fcs;
    command[index++] = 0x16;

    Serial.print("Setting output filter type command to radar: ");
    for (int i = 0; i < 13; i++)
    {
        Serial.print("0x");
        if (command[i] < 0x10)
        {
            Serial.print("0");
        }
        Serial.print(command[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    _serial.write(command, 13);
    _serial.flush();
    return ERR_OK;
}

//``````````````````````````````````````````````````````````` RECEIVE OUTPUT FILTER TYPE ACKNOWLEDGEMENT FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::receiveSetOutputFilterAcknowledgement(uint8_t destAddress, uint32_t timeout)
{
    if (timeout == 0)
    {
        return ERR_TIMEOUT;
    }

    uint32_t startTime = millis();
    uint8_t buffer[9];
    uint16_t index = 0;
    uint8_t byte;

    // Wait for response with timeout protection
    while ((millis() - startTime) < timeout)
    {
        if (_serial.available())
        {
            byte = _serial.read();
            buffer[index++] = byte;

            if (byte == 0x16 && index >= 9)
            {
                debugPrintHexFrame("Received output filter acknowledgement: ", buffer, index);

                if (index == 9 &&
                    buffer[0] == 0x68 && buffer[1] == 0x03 && buffer[2] == 0x03 &&
                    buffer[3] == 0x68 && buffer[4] == 0x01 && buffer[5] == destAddress &&
                    buffer[6] == 0xD5 && buffer[8] == 0x16)
                {
                    uint8_t expectedFCS = calculateFCS(buffer, 4, 6);
                    if (buffer[7] == expectedFCS)
                    {
                        return ERR_OK;
                    }
                    else
                    {
                        return ERR_INVALID_CHECKSUM;
                    }
                }
                else
                {
                    return ERR_COMMAND_RX_FRAME_DAMAGED;
                }
            }

            if (index > 9)
            {
                return ERR_COMMAND_MAX_DATA_OVERFLOW;
            }
        }
    }

    return ERR_COMMAND_NO_DATA_RECEIVED;
}

//``````````````````````````````````````````````````````````` GET OUTPUT FILTER TYPE FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::iSYS_getOutputFilterType(iSYSOutputNumber_t outputnumber, iSYSOutput_filter_t *filter, uint8_t destAddress, uint32_t timeout)
{

    if (filter == nullptr)
    {
        return ERR_NULL_POINTER;
    }

    iSYSResult_t res = sendGetOutputFilterRequest(outputnumber, destAddress);
    if (res != ERR_OK)
    {
        return res;
    }

    res = receiveGetOutputFilterResponse(filter, destAddress, timeout);
    if (res != ERR_OK)
    {
        return res;
    }

    return ERR_OK;
}

//``````````````````````````````````````````````````````````` SEND OUTPUT FILTER TYPE REQUEST FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::sendGetOutputFilterRequest(iSYSOutputNumber_t outputnumber, uint8_t destAddress)
{

    uint8_t command[11];
    uint8_t index = 0;

    command[index++] = 0x68;
    command[index++] = 0x05;
    command[index++] = 0x05;
    command[index++] = 0x68;
    command[index++] = destAddress;
    command[index++] = 0x01;
    command[index++] = 0xD4;
    command[index++] = outputnumber;
    command[index++] = 0x15;

    uint8_t fcs = calculateFCS(command, 4, index - 1);
    command[index++] = fcs;
    command[index++] = 0x16;

    Serial.print("Getting output filter type command to radar: ");
    for (int i = 0; i < 11; i++)
    {
        Serial.print("0x");
        if (command[i] < 0x10)
        {
            Serial.print("0");
        }
        Serial.print(command[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    _serial.write(command, 11);
    _serial.flush();
    return ERR_OK;
}

//``````````````````````````````````````````````````````````` RECEIVE OUTPUT FILTER TYPE RESPONSE FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::receiveGetOutputFilterResponse(iSYSOutput_filter_t *filter, uint8_t destAddress, uint32_t timeout)
{
    if (timeout == 0)
    {
        return ERR_TIMEOUT;
    }

    uint32_t startTime = millis();
    uint8_t buffer[11];
    uint16_t index = 0;
    uint8_t byte;

    while ((millis() - startTime) < timeout)
    {
        if (_serial.available())
        {
            byte = _serial.read();
            buffer[index++] = byte;

            if (byte == 0x16 && index >= 11)
            {

                debugPrintHexFrame("Received output filter response: ", buffer, index);

                if (index == 11 &&
                    buffer[0] == 0x68 && buffer[1] == 0x05 && buffer[2] == 0x05 &&
                    buffer[3] == 0x68 && buffer[4] == 0x01 && buffer[5] == destAddress &&
                    buffer[6] == 0xD4 && buffer[7] == 0x00 && buffer[10] == 0x16)
                {
                    uint8_t expectedFCS = calculateFCS(buffer, 4, 8);
                    if (buffer[9] == expectedFCS)
                    {
                        *filter = (iSYSOutput_filter_t)buffer[8];
                        return ERR_OK;
                    }
                    else
                    {
                        return ERR_INVALID_CHECKSUM;
                    }
                }
                else
                {
                    return ERR_COMMAND_RX_FRAME_DAMAGED;
                }
            }

            if (index > 11)
            {
                return ERR_COMMAND_MAX_DATA_OVERFLOW;
            }
        }
    }

    return ERR_COMMAND_NO_DATA_RECEIVED;
}

//``````````````````````````````````````````````````````````` SET OUTPUT SIGNAL FILTER FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::iSYS_setOutputSignalFilter(iSYSOutputNumber_t outputnumber, iSYSFilter_signal_t signal, uint8_t destAddress, uint32_t timeout)
{

    iSYSResult_t res = sendSetOutputSignalFilterRequest(outputnumber, signal, destAddress);
    if (res != ERR_OK)
    {
        return res;
    }

    res = receiveSetOutputSignalFilterAcknowledgement(destAddress, timeout);
    if (res != ERR_OK)
    {
        return res;
    }

    return ERR_OK;
}

//``````````````````````````````````````````````````````````` SEND OUTPUT SIGNAL FILTER REQUEST FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::sendSetOutputSignalFilterRequest(iSYSOutputNumber_t outputnumber, iSYSFilter_signal_t signal, uint8_t destAddress)
{

    uint8_t command[13];
    uint8_t index = 0;

    command[index++] = 0x68;
    command[index++] = 0x07;
    command[index++] = 0x07;
    command[index++] = 0x68;
    command[index++] = destAddress;
    command[index++] = 0x01;
    command[index++] = 0xD5;
    command[index++] = outputnumber;
    command[index++] = 0x16;
    command[index++] = 0x00;
    command[index++] = (uint8_t)signal;

    uint8_t fcs = calculateFCS(command, 4, index - 1);
    command[index++] = fcs;
    command[index++] = 0x16;

    debugPrintHexFrame("Setting output signal filter command to radar: ", command, 13);

    _serial.write(command, 13);
    _serial.flush();
    return ERR_OK;
}

//``````````````````````````````````````````````````````````` RECEIVE OUTPUT SIGNAL FILTER ACKNOWLEDGEMENT FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::receiveSetOutputSignalFilterAcknowledgement(uint8_t destAddress, uint32_t timeout)
{
    if (timeout == 0)
    {
        return ERR_TIMEOUT;
    }

    uint32_t startTime = millis();
    uint8_t buffer[9];
    uint16_t index = 0;
    uint8_t byte;

    while ((millis() - startTime) < timeout)
    {
        if (_serial.available())
        {
            byte = _serial.read();
            buffer[index++] = byte;

            if (byte == 0x16 && index >= 9)
            {
                debugPrintHexFrame("Received output signal filter acknowledgement: ", buffer, index);

                if (index == 9 &&
                    buffer[0] == 0x68 && buffer[1] == 0x03 && buffer[2] == 0x03 &&
                    buffer[3] == 0x68 && buffer[4] == 0x01 && buffer[5] == destAddress &&
                    buffer[6] == 0xD5 && buffer[8] == 0x16)
                {
                    uint8_t expectedFCS = calculateFCS(buffer, 4, 6);
                    if (buffer[7] == expectedFCS)
                    {
                        return ERR_OK;
                    }
                    else
                    {
                        return ERR_INVALID_CHECKSUM;
                    }
                }
                else
                {
                    return ERR_COMMAND_RX_FRAME_DAMAGED;
                }
            }

            if (index > 9)
            {
                return ERR_COMMAND_MAX_DATA_OVERFLOW;
            }
        }
    }

    return ERR_COMMAND_NO_DATA_RECEIVED;
}

//``````````````````````````````````````````````````````````` GET OUTPUT SIGNAL FILTER FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::iSYS_getOutputSignalFilter(iSYSOutputNumber_t outputnumber, iSYSFilter_signal_t *signal, uint8_t destAddress, uint32_t timeout)
{

    if (signal == nullptr)
    {
        return ERR_NULL_POINTER;
    }

    iSYSResult_t res = sendGetOutputSignalFilterRequest(outputnumber, destAddress);
    if (res != ERR_OK)
    {
        return res;
    }

    res = receiveGetOutputSignalFilterResponse(signal, destAddress, timeout);
    if (res != ERR_OK)
    {
        return res;
    }

    return ERR_OK;
}

//``````````````````````````````````````````````````````````` SEND OUTPUT SIGNAL FILTER REQUEST FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::sendGetOutputSignalFilterRequest(iSYSOutputNumber_t outputnumber, uint8_t destAddress)
{

    uint8_t command[11];
    uint8_t index = 0;

    command[index++] = 0x68;
    command[index++] = 0x05;
    command[index++] = 0x05;
    command[index++] = 0x68;
    command[index++] = destAddress;
    command[index++] = 0x01;
    command[index++] = 0xD4;
    command[index++] = outputnumber;
    command[index++] = 0x16;

    uint8_t fcs = calculateFCS(command, 4, index - 1);
    command[index++] = fcs;
    command[index++] = 0x16;

    Serial.print("Getting output signal filter command to radar: ");
    for (int i = 0; i < 11; i++)
    {
        Serial.print("0x");
        if (command[i] < 0x10)
        {
            Serial.print("0");
        }
        Serial.print(command[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    _serial.write(command, 11);
    _serial.flush();
    return ERR_OK;
}

//``````````````````````````````````````````````````````````` RECEIVE OUTPUT SIGNAL FILTER RESPONSE FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::receiveGetOutputSignalFilterResponse(iSYSFilter_signal_t *signal, uint8_t destAddress, uint32_t timeout)
{
    if (timeout == 0)
    {
        return ERR_TIMEOUT;
    }

    uint32_t startTime = millis();
    uint8_t buffer[11];
    uint16_t index = 0;
    uint8_t byte;

    // Wait for response with timeout protection
    while ((millis() - startTime) < timeout)
    {
        if (_serial.available())
        {
            byte = _serial.read();
            buffer[index++] = byte;

            if (byte == 0x16 && index >= 11)
            {

                debugPrintHexFrame("Received output signal filter response: ", buffer, index);

                if (index == 11 &&
                    buffer[0] == 0x68 && buffer[1] == 0x05 && buffer[2] == 0x05 &&
                    buffer[3] == 0x68 && buffer[4] == 0x01 && buffer[5] == destAddress &&
                    buffer[6] == 0xD4 && buffer[7] == 0x00 && buffer[10] == 0x16)
                {
                    uint8_t expectedFCS = calculateFCS(buffer, 4, 8);
                    if (buffer[9] == expectedFCS)
                    {

                        *signal = (iSYSFilter_signal_t)buffer[8];
                        return ERR_OK;
                    }
                    else
                    {
                        return ERR_INVALID_CHECKSUM;
                    }
                }
                else
                {
                    return ERR_COMMAND_RX_FRAME_DAMAGED;
                }
            }

            if (index > 11)
            {
                return ERR_COMMAND_MAX_DATA_OVERFLOW;
            }
        }
    }

    return ERR_COMMAND_NO_DATA_RECEIVED;
}
