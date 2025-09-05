#include "iSYS4001.h"

iSYS4001::iSYS4001(HardwareSerial &serial, uint32_t baud)
    : _serial(serial), _baud(baud), _debugEnabled(false), _debugStream(nullptr)
{
}

// =============================
// Debug configuration methods
// =============================
void iSYS4001::setDebugEnabled(bool enabled)
{
    _debugEnabled = enabled;
}

void iSYS4001::setDebugStream(Stream &stream)
{
    _debugStream = &stream;
}

void iSYS4001::setDebug(Stream &stream, bool enabled)
{
    _debugStream = &stream;
    _debugEnabled = enabled;
}

// Internal debug helpers
void iSYS4001::debugPrint(const char *msg)
{
    if (_debugEnabled && _debugStream)
    {
        _debugStream->print(msg);
    }
}

void iSYS4001::debugPrintln(const char *msg)
{
    if (_debugEnabled && _debugStream)
    {
        _debugStream->println(msg);
    }
}

void iSYS4001::debugPrintHexFrame(const char *prefix, const uint8_t *data, size_t length)
{
    if (!(_debugEnabled && _debugStream))
        return;
    if (prefix && *prefix)
    {
        _debugStream->print(prefix);
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
}

/***************************************************************
 *  GET TARGET LIST FUNCTIONS
 ***************************************************************/

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
 *  SET RANGE MIN/MAX FUNCTIONS
 ***************************************************************/

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

/***************************************************************
 *  GET RANGE MIN/MAX FUNCTIONS
 ***************************************************************/

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
 *  SET VELOCITY MIN/MAX FUNCTIONS
 ***************************************************************/

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

/***************************************************************
 *  GET VELOCITY MIN/MAX FUNCTIONS
 ***************************************************************/

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
 *  SET SIGNAL MIN/MAX FUNCTIONS
 ***************************************************************/

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

/***************************************************************
 *  GET SIGNAL MIN/MAX FUNCTIONS
 ***************************************************************/

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

// Main function to send EEPROM commands to the radar sensor
// Parameters: subFunction - EEPROM sub-function code (factory settings, save sensor, etc.)
//             destAddress - destination address for the radar sensor
//             timeout - maximum time to wait for response in milliseconds
// Returns: iSYSResult_t - error code indicating success or failure
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

// Function to send EEPROM command frame to the radar sensor
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

// Helper function to calculate Frame Check Sequence (FCS)
// Parameters: data - array containing the frame data
//             startIndex - starting index for FCS calculation
//             endIndex - ending index for FCS calculation (inclusive)
// Returns: uint8_t - calculated FCS value
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

// Function to set a new device address on the sensor
// Parameters: deviceaddress - the new sensor address to set
//             destAddress   - the current address of the sensor (or broadcast if supported)
//             timeout       - maximum time to wait for acknowledgement in milliseconds
// Returns: iSYSResult_t - error code indicating success or failure
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

// Function to request and read the current device address
// Parameters: deviceaddress - pointer where the found address will be stored
//             destAddress   - destination address (use 0x00 for broadcast if unknown)
//             timeout       - maximum time to wait for response in milliseconds
// Returns: iSYSResult_t - error code indicating success or failure
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

// Main function to start data acquisition from the iSYS radar sensor
// Parameters: destAddress - destination address for the radar sensor
//             timeout - maximum time to wait for response in milliseconds
// Returns: iSYSResult_t - error code indicating success or failure
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

// Main function to stop data acquisition from the iSYS radar sensor
// Parameters: destAddress - destination address for the radar sensor
//             timeout - maximum time to wait for response in milliseconds
// Returns: iSYSResult_t - error code indicating success or failure
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

// Function to send acquisition start/stop command frame to the radar sensor
// Parameters: destAddress - destination address for the radar sensor
//             start - true to start acquisition, false to stop acquisition
// Returns: iSYSResult_t - error code (always ERR_OK for this function)
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

// Function to receive and verify acquisition acknowledgement from radar sensor
// Parameters: destAddress - destination address for the radar sensor
//             timeout - maximum time to wait for response in milliseconds
// Returns: iSYSResult_t - error code indicating success or failure
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

// Function to set the single target filter type for a selected output
// Parameters: outputnumber - specifies which output to use (1, 2, or 3)
//             filterType - the filter type to set (highest amplitude, mean, median, min, max)
//             destAddress - destination address for the radar sensor
//             timeout - maximum time to wait for response in milliseconds
// Returns: iSYSResult_t - error code indicating success or failure
iSYSResult_t iSYS4001::iSYS_setOutputFilter(iSYSOutputNumber_t outputnumber, iSYSOutput_filter_t filter, uint8_t destAddress, uint32_t timeout)
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

// Function to send set output filter request command to the radar sensor
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

// Function to receive and verify set output filter acknowledgement from radar sensor
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

// Function to get the single target filter type from a selected output
iSYSResult_t iSYS4001::iSYS_getOutputFilter(iSYSOutputNumber_t outputnumber, iSYSOutput_filter_t *filter, uint8_t destAddress, uint32_t timeout)
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

// Function to send get output filter request command to the radar sensor
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

// Function to receive and decode get output filter response from radar sensor
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

// Function to set the single target filter signal for a selected output
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

// Function to send set output signal filter request command to the radar sensor
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

// Function to receive and verify set output signal filter acknowledgement from radar sensor
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

// Function to get the single target filter signal from a selected output
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

// Function to send get output signal filter request command to the radar sensor

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

// Function to receive and decode get output signal filter response from radar sensor

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
