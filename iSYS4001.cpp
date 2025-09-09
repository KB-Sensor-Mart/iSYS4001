#include "iSYS4001.h"

/**
 * @brief Constructor for iSYS4001 radar sensor interface
 *
 * Initializes the iSYS4001 radar sensor communication interface with the specified
 * serial port and baud rate. Sets up internal state variables for debug logging
 * and communication parameters.
 *
 * @param serial Reference to the HardwareSerial object for UART communication
 * @param baud Baud rate for serial communication (115200 for iSYS-4001)
 *
 * @note The constructor does not establish communication with the radar device.
 *       Use the public API functions to configure and communicate with the device.
 *
 * @example
 *   // Initialize with Serial2 at 115200 baud
 *   iSYS4001 radar(Serial2, 115200);
 *
 */
iSYS4001::iSYS4001(HardwareSerial &serial, uint32_t baud)
    : _serial(serial), _baud(baud), _debugEnabled(false), _debugStream(nullptr)
{
}

/***************************************************************
 *  DEBUG CONFIGURATION FUNCTIONS
 ***************************************************************/
/**
 * @brief Enable or disable debug output for serial monitoring
 *
 * Sets the debug output stream and enables or disables debug messages for
 * command/response frames sent to and received from the iSYS-4001 radar device.
 *
 * @param stream Reference to a Stream object (e.g., Serial) to send debug messages
 * @param enabled Boolean flag to enable (true) or disable (false) debug output
 *
 * @return iSYSResult_t ERR_OK if the debug stream is successfully set, or
 *         ERR_NULL_POINTER if the provided stream is invalid
 *
 * @note When debug is enabled, internal frame transmissions and receptions
 *       will be printed to the provided stream for monitoring purposes.
 *
 * @example
 *   // Enable debug output on Serial
 *   iSYSResult_t res = radar.setDebug(Serial, true);
 *   if (res == ERR_OK) {
 *       Serial.println("Debug enabled");
 *   } else {
 *       Serial.println("Failed to enable debug output");
 *   }
 */
iSYSResult_t iSYS4001::setDebug(Stream &stream, bool enabled)
{
    _debugStream = &stream;
    _debugEnabled = enabled;
    return (_debugStream != nullptr) ? ERR_OK : ERR_NULL_POINTER;
}

/**
 * @brief Internal debug helper function for printing text messages
 *
 * Prints debug messages to the configured debug stream if debug logging is enabled.
 * This is an internal helper function used throughout the library for debug output.
 * The function automatically checks if debug is enabled and a valid stream is configured.
 *
 * @param msg Pointer to the message string to print (can be nullptr)
 * @param newline If true, adds a newline after the message; if false, prints without newline
 *
 * @note This function is a no-op if debug is disabled, stream is not configured, or msg is nullptr
 * @note Used internally by the library - not intended for direct user calls
 *
 * @example
 *   // Internal usage within library functions
 *   debugPrint("Starting communication", true);
 *   debugPrint("Value: ", false);
 *   debugPrint("123", true);
 */
void iSYS4001::debugPrint(const char *msg, bool newline)
{
    if (_debugEnabled && _debugStream && msg)
    {
        if (newline)
            _debugStream->println(msg);
        else
            _debugStream->print(msg);
    }
}

/**
 * @brief Internal debug helper function for printing hex data frames
 *
 * Prints binary data as hexadecimal values to the configured debug stream.
 * This is an internal helper function used throughout the library for debugging
 * UART communication frames and protocol data. The function formats each byte
 * as "0xXX " and adds a newline at the end.
 *
 * @param prefix Optional prefix string to print before the hex data (can be nullptr)
 * @param data Pointer to the byte array to print as hex (can be nullptr if length is 0)
 * @param length Number of bytes to print from the data array
 *
 * @note This function is a no-op if debug is disabled, stream is not configured,
 *       or if data is nullptr while length > 0
 * @note Used internally by the library - not intended for direct user calls
 *
 * @example
 *   // Internal usage within library functions
 *   uint8_t frame[] = {0x68, 0x05, 0x05, 0x68, 0x80};
 *   debugPrintHexFrame("Sending command: ", frame, 5);
 *   // Output: "Sending command: 0x68 0x05 0x05 0x68 0x80 "
 */
void iSYS4001::debugPrintHexFrame(const char *prefix, const uint8_t *data, size_t length)
{
    if (!(_debugEnabled && _debugStream) || (data == nullptr && length > 0))
        return;

    if (prefix && *prefix)
        _debugStream->print(prefix);

    for (size_t i = 0; i < length; i++)
    {
        _debugStream->print(data[i] < 0x10 ? "0x0" : "0x");
        _debugStream->print(data[i], HEX);
        _debugStream->print(" ");
    }
    _debugStream->println();
}

/***************************************************************
 *  GET TARGET LIST FUNCTIONS
 ***************************************************************/

/**
 * @brief Retrieve 16-bit target list from radar device
 *
 * Requests and receives a list of detected targets from the iSYS-4001 radar
 * using the 16-bit data format. The function first sends a target list request
 * to the specified output channel, then waits for and parses the response into
 * the provided iSYSTargetList_t structure.
 *
 * @param pTargetList Pointer to iSYSTargetList_t structure that will receive the target list data
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for target list response
 * @param outputnumber Output channel to query (ISYS_OUTPUT_1, ISYS_OUTPUT_2, or ISYS_OUTPUT_3)
 *
 * @return iSYSResult_t ERR_OK on success, or error code for failure conditions:
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response received within timeout
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame shorter than expected
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Response frame invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *         - ERR_OUTPUT_OUT_OF_RANGE: Invalid output channel number
 *
 * @note This function clears the target list structure before filling it.
 * @note Internally, this calls sendTargetListRequest() and receiveTargetListResponse().
 *
 * @example
 *   // Get 16-bit target list from output 1
 *   iSYSTargetList_t targetList;
 *   iSYSResult_t res = radar.getTargetList16(&targetList, 0x80, 300, ISYS_OUTPUT_1);
 *   if (res == ERR_OK) {
 *       Serial.print("Number of targets: ");
 *       Serial.println(targetList.count);
 *   } else {
 *       Serial.print("Failed to get target list, error: ");
 *       Serial.println(res);
 *   }
 */
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

/**
 * @brief Retrieve 32-bit target list from radar device
 *
 * Requests and receives a list of detected targets from the iSYS-4001 radar
 * using the 32-bit data format. The function first sends a target list request
 * to the specified output channel, then waits for and parses the response into
 * the provided iSYSTargetList_t structure.
 *
 * @param pTargetList Pointer to iSYSTargetList_t structure that will receive the target list data
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for target list response
 * @param outputnumber Output channel to query (ISYS_OUTPUT_1, ISYS_OUTPUT_2, or ISYS_OUTPUT_3)
 *
 * @return iSYSResult_t ERR_OK on success, or error code for failure conditions:
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response received within timeout
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame shorter than expected
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Response frame invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *         - ERR_OUTPUT_OUT_OF_RANGE: Invalid output channel number
 *
 * @note This function clears the target list structure before filling it.
 * @note Internally, this calls sendTargetListRequest() and receiveTargetListResponse().
 *
 * @example
 *   // Get 32-bit target list from output 2
 *   iSYSTargetList_t targetList;
 *   iSYSResult_t res = radar.getTargetList32(&targetList, 0x80, 300, ISYS_OUTPUT_2);
 *   if (res == ERR_OK) {
 *       Serial.print("Number of targets: ");
 *       Serial.println(targetList.count);
 *   } else {
 *       Serial.print("Failed to get target list, error: ");
 *       Serial.println(res);
 *   }
 */
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

/**
 * @brief Internal function to send target list request command to radar device
 *
 * Constructs and transmits a command frame to request target list data from
 * the iSYS-4001 radar device. The command specifies the output channel and data
 * precision (16-bit or 32-bit). This is an internal helper function used by
 * getTargetList16() and getTargetList32().
 *
 * @param outputnumber Output channel to request data from (ISYS_OUTPUT_1, ISYS_OUTPUT_2, or ISYS_OUTPUT_3)
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param bitrate Data precision: 16 for 16-bit format, 32 for 32-bit format
 *
 * @return iSYSResult_t ERR_OK on success, or error code if command transmission fails
 *
 * @note This function only sends the command - it does not wait for or process the response
 * @note Used internally by getTargetList16() and getTargetList32() functions
 * @note The command frame includes proper framing, address, and checksum validation
 *
 * @example
 *   // Internal usage - called by getTargetList16()
 *   iSYSResult_t res = sendTargetListRequest(ISYS_OUTPUT_1, 0x80, 16);
 *   if (res != ERR_OK) {
 *       // Handle error
 *   }
 */
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

/**
 * @brief Internal function to receive and process target list response from radar device
 *
 * Waits for and receives the target list response frame from the iSYS-4001 radar device.
 * The function handles the variable-length response based on the number of detected targets
 * and the data precision (16-bit or 32-bit). It validates the frame structure and
 * passes the data to decodeTargetFrame() for processing.
 *
 * @param pTargetList Pointer to target list structure that will receive the decoded data
 * @param timeout Maximum time in milliseconds to wait for the complete response
 * @param bitrate Data precision: 16 for 16-bit format, 32 for 32-bit format
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions:
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response received before timeout
 *         - ERR_FRAME_INCOMPLETE: Response frame was truncated
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Frame structure validation failed
 *         - ERR_COMMAND_MAX_DATA_OVERFLOW: Too many targets in response
 *
 * @note This function handles the variable-length nature of target list responses
 * @note Used internally by getTargetList16() and getTargetList32() functions
 * @note The function first reads the header to determine expected response length
 *
 * @example
 *   // Internal usage - called by getTargetList32()
 *   iSYSTargetList_t targets;
 *   iSYSResult_t res = receiveTargetListResponse(&targets, 300, 32);
 *   if (res != ERR_OK) {
 *       // Handle communication error
 *   }
 */
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

/**
 * @brief Internal function to decode target list frame data into structured format
 *
 * Parses the raw frame data received from the radar device and extracts
 * target information into the iSYSTargetList_t structure. Handles both 16-bit
 * and 32-bit data formats with appropriate scaling and conversion. Validates
 * frame structure and populates target data including signal strength, velocity,
 * range, and angle for each detected target.
 *
 * @param frame_array Pointer to the raw frame data received from the device
 * @param nrOfElements Total number of bytes in the frame array
 * @param bitrate Data precision: 16 for 16-bit format, 32 for 32-bit format
 * @param targetList Pointer to target list structure to populate with decoded data
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions:
 *         - ERR_COMMAND_NO_VALID_FRAME_FOUND: Frame structure validation failed
 *         - ERR_COMMAND_FAILURE: Invalid number of targets in frame
 *
 * @note This function handles the complex frame parsing and data conversion
 * @note Used internally by receiveTargetListResponse() function
 * @note Supports both 16-bit (7 bytes per target) and 32-bit (14 bytes per target) formats
 * @note Automatically handles clipping flag (0xFF) when device data is saturated
 *
 * @example
 *   // Internal usage - called by receiveTargetListResponse()
 *   uint8_t frame[100];
 *   iSYSTargetList_t targets;
 *   iSYSResult_t res = decodeTargetFrame(frame, 50, 32, &targets);
 *   if (res == ERR_OK) {
 *       // targets structure now contains decoded target data
 *   }
 */
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
 * @brief Set the minimum detection range for a specified output channel
 *
 * Configures the minimum detection range threshold for the specified output channel
 * on the iSYS-4001 radar device. The range value is provided in meters and is
 * automatically converted to the device's internal 0.1m fixed-point format.
 *
 * @param outputnumber Output channel to configure (ISYS_OUTPUT_1, ISYS_OUTPUT_2, or ISYS_OUTPUT_3)
 * @param range Minimum range in meters (0 to 149.9)
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for acknowledgement
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions:
 *         - ERR_OUTPUT_OUT_OF_RANGE: Invalid output channel number
 *         - ERR_PARAMETER_OUT_OF_RANGE: Range value outside valid limits
 *         - ERR_TIMEOUT: Invalid timeout parameter (0)
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response from device
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame too short
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Response frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *
 * @note The function automatically converts meters to 0.1m fixed-point format (multiply by 10)
 * @note Integer inputs are recommended; floating-point values should be avoided to ensure consistent processing.
 * @note Changes take effect immediately but should be saved with saveApplicationSettings()
 * @note Recommended timeout is >= 100ms (300ms used in examples)
 *
 * @example
 *   // Set minimum range to 5 meters for output 1
 *   iSYSResult_t res = radar.iSYS_setOutputRangeMin(ISYS_OUTPUT_1, 5, 0x80, 300);
 *   if (res == ERR_OK) {
 *       Serial.println("Minimum range set successfully");
 *   }
 */

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

    debugPrintHexFrame("Sending SET range min command to radar: ", command, 13);

    size_t bytesWritten = _serial.write(command, 13);
    _serial.flush();

    if (bytesWritten != 13)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

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

    debugPrintHexFrame("Received SET range min acknowledgement: ", response, minIndex);

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

/**
 * @brief Set the maximum detection range for a specified output channel
 *
 * Configures the maximum detection range threshold for the specified output channel
 * on the iSYS-4001 radar device. The range value is provided in meters and is
 * automatically converted to the device's internal 0.1m fixed-point format.
 *
 * @param outputnumber Output channel to configure (ISYS_OUTPUT_1, ISYS_OUTPUT_2, or ISYS_OUTPUT_3)
 * @param range Maximum range in meters (0.1 to 150.0)
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for acknowledgement
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions:
 *         - ERR_OUTPUT_OUT_OF_RANGE: Invalid output channel number
 *         - ERR_PARAMETER_OUT_OF_RANGE: Range value outside valid limits
 *         - ERR_TIMEOUT: Invalid timeout parameter (0)
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response from device
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame too short
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Response frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *
 * @note The function automatically converts meters to 0.1m fixed-point format (multiply by 10)
 * @note Integer inputs are recommended; floating-point values should be avoided to ensure consistent processing.
 * @note Changes take effect immediately but should be saved with saveApplicationSettings()
 * @note Recommended timeout is >= 100ms (300ms used in examples)
 *
 * @example
 *   // Set maximum range to 100 meters for output 1
 *   iSYSResult_t res = radar.iSYS_setOutputRangeMax(ISYS_OUTPUT_1, 100, 0x80, 300);
 *   if (res == ERR_OK) {
 *       Serial.println("Maximum range set successfully");
 *   }
 */
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

    debugPrintHexFrame("Sending SET range max command to radar: ", command, 13);

    size_t bytesWritten = _serial.write(command, 13);
    _serial.flush();

    if (bytesWritten != 13)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

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

    debugPrintHexFrame("Received SET range max acknowledgement: ", response, maxIndex);

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

/**
 * @brief Get the current minimum detection range for a specified output channel
 *
 * Retrieves the current minimum detection range threshold for the specified output channel
 * from the iSYS-4001 radar device. The range value is returned in meters after automatic
 * conversion from the device's internal 0.1m fixed-point format.
 *
 * @param outputnumber Output channel to query (ISYS_OUTPUT_1, ISYS_OUTPUT_2, or ISYS_OUTPUT_3)
 * @param range Pointer to float variable that will receive the minimum range in meters
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for response
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions:
 *         - ERR_NULL_POINTER: range pointer is null
 *         - ERR_OUTPUT_OUT_OF_RANGE: Invalid output channel number
 *         - ERR_TIMEOUT: Invalid timeout parameter (0)
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response from device
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame too short
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Response frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *
 * @note The function automatically converts from 0.1m fixed-point format to meters (divide by 10)
 * @note Recommended timeout is >= 100ms (300ms used in examples)
 *
 * @example
 *   // Get current minimum range for output 1
 *   float minRange = 0;
 *   iSYSResult_t res = radar.iSYS_getOutputRangeMin(ISYS_OUTPUT_1, &minRange, 0x80, 300);
 *   if (res == ERR_OK) {
 *       Serial.print("Current minimum range: ");
 *       Serial.print(minRange);
 *       Serial.println(" meters");
 *   }
 */
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

    debugPrintHexFrame("Sending GET Range Min command: ", command, sizeof(command));

    size_t bytesWritten = _serial.write(command, 11);
    _serial.flush();

    if (bytesWritten != sizeof(command))
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

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

/**
 * @brief Get the current maximum detection range for a specified output channel
 *
 * Retrieves the current maximum detection range threshold for the specified output channel
 * from the iSYS-4001 radar device. The range value is returned in meters after automatic
 * conversion from the device's internal 0.1m fixed-point format.
 *
 * @param outputnumber Output channel to query (ISYS_OUTPUT_1, ISYS_OUTPUT_2, or ISYS_OUTPUT_3)
 * @param range Pointer to float variable that will receive the maximum range in meters
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for response
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions:
 *         - ERR_NULL_POINTER: range pointer is null
 *         - ERR_OUTPUT_OUT_OF_RANGE: Invalid output channel number
 *         - ERR_TIMEOUT: Invalid timeout parameter (0)
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response from device
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame too short
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Response frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *
 * @note The function automatically converts from 0.1m fixed-point format to meters (divide by 10)
 * @note Recommended timeout is >= 100ms (300ms used in examples)
 *
 * @example
 *   // Get current maximum range for output 1
 *   float maxRange = 0;
 *   iSYSResult_t res = radar.iSYS_getOutputRangeMax(ISYS_OUTPUT_1, &maxRange, 0x80, 300);
 *   if (res == ERR_OK) {
 *       Serial.print("Current maximum range: ");
 *       Serial.print(maxRange);
 *       Serial.println(" meters");
 *   }
 */
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

    debugPrintHexFrame("Sending GET Range Max command: ", command, sizeof(command));

    size_t bytesWritten = _serial.write(command, 11);
    _serial.flush();

    if (bytesWritten != sizeof(command))
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

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
 * @brief Set the minimum velocity threshold for a specified output channel
 *
 * Configures the minimum velocity threshold for the specified output channel
 * on the iSYS-4001 radar device. The velocity value is provided in km/h and is
 * automatically converted to the device's internal m/s format with 0.1m/s precision.
 *
 * @param outputnumber Output channel to configure (ISYS_OUTPUT_1, ISYS_OUTPUT_2, or ISYS_OUTPUT_3)
 * @param velocity Minimum velocity in km/h (0 to 249.9)
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for acknowledgement
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions:
 *         - ERR_OUTPUT_OUT_OF_RANGE: Invalid output channel number
 *         - ERR_PARAMETER_OUT_OF_RANGE: Velocity value outside valid limits
 *         - ERR_TIMEOUT: Invalid timeout parameter (0)
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response from device
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame too short
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Response frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *
 * @note The function automatically converts km/h to m/s: scaled = round((kmh / 3.6) * 10)
 * @note Changes take effect immediately but should be saved with saveApplicationSettings()
 * @note Recommended timeout is >= 100ms (300ms used in examples)
 *
 * @example
 *   // Set minimum velocity to 5 km/h for output 1
 *   iSYSResult_t res = radar.iSYS_setOutputVelocityMin(ISYS_OUTPUT_1, 5, 0x80, 300);
 *   if (res == ERR_OK) {
 *       Serial.println("Minimum velocity set successfully");
 *   }
 */
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

    debugPrintHexFrame("Sending SET velocity min command to radar: ", command, 13);

    size_t bytesWritten = _serial.write(command, 13);
    _serial.flush();

    if (bytesWritten != 13)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

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

    debugPrintHexFrame("Received SET velocity min acknowledgement: ", response, minIndex);

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

/**
 * @brief Set the maximum velocity threshold for a specified output channel
 *
 * Configures the maximum velocity threshold for the specified output channel
 * on the iSYS-4001 radar device. The velocity value is provided in km/h and is
 * automatically converted to the device's internal m/s format with 0.1m/s precision.
 *
 * @param outputnumber Output channel to configure (ISYS_OUTPUT_1, ISYS_OUTPUT_2, or ISYS_OUTPUT_3)
 * @param velocity Maximum velocity in km/h (0.5 to 250.0)
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for acknowledgement
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions:
 *         - ERR_OUTPUT_OUT_OF_RANGE: Invalid output channel number
 *         - ERR_PARAMETER_OUT_OF_RANGE: Velocity value outside valid limits
 *         - ERR_TIMEOUT: Invalid timeout parameter (0)
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response from device
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame too short
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Response frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *
 * @note The function automatically converts km/h to m/s: scaled = round((kmh / 3.6) * 10)
 * @note Changes take effect immediately but should be saved with saveApplicationSettings()
 * @note Recommended timeout is >= 100ms (300ms used in examples)
 *
 * @example
 *   // Set maximum velocity to 120 km/h for output 1
 *   iSYSResult_t res = radar.iSYS_setOutputVelocityMax(ISYS_OUTPUT_1, 120, 0x80, 300);
 *   if (res == ERR_OK) {
 *       Serial.println("Maximum velocity set successfully");
 *   }
 */
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

    debugPrintHexFrame("Sending SET velocity max command to radar: ", command, 13);

    size_t bytesWritten = _serial.write(command, 13);
    _serial.flush();

    if (bytesWritten != 13)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

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

    debugPrintHexFrame("Received SET velocity max acknowledgement: ", response, maxIndex);

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

/**
 * @brief Get the current minimum velocity threshold for a specified output channel
 *
 * Retrieves the current minimum velocity threshold for the specified output channel
 * from the iSYS-4001 radar device. The velocity value is returned in km/h after automatic
 * conversion from the device's internal m/s format.
 *
 * @param outputnumber Output channel to query (ISYS_OUTPUT_1, ISYS_OUTPUT_2, or ISYS_OUTPUT_3)
 * @param velocity Pointer to float variable that will receive the minimum velocity in km/h
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for response
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions:
 *         - ERR_NULL_POINTER: velocity pointer is null
 *         - ERR_OUTPUT_OUT_OF_RANGE: Invalid output channel number
 *         - ERR_TIMEOUT: Invalid timeout parameter (0)
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response from device
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame too short
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Response frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *
 * @note The function automatically converts from m/s to km/h: kmh = (raw / 10) * 3.6
 * @note Recommended timeout is >= 100ms (300ms used in examples)
 *
 * @example
 *   // Get current minimum velocity for output 1
 *   float minVelocity = 0;
 *   iSYSResult_t res = radar.iSYS_getOutputVelocityMin(ISYS_OUTPUT_1, &minVelocity, 0x80, 300);
 *   if (res == ERR_OK) {
 *       Serial.print("Current minimum velocity: ");
 *       Serial.print(minVelocity);
 *       Serial.println(" km/h");
 *   }
 */
iSYSResult_t iSYS4001::iSYS_getOutputVelocityMin(iSYSOutputNumber_t outputnumber, float *velocity, uint8_t destAddress, uint32_t timeout)
{
    if (velocity == NULL)
        return ERR_NULL_POINTER;

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
    command[index++] = 0x0C;
    uint8_t fcs = calculateFCS(command, 4, 8);
    command[index++] = fcs;
    command[index++] = 0x16;

    debugPrintHexFrame("Sending GET Velocity min command: ", command, sizeof(command));

    size_t bytesWritten = _serial.write(command, sizeof(command));
    _serial.flush();

    if (bytesWritten != sizeof(command))
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

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

/**
 * @brief Get the current maximum velocity threshold for a specified output channel
 *
 * Retrieves the current maximum velocity threshold for the specified output channel
 * from the iSYS-4001 radar device. The velocity value is returned in km/h after automatic
 * conversion from the device's internal m/s format.
 *
 * @param outputnumber Output channel to query (ISYS_OUTPUT_1, ISYS_OUTPUT_2, or ISYS_OUTPUT_3)
 * @param velocity Pointer to float variable that will receive the maximum velocity in km/h
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for response
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions:
 *         - ERR_NULL_POINTER: velocity pointer is null
 *         - ERR_OUTPUT_OUT_OF_RANGE: Invalid output channel number
 *         - ERR_TIMEOUT: Invalid timeout parameter (0)
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response from device
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame too short
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Response frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *
 * @note The function automatically converts from m/s to km/h: kmh = (raw / 10) * 3.6
 * @note Recommended timeout is >= 100ms (300ms used in examples)
 *
 * @example
 *   // Get current maximum velocity for output 1
 *   float maxVelocity = 0;
 *   iSYSResult_t res = radar.iSYS_getOutputVelocityMax(ISYS_OUTPUT_1, &maxVelocity, 0x80, 300);
 *   if (res == ERR_OK) {
 *       Serial.print("Current maximum velocity: ");
 *       Serial.print(maxVelocity);
 *       Serial.println(" km/h");
 *   }
 */
iSYSResult_t iSYS4001::iSYS_getOutputVelocityMax(iSYSOutputNumber_t outputnumber, float *velocity, uint8_t destAddress, uint32_t timeout)
{
    if (velocity == NULL)
        return ERR_NULL_POINTER;

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
    command[index++] = 0x0D;
    uint8_t fcs = calculateFCS(command, 4, 8);
    command[index++] = fcs;
    command[index++] = 0x16;

    debugPrintHexFrame("Sending GET Velocity max command: ", command, sizeof(command));

    size_t bytesWritten = _serial.write(command, sizeof(command));
    _serial.flush();

    if (bytesWritten != sizeof(command))
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

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
 * @brief Set the minimum signal strength threshold for a specified output channel
 *
 * Configures the minimum signal strength threshold for the specified output channel
 * on the iSYS-4001 radar device. The signal value is provided in dB and is
 * automatically converted to the device's internal 0.1dB fixed-point format.
 *
 * @param outputnumber Output channel to configure (ISYS_OUTPUT_1, ISYS_OUTPUT_2, or ISYS_OUTPUT_3)
 * @param signal Minimum signal strength in dB (0.0 to 249.9)
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for acknowledgement
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions:
 *         - ERR_OUTPUT_OUT_OF_RANGE: Invalid output channel number
 *         - ERR_PARAMETER_OUT_OF_RANGE: Signal value outside valid limits
 *         - ERR_TIMEOUT: Invalid timeout parameter (0)
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response from device
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame too short
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Response frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *
 * @note The function automatically converts dB to 0.1dB fixed-point format (multiply by 10)
 * @note Changes take effect immediately but should be saved with saveApplicationSettings()
 * @note Recommended timeout is >= 100ms (300ms used in examples)
 *
 * @example
 *   // Set minimum signal to 10 dB for output 1
 *   iSYSResult_t res = radar.iSYS_setOutputSignalMin(ISYS_OUTPUT_1, 10, 0x80, 300);
 *   if (res == ERR_OK) {
 *       Serial.println("Minimum signal threshold set successfully");
 *   }
 */
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

    debugPrintHexFrame("Sending SET signal min command to radar: ", command, 13);

    size_t bytesWritten = _serial.write(command, 13);
    _serial.flush();

    if (bytesWritten != 13)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

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

    debugPrintHexFrame("Received SET signal min acknowledgement: ", response, minIndex);

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

/**
 * @brief Set the maximum signal strength threshold for a specified output channel
 *
 * Configures the maximum signal strength threshold for the specified output channel
 * on the iSYS-4001 radar device. The signal value is provided in dB and is
 * automatically converted to the device's internal 0.1dB fixed-point format.
 *
 * @param outputnumber Output channel to configure (ISYS_OUTPUT_1, ISYS_OUTPUT_2, or ISYS_OUTPUT_3)
 * @param signal Maximum signal strength in dB (0.1 to 250.0)
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for acknowledgement
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions:
 *         - ERR_OUTPUT_OUT_OF_RANGE: Invalid output channel number
 *         - ERR_PARAMETER_OUT_OF_RANGE: Signal value outside valid limits
 *         - ERR_TIMEOUT: Invalid timeout parameter (0)
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response from device
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame too short
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Response frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *
 * @note The function automatically converts dB to 0.1dB fixed-point format (multiply by 10)
 * @note Changes take effect immediately but should be saved with saveApplicationSettings()
 * @note Recommended timeout is >= 100ms (300ms used in examples)
 *
 * @example
 *   // Set maximum signal to 50 dB for output 1
 *   iSYSResult_t res = radar.iSYS_setOutputSignalMax(ISYS_OUTPUT_1, 50, 0x80, 300);
 *   if (res == ERR_OK) {
 *       Serial.println("Maximum signal threshold set successfully");
 *   }
 */
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

    debugPrintHexFrame("Sending SET signal max command to radar: ", command, 13);

    size_t bytesWritten = _serial.write(command, 13);
    _serial.flush();

    if (bytesWritten != 13)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

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

    debugPrintHexFrame("Received SET signal max acknowledgement: ", response, maxIndex);

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

/**
 * @brief Get the current minimum signal strength threshold for a specified output channel
 *
 * Retrieves the current minimum signal strength threshold for the specified output channel
 * from the iSYS-4001 radar device. The signal value is returned in dB after automatic
 * conversion from the device's internal 0.1dB fixed-point format.
 *
 * @param outputnumber Output channel to query (ISYS_OUTPUT_1, ISYS_OUTPUT_2, or ISYS_OUTPUT_3)
 * @param signal Pointer to float variable that will receive the minimum signal strength in dB
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for response
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions:
 *         - ERR_NULL_POINTER: signal pointer is null
 *         - ERR_OUTPUT_OUT_OF_RANGE: Invalid output channel number
 *         - ERR_TIMEOUT: Invalid timeout parameter (0)
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response from device
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame too short
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Response frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *
 * @note The function automatically converts from 0.1dB fixed-point format to dB (divide by 10)
 * @note Recommended timeout is >= 100ms (300ms used in examples)
 *
 * @example
 *   // Get current minimum signal for output 1
 *   float minSignal = 0;
 *   iSYSResult_t res = radar.iSYS_getOutputSignalMin(ISYS_OUTPUT_1, &minSignal, 0x80, 300);
 *   if (res == ERR_OK) {
 *       Serial.print("Current minimum signal: ");
 *       Serial.print(minSignal);
 *       Serial.println(" dB");
 *   }
 */
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

    debugPrintHexFrame("Sending GET Signal Min command: ", command, sizeof(command));

    size_t bytesWritten = _serial.write(command, sizeof(command));
    _serial.flush();

    if (bytesWritten != sizeof(command))
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

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

    debugPrintHexFrame("Received Signal Min response: ", response, rIdx);

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

/**
 * @brief Get the current maximum signal strength threshold for a specified output channel
 *
 * Retrieves the current maximum signal strength threshold for the specified output channel
 * from the iSYS-4001 radar device. The signal value is returned in dB after automatic
 * conversion from the device's internal 0.1dB fixed-point format.
 *
 * @param outputnumber Output channel to query (ISYS_OUTPUT_1, ISYS_OUTPUT_2, or ISYS_OUTPUT_3)
 * @param signal Pointer to float variable that will receive the maximum signal strength in dB
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for response
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions:
 *         - ERR_NULL_POINTER: signal pointer is null
 *         - ERR_OUTPUT_OUT_OF_RANGE: Invalid output channel number
 *         - ERR_TIMEOUT: Invalid timeout parameter (0)
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response from device
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame too short
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Response frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *
 * @note The function automatically converts from 0.1dB fixed-point format to dB (divide by 10)
 * @note Recommended timeout is >= 100ms (300ms used in examples)
 *
 * @example
 *   // Get current maximum signal for output 1
 *   float maxSignal = 0;
 *   iSYSResult_t res = radar.iSYS_getOutputSignalMax(ISYS_OUTPUT_1, &maxSignal, 0x80, 300);
 *   if (res == ERR_OK) {
 *       Serial.print("Current maximum signal: ");
 *       Serial.print(maxSignal);
 *       Serial.println(" dB");
 *   }
 */
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

    debugPrintHexFrame("Sending GET Signal Max command: ", command, sizeof(command));

    size_t bytesWritten = _serial.write(command, sizeof(command));
    _serial.flush();

    if (bytesWritten != sizeof(command))
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

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

    debugPrintHexFrame("Received Signal Max response: ", response, rIdx);

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
 * @brief Set the target velocity direction filter for a specified output channel
 *
 * Configures which target directions(APPROACHING, RECEDING, BOTH) are considered for the specified output channel
 * on the iSYS-4001 radar device. This allows filtering targets based on their
 * movement direction relative to the radar sensor.
 *
 * @param outputnumber Output channel to configure (ISYS_OUTPUT_1, ISYS_OUTPUT_2, or ISYS_OUTPUT_3)
 * @param direction Target direction filter (ISYS_TARGET_DIRECTION_APPROACHING, ISYS_TARGET_DIRECTION_RECEDING, or ISYS_TARGET_DIRECTION_BOTH)
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for acknowledgement
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions:
 *         - ERR_OUTPUT_OUT_OF_RANGE: Invalid output channel number
 *         - ERR_TIMEOUT: Invalid timeout parameter (0)
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response from device
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame too short
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Response frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *
 * @note Setting direction filters which targets are reported by the device for the selected output
 * @note Changes take effect immediately but should be saved with saveApplicationSettings()
 * @note Recommended timeout is >= 100ms (300ms used in examples)
 *
 * @example
 *   // Set output 1 to detect both approaching and receding targets
 *   iSYSResult_t res = radar.iSYS_setOutputDirection(ISYS_OUTPUT_1, ISYS_TARGET_DIRECTION_BOTH, 0x80, 300);
 *   if (res == ERR_OK) {
 *       Serial.println("Direction filter set successfully");
 *   }
 */
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

    debugPrintHexFrame("Sending SET direction command to radar: ", command, 13);

    size_t bytesWritten = _serial.write(command, 13);
    _serial.flush();

    if (bytesWritten != 13)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    uint8_t response[9];
    size_t responseIndex = 0;
    uint32_t startTime = millis();

    while ((millis() - startTime) < timeout && responseIndex < sizeof(response))
    {
        if (_serial.available())
        {
            response[responseIndex++] = _serial.read();

            if (response[responseIndex - 1] == 0x16)
                break;
        }
    }

    debugPrintHexFrame("Received SET direction acknowledgement: ", response, responseIndex);

    if (responseIndex == 0)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    if (responseIndex < 9)
    {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }

    if (response[0] != 0x68 || response[1] != 0x03 || response[2] != 0x03 ||
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress ||
        response[6] != 0xD5 || response[8] != 0x16)
    {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }

    if (responseIndex >= 9)
    {
        uint8_t expectedFCS = calculateFCS(response, 4, 6);
        if (response[7] != expectedFCS)
        {
            return ERR_INVALID_CHECKSUM;
        }
    }

    return ERR_OK;
}

/**
 * @brief Get the current target velocity direction for a specified output channel
 *
 * Retrieves the current target direction (APPROACHING, RECEDING, BOTH) setting for the specified output channel
 * from the iSYS-4001 radar device. This shows which target directions are currently
 * being considered for the selected output.
 *
 * @param outputnumber Output channel to query (ISYS_OUTPUT_1, ISYS_OUTPUT_2, or ISYS_OUTPUT_3)
 * @param direction Pointer to iSYSDirection_type_t variable that will receive the current direction
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for response
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions:
 *         - ERR_OUTPUT_OUT_OF_RANGE: Invalid output channel number
 *         - ERR_NULL_POINTER: direction pointer is null
 *         - ERR_TIMEOUT: Invalid timeout parameter (0)
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response from device
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame too short
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Response frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *
 * @note The returned direction value indicates which targets are currently being reported
 * @note Recommended timeout is >= 100ms (300ms used in examples)
 *
 * @example
 *   // Get current direction for output 1
 *   iSYSDirection_type_t currentDirection;
 *   iSYSResult_t res = radar.iSYS_getOutputDirection(ISYS_OUTPUT_1, &currentDirection, 0x80, 300);
 *   if (res == ERR_OK) {
 *       Serial.print("Current direction : ");
 *       Serial.println(currentDirection);
 *   }
 */
iSYSResult_t iSYS4001::iSYS_getOutputDirection(iSYSOutputNumber_t outputnumber, iSYSDirection_type_t *direction, uint8_t destAddress, uint32_t timeout)
{
    if (outputnumber < ISYS_OUTPUT_1 || outputnumber > ISYS_OUTPUT_3)
    {
        return ERR_OUTPUT_OUT_OF_RANGE;
    }

    if (direction == NULL)
        return ERR_NULL_POINTER;

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
    command[index++] = 0x0E;
    uint8_t fcs = calculateFCS(command, 4, 8);
    command[index++] = fcs;
    command[index++] = 0x16;

    debugPrintHexFrame("Sending GET Direction command: ", command, sizeof(command));

    size_t bytesWritten = _serial.write(command, sizeof(command));
    _serial.flush();

    if (bytesWritten != sizeof(command))
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    uint8_t response[11];
    size_t responseIndex = 0;
    uint32_t startTime = millis();

    while ((millis() - startTime) < timeout && responseIndex < sizeof(response))
    {
        if (_serial.available())
        {
            response[responseIndex++] = _serial.read();

            if (response[responseIndex - 1] == 0x16)
                break;
        }
    }

    debugPrintHexFrame("Received Direction response: ", response, responseIndex);

    if (responseIndex == 0)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    if (responseIndex < 11)
    {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }

    if (response[0] != 0x68 || response[1] != 0x05 || response[2] != 0x05 ||
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress ||
        response[6] != 0xD4 || response[7] != 0x00 || response[10] != 0x16)
    {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }

    if (responseIndex >= 11)
    {
        uint8_t expectedFCS = calculateFCS(response, 4, 8);
        if (response[9] != expectedFCS)
        {
            return ERR_INVALID_CHECKSUM;
        }
    }

    *direction = (iSYSDirection_type_t)response[8];
    return ERR_OK;
}

/***************************************************************
 *  EEPROM COMMAND FUNCTIONS
 ***************************************************************/

/**
 * @brief Internal function to send EEPROM command and wait for acknowledgement
 *
 * Sends an EEPROM command to the iSYS-4001 radar device and waits for acknowledgement.
 * This is an internal helper function used by the public EEPROM convenience functions
 * (setFactorySettings, saveSensorSettings, saveApplicationSettings, saveAllSettings).
 *
 * @param subFunction EEPROM sub-function to execute (ISYS_EEPROM_SET_FACTORY_SETTINGS, ISYS_EEPROM_SAVE_SENSOR_SETTINGS, etc.)
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for acknowledgement
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions:
 *         - ERR_TIMEOUT: Invalid timeout parameter (0)
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response from device
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame too short
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Response frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *
 * @note This function performs a two-step process: send command frame, then validate acknowledgement
 * @note Used internally by public EEPROM convenience functions
 * @note EEPROM writes take effect immediately after acknowledgement
 *
 * @example
 *   // Internal usage - called by saveApplicationSettings()
 *   iSYSResult_t res = sendEEPROMCommand(ISYS_EEPROM_SAVE_APPLICATION_SETTINGS, 0x80, 300);
 *   if (res != ERR_OK) {
 *       // Handle EEPROM command failure
 *   }
 */
iSYSResult_t iSYS4001::sendEEPROMCommand(iSYSEEPROMSubFunction_t subFunction, uint8_t destAddress, uint32_t timeout)
{
    if (timeout == 0)
    {
        return ERR_TIMEOUT;
    }

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

/**
 * @brief Convenience function to restore factory default settings
 *
 * Restores the iSYS-4001 radar device to its factory default configuration.
 * This will reset all sensor and application parameters to their original values.
 *
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for acknowledgement
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions
 *
 * @note This function permanently changes device configuration - use with care
 * @note Recommended timeout is >= 100ms (300ms used in examples)
 *
 * @example
 *   // Restore factory settings
 *   iSYSResult_t res = radar.setFactorySettings(0x80, 300);
 *   if (res == ERR_OK) {
 *       Serial.println("Factory settings restored");
 *   }
 */
iSYSResult_t iSYS4001::setFactorySettings(uint8_t destAddress, uint32_t timeout)
{
    return sendEEPROMCommand(ISYS_EEPROM_SET_FACTORY_SETTINGS, destAddress, timeout);
}

/**
 * @brief Convenience function to save sensor settings to EEPROM
 *
 * Persists the current sensor-side parameters to the device's EEPROM memory.
 * This ensures that sensor settings survive power cycles and device resets.
 *
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for acknowledgement
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions
 *
 * @note This function saves sensor-specific parameters to EEPROM
 * @note Recommended timeout is >= 100ms (300ms used in examples)
 *
 * @example
 *   // Save sensor settings
 *   iSYSResult_t res = radar.saveSensorSettings(0x80, 300);
 *   if (res == ERR_OK) {
 *       Serial.println("Sensor settings saved");
 *   }
 */
iSYSResult_t iSYS4001::saveSensorSettings(uint8_t destAddress, uint32_t timeout)
{
    return sendEEPROMCommand(ISYS_EEPROM_SAVE_SENSOR_SETTINGS, destAddress, timeout);
}

/**
 * @brief Convenience function to save application settings to EEPROM
 *
 * Persists the current application-side parameters to the device's EEPROM memory.
 * This includes range, velocity, signal, and direction settings for all outputs.
 *
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for acknowledgement
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions
 *
 * @note This function saves application-specific parameters to EEPROM
 * @note Recommended timeout is >= 100ms (300ms used in examples)
 *
 * @example
 *   // Save application settings
 *   iSYSResult_t res = radar.saveApplicationSettings(0x80, 300);
 *   if (res == ERR_OK) {
 *       Serial.println("Application settings saved");
 *   }
 */
iSYSResult_t iSYS4001::saveApplicationSettings(uint8_t destAddress, uint32_t timeout)
{
    return sendEEPROMCommand(ISYS_EEPROM_SAVE_APPLICATION_SETTINGS, destAddress, timeout);
}

/**
 * @brief Convenience function to save all settings to EEPROM
 *
 * Persists both sensor and application parameters to the device's EEPROM memory.
 * This is equivalent to calling both saveSensorSettings() and saveApplicationSettings().
 *
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for acknowledgement
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions
 *
 * @note This function saves both sensor and application parameters to EEPROM
 * @note Recommended timeout is >= 100ms (300ms used in examples)
 *
 * @example
 *   // Save all settings
 *   iSYSResult_t res = radar.saveAllSettings(0x80, 300);
 *   if (res == ERR_OK) {
 *       Serial.println("All settings saved");
 *   }
 */
iSYSResult_t iSYS4001::saveAllSettings(uint8_t destAddress, uint32_t timeout)
{
    return sendEEPROMCommand(ISYS_EEPROM_SAVE_ALL_SETTINGS, destAddress, timeout);
}

/**
 * @brief Internal function to send EEPROM command frame to radar device
 *
 * Constructs and transmits a command frame for EEPROM operations to the
 * iSYS-4001 radar device. This is an internal helper function used by
 * sendEEPROMCommand() to send the actual command frame.
 *
 * @param subFunction EEPROM sub-function to execute (ISYS_EEPROM_SET_FACTORY_SETTINGS, ISYS_EEPROM_SAVE_SENSOR_SETTINGS, etc.)
 * @param destAddress Device Radar Destination address (typically 0x80)
 *
 * @return iSYSResult_t ERR_OK on success, or error code if command transmission fails
 *
 * @note This function only sends the command - it does not wait for or process the response
 * @note Used internally by sendEEPROMCommand() function
 * @note The command frame includes proper framing, address, and checksum validation
 *
 * @example
 *   // Internal usage - called by sendEEPROMCommand()
 *   iSYSResult_t res = sendEEPROMCommandFrame(ISYS_EEPROM_SAVE_APPLICATION_SETTINGS, 0x80);
 *   if (res != ERR_OK) {
 *       // Handle command transmission error
 *   }
 */
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

    size_t bytesWritten = _serial.write(command, 10);
    _serial.flush();

    if (bytesWritten != 10)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    return ERR_OK;
}
/**
 * @brief Internal function to receive EEPROM command acknowledgement from radar device
 *
 * Waits for and validates the acknowledgement response from the iSYS-4001 radar device
 * after sending an EEPROM command. This is an internal helper function used by
 * sendEEPROMCommand() to validate the command was received and processed.
 *
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for acknowledgement
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions:
 *         - ERR_TIMEOUT: Invalid timeout parameter (0)
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response from device
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame too short
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Response frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *
 * @note This function validates the acknowledgement frame structure and checksum
 * @note Used internally by sendEEPROMCommand() function
 * @note EEPROM operations take effect immediately after acknowledgement
 * @note Recommended timeout is >= 100ms (300ms used in examples)
 *
 * @example
 *   // Internal usage - called by sendEEPROMCommand()
 *   iSYSResult_t res = receiveEEPROMAcknowledgement(0x80, 300);
 *   if (res != ERR_OK) {
 *       // Handle acknowledgement error
 *   }
 */
iSYSResult_t iSYS4001::receiveEEPROMAcknowledgement(uint8_t destAddress, uint32_t timeout)
{
    if (timeout == 0)
    {
        return ERR_TIMEOUT;
    }

    uint8_t response[9];
    size_t responseIndex = 0;
    uint32_t startTime = millis();

    while ((millis() - startTime) < timeout && responseIndex < sizeof(response))
    {
        if (_serial.available())
        {
            response[responseIndex++] = _serial.read();

            if (response[responseIndex - 1] == 0x16)
                break;
        }
    }

    debugPrintHexFrame("Received EEPROM acknowledgement: ", response, responseIndex);

    if (responseIndex == 0)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    if (responseIndex < 9)
    {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }

    if (response[0] != 0x68 || response[1] != 0x03 || response[2] != 0x03 ||
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress ||
        response[6] != 0xDF || response[8] != 0x16)
    {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }

    if (responseIndex >= 9)
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
 * @brief Calculate frame checksum (FCS) for radar communication
 *
 * Computes an additive checksum across a specified range of bytes in a frame.
 * The checksum is used by the iSYS-4001 radar device to validate the integrity
 * of transmitted and received command/response frames.
 *
 * @param data Pointer to byte array containing the frame data
 * @param startIndex Index of the first byte to include in the checksum
 * @param endIndex Index of the last byte to include in the checksum
 *
 * @return uint8_t The computed checksum value
 *
 * @note The checksum is calculated as the 8-bit sum of all bytes between
 *       startIndex and endIndex (inclusive).
 *
 * @example
 *   // Example: calculate checksum for command frame
 *   uint8_t command[] = {0x68, 0x05, 0x05, 0x68, 0x80, 0x01, 0xD4, 0x01, 0x16};
 *   uint8_t fcs = radar.calculateFCS(command, 4, 7);
 *   Serial.print("Calculated FCS: ");
 *   Serial.println(fcs, HEX);
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
 * @brief Set the device address for the radar sensor
 *
 * Changes the DESTINATION address of the iSYS-4001 radar device.
 * After setting a new address, all subsequent communications must use the new address.
 *
 * @param deviceaddress New device address to set (typically 0x80-0xFF)
 * @param destAddress Current known address of the device (0x00 can be used if unknown)
 * @param timeout Maximum time in milliseconds to wait for acknowledgement
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions:
 *         - ERR_TIMEOUT: Invalid timeout parameter (0)
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response from device
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame too short
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Response frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *
 * @note After setting a new address, subsequent communications must use the new address
 * @note Consider persisting configuration with EEPROM commands if required
 * @note Recommended timeout is >= 100ms (300ms used in examples)
 *
 * @example
 *   // Set device address to 0x81, communicating to current device at 0x80
 *   iSYSResult_t res = radar.iSYS_setDeviceAddress(0x81, 0x80, 300);
 *   if (res == ERR_OK) {
 *       Serial.println("Device address changed to 0x81");
 *       // Now use 0x81 for all subsequent communications
 *   }
 */
iSYSResult_t iSYS4001::iSYS_setDeviceAddress(uint8_t deviceaddress, uint8_t destAddress, uint32_t timeout)
{
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
    command[index++] = 0xD3;
    command[index++] = 0x00;
    command[index++] = 0x01;
    command[index++] = 0x00;
    command[index++] = deviceaddress;

    uint8_t fcs = calculateFCS(command, 4, 10);
    command[index++] = fcs;
    command[index++] = 0x16;

    debugPrintHexFrame("Sending SET address command: ", command, sizeof(command));

    size_t bytesWritten = _serial.write(command, sizeof(command));
    _serial.flush();

    if (bytesWritten != sizeof(command))
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    uint8_t response[9];
    size_t responseIndex = 0;
    uint32_t startTime = millis();

    while ((millis() - startTime) < timeout && responseIndex < sizeof(response))
    {
        if (_serial.available())
        {
            response[responseIndex++] = _serial.read();

            if (response[responseIndex - 1] == 0x16)
                break;
        }
    }

    debugPrintHexFrame("Received SET address acknowledgement: ", response, responseIndex);

    if (responseIndex == 0)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    if (responseIndex < 9)
    {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }

    if (response[0] != 0x68 || response[1] != 0x03 || response[2] != 0x03 ||
        response[3] != 0x68 || response[4] != 0x01 || response[5] != deviceaddress ||
        response[6] != 0xD3 || response[8] != 0x16)
    {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }

    if (responseIndex >= 9)
    {
        uint8_t expectedFCS = calculateFCS(response, 4, 6);
        if (response[7] != expectedFCS)
        {
            return ERR_INVALID_CHECKSUM;
        }
    }

    return ERR_OK;
}

/**
 * @brief Get the current device address of the radar sensor
 *
 * Retrieves the current device/destination address of the iSYS-4001 radar device. This is useful
 * for discovering the address of a device when it's unknown, or for verification
 * after setting a new address.
 *
 * @param deviceaddress Pointer to uint8_t variable that will receive the current device address
 * @param destAddress Current known address of the device (0x00 can be used if unknown)
 * @param timeout Maximum time in milliseconds to wait for response
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions:
 *         - ERR_NULL_POINTER: deviceaddress pointer is null
 *         - ERR_TIMEOUT: Invalid timeout parameter (0)
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response from device
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame too short
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Response frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *
 * @note This function can be used to discover the address of an unknown device
 * @note Recommended timeout is >= 100ms (300ms used in examples)
 *
 * @example
 *   // Get current device address
 *   uint8_t currentAddress = 0;
 *   iSYSResult_t res = radar.iSYS_getDeviceAddress(&currentAddress, 0x00, 300);
 *   if (res == ERR_OK) {
 *       Serial.print("Current device address: 0x");
 *       Serial.println(currentAddress, HEX);
 *   }
 */
iSYSResult_t iSYS4001::iSYS_getDeviceAddress(uint8_t *deviceaddress, uint8_t destAddress, uint32_t timeout)
{
    if (deviceaddress == NULL)
        return ERR_NULL_POINTER;

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
    command[index++] = 0x00;
    command[index++] = 0x01;
    command[index++] = 0xD2;
    command[index++] = 0x00;
    command[index++] = 0x01;
    uint8_t fcs = calculateFCS(command, 4, 8);
    command[index++] = fcs;
    command[index++] = 0x16;

    debugPrintHexFrame("Sending GET address command: ", command, sizeof(command));

    size_t bytesWritten = _serial.write(command, sizeof(command));
    _serial.flush();

    if (bytesWritten != sizeof(command))
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    uint8_t response[11];
    size_t responseIndex = 0;
    uint32_t startTime = millis();

    while ((millis() - startTime) < timeout && responseIndex < sizeof(response))
    {
        if (_serial.available())
        {
            response[responseIndex++] = _serial.read();

            if (response[responseIndex - 1] == 0x16)
                break;
        }
    }

    debugPrintHexFrame("Received GET address response: ", response, responseIndex);

    if (responseIndex == 0)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    if (responseIndex < 11)
    {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }

    if (response[0] != 0x68 || response[1] != 0x05 || response[2] != 0x05 ||
        response[3] != 0x68 || response[6] != 0xD2 || response[10] != 0x16)
    {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }

    if (responseIndex >= 11)
    {
        uint8_t expectedFCS = calculateFCS(response, 4, 8);
        if (response[9] != expectedFCS)
        {
            return ERR_INVALID_CHECKSUM;
        }
    }

    *deviceaddress = response[8];
    return ERR_OK;
}

/***************************************************************
 *  ACQUISITION CONTROL FUNCTIONS
 ***************************************************************/

/**
 * @brief Start radar acquisition on the device
 *
 * Initiates the measurement cycle on the iSYS-4001 radar device. This function
 * must be called before attempting to request target data or stream measurements.
 * The device will begin actively scanning and detecting targets after this command.
 *
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for acknowledgement
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions:
 *         - ERR_TIMEOUT: Invalid timeout parameter (0)
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response from device
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame too short
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Response frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *         - ERR_COMMAND_MAX_DATA_OVERFLOW: Response exceeds internal buffer
 *
 * @note Start acquisition before attempting to stream or repeatedly request data
 * @note Recommended timeout is >= 100ms (300ms used in examples)
 * @note Flush stale UART bytes before issuing commands for improved reliability
 *
 * @example
 *   // Start radar acquisition
 *   iSYSResult_t res = radar.iSYS_startAcquisition(0x80, 300);
 *   if (res == ERR_OK) {
 *       Serial.println("Radar acquisition started");
 *       // Now can request target data
 *   }
 */
iSYSResult_t iSYS4001::iSYS_startAcquisition(uint8_t destAddress, uint32_t timeout)
{
    if (timeout == 0)
    {
        return ERR_TIMEOUT;
    }

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

/**
 * @brief Stop radar acquisition on the device
 *
 * Halts the measurement cycle on the iSYS-4001 radar device. This function
 * stops the device from actively scanning and detecting targets.
 *
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for acknowledgement
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions:
 *         - ERR_TIMEOUT: Invalid timeout parameter (0)
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response from device
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame too short
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Response frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *         - ERR_COMMAND_MAX_DATA_OVERFLOW: Response exceeds internal buffer
 *
 * @note Recommended timeout is >= 100ms (300ms used in examples)
 * @note Flush stale  bytes before issuing commands for improved reliability
 *
 * @example
 *   // Stop radar acquisition
 *   iSYSResult_t res = radar.iSYS_stopAcquisition(0x80, 300);
 *   if (res == ERR_OK) {
 *       Serial.println("Radar acquisition stopped");
 *       // Can now change device settings if needed
 *   }
 */
iSYSResult_t iSYS4001::iSYS_stopAcquisition(uint8_t destAddress, uint32_t timeout)
{
    if (timeout == 0)
    {
        return ERR_TIMEOUT;
    }

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

/**
 * @brief Internal function to send acquisition start/stop command to radar device
 *
 * Constructs and transmits a command frame to start or stop radar acquisition
 * on the iSYS-4001 device. This is an internal helper function used by
 * iSYS_startAcquisition() and iSYS_stopAcquisition().
 *
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param start If true, starts acquisition; if false, stops acquisition
 *
 * @return iSYSResult_t ERR_OK on success, or error code if command transmission fails
 *
 * @note This function only sends the command - it does not wait for or process the response
 * @note Used internally by iSYS_startAcquisition() and iSYS_stopAcquisition() functions
 * @note The command frame includes proper framing, address, and checksum validation
 *
 * @example
 *   // Internal usage - called by iSYS_startAcquisition()
 *   iSYSResult_t res = sendAcquisitionCommand(0x80, true);
 *   if (res != ERR_OK) {
 *       // Handle error
 *   }
 */
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

    size_t bytesWritten = _serial.write(command, 11);
    _serial.flush();

    if (bytesWritten != 11)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    return ERR_OK;
}

/**
 * @brief Internal function to receive acquisition command acknowledgement from radar device
 *
 * Waits for and validates the acknowledgement response from the iSYS-4001 radar device
 * after sending an acquisition start/stop command. This is an internal helper function
 * used by iSYS_startAcquisition() and iSYS_stopAcquisition().
 *
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for acknowledgement
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions:
 *         - ERR_TIMEOUT: Invalid timeout parameter (0)
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response from device
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame too short
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Response frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *
 * @note This function validates the acknowledgement frame structure and checksum
 * @note Used internally by iSYS_startAcquisition() and iSYS_stopAcquisition() functions
 * @note Recommended timeout is >= 100ms (300ms used in examples)
 *
 * @example
 *   // Internal usage - called by iSYS_startAcquisition()
 *   iSYSResult_t res = receiveAcquisitionAcknowledgement(0x80, 300);
 *   if (res != ERR_OK) {
 *       // Handle acknowledgement error
 *   }
 */
iSYSResult_t iSYS4001::receiveAcquisitionAcknowledgement(uint8_t destAddress, uint32_t timeout)
{
    if (timeout == 0)
    {
        return ERR_TIMEOUT;
    }

    uint8_t response[9];
    size_t responseIndex = 0;
    uint32_t startTime = millis();

    while ((millis() - startTime) < timeout && responseIndex < sizeof(response))
    {
        if (_serial.available())
        {
            response[responseIndex++] = _serial.read();

            if (response[responseIndex - 1] == 0x16)
                break;
        }
    }

    debugPrintHexFrame("Received acquisition acknowledgement: ", response, responseIndex);

    if (responseIndex == 0)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    if (responseIndex < 9)
    {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }

    if (response[0] != 0x68 || response[1] != 0x03 || response[2] != 0x03 ||
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress ||
        response[6] != 0xD1 || response[8] != 0x16)
    {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }

    if (responseIndex >= 9)
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
 *  OUTPUT MULTIPLE TARGET FILTER FUNCTIONS
 ***************************************************************/

/**
 * @brief Set multiple target filter for a specified output channel
 *
 * Configures the multiple target filtering for the specified output channel
 * on the iSYS-4001 radar device. This function specifically handles signal
 * selection for multiple target filtering and always sets the signal filter
 * to ISYS_OFF for multiple target operation.This is used when users want to 
 * scan multiple targets.
 *
 * @param outputnumber Output channel to configure (ISYS_OUTPUT_1, ISYS_OUTPUT_2, or ISYS_OUTPUT_3)
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for acknowledgement
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions:
 *         - ERR_OUTPUT_OUT_OF_RANGE: Invalid output channel number
 *         - ERR_TIMEOUT: Invalid timeout parameter (0)
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response from device
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame too short
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Response frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *
 * @note This function is specifically for multiple target filtering
 * @note The signal filter is always set to ISYS_OFF for multiple target filtering
 * @note Changes take effect immediately but should be saved with saveAllSettings()
 * @note Recommended timeout is >= 100ms (300ms used in examples)
 *
 * @example
 *   // Set multiple target filter for output 1
 *   iSYSResult_t res = radar.iSYS_setMultipleTargetFilter(ISYS_OUTPUT_1, 0x80, 300);
 *   if (res == ERR_OK) {
 *       Serial.println("Multiple target filter set successfully");
 *   }
 */
iSYSResult_t iSYS4001::iSYS_setMultipleTargetFilter(iSYSOutputNumber_t outputnumber, uint8_t destAddress, uint32_t timeout)
{
    if (outputnumber < ISYS_OUTPUT_1 || outputnumber > ISYS_OUTPUT_3)
    {
        return ERR_OUTPUT_OUT_OF_RANGE;
    }

    if (timeout == 0)
    {
        return ERR_TIMEOUT;
    }

    iSYSResult_t res = sendSetMultipleTargetFilterRequest(outputnumber, destAddress);
    if (res != ERR_OK)
    {
        return res;
    }

    res = receiveSetMultipleTargetFilterAcknowledgement(destAddress, timeout);
    if (res != ERR_OK)
    {
        return res;
    }

    return ERR_OK;
}

/**
 * @brief Internal function to send multiple target filter request to radar device
 *
 * Constructs and transmits a command frame to set multiple target filtering
 * for the specified output channel on the iSYS-4001 radar device. This is an
 * internal helper function used by iSYS_setMultipleTargetFilter().
 *
 * @param outputnumber Output channel to configure (ISYS_OUTPUT_1, ISYS_OUTPUT_2, or ISYS_OUTPUT_3)
 * @param destAddress Device Radar Destination address (typically 0x80)
 *
 * @return iSYSResult_t ERR_OK on success, or error code if command transmission fails
 *
 * @note This function only sends the command - it does not wait for or process the response
 * @note Used internally by iSYS_setMultipleTargetFilter() function
 * @note The signal filter is always set to ISYS_OFF for multiple target filtering
 * @note The command frame includes proper framing, address, and checksum validation
 *
 * @example
 *   // Internal usage - called by iSYS_setMultipleTargetFilter()
 *   iSYSResult_t res = sendSetMultipleTargetFilterRequest(ISYS_OUTPUT_1, 0x80);
 *   if (res != ERR_OK) {
 *       // Handle command transmission error
 *   }
 */
iSYSResult_t iSYS4001::sendSetMultipleTargetFilterRequest(iSYSOutputNumber_t outputnumber, uint8_t destAddress)
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
    command[index++] = (uint8_t)ISYS_OFF; // Always set to ISYS_OFF

    uint8_t fcs = calculateFCS(command, 4, index - 1);
    command[index++] = fcs;
    command[index++] = 0x16;

    debugPrintHexFrame("Setting multiple target filter command to radar: ", command, 13);

    size_t bytesWritten = _serial.write(command, 13);
    _serial.flush();

    if (bytesWritten != 13)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    return ERR_OK;
}

/**
 * @brief Internal function to receive multiple target filter acknowledgement from radar device
 *
 * Waits for and validates the acknowledgement response from the iSYS-4001 radar device
 * after sending a multiple target filter command. This is an internal helper function
 * used by iSYS_setMultipleTargetFilter().
 *
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for acknowledgement
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions:
 *         - ERR_TIMEOUT: Invalid timeout parameter (0)
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response from device
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame too short
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Response frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *
 * @note This function validates the acknowledgement frame structure and checksum
 * @note Used internally by iSYS_setMultipleTargetFilter() function
 * @note Recommended timeout is >= 100ms (300ms used in examples)
 *
 * @example
 *   // Internal usage - called by iSYS_setMultipleTargetFilter()
 *   iSYSResult_t res = receiveSetMultipleTargetFilterAcknowledgement(0x80, 300);
 *   if (res != ERR_OK) {
 *       // Handle acknowledgement error
 *   }
 */
iSYSResult_t iSYS4001::receiveSetMultipleTargetFilterAcknowledgement(uint8_t destAddress, uint32_t timeout)
{
    if (timeout == 0)
    {
        return ERR_TIMEOUT;
    }

    uint8_t response[9];
    size_t responseIndex = 0;
    uint32_t startTime = millis();

    while ((millis() - startTime) < timeout && responseIndex < sizeof(response))
    {
        if (_serial.available())
        {
            response[responseIndex++] = _serial.read();

            if (response[responseIndex - 1] == 0x16)
                break;
        }
    }

    debugPrintHexFrame("Received multiple target filter acknowledgement: ", response, responseIndex);

    if (responseIndex == 0)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    if (responseIndex < 9)
    {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }

    if (response[0] != 0x68 || response[1] != 0x03 || response[2] != 0x03 ||
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress ||
        response[6] != 0xD5 || response[8] != 0x16)
    {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }

    if (responseIndex >= 9)
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
 *  OUTPUT SINGLE TARGET FILTER FUNCTIONS
 ***************************************************************/

/**
 * @brief Set the output filter type for single target filtering
 *
 * Configures the filter type for single target output on the specified output channel
 * of the iSYS-4001 radar device. This determines how multiple targets are processed
 * to produce a single output value (highest signal, mean, median, min, or max).
 *
 * @param outputnumber Output channel to configure (ISYS_OUTPUT_1, ISYS_OUTPUT_2, or ISYS_OUTPUT_3)
 * @param filter Filter type (ISYS_OUTPUT_FILTER_HIGHEST_SIGNAL, ISYS_OUTPUT_FILTER_MEAN, ISYS_OUTPUT_FILTER_MEDIAN, ISYS_OUTPUT_FILTER_MIN, or ISYS_OUTPUT_FILTER_MAX)
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for acknowledgement
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions:
 *         - ERR_OUTPUT_OUT_OF_RANGE: Invalid output channel number
 *         - ERR_TIMEOUT: Invalid timeout parameter (0)
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response from device
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame too short
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Response frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *
 * @note This function configures how multiple targets are processed for single target output
 * @note When using HIGHEST_SIGNAL, signal selection is not required
 * @note Changes take effect immediately but should be saved with saveAllSettings()
 * @note Recommended timeout is >= 100ms (300ms used in examples)
 *
 * @example
 *   // Set output 1 to use median filter for single target output
 *   iSYSResult_t res = radar.iSYS_setOutputFilterType(ISYS_OUTPUT_1, ISYS_OUTPUT_FILTER_MEDIAN, 0x80, 300);
 *   if (res == ERR_OK) {
 *       Serial.println("Output filter type set successfully");
 *   }
 */
iSYSResult_t iSYS4001::iSYS_setOutputFilterType(iSYSOutputNumber_t outputnumber, iSYSOutput_filter_t filter, uint8_t destAddress, uint32_t timeout)
{
    if (outputnumber < ISYS_OUTPUT_1 || outputnumber > ISYS_OUTPUT_3)
    {
        return ERR_OUTPUT_OUT_OF_RANGE;
    }

    if (timeout == 0)
    {
        return ERR_TIMEOUT;
    }

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

/**
 * @brief Internal function to send output filter type request to radar device
 *
 * Constructs and transmits a command frame to set the output filter type
 * for single target filtering on the specified output channel of the iSYS-4001
 * radar device. This is an internal helper function used by iSYS_setOutputFilterType().
 *
 * @param outputnumber Output channel to configure (ISYS_OUTPUT_1, ISYS_OUTPUT_2, or ISYS_OUTPUT_3)
 * @param filter Filter type (ISYS_OUTPUT_FILTER_HIGHEST_SIGNAL, ISYS_OUTPUT_FILTER_MEAN, ISYS_OUTPUT_FILTER_MEDIAN, ISYS_OUTPUT_FILTER_MIN, or ISYS_OUTPUT_FILTER_MAX)
 * @param destAddress Device Radar Destination address (typically 0x80)
 *
 * @return iSYSResult_t ERR_OK on success, or error code if command transmission fails
 *
 * @note This function only sends the command - it does not wait for or process the response
 * @note Used internally by iSYS_setOutputFilterType() function
 * @note The command frame includes proper framing, address, and checksum validation
 *
 * @example
 *   // Internal usage - called by iSYS_setOutputFilterType()
 *   iSYSResult_t res = sendSetOutputFilterRequest(ISYS_OUTPUT_1, ISYS_OUTPUT_FILTER_MEDIAN, 0x80);
 *   if (res != ERR_OK) {
 *       // Handle command transmission error
 *   }
 */
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

    debugPrintHexFrame("Setting output filter type command to radar: ", command, 13);

    size_t bytesWritten = _serial.write(command, 13);
    _serial.flush();

    if (bytesWritten != 13)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    return ERR_OK;
}

/**
 * @brief Internal function to receive output filter type acknowledgement from radar device
 *
 * Waits for and validates the acknowledgement response from the iSYS-4001 radar device
 * after sending an output filter type command. This is an internal helper function
 * used by iSYS_setOutputFilterType().
 *
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for acknowledgement
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions:
 *         - ERR_TIMEOUT: Invalid timeout parameter (0)
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response from device
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame too short
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Response frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *
 * @note This function validates the acknowledgement frame structure and checksum
 * @note Used internally by iSYS_setOutputFilterType() function
 * @note Recommended timeout is >= 100ms (300ms used in examples)
 *
 * @example
 *   // Internal usage - called by iSYS_setOutputFilterType()
 *   iSYSResult_t res = receiveSetOutputFilterAcknowledgement(0x80, 300);
 *   if (res != ERR_OK) {
 *       // Handle acknowledgement error
 *   }
 */
iSYSResult_t iSYS4001::receiveSetOutputFilterAcknowledgement(uint8_t destAddress, uint32_t timeout)
{
    if (timeout == 0)
    {
        return ERR_TIMEOUT;
    }

    uint8_t response[9];
    size_t responseIndex = 0;
    uint32_t startTime = millis();

    while ((millis() - startTime) < timeout && responseIndex < sizeof(response))
    {
        if (_serial.available())
        {
            response[responseIndex++] = _serial.read();

            if (response[responseIndex - 1] == 0x16)
                break;
        }
    }

    debugPrintHexFrame("Received output filter acknowledgement: ", response, responseIndex);

    if (responseIndex == 0)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    if (responseIndex < 9)
    {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }

    if (response[0] != 0x68 || response[1] != 0x03 || response[2] != 0x03 ||
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress ||
        response[6] != 0xD5 || response[8] != 0x16)
    {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }

    if (responseIndex >= 9)
    {
        uint8_t expectedFCS = calculateFCS(response, 4, 6);
        if (response[7] != expectedFCS)
        {
            return ERR_INVALID_CHECKSUM;
        }
    }

    return ERR_OK;
}

/**
 * @brief Get the current output filter type for single target filtering
 *
 * Retrieves the current filter type setting for single target output on the
 * specified output channel from the iSYS-4001 radar device. This shows how
 * multiple targets are currently being processed to produce a single output value.
 *
 * @param outputnumber Output channel to query (ISYS_OUTPUT_1, ISYS_OUTPUT_2, or ISYS_OUTPUT_3)
 * @param filter Pointer to iSYSOutput_filter_t variable that will receive the current filter type
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for response
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions:
 *         - ERR_OUTPUT_OUT_OF_RANGE: Invalid output channel number
 *         - ERR_NULL_POINTER: filter pointer is null
 *         - ERR_TIMEOUT: Invalid timeout parameter (0)
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response from device
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame too short
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Response frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *
 * @note The returned filter value indicates how multiple targets are processed for single target output
 * @note Recommended timeout is >= 100ms (300ms used in examples)
 *
 * @example
 *   // Get current filter type for output 1
 *   iSYSOutput_filter_t currentFilter;
 *   iSYSResult_t res = radar.iSYS_getOutputFilterType(ISYS_OUTPUT_1, &currentFilter, 0x80, 300);
 *   if (res == ERR_OK) {
 *       Serial.print("Current filter type: ");
 *       Serial.println(currentFilter);
 *   }
 */
iSYSResult_t iSYS4001::iSYS_getOutputFilterType(iSYSOutputNumber_t outputnumber, iSYSOutput_filter_t *filter, uint8_t destAddress, uint32_t timeout)
{
    if (outputnumber < ISYS_OUTPUT_1 || outputnumber > ISYS_OUTPUT_3)
    {
        return ERR_OUTPUT_OUT_OF_RANGE;
    }

    if (filter == nullptr)
    {
        return ERR_NULL_POINTER;
    }

    if (timeout == 0)
    {
        return ERR_TIMEOUT;
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

/**
 * @brief Internal function to send get output filter type request to radar device
 *
 * Constructs and transmits a command frame to request the current output filter type
 * setting for single target filtering on the specified output channel of the iSYS-4001
 * radar device. This is an internal helper function used by iSYS_getOutputFilterType().
 *
 * @param outputnumber Output channel to query (ISYS_OUTPUT_1, ISYS_OUTPUT_2, or ISYS_OUTPUT_3)
 * @param destAddress Device Radar Destination address (typically 0x80)
 *
 * @return iSYSResult_t ERR_OK on success, or error code if command transmission fails
 *
 * @note This function only sends the command - it does not wait for or process the response
 * @note Used internally by iSYS_getOutputFilterType() function
 * @note The command frame includes proper framing, address, and checksum validation
 *
 * @example
 *   // Internal usage - called by iSYS_getOutputFilterType()
 *   iSYSResult_t res = sendGetOutputFilterRequest(ISYS_OUTPUT_1, 0x80);
 *   if (res != ERR_OK) {
 *       // Handle command transmission error
 *   }
 */
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

    debugPrintHexFrame("Getting output filter type command to radar: ", command, 11);

    size_t bytesWritten = _serial.write(command, 11);
    _serial.flush();

    if (bytesWritten != 11)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    return ERR_OK;
}

/**
 * @brief Receive and parse output filter type response from radar
 *
 * Waits for and validates the response frame from the iSYS-4001 radar device
 * after sending a request for the current output filter type. The function
 * extracts the filter type from the response and returns it via the provided
 * pointer.
 *
 * @param filter Pointer to iSYSOutput_filter_t variable that will receive the current output filter type
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for response
 *
 * @return iSYSResult_t ERR_OK on success, or error code for failure conditions:
 *         - ERR_TIMEOUT: Timeout parameter is zero
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response received within timeout
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame shorter than expected
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Response frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *
 * @note This function should be called after sending the corresponding
 *       request function (sendGetOutputFilterRequest()).
 * @note The received filter value indicates which type of output filtering
 *       is currently active on the device.
 *
 * @example
 *   // Receive and process response for output filter type
 *   iSYSOutput_filter_t currentFilter;
 *   iSYSResult_t res = radar.receiveGetOutputFilterResponse(&currentFilter, 0x80, 300);
 *   if (res == ERR_OK) {
 *       Serial.print("Output filter type received: ");
 *       Serial.println(currentFilter);
 *   } else {
 *       Serial.print("Error receiving filter type: ");
 *       Serial.println(res);
 *   }
 */

iSYSResult_t iSYS4001::receiveGetOutputFilterResponse(iSYSOutput_filter_t *filter, uint8_t destAddress, uint32_t timeout)
{
    if (timeout == 0)
    {
        return ERR_TIMEOUT;
    }

    uint8_t response[11];
    size_t responseIndex = 0;
    uint32_t startTime = millis();

    while ((millis() - startTime) < timeout && responseIndex < sizeof(response))
    {
        if (_serial.available())
        {
            response[responseIndex++] = _serial.read();

            if (response[responseIndex - 1] == 0x16)
                break;
        }
    }

    debugPrintHexFrame("Received output filter response: ", response, responseIndex);

    if (responseIndex == 0)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    if (responseIndex < 11)
    {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }

    if (response[0] != 0x68 || response[1] != 0x05 || response[2] != 0x05 ||
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress ||
        response[6] != 0xD4 || response[7] != 0x00 || response[10] != 0x16)
    {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }

    if (responseIndex >= 11)
    {
        uint8_t expectedFCS = calculateFCS(response, 4, 8);
        if (response[9] != expectedFCS)
        {
            return ERR_INVALID_CHECKSUM;
        }
    }

    *filter = (iSYSOutput_filter_t)response[8];
    return ERR_OK;
}

/**
 * @brief Set the output signal filter for a channel
 *
 * Sends a request to the iSYS-4001 radar device to configure the signal filter
 * type for a specified output channel, and waits for the acknowledgement frame
 * to confirm successful application of the setting.
 *
 * @param outputnumber Output channel to configure (ISYS_OUTPUT_1, ISYS_OUTPUT_2, or ISYS_OUTPUT_3)
 * @param signal Desired signal filter type to set (value from iSYSFilter_signal_t)
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for acknowledgement
 *
 * @return iSYSResult_t ERR_OK on success, or error code for failure conditions:
 *         - ERR_OUTPUT_OUT_OF_RANGE: Invalid output channel number
 *         - ERR_TIMEOUT: Timeout parameter is zero
 *         - ERR_COMMAND_NO_DATA_RECEIVED: Failed to send request or no acknowledgement received
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Acknowledgement frame shorter than expected
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Acknowledgement frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Acknowledgement checksum validation failed
 *
 * @note This is a high-level function that internally calls
 *       sendSetOutputSignalFilterRequest() and
 *       receiveSetOutputSignalFilterAcknowledgement().
 *
 * @example
 *   // Set output 2 to use HIGHEST_SIGNAL filter type
 *   iSYSResult_t res = radar.iSYS_setOutputSignalFilter(ISYS_OUTPUT_2, ISYS_FILTER_SIGNAL_HIGHEST, 0x80, 300);
 *   if (res == ERR_OK) {
 *       Serial.println("Output signal filter successfully configured");
 *   } else {
 *       Serial.print("Failed to set output signal filter, error: ");
 *       Serial.println(res);
 *   }
 */

iSYSResult_t iSYS4001::iSYS_setOutputSignalFilter(iSYSOutputNumber_t outputnumber, iSYSFilter_signal_t signal, uint8_t destAddress, uint32_t timeout)
{
    if (outputnumber < ISYS_OUTPUT_1 || outputnumber > ISYS_OUTPUT_3)
    {
        return ERR_OUTPUT_OUT_OF_RANGE;
    }

    if (timeout == 0)
    {
        return ERR_TIMEOUT;
    }

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

/**
 * @brief Send request to set the output signal filter for a channel
 *
 * Constructs and transmits a command frame to the iSYS-4001 radar device
 * instructing it to change the signal filter setting for the specified
 * output channel. The radar should reply with an acknowledgement frame,
 * which must be validated using receiveSetOutputSignalFilterAcknowledgement().
 *
 * @param outputnumber Output channel to configure (ISYS_OUTPUT_1, ISYS_OUTPUT_2, or ISYS_OUTPUT_3)
 * @param signal Desired signal filter type to set (value from iSYSFilter_signal_t)
 * @param destAddress Device Radar Destination address (typically 0x80)
 *
 * @return iSYSResult_t ERR_OK on success, or error code for failure conditions:
 *         - ERR_COMMAND_NO_DATA_RECEIVED: Failed to write full request frame to serial
 *
 * @note This function only sends the request; the acknowledgement must be
 *       received separately to confirm success.
 *
 * @example
 *   // Set signal filter for output 1 to HIGHEST_SIGNAL
 *   iSYSResult_t res = radar.sendSetOutputSignalFilterRequest(ISYS_OUTPUT_1, ISYS_FILTER_SIGNAL_HIGHEST, 0x80);
 *   if (res == ERR_OK) {
 *       // Wait for acknowledgement
 *       res = radar.receiveSetOutputSignalFilterAcknowledgement(0x80, 300);
 *       if (res == ERR_OK) {
 *           Serial.println("Signal filter successfully updated");
 *       } else {
 *           Serial.println("Failed to receive acknowledgement");
 *       }
 *   } else {
 *       Serial.println("Failed to send set filter command");
 *   }
 */

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

    size_t bytesWritten = _serial.write(command, 13);
    _serial.flush();

    if (bytesWritten != 13)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    return ERR_OK;
}

/**
 * @brief Receive and validate acknowledgement for setting output signal filter
 *
 * Waits for and parses the acknowledgement frame from the iSYS-4001 radar device
 * after sending a "set output signal filter" command. The acknowledgement
 * confirms that the radar has accepted and applied the requested filter change.
 *
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for acknowledgement
 *
 * @return iSYSResult_t ERR_OK on success, or error code for failure conditions:
 *         - ERR_TIMEOUT: Timeout parameter is zero
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No acknowledgement received within timeout
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Acknowledgement frame shorter than expected
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Frame structure invalid (header, address, or command mismatch)
 *         - ERR_INVALID_CHECKSUM: Acknowledgement checksum validation failed
 *
 * @note This function should be called after sending a set output signal filter request
 * @note The acknowledgement frame does not return a signal value, only confirmation
 *
 * @example
 *   // Receive acknowledgement after setting signal filter
 *   iSYSResult_t res = radar.receiveSetOutputSignalFilterAcknowledgement(0x80, 300);
 *   if (res == ERR_OK) {
 *       Serial.println("Signal filter successfully set");
 *   } else {
 *       Serial.print("Failed to receive acknowledgement, error: ");
 *       Serial.println(res);
 *   }
 */

iSYSResult_t iSYS4001::receiveSetOutputSignalFilterAcknowledgement(uint8_t destAddress, uint32_t timeout)
{
    if (timeout == 0)
    {
        return ERR_TIMEOUT;
    }

    uint8_t response[9];
    size_t responseIndex = 0;
    uint32_t startTime = millis();

    while ((millis() - startTime) < timeout && responseIndex < sizeof(response))
    {
        if (_serial.available())
        {
            response[responseIndex++] = _serial.read();

            if (response[responseIndex - 1] == 0x16)
                break;
        }
    }

    debugPrintHexFrame("Received output signal filter acknowledgement: ", response, responseIndex);

    if (responseIndex == 0)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    if (responseIndex < 9)
    {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }

    if (response[0] != 0x68 || response[1] != 0x03 || response[2] != 0x03 ||
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress ||
        response[6] != 0xD5 || response[8] != 0x16)
    {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }

    if (responseIndex >= 9)
    {
        uint8_t expectedFCS = calculateFCS(response, 4, 6);
        if (response[7] != expectedFCS)
        {
            return ERR_INVALID_CHECKSUM;
        }
    }

    return ERR_OK;
}

/**
 * @brief Get the current output signal filter setting for single target filtering
 *
 * Retrieves the current signal filter setting for single target output on the
 * specified output channel from the iSYS-4001 radar device. This shows which
 * signal type is currently being used for filtering multiple targets.
 *
 * @param outputnumber Output channel to query (ISYS_OUTPUT_1, ISYS_OUTPUT_2, or ISYS_OUTPUT_3)
 * @param signal Pointer to iSYSFilter_signal_t variable that will receive the current signal filter
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for response
 *
 * @return iSYSResult_t ERR_OK on success, or error code for various failure conditions:
 *         - ERR_OUTPUT_OUT_OF_RANGE: Invalid output channel number
 *         - ERR_NULL_POINTER: signal pointer is null
 *         - ERR_TIMEOUT: Invalid timeout parameter (0)
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response from device
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame too short
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Response frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *
 * @note The returned signal value indicates which signal type is used for filtering
 * @note When using HIGHEST_SIGNAL filter type, signal selection is not required
 * @note Recommended timeout is >= 100ms (300ms used in examples)
 *
 * @example
 *   // Get current signal filter for output 1
 *   iSYSFilter_signal_t currentSignal;
 *   iSYSResult_t res = radar.iSYS_getOutputSignalFilter(ISYS_OUTPUT_1, &currentSignal, 0x80, 300);
 *   if (res == ERR_OK) {
 *       Serial.print("Current signal filter: ");
 *       Serial.println(currentSignal);
 *   }
 */
iSYSResult_t iSYS4001::iSYS_getOutputSignalFilter(iSYSOutputNumber_t outputnumber, iSYSFilter_signal_t *signal, uint8_t destAddress, uint32_t timeout)
{
    if (outputnumber < ISYS_OUTPUT_1 || outputnumber > ISYS_OUTPUT_3)
    {
        return ERR_OUTPUT_OUT_OF_RANGE;
    }

    if (signal == nullptr)
    {
        return ERR_NULL_POINTER;
    }

    if (timeout == 0)
    {
        return ERR_TIMEOUT;
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

/**
 * @brief Send request to retrieve current output signal filter setting
 *
 * Constructs and transmits a request frame to the iSYS-4001 radar device asking
 * for the current signal filter setting on the specified output channel. The
 * radar will respond with a frame containing the active filter configuration.
 *
 * @param outputnumber Output channel to query (ISYS_OUTPUT_1, ISYS_OUTPUT_2, or ISYS_OUTPUT_3)
 * @param destAddress Device Radar Destination address (typically 0x80)
 *
 * @return iSYSResult_t ERR_OK on success, or error code for failure conditions:
 *         - ERR_COMMAND_NO_DATA_RECEIVED: Failed to write full request frame to serial
 *
 * @note The response must be parsed using iSYS4001::receiveGetOutputSignalFilterResponse()
 * @note This function only sends the request; it does not wait for or validate the response
 *
 * @example
 *   // Send request to get current signal filter for output 1
 *   iSYSResult_t res = radar.sendGetOutputSignalFilterRequest(ISYS_OUTPUT_1, 0x80);
 *   if (res != ERR_OK) {
 *       Serial.println("Failed to send request frame");
 *   }
 */

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

    debugPrintHexFrame("Getting output signal filter command to radar: ", command, 11);

    size_t bytesWritten = _serial.write(command, 11);
    _serial.flush();

    if (bytesWritten != 11)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    return ERR_OK;
}

/**
 * @brief Receive and parse output signal filter response from radar
 *
 * Waits for and validates the response frame from the iSYS-4001 radar device
 * after sending a request for the current signal filter configuration. The
 * function extracts the signal filter type from the response and returns it
 * via the provided pointer.
 *
 * @param signal Pointer to iSYSFilter_signal_t variable that will receive the current signal filter
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for response
 *
 * @return iSYSResult_t ERR_OK on success, or error code for failure conditions:
 *         - ERR_TIMEOUT: Timeout parameter is zero
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response received within timeout
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame shorter than expected
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Frame structure invalid (header, address, or command mismatch)
 *         - ERR_INVALID_CHECKSUM: Response checksum validation failed
 *
 * @note This function should be called after sendGetOutputSignalFilterRequest()
 * @note The received signal value indicates which filter type is active on the device
 *
 * @example
 *   // Receive and process response for output signal filter
 *   iSYSFilter_signal_t currentSignal;
 *   iSYSResult_t res = radar.receiveGetOutputSignalFilterResponse(&currentSignal, 0x80, 300);
 *   if (res == ERR_OK) {
 *       Serial.print("Signal filter received: ");
 *       Serial.println(currentSignal);
 *   } else {
 *       Serial.print("Error receiving filter response: ");
 *       Serial.println(res);
 *   }
 */

iSYSResult_t iSYS4001::receiveGetOutputSignalFilterResponse(iSYSFilter_signal_t *signal, uint8_t destAddress, uint32_t timeout)
{
    if (timeout == 0)
    {
        return ERR_TIMEOUT;
    }

    uint8_t response[11];
    size_t responseIndex = 0;
    uint32_t startTime = millis();

    while ((millis() - startTime) < timeout && responseIndex < sizeof(response))
    {
        if (_serial.available())
        {
            response[responseIndex++] = _serial.read();

            if (response[responseIndex - 1] == 0x16)
                break;
        }
    }

    debugPrintHexFrame("Received output signal filter response: ", response, responseIndex);

    if (responseIndex == 0)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    if (responseIndex < 11)
    {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }

    if (response[0] != 0x68 || response[1] != 0x05 || response[2] != 0x05 ||
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress ||
        response[6] != 0xD4 || response[7] != 0x00 || response[10] != 0x16)
    {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }

    if (responseIndex >= 11)
    {
        uint8_t expectedFCS = calculateFCS(response, 4, 8);
        if (response[9] != expectedFCS)
        {
            return ERR_INVALID_CHECKSUM;
        }
    }

    *signal = (iSYSFilter_signal_t)response[8];
    return ERR_OK;
}


/***************************************************************
 *  SET/GET RANGE BOUND FUNCTIONS
 **************************************************************/

/**
 * @brief Set the sensor range bound (0–50 m or 0–150 m)
 *
 * Configures the overall range window of the iSYS-4001 device. This setting
 * switches the operating range between 0–50 m and 0–150 m. The device should
 * not be acquiring when changing this bound; stop acquisition first if needed.
 *
 * @param bound Desired range bound
 *        - ISYS_RANGE_0_TO_50  → 0 to 50 meters
 *        - ISYS_RANGE_0_TO_150 → 0 to 150 meters
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for acknowledgement
 *
 * @return iSYSResult_t ERR_OK on success, or error code for failure conditions:
 *         - ERR_TIMEOUT: Timeout parameter is zero
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No acknowledgement received
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Acknowledgement frame too short
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Acknowledgement checksum invalid
 *
 * @note Call iSYS_stopAcquisition() before changing range bound; restart afterwards.
 * @note Persist with saveApplicationSettings() if you want the setting to survive power cycles.
 *
 * @example
 *   // Set device to extended 150 m range
 *   // Ensure acquisition is stopped before calling this
 *   iSYSResult_t res = radar.iSYS_setRangeBound(ISYS_RANGE_0_TO_150, 0x80, 300);
 *   if (res == ERR_OK) {
 *       Serial.println("Range bound set to 0–150 m");
 *   }
 */
iSYSResult_t iSYS4001::iSYS_setRangeBound(iSYSRangeBound_t bound, uint8_t destAddress, uint32_t timeout)
{
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
    command[index++] = 0xD3;
    command[index++] = 0x00;
    command[index++] = 0x10;
    command[index++] = 0x00;
    command[index++] = (bound == ISYS_RANGE_0_TO_150) ? 0x01 : 0x00;

    uint8_t fcs = calculateFCS(command, 4, 10);
    command[index++] = fcs;
    command[index++] = 0x16;

    debugPrintHexFrame("Sending SET range bound command: ", command, sizeof(command));

    size_t bytesWritten = _serial.write(command, sizeof(command));
    _serial.flush();

    if (bytesWritten != sizeof(command))
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    uint8_t response[9];
    size_t responseIndex = 0;
    uint32_t startTime = millis();

    while ((millis() - startTime) < timeout && responseIndex < sizeof(response))
    {
        if (_serial.available())
        {
            response[responseIndex++] = _serial.read();
            if (response[responseIndex - 1] == 0x16)
                break;
        }
    }

    debugPrintHexFrame("Received SET range bound acknowledgement: ", response, responseIndex);

    if (responseIndex == 0)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    if (responseIndex < 9)
    {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }

    if (response[0] != 0x68 || response[1] != 0x03 || response[2] != 0x03 ||
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress ||
        response[6] != 0xD3 || response[8] != 0x16)
    {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }

    if (responseIndex >= 9)
    {
        uint8_t expectedFCS = calculateFCS(response, 4, 6);
        if (response[7] != expectedFCS)
        {
            return ERR_INVALID_CHECKSUM;
        }
    }

    return ERR_OK;
}

/**
 * @brief Get the current sensor range bound (0–50 m or 0–150 m)
 *
 * Queries the iSYS-4001 device to determine which range window is active.
 * This does not require acquisition to be stopped.
 *
 * @param bound Output parameter that will receive the current range bound
 *        - ISYS_RANGE_0_TO_50  → 0 to 50 meters
 *        - ISYS_RANGE_0_TO_150 → 0 to 150 meters
 * @param destAddress Device Radar Destination address (typically 0x80)
 * @param timeout Maximum time in milliseconds to wait for the response
 *
 * @return iSYSResult_t ERR_OK on success, or error code for failure conditions:
 *         - ERR_NULL_POINTER: bound pointer is null
 *         - ERR_TIMEOUT: Timeout parameter is zero
 *         - ERR_COMMAND_NO_DATA_RECEIVED: No response received
 *         - ERR_COMMAND_RX_FRAME_LENGTH: Response frame too short
 *         - ERR_COMMAND_RX_FRAME_DAMAGED: Frame structure invalid
 *         - ERR_INVALID_CHECKSUM: Response checksum invalid
 *
 * @example
 *   iSYSRangeBound_t current;
 *   iSYSResult_t res = radar.iSYS_getRangeBound(&current, 0x80, 300);
 *   if (res == ERR_OK) {
 *       // use 'current'
 *   }
 */
iSYSResult_t iSYS4001::iSYS_getRangeBound(iSYSRangeBound_t *bound, uint8_t destAddress, uint32_t timeout)
{
    if (bound == NULL)
        return ERR_NULL_POINTER;

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
    command[index++] = 0xD2;
    command[index++] = 0x00;
    command[index++] = 0x10;
    uint8_t fcs = calculateFCS(command, 4, 8);
    command[index++] = fcs;
    command[index++] = 0x16;

    debugPrintHexFrame("Sending GET range bound command: ", command, sizeof(command));

    size_t bytesWritten = _serial.write(command, sizeof(command));
    _serial.flush();
    if (bytesWritten != sizeof(command))
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    uint8_t response[11];
    size_t responseIndex = 0;
    uint32_t startTime = millis();

    while ((millis() - startTime) < timeout && responseIndex < sizeof(response))
    {
        if (_serial.available())
        {
            response[responseIndex++] = _serial.read();
            if (response[responseIndex - 1] == 0x16)
                break;
        }
    }

    debugPrintHexFrame("Received GET range bound response: ", response, responseIndex);

    if (responseIndex == 0)
    {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }

    if (responseIndex < 11)
    {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }

    if (response[0] != 0x68 || response[1] != 0x05 || response[2] != 0x05 ||
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress ||
        response[6] != 0xD2 || response[10] != 0x16)
    {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }

    if (responseIndex >= 11)
    {
        uint8_t expectedFCS = calculateFCS(response, 4, 8);
        if (response[9] != expectedFCS)
        {
            return ERR_INVALID_CHECKSUM;
        }
    }

    // Payload carries value 0x00 (near/50m) or 0x01 (far/150m)
    *bound = (response[8] == 0x01) ? ISYS_RANGE_0_TO_150 : ISYS_RANGE_0_TO_50;
    return ERR_OK;
}





































 