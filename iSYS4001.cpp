#include "iSYS4001.h"

// Constructor for iSYS4001 class
// Parameters: serial - reference to HardwareSerial object for UART communication
//             baud - baud rate for serial communication
iSYS4001::iSYS4001(HardwareSerial& serial, uint32_t baud) 
    : _serial(serial), _baud(baud)
{
    // UART will be initialized by the sketch (allows specifying pins on ESP32)
    // This constructor stores the serial interface and baud rate for later use
}


/***************************************************************  
 *  GET TARGET LIST FUNCTION  
 ***************************************************************/

// Main function to get target list from iSYS4001 radar sensor
// Parameters: pTargetList - pointer to structure that will hold the target data
//             destAddress - destination address for the radar sensor
//             timeout - maximum time to wait for response in milliseconds
//             outputnumber - specifies which output to use (defaults to ISYS_OUTPUT_1 if not specified)
// Returns: iSYSResult_t - error code indicating success or failure
iSYSResult_t iSYS4001::getTargetList32(iSYSTargetList_t *pTargetList,uint8_t destAddress,uint32_t timeout,iSYSOutputNumber_t outputnumber) 
{
    // Initialize target list structure with zeros to ensure clean state
    memset(pTargetList, 0, sizeof(iSYSTargetList_t));
    
    // Send the target list request command to the radar sensor
    iSYSResult_t res = sendTargetListRequest(outputnumber, destAddress);
    if (res != ERR_OK) 
    {
        return res;  // If sending failed, return error immediately
    }
    
    // Receive and decode the response from the radar sensor
    res = receiveTargetListResponse(pTargetList, timeout);
    if (res != ERR_OK) 
    {
        return res;  // If receiving/decoding failed, return error
    }
    
    return ERR_OK;  // Success - target list has been populated
}

// Function to send target list request command to the radar sensor
// Parameters: outputnumber - specifies which output to use
//             destAddress - destination address for the radar sensor
// Returns: iSYSResult_t - error code (always ERR_OK for this function)
iSYSResult_t iSYS4001::sendTargetListRequest(iSYSOutputNumber_t outputnumber, uint8_t destAddress) 
{
    // Based on the protocol from the image: 68 05 05 68 80 01 DA 01 20 7C 16    68 05 05 68 80 01 DA 01 20 7C 16
    // Frame structure: SD2 LE LEr SD2 DA SA FC PDU FCS ED
    // SD2 = Start Delimiter 2 (0x68), LE = Length, LEr = Length repeat, DA = Destination Address, 
    // SA = Source Address, FC = Function Code, PDU = Protocol Data Unit, FCS = Frame Check Sequence, ED = End Delimiter
    
    uint8_t command[11];  // Array to hold the complete command frame
    uint8_t index = 0;    // Index for building the command array
    
    // Build the command frame byte by byte according to iSYS protocol
    command[index++] = 0x68;  // SD2
    command[index++] = 0x05;  // LE
    command[index++] = 0x05;  // LEr
    command[index++] = 0x68;  // 
    command[index++] = destAddress;  // DA
    command[index++] = 0x01;  // SA
    command[index++] = 0xDA;  // FC
    command[index++] = outputnumber;  // PDU - Output number
    command[index++] = 0x20;  // 32-bit resolution flag (0x20 = 32-bit mode)
    
    // Calculate FCS (Frame Check Sequence) - sum of bytes from DA to PDU
    uint8_t fcs = calculateFCS(command, 3+1, index-1);
    command[index++] = fcs;  // FCS - Frame Check Sequence for error detection
    command[index++] = 0x16; // ED - End Delimiter
    
    // Debug: Print the command frame being sent to radar
    Serial.print("Sending command to radar: ");
    for (int i = 0; i < 11; i++) 
    {
        Serial.print("0x");
        if (command[i] < 0x10) 
        {
            Serial.print("0");  // Add leading zero for single digit hex
        }
        Serial.print(command[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    // Send the complete command frame over serial interface
    _serial.write(command, 11);
    _serial.flush();  // Ensure all data is transmitted before continuing
    return ERR_OK;
}

// Function to receive and process target list response from radar sensor
// Parameters: pTargetList - pointer to structure that will hold the decoded target data
//             timeout - maximum time to wait for response in milliseconds
// Returns: iSYSResult_t - error code indicating success or failure
iSYSResult_t iSYS4001::receiveTargetListResponse(iSYSTargetList_t *pTargetList, uint32_t timeout) 
{
    uint32_t startTime = millis();
    std::vector<uint8_t> buffer;

    // Read the first 6 bytes (header)
    while ((millis() - startTime) < timeout && buffer.size() < 6) {
        if (_serial.available()) {
            buffer.push_back(_serial.read());
        }
    }

    if (buffer.size() < 6) {
        return ERR_COMMAND_NO_DATA_RECEIVED; // Timeout before header complete
    }

    // Extract number of targets (6th byte)
    uint8_t nrOfTargets = buffer[5];
    if (nrOfTargets > MAX_TARGETS) {
        return ERR_COMMAND_MAX_DATA_OVERFLOW; // Too many targets
    }

    // Calculate expected frame length
    uint16_t expectedLength = 6 + (14 * nrOfTargets) + 2;
    buffer.reserve(expectedLength);

    // Read until full frame is received
    while ((millis() - startTime) < timeout && buffer.size() < expectedLength) {
        if (_serial.available()) {
            buffer.push_back(_serial.read());
        }
    }

    if (buffer.size() != expectedLength) {
        return ERR_COMMAND_NO_DATA_RECEIVED; // Incomplete frame
    }

    // Validate end delimiter (last byte must be 0x16)
    if (buffer.back() != 0x16) {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }

    // Debug: print received frame
    Serial.print("Received response from radar: ");
    for (size_t i = 0; i < buffer.size(); i++) {
        Serial.printf("0x%02X ", buffer[i]);
    }
    Serial.println();

        
// Minimum valid response length reached (11 bytes minimum)
// Decode the received frame using the decodeTargetFrame function
    iSYSResult_t res = decodeTargetFrame(buffer.data(),buffer.size(),4001, 32, pTargetList);
    return res;  // Return the result of decoding
}


// Function to decode target frame data from radar sensor response
// Parameters: frame_array - array containing the received frame data
//             nrOfElements - number of bytes in the frame array
//             productcode - product code of the radar sensor (4001, 4004, 6003, etc.)
//             bitrate - resolution mode (16-bit or 32-bit)
//             targetList - pointer to structure that will hold decoded target data
// Returns: iSYSResult_t - error code indicating success or failure
iSYSResult_t iSYS4001::decodeTargetFrame(uint8_t *frame_array, uint16_t nrOfElements,uint16_t productcode, uint8_t bitrate,iSYSTargetList_t *targetList) 
{
    // Validate input parameters
    if (frame_array == NULL || targetList == NULL)
    {
        return ERR_NULL_POINTER;  // Return error if pointers are null
    }
    
    if (nrOfElements < 6)
    {
        return ERR_COMMAND_RX_FRAME_LENGTH;  // Frame too short to be valid
    }

     if (frame_array[nrOfElements - 1] != 0x16) 
    {
        return ERR_COMMAND_NO_VALID_FRAME_FOUND; 
    }

    // Determine frame type and calculate frame control offset
    uint16_t ui16_fc;
    if (frame_array[0] == 0x68) 
    {
        ui16_fc = 6; // variable-length frame (starts with 0x68)
    } 
    else if (frame_array[0] == 0xA2) 
    {
        ui16_fc = 3; // fixed-length frame (different start byte)
    }
    else 
    {
        return ERR_COMMAND_NO_VALID_FRAME_FOUND;
    }

    // Extract frame header information
    uint8_t output_number = static_cast<uint8_t>(frame_array[ui16_fc + 1] & 0xFF);  // Output number
    uint8_t nrOfTargets = static_cast<uint8_t>(frame_array[ui16_fc + 2] & 0xFF);    // Number of targets
    uint8_t* pData = &frame_array[ui16_fc + 3];  // Pointer to start of target data

    // Validate number of targets (0xFF indicates clipping, other values must be <= MAX_TARGETS)
    if ((nrOfTargets > MAX_TARGETS) && (nrOfTargets != 0xFF)) 
    {
        return ERR_COMMAND_FAILURE;  // Too many targets
    }

    // Process target data if not in clipping mode
    if (nrOfTargets != 0xFF) 
    {
        // Initialize all target structures with zeros
        for (uint8_t i = 0; i < MAX_TARGETS; i++) 
        {
            targetList->targets[i].angle = 0;    // Target angle in degrees
            targetList->targets[i].range = 0;    // Target range in meters
            targetList->targets[i].signal = 0;   // Target signal strength
            targetList->targets[i].velocity = 0; // Target velocity in m/s
        }

        // Set target list header information
        targetList->nrOfTargets = nrOfTargets;      // Number of valid targets
        targetList->clippingFlag = 0;               // No clipping detected
        targetList->outputNumber = output_number;   // Output number from frame

        // Decode target data based on resolution mode (32-bit or 16-bit)
        if (bitrate == 32) 
        {
            // 32-bit resolution mode - higher precision data
            for (uint8_t i = 0; i < nrOfTargets; i++) 
            {
                // Decode signal strength (16-bit, scaled by 0.01)
                int16_t tmp = (static_cast<int16_t>(pData[0]) << 8) | pData[1];
                pData += 2;  // Move pointer to next data field
                targetList->targets[i].signal = static_cast<float>(tmp) * 0.01f;

                // Decode velocity (32-bit, scaled by 0.001 m/s)
                int32_t tmp32 = (static_cast<int32_t>(pData[0]) << 24) |
                                 (static_cast<int32_t>(pData[1]) << 16) |
                                 (static_cast<int32_t>(pData[2]) << 8)  |
                                  static_cast<int32_t>(pData[3]);
                pData += 4;  // Move pointer to next data field
                targetList->targets[i].velocity = static_cast<float>(tmp32) * 0.001f;

                // Decode range (32-bit, scaled by 1e-6 meters)
                tmp32 = (static_cast<int32_t>(pData[0]) << 24) |
                        (static_cast<int32_t>(pData[1]) << 16) |
                        (static_cast<int32_t>(pData[2]) << 8)  |
                         static_cast<int32_t>(pData[3]);
                pData += 4;  // Move pointer to next data field
                targetList->targets[i].range = static_cast<float>(tmp32) * 1e-6f;

                // Decode angle (32-bit, scaled by 0.01 degrees)
                tmp32 = (static_cast<int32_t>(pData[0]) << 24) |
                        (static_cast<int32_t>(pData[1]) << 16) |
                        (static_cast<int32_t>(pData[2]) << 8)  |
                         static_cast<int32_t>(pData[3]);
                pData += 4;  // Move pointer to next data field
                targetList->targets[i].angle = static_cast<float>(tmp32) * 0.001f;
            }
        }

    } 
    else 
    {
        // Clipping mode detected (0xFF targets) - sensor is overloaded
        targetList->clippingFlag = 1;  // Set clipping flag to indicate overload
    }

    // Set error status based on number of targets
    if (nrOfTargets == MAX_TARGETS) 
    {
        targetList->error.iSYSTargetListError = TARGET_LIST_FULL;  // Maximum targets reached
    } 
    else 
    {
        targetList->error.iSYSTargetListError = TARGET_LIST_OK;    // Normal operation
    }
    
    return ERR_OK;  // Success - target list has been decoded
}




/***************************************************************  
 *  SET RANGE MIN/MAX FUNCTIONS
 ***************************************************************/

//``````````````````````````````````````````````````````````` SET RANGE MIN FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::iSYS_setOutputRangeMin(iSYSOutputNumber_t outputnumber, uint16_t range, uint8_t destAddress, uint32_t timeout)
{
    // Input parameter validation
    if (outputnumber < ISYS_OUTPUT_1 || outputnumber > ISYS_OUTPUT_3) {
        return ERR_PARAMETER_OUT_OF_RANGE;
    }
    
    if (range > 150) {
        return ERR_PARAMETER_OUT_OF_RANGE;
    }
    
    if (timeout == 0) {
        return ERR_TIMEOUT;
    }
    

    uint8_t command[13];
    uint8_t index = 0;
    uint16_t scaledRange = range * 10;
    uint8_t minHighByte = (scaledRange >> 8) & 0xFF;
    uint8_t minLowByte = scaledRange & 0xFF;

    // Build command frame
    command[index++] = 0x68; // SD2
    command[index++] = 0x07; // LE
    command[index++] = 0x07; // LEr
    command[index++] = 0x68; // SD2
    command[index++] = destAddress; // DA
    command[index++] = 0x01; // SA
    command[index++] = 0xD5; // FC
    command[index++] = outputnumber; // PDU (output number)
    command[index++] = 0x08; // Mode/flag for min range
    command[index++] = minHighByte; // High byte of range
    command[index++] = minLowByte; // Low byte of range

    // Calculate checksum (sum of bytes 4 to 10)
    uint8_t fcs = calculateFCS(command, 4, 10);
    
    command[11] = fcs; // Checksum
    command[12] = 0x16;

    for (int i = 0; i < 13; i++) 
    {
        Serial.print("0x");
        if (command[i] < 0x10) Serial.print("0");
        Serial.print(command[i], HEX);
        Serial.print(" ");
    }
    Serial.println();


    
    // Send command and check if write was successful
    size_t bytesWritten = _serial.write(command, 13);
    _serial.flush();

    // Response Buffer
    uint8_t response[9];
    size_t minIndex = 0;
    uint32_t startTime = millis();

    // Read response with timeout
    while ((millis() - startTime) < timeout && minIndex < sizeof(response)) {
        if (_serial.available()) {
            response[minIndex++] = _serial.read();

            if (response[minIndex-1] == 0x16) break;
        }
    }
    

    for (int i = 0; i < minIndex; i++) 
    {
        Serial.print("0x");
        if (response[i] < 0x10) Serial.print("0");
        Serial.print(response[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    // Comprehensive response validation
    if (minIndex == 0) {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }
    
    if (minIndex < 9) {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }
    
    // Validate response frame structure
    if (response[0] != 0x68 || response[1] != 0x03 || response[2] != 0x03 || 
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress || 
        response[6] != 0xD5 || response[8] != 0x16) {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }
    
    // Validate checksum if present in response
    if (minIndex >= 9) {
        uint8_t expectedFCS = calculateFCS(response, 4, 6); // Calculate expected FCS for DA..FC
        if (response[7] != expectedFCS) {
            return ERR_INVALID_CHECKSUM;
        }
    }
    return ERR_OK;
}



//``````````````````````````````````````````````````````````` SET RANGE MAX FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::iSYS_setOutputRangeMax(iSYSOutputNumber_t outputnumber, uint16_t range, uint8_t destAddress, uint32_t timeout)
{
    // Input parameter validation
    if (outputnumber < ISYS_OUTPUT_1 || outputnumber > ISYS_OUTPUT_3) {
        return ERR_PARAMETER_OUT_OF_RANGE;
    }
    
    if (range > 150) {
        return ERR_PARAMETER_OUT_OF_RANGE;
    }
    
    if (timeout == 0) {
        return ERR_TIMEOUT;
    }
    

    uint8_t command[13];
    uint8_t index = 0;
    uint16_t scaledRange = range * 10;
    uint8_t maxHighByte = (scaledRange >> 8) & 0xFF;
    uint8_t maxLowByte = scaledRange & 0xFF;

    // Build command frame
    command[index++] = 0x68; // SD2
    command[index++] = 0x07; // LE
    command[index++] = 0x07; // LEr
    command[index++] = 0x68; // SD2
    command[index++] = destAddress; // DA
    command[index++] = 0x01; // SA
    command[index++] = 0xD5; // FC
    command[index++] = outputnumber; // PDU (output number)
    command[index++] = 0x09; // Mode/flag for max range
    command[index++] = maxHighByte; // High byte of range
    command[index++] = maxLowByte; // Low byte of range

    // Calculate checksum (sum of bytes 4 to 10)
    uint8_t fcs = calculateFCS(command, 4, 10);
    
    command[index++] = fcs; // Checksum
    command[index++] = 0x16;
    
    for (int i = 0; i < 13; i++) 
    {
        Serial.print("0x");
        if (command[i] < 0x10) Serial.print("0");
        Serial.print(command[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    
    // Send command and check if write was successful
    size_t bytesWritten = _serial.write(command, 13);
    _serial.flush();

    uint8_t response[9];
    uint32_t startTime = millis();
    size_t maxIndex = 0;
    
    
    // Read response with timeout
    while ((millis() - startTime) < timeout && maxIndex < 9) {
        if (_serial.available()) {
            response[maxIndex++] = _serial.read();
            if (response[maxIndex-1] == 0x16) break; // End delimiter
        }
    }

    for (int i = 0; i < maxIndex; i++) 
    {
        Serial.print("0x");
        if (response[i] < 0x10) Serial.print("0");
        Serial.print(response[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    // Comprehensive response validation
    if (maxIndex == 0) {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }
    
    if (maxIndex < 9) {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }
    
    // Validate response frame structure
    if (response[0] != 0x68 || response[1] != 0x03 || response[2] != 0x03 || 
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress || 
        response[6] != 0xD5 || response[8] != 0x16) {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }
    
    // Validate checksum if present in response
    if (maxIndex >= 9) {
        uint8_t expectedFCS = calculateFCS(response, 4, 6); // Calculate expected FCS for DA..FC
        if (response[7] != expectedFCS) {
            return ERR_INVALID_CHECKSUM;
        }
    }
    
    return ERR_OK;
}



/***************************************************************  
 *  SET VELOCITY MIN/MAX FUNCTIONS 
 ***************************************************************/

//``````````````````````````````````````````````````````````` SET VELOCITY MIN FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::iSYS_setOutputVelocityMin(iSYSOutputNumber_t outputnumber, uint16_t velocity, uint8_t destAddress, uint32_t timeout)
{
    // Input parameter validation
    if (outputnumber < ISYS_OUTPUT_1 || outputnumber > ISYS_OUTPUT_3) {
        return ERR_PARAMETER_OUT_OF_RANGE;
    }
    
    if (velocity > 250) {
        return ERR_PARAMETER_OUT_OF_RANGE;
    }
    
    if (timeout == 0) {
        return ERR_TIMEOUT;
    }
    

    uint8_t command[13];
    uint8_t index = 0;
    uint16_t scaledVelocity = (uint16_t)round((velocity / 3.6f) * 10.0f);
    uint8_t minHighByte = (scaledVelocity >> 8) & 0xFF;
    uint8_t minLowByte = scaledVelocity & 0xFF;

    // Build command frame
    command[index++] = 0x68; // SD2
    command[index++] = 0x07; // LE
    command[index++] = 0x07; // LEr
    command[index++] = 0x68; // SD2
    command[index++] = destAddress; // DA
    command[index++] = 0x01; // SA
    command[index++] = 0xD5; // FC
    command[index++] = outputnumber; // PDU (output number)
    command[index++] = 0x0C; // Mode/flag for min range
    command[index++] = minHighByte; // High byte of range
    command[index++] = minLowByte; // Low byte of range

    // Calculate checksum (sum of bytes 4 to 10)
    uint8_t fcs = calculateFCS(command, 4, 10);
    
    command[11] = fcs; // Checksum
    command[12] = 0x16;

    for (int i = 0; i < 13; i++) 
    {
        Serial.print("0x");
        if (command[i] < 0x10) Serial.print("0");
        Serial.print(command[i], HEX);
        Serial.print(" ");
    }
    Serial.println();


    
    // Send command and check if write was successful
    size_t bytesWritten = _serial.write(command, 13);
    _serial.flush();

    // Response Buffer
    uint8_t response[9];
    size_t minIndex = 0;
    uint32_t startTime = millis();

    // Read response with timeout
    while ((millis() - startTime) < timeout && minIndex < sizeof(response)) {
        if (_serial.available()) {
            response[minIndex++] = _serial.read();

            if (response[minIndex-1] == 0x16) break;
        }
    }
    

    for (int i = 0; i < minIndex; i++) 
    {
        Serial.print("0x");
        if (response[i] < 0x10) Serial.print("0");
        Serial.print(response[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    // Comprehensive response validation
    if (minIndex == 0) {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }
    
    if (minIndex < 9) {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }
    
    // Validate response frame structure
    if (response[0] != 0x68 || response[1] != 0x03 || response[2] != 0x03 || 
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress || 
        response[6] != 0xD5 || response[8] != 0x16) {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }
    
    // Validate checksum if present in response
    if (minIndex >= 9) {
        uint8_t expectedFCS = calculateFCS(response, 4, 6); // Calculate expected FCS for DA..FC
        if (response[7] != expectedFCS) {
            return ERR_INVALID_CHECKSUM;
        }
    }
    return ERR_OK;
}



//``````````````````````````````````````````````````````````` SET VELOCITY MAX FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::iSYS_setOutputVelocityMax(iSYSOutputNumber_t outputnumber, uint16_t velocity, uint8_t destAddress, uint32_t timeout)
{
    // Input parameter validation
    if (outputnumber < ISYS_OUTPUT_1 || outputnumber > ISYS_OUTPUT_3) {
        return ERR_PARAMETER_OUT_OF_RANGE;
    }
    
    if (velocity > 250) {
        return ERR_PARAMETER_OUT_OF_RANGE;
    }
    
    if (timeout == 0) {
        return ERR_TIMEOUT;
    }
    

    uint8_t command[13];
    uint8_t index = 0;
    uint16_t scaledVelocity = (uint16_t)round((velocity / 3.6f) * 10.0f);
    uint8_t maxHighByte = (scaledVelocity >> 8) & 0xFF;
    uint8_t maxLowByte = scaledVelocity & 0xFF;

    // Build command frame
    command[index++] = 0x68; // SD2
    command[index++] = 0x07; // LE
    command[index++] = 0x07; // LEr
    command[index++] = 0x68; // SD2
    command[index++] = destAddress; // DA
    command[index++] = 0x01; // SA
    command[index++] = 0xD5; // FC
    command[index++] = outputnumber; // PDU (output number)
    command[index++] = 0x0D; // Mode/flag for max range
    command[index++] = maxHighByte; // High byte of range
    command[index++] = maxLowByte; // Low byte of range

    // Calculate checksum (sum of bytes 4 to 10)
    uint8_t fcs = calculateFCS(command, 4, 10);
    
    command[index++] = fcs; // Checksum
    command[index++] = 0x16;
    
    for (int i = 0; i < 13; i++) 
    {
        Serial.print("0x");
        if (command[i] < 0x10) Serial.print("0");
        Serial.print(command[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    
    // Send command and check if write was successful
    size_t bytesWritten = _serial.write(command, 13);
    _serial.flush();

    uint8_t response[9];
    uint32_t startTime = millis();
    size_t maxIndex = 0;
    
    
    // Read response with timeout
    while ((millis() - startTime) < timeout && maxIndex < 9) {
        if (_serial.available()) {
            response[maxIndex++] = _serial.read();
            if (response[maxIndex-1] == 0x16) break; // End delimiter
        }
    }

    for (int i = 0; i < maxIndex; i++) 
    {
        Serial.print("0x");
        if (response[i] < 0x10) Serial.print("0");
        Serial.print(response[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    // Comprehensive response validation
    if (maxIndex == 0) {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }
    
    if (maxIndex < 9) {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }
    
    // Validate response frame structure
    if (response[0] != 0x68 || response[1] != 0x03 || response[2] != 0x03 || 
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress || 
        response[6] != 0xD5 || response[8] != 0x16) {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }
    
    // Validate checksum if present in response
    if (maxIndex >= 9) {
        uint8_t expectedFCS = calculateFCS(response, 4, 6); // Calculate expected FCS for DA..FC
        if (response[7] != expectedFCS) {
            return ERR_INVALID_CHECKSUM;
        }
    }
    
    return ERR_OK;
}





/***************************************************************  
 *  SIGNAL MIN MAX FUNCTIONS
 ***************************************************************/

//``````````````````````````````````````````````````````````` SET SIGNAL MIN FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::iSYS_setOutputSignalMin(iSYSOutputNumber_t outputnumber, uint16_t signal, uint8_t destAddress, uint32_t timeout)
{
    // Input parameter validation
    if (outputnumber < ISYS_OUTPUT_1 || outputnumber > ISYS_OUTPUT_3) {
        return ERR_PARAMETER_OUT_OF_RANGE;
    }
    
    if (signal > 150) {
        return ERR_PARAMETER_OUT_OF_RANGE;
    }
    
    if (timeout == 0) {
        return ERR_TIMEOUT;
    }
    

    uint8_t command[13];
    uint8_t index = 0;
    uint16_t scaledSignal = signal * 10;
    uint8_t minHighByte = (scaledSignal >> 8) & 0xFF;
    uint8_t minLowByte = scaledSignal & 0xFF;

    // Build command frame
    command[index++] = 0x68; // SD2
    command[index++] = 0x07; // LE
    command[index++] = 0x07; // LEr
    command[index++] = 0x68; // SD2
    command[index++] = destAddress; // DA
    command[index++] = 0x01; // SA
    command[index++] = 0xD5; // FC
    command[index++] = outputnumber; // PDU (output number)
    command[index++] = 0x0A; // Mode/flag for min range
    command[index++] = minHighByte; // High byte of range
    command[index++] = minLowByte; // Low byte of range

    // Calculate checksum (sum of bytes 4 to 10)
    uint8_t fcs = calculateFCS(command, 4, 10);
    
    command[11] = fcs; // Checksum
    command[12] = 0x16;

    for (int i = 0; i < 13; i++) 
    {
        Serial.print("0x");
        if (command[i] < 0x10) Serial.print("0");
        Serial.print(command[i], HEX);
        Serial.print(" ");
    }
    Serial.println();


    
    // Send command and check if write was successful
    size_t bytesWritten = _serial.write(command, 13);
    _serial.flush();

    // Response Buffer
    uint8_t response[9];
    size_t minIndex = 0;
    uint32_t startTime = millis();

    // Read response with timeout
    while ((millis() - startTime) < timeout && minIndex < sizeof(response)) {
        if (_serial.available()) {
            response[minIndex++] = _serial.read();

            if (response[minIndex-1] == 0x16) break;
        }
    }
    

    for (int i = 0; i < minIndex; i++) 
    {
        Serial.print("0x");
        if (response[i] < 0x10) Serial.print("0");
        Serial.print(response[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    // Comprehensive response validation
    if (minIndex == 0) {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }
    
    if (minIndex < 9) {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }
    
    // Validate response frame structure
    if (response[0] != 0x68 || response[1] != 0x03 || response[2] != 0x03 || 
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress || 
        response[6] != 0xD5 || response[8] != 0x16) {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }
    
    // Validate checksum if present in response
    if (minIndex >= 9) {
        uint8_t expectedFCS = calculateFCS(response, 4, 6); // Calculate expected FCS for DA..FC
        if (response[7] != expectedFCS) {
            return ERR_INVALID_CHECKSUM;
        }
    }
    return ERR_OK;
}



//``````````````````````````````````````````````````````````` SET SIGNAL MAX FUNCTION ```````````````````````````````````````````````````````````//

iSYSResult_t iSYS4001::iSYS_setOutputSignalMax(iSYSOutputNumber_t outputnumber, uint16_t signal, uint8_t destAddress, uint32_t timeout)
{
    // Input parameter validation
    if (outputnumber < ISYS_OUTPUT_1 || outputnumber > ISYS_OUTPUT_3) {
        return ERR_PARAMETER_OUT_OF_RANGE;
    }
    
    if (signal > 150) {
        return ERR_PARAMETER_OUT_OF_RANGE;
    }
    
    if (timeout == 0) {
        return ERR_TIMEOUT;
    }
    

    uint8_t command[13];
    uint8_t index = 0;
    uint16_t scaledRange = signal * 10;
    uint8_t maxHighByte = (scaledRange >> 8) & 0xFF;
    uint8_t maxLowByte = scaledRange & 0xFF;

    // Build command frame
    command[index++] = 0x68; // SD2
    command[index++] = 0x07; // LE
    command[index++] = 0x07; // LEr
    command[index++] = 0x68; // SD2
    command[index++] = destAddress; // DA
    command[index++] = 0x01; // SA
    command[index++] = 0xD5; // FC
    command[index++] = outputnumber; // PDU (output number)
    command[index++] = 0x0B; // Mode/flag for max range
    command[index++] = maxHighByte; // High byte of range
    command[index++] = maxLowByte; // Low byte of range

    // Calculate checksum (sum of bytes 4 to 10)
    uint8_t fcs = calculateFCS(command, 4, 10);
    
    command[index++] = fcs; // Checksum
    command[index++] = 0x16;
    
    for (int i = 0; i < 13; i++) 
    {
        Serial.print("0x");
        if (command[i] < 0x10) Serial.print("0");
        Serial.print(command[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    
    // Send command and check if write was successful
    size_t bytesWritten = _serial.write(command, 13);
    _serial.flush();

    uint8_t response[9];
    uint32_t startTime = millis();
    size_t maxIndex = 0;
    
    
    // Read response with timeout
    while ((millis() - startTime) < timeout && maxIndex < 9) {
        if (_serial.available()) {
            response[maxIndex++] = _serial.read();
            if (response[maxIndex-1] == 0x16) break; // End delimiter
        }
    }

    for (int i = 0; i < maxIndex; i++) 
    {
        Serial.print("0x");
        if (response[i] < 0x10) Serial.print("0");
        Serial.print(response[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    // Comprehensive response validation
    if (maxIndex == 0) {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }
    
    if (maxIndex < 9) {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }
    
    // Validate response frame structure
    if (response[0] != 0x68 || response[1] != 0x03 || response[2] != 0x03 || 
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress || 
        response[6] != 0xD5 || response[8] != 0x16) {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }
    
    // Validate checksum if present in response
    if (maxIndex >= 9) {
        uint8_t expectedFCS = calculateFCS(response, 4, 6); // Calculate expected FCS for DA..FC
        if (response[7] != expectedFCS) {
            return ERR_INVALID_CHECKSUM;
        }
    }
    
    return ERR_OK;
}



/***************************************************************  
 *  SET VELOCITY DIRECTION FUNCTION 
 ***************************************************************/

iSYSResult_t iSYS4001::iSYS_setOutputDirection(iSYSOutputNumber_t outputnumber, iSYSDirection_type_t direction, uint8_t destAddress, uint32_t timeout)
{
    // Input parameter validation
    if (outputnumber < ISYS_OUTPUT_1 || outputnumber > ISYS_OUTPUT_3) {
        return ERR_PARAMETER_OUT_OF_RANGE;
    }

    
    if (timeout == 0) {
        return ERR_TIMEOUT;
    }
    

    uint8_t command[13];
    uint8_t index = 0;


    // Build command frame
    command[index++] = 0x68; // SD2
    command[index++] = 0x07; // LE
    command[index++] = 0x07; // LEr
    command[index++] = 0x68; // SD2
    command[index++] = destAddress; // DA
    command[index++] = 0x01; // SA
    command[index++] = 0xD5; // FC
    command[index++] = outputnumber; // PDU (output number)
    command[index++] = 0x0E; // Mode/flag for max range
    command[index++] = 0X00; // High byte of range
    command[index++] = (uint8_t)direction; // Low byte of range

    // Calculate checksum (sum of bytes 4 to 10)
    uint8_t fcs = calculateFCS(command, 4, 10);
    
    command[index++] = fcs; // Checksum
    command[index++] = 0x16;
    
    for (int i = 0; i < 13; i++) 
    {
        Serial.print("0x");
        if (command[i] < 0x10) Serial.print("0");
        Serial.print(command[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    
    // Send command and check if write was successful
    size_t bytesWritten = _serial.write(command, 13);
    _serial.flush();

    uint8_t response[9];
    uint32_t startTime = millis();
    size_t maxIndex = 0;
    
    
    // Read response with timeout
    while ((millis() - startTime) < timeout && maxIndex < 9) {
        if (_serial.available()) {
            response[maxIndex++] = _serial.read();
            if (response[maxIndex-1] == 0x16) break; // End delimiter
        }
    }

    for (int i = 0; i < maxIndex; i++) 
    {
        Serial.print("0x");
        if (response[i] < 0x10) Serial.print("0");
        Serial.print(response[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    // Comprehensive response validation
    if (maxIndex == 0) {
        return ERR_COMMAND_NO_DATA_RECEIVED;
    }
    
    if (maxIndex < 9) {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }
    
    // Validate response frame structure
    if (response[0] != 0x68 || response[1] != 0x03 || response[2] != 0x03 || 
        response[3] != 0x68 || response[4] != 0x01 || response[5] != destAddress || 
        response[6] != 0xD5 || response[8] != 0x16) {
        return ERR_COMMAND_RX_FRAME_DAMAGED;
    }
    
    // Validate checksum if present in response
    if (maxIndex >= 9) {
        uint8_t expectedFCS = calculateFCS(response, 4, 6); // Calculate expected FCS for DA..FC
        if (response[7] != expectedFCS) {
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
iSYSResult_t iSYS4001::sendEEPROMCommand(iSYSEEPROMSubFunction_t subFunction,uint8_t destAddress,uint32_t timeout) 
{
    // Send the EEPROM command frame to the radar sensor
    iSYSResult_t res = sendEEPROMCommandFrame(subFunction, destAddress);
    if (res != ERR_OK) 
    {
        return res;  // If sending failed, return error immediately
    }
    
    // Receive and verify the acknowledgement response from the radar sensor
    res = receiveEEPROMAcknowledgement(destAddress, timeout);
    if (res != ERR_OK) 
    {
        return res;  // If receiving/verification failed, return error
    }
    
    return ERR_OK;  // Success - EEPROM command acknowledged
}

// Convenience function to set factory settings
// Parameters: destAddress - destination address for the radar sensor
//             timeout - maximum time to wait for response in milliseconds
// Returns: iSYSResult_t - error code indicating success or failure
iSYSResult_t iSYS4001::setFactorySettings(uint8_t destAddress, uint32_t timeout) 
{
    return sendEEPROMCommand(ISYS_EEPROM_SET_FACTORY_SETTINGS, destAddress, timeout);
}

// Convenience function to save sensor settings
// Parameters: destAddress - destination address for the radar sensor
//             timeout - maximum time to wait for response in milliseconds
// Returns: iSYSResult_t - error code indicating success or failure
iSYSResult_t iSYS4001::saveSensorSettings(uint8_t destAddress, uint32_t timeout) 
{
    return sendEEPROMCommand(ISYS_EEPROM_SAVE_SENSOR_SETTINGS, destAddress, timeout);
}

// Convenience function to save application settings
// Parameters: destAddress - destination address for the radar sensor
//             timeout - maximum time to wait for response in milliseconds
// Returns: iSYSResult_t - error code indicating success or failure
iSYSResult_t iSYS4001::saveApplicationSettings(uint8_t destAddress, uint32_t timeout) 
{
    return sendEEPROMCommand(ISYS_EEPROM_SAVE_APPLICATION_SETTINGS, destAddress, timeout);
}

// Convenience function to save all settings (sensor + application)
// Parameters: destAddress - destination address for the radar sensor
//             timeout - maximum time to wait for response in milliseconds
// Returns: iSYSResult_t - error code indicating success or failure
iSYSResult_t iSYS4001::saveAllSettings(uint8_t destAddress, uint32_t timeout) 
{
    return sendEEPROMCommand(ISYS_EEPROM_SAVE_ALL_SETTINGS, destAddress, timeout);
}

// Function to send EEPROM command frame to the radar sensor
// Parameters: subFunction - EEPROM sub-function code
//             destAddress - destination address for the radar sensor
// Returns: iSYSResult_t - error code (always ERR_OK for this function)
iSYSResult_t iSYS4001::sendEEPROMCommandFrame(iSYSEEPROMSubFunction_t subFunction, uint8_t destAddress) 
{
    // Based on the protocol documentation:
    // Frame structure: SD2 LE LEr SD2 DA SA FC PDU FCS ED
    // For EEPROM commands: FC = 0xDF, PDU = sub-function code
    
    uint8_t command[10];
    uint8_t index = 0;
    
    // Build the command frame byte by byte according to iSYS protocol
    command[index++] = 0x68;  // SD2
    command[index++] = 0x04;  // LE
    command[index++] = 0x04;  // LEr
    command[index++] = 0x68;  // SD2
    command[index++] = destAddress;  // DA
    command[index++] = 0x01;  // SA
    command[index++] = 0xDF;  // FC
    command[index++] = subFunction;  // PDU - Sub-function code
    
    // Calculate FCS (Frame Check Sequence) - sum of bytes from DA to PDU
    uint8_t fcs = calculateFCS(command, 4, 7);
    command[index++] = fcs;  // FCS
    command[index++] = 0x16; // ED
    
    // Debug
    Serial.print("Sending EEPROM command to radar: ");
    for (int i = 0; i < 10; i++) 
    {
        Serial.print("0x");
        if (command[i] < 0x10) 
        {
            Serial.print("0");  // Add leading zero for single digit hex
        }
        Serial.print(command[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    
    _serial.write(command, 10);
    _serial.flush();
    return ERR_OK;
}

// Function to receive and verify EEPROM acknowledgement from radar sensor
// Parameters: timeout - maximum time to wait for response in milliseconds
// Returns: iSYSResult_t - error code indicating success or failure
iSYSResult_t iSYS4001::receiveEEPROMAcknowledgement(uint8_t destAddress,uint32_t timeout) 
{
    uint32_t startTime = millis();  // Record start time for timeout calculation
    uint8_t buffer[256];            // Buffer to store incoming response data
    uint16_t index = 0;             // Index for storing received bytes
    uint8_t byte;
    
    // Wait for response with timeout protection
    while ((millis() - startTime) < timeout) 
    {
        if (_serial.available()) 
        {  // Check if data is available to read
            byte = _serial.read();  // Read one byte from serial
            buffer[index++] = byte;         // Store byte in buffer
            
            // Check for end delimiter (0x16) which indicates end of frame
            if (byte == 0x16 && index >= 9) 
            {
                // Debug: Print the received acknowledgement frame from radar
                for (int i = 0; i < index; i++)
                {
                    Serial.print("0x");
                    if (buffer[i] < 0x10) 
                    {
                        Serial.print("0");  // Add leading zero for single digit hex
                    }
                    Serial.print(buffer[i], HEX);
                    Serial.print(" ");
                }
                Serial.println();
                
                // Verify the acknowledgement frame structure
                // Expected: 68 03 03 68 01 80 DF 60 16
                if (index == 9 && 
                    buffer[0] == 0x68 && buffer[1] == 0x03 && buffer[2] == 0x03 &&
                    buffer[3] == 0x68 && buffer[4] == 0x01 && buffer[5] == destAddress &&
                    buffer[6] == 0xDF && buffer[8] == 0x16) 
                    //&& buffer[7] == 0x60
                {
                    return ERR_OK;  // Valid acknowledgement received
                }
                else 
                {
                    return ERR_COMMAND_RX_FRAME_DAMAGED;  // Invalid frame structure
                }
            }
            
            // Prevent buffer overflow by checking array bounds
            if (index >= 256) 
            {
                return ERR_COMMAND_MAX_DATA_OVERFLOW;  // Buffer overflow error
            }
        }
    }
    
    return ERR_COMMAND_NO_DATA_RECEIVED; // Timeout error - no response received
}





/***************************************************************  
 *  CALCULATE CHECKSUM FUNCTION
 ***************************************************************/


// Helper function to calculate Frame Check Sequence (FCS)
// Parameters: data - array containing the frame data
//             startIndex - starting index for FCS calculation
//             endIndex - ending index for FCS calculation (inclusive)
// Returns: uint8_t - calculated FCS value
uint8_t iSYS4001::calculateFCS(const uint8_t* data, uint8_t startIndex, uint8_t endIndex) 
{
    uint8_t fcs = 0;
    for (uint8_t i = startIndex; i <= endIndex; i++) 
    {
        fcs = (uint8_t)(fcs + data[i]);  // Sum of bytes from startIndex to endIndex
    }
    return fcs;
}






/***************************************************************  
 *  DEVICE ADDRESS FUNCTIONS  
 ***************************************************************/

// Function to set a new RS485 device address on the sensor
// Parameters: deviceaddress - the new sensor address to set
//             destAddress   - the current address of the sensor (or broadcast if supported)
//             timeout       - maximum time to wait for acknowledgement in milliseconds
// Returns: iSYSResult_t - error code indicating success or failure
iSYSResult_t iSYS4001::iSYS_setDeviceAddress(uint8_t deviceaddress, uint8_t destAddress , uint32_t timeout)
{
    // Frame based on documentation example:
    // 68 07 07 68 DA 01 D3 00 01 00 <NEW_ADDR> FCS 16
    uint8_t command[13];
    uint8_t index = 0;

    command[index++] = 0x68;   // SD2
    command[index++] = 0x07;   // LE
    command[index++] = 0x07;   // LEr
    command[index++] = 0x68;   // SD2
    command[index++] = destAddress; // DA (old/current address)
    command[index++] = 0x01;   // SA (master)
    command[index++] = 0xD3;   // FC (write/read address)
    command[index++] = 0x00;   // PDU[0] sub-function high byte
    command[index++] = 0x01;   // PDU[1] sub-function low byte
    command[index++] = 0x00;   // PDU[2] reserved
    command[index++] = deviceaddress; // PDU[3] new address

    uint8_t fcs = calculateFCS(command, 4, 10);
    command[index++] = fcs;    // FCS
    command[index++] = 0x16;   // ED

    // Debug: print outbound frame
    Serial.print("Sending SET address command: ");
    for (int i = 0; i < (int)sizeof(command); i++)
    {
        Serial.print("0x");
        if (command[i] < 0x10) Serial.print("0");
        Serial.print(command[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    _serial.write(command, sizeof(command));
    _serial.flush();

    // Wait for acknowledgement frame: 68 03 03 68 01 <NEW_ADDR> D3 <FCS> 16
    uint32_t startTime = millis();
    uint8_t buffer[32];
    uint8_t count = 0;
    while ((millis() - startTime) < timeout)
    {
        if (_serial.available())
        {
            uint8_t b = _serial.read();
            if (count < sizeof(buffer)) buffer[count++] = b; else return ERR_COMMAND_MAX_DATA_OVERFLOW;
            if (b == 0x16 && count >= 9)
            {
                // Optional: print frame
                Serial.print("Received SET address ack: ");
                for (uint8_t i = 0; i < count; i++)
                {
                    Serial.print("0x");
                    if (buffer[i] < 0x10) Serial.print("0");
                    Serial.print(buffer[i], HEX);
                    Serial.print(" ");
                }
                Serial.println();

                // Validate minimal structure
                if (count == 9 && buffer[0] == 0x68 && buffer[1] == 0x03 && buffer[2] == 0x03 &&
                    buffer[3] == 0x68 && buffer[4] == 0x01 && buffer[5] == deviceaddress &&
                    buffer[6] == 0xD3 && buffer[8] == 0x16)
                {
                    uint8_t calc = calculateFCS(buffer, 4, 6); // DA..FC for LE=03
                    if (calc == buffer[7]) return ERR_OK;
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
    if (deviceaddress == NULL) return ERR_NULL_POINTER;

    // Request frame example:
    // 68 05 05 68 DA 01 D2 00 01 FCS 16
    uint8_t command[11];
    uint8_t index = 0;
    command[index++] = 0x68;   // SD2
    command[index++] = 0x05;   // LE
    command[index++] = 0x05;   // LEr
    command[index++] = 0x68;   // SD2
    command[index++] = 0x00; // DA (often 0x00 broadcast)
    command[index++] = 0x01;   // SA (master)
    command[index++] = 0xD2;   // FC (read address)
    command[index++] = 0x00;   // PDU[0] sub-function high
    command[index++] = 0x01;   // PDU[1] sub-function low
    uint8_t fcs = calculateFCS(command, 4, 8);
    command[index++] = fcs;    // FCS
    command[index++] = 0x16;   // ED

    // Debug: print outbound frame
    Serial.print("Sending GET address command: ");
    for (int i = 0; i < (int)sizeof(command); i++)
    {
        Serial.print("0x");
        if (command[i] < 0x10) Serial.print("0");
        Serial.print(command[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    _serial.write(command, sizeof(command));
    _serial.flush();

    // Expected response example:
    // 68 05 05 68 01 SA D2 00 <ADDR> FCS 16
    uint32_t startTime = millis();
    uint8_t buffer[32];
    uint8_t count = 0;
    while ((millis() - startTime) < timeout)
    {
        if (_serial.available())
        {
            uint8_t b = _serial.read();
            if (count < sizeof(buffer)) buffer[count++] = b; else return ERR_COMMAND_MAX_DATA_OVERFLOW;
            if (b == 0x16 && count >= 11)
            {
                // Optional: print frame
                Serial.print("Received GET address response: ");
                for (uint8_t i = 0; i < count; i++)
                {
                    Serial.print("0x");
                    if (buffer[i] < 0x10) Serial.print("0");
                    Serial.print(buffer[i], HEX);
                    Serial.print(" ");
                }
                Serial.println();

                if (count == 11 && buffer[0] == 0x68 && buffer[1] == 0x05 && buffer[2] == 0x05 &&
                    buffer[3] == 0x68 && buffer[6] == 0xD2 && buffer[10] == 0x16)
                {
                    uint8_t calc = calculateFCS(buffer, 4, 8); // DA..PDU
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
    // Send the start acquisition command to the radar sensor
    iSYSResult_t res = sendAcquisitionCommand(destAddress, true);
    if (res != ERR_OK) 
    {
        return res;  // If sending failed, return error immediately
    }
    
    // Receive and verify the acknowledgement response from the radar sensor
    res = receiveAcquisitionAcknowledgement(destAddress, timeout);
    if (res != ERR_OK) 
    {
        return res;  // If receiving/verification failed, return error
    }
    
    return ERR_OK;  // Success - acquisition started
}

// Main function to stop data acquisition from the iSYS radar sensor
// Parameters: destAddress - destination address for the radar sensor
//             timeout - maximum time to wait for response in milliseconds
// Returns: iSYSResult_t - error code indicating success or failure
iSYSResult_t iSYS4001::iSYS_stopAcquisition(uint8_t destAddress, uint32_t timeout)
{
    // Send the stop acquisition command to the radar sensor
    iSYSResult_t res = sendAcquisitionCommand(destAddress, false);
    if (res != ERR_OK) 
    {
        return res;  // If sending failed, return error immediately
    }
    
    // Receive and verify the acknowledgement response from the radar sensor
    res = receiveAcquisitionAcknowledgement(destAddress, timeout);
    if (res != ERR_OK) 
    {
        return res;  // If receiving/verification failed, return error
    }
    
    return ERR_OK;  // Success - acquisition stopped
}

// Function to send acquisition start/stop command frame to the radar sensor
// Parameters: destAddress - destination address for the radar sensor
//             start - true to start acquisition, false to stop acquisition
// Returns: iSYSResult_t - error code (always ERR_OK for this function)
iSYSResult_t iSYS4001::sendAcquisitionCommand(uint8_t destAddress, bool start)
{
    // Based on the protocol from the documentation:
    // Start acquisition: 68 05 05 68 80 01 D1 00 00 52 16
    // Stop acquisition:  68 05 05 68 80 01 D1 00 01 53 16
    // Frame structure: SD2 LE LEr SD2 DA SA FC PDU FCS ED
    // SD2 = Start Delimiter 2 (0x68), LE = Length, LEr = Length repeat, DA = Destination Address, 
    // SA = Source Address, FC = Function Code (0xD1), PDU = Protocol Data Unit (0x00 for start, 0x01 for stop), 
    // FCS = Frame Check Sequence, ED = End Delimiter (0x16)
    
    uint8_t command[11];  // Array to hold the complete command frame
    uint8_t index = 0;    // Index for building the command array
    
    // Build the command frame byte by byte according to iSYS protocol
    command[index++] = 0x68;  // SD2
    command[index++] = 0x05;  // LE
    command[index++] = 0x05;  // LEr
    command[index++] = 0x68;  // SD2
    command[index++] = destAddress;  // DA
    command[index++] = 0x01;  // SA
    command[index++] = 0xD1;  // FC - Acquisition control function code
    command[index++] = 0x00;  // PDU[0] - Sub-function high byte (always 0x00)
    command[index++] = start ? 0x00 : 0x01;  // PDU[1] - Sub-function low byte (0x00 = start, 0x01 = stop)
    
    // Calculate FCS (Frame Check Sequence) - sum of bytes from DA to PDU
    uint8_t fcs = calculateFCS(command, 4, 8);
    command[index++] = fcs;  // FCS - Frame Check Sequence for error detection
    command[index++] = 0x16; // ED - End Delimiter
    
    // Debug: Print the command frame being sent to radar
    Serial.print(start ? "Starting" : "Stopping");
    Serial.print(" acquisition command to radar: ");
    for (int i = 0; i < 11; i++) 
    {
        Serial.print("0x");
        if (command[i] < 0x10) 
        {
            Serial.print("0");  // Add leading zero for single digit hex
        }
        Serial.print(command[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    // Send the complete command frame over serial interface
    _serial.write(command, 11);
    _serial.flush();  // Ensure all data is transmitted before continuing
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


    uint32_t startTime = millis();  // Record start time for timeout calculation
    uint8_t buffer[9];            // Buffer to store incoming response data
    uint16_t index = 0;             // Index for storing received bytes
    uint8_t byte;
    
    // Wait for response with timeout protection
    while ((millis() - startTime) < timeout) 
    {
        if (_serial.available()) 
        {  // Check if data is available to read
            byte = _serial.read();  // Read one byte from serial
            buffer[index++] = byte;         // Store byte in buffer
            
            // Check for end delimiter (0x16) which indicates end of frame
            if (byte == 0x16 && index >= 9) 
            {
               
                for (int i = 0; i < index; i++)
                {
                    Serial.print("0x");
                    if (buffer[i] < 0x10) 
                    {
                        Serial.print("0");  // Add leading zero for single digit hex
                    }
                    Serial.print(buffer[i], HEX);
                    Serial.print(" ");
                }
                Serial.println();
                
                // Verify the acknowledgement frame structure
                // Expected: 68 03 03 68 01 destAddress D1 FCS 16
                if (index == 9 && 
                    buffer[0] == 0x68 && buffer[1] == 0x03 && buffer[2] == 0x03 &&
                    buffer[3] == 0x68 && buffer[4] == 0x01 && buffer[5] == destAddress &&
                    buffer[6] == 0xD1 && buffer[8] == 0x16) 
                {
                    
                    uint8_t expectedFCS = calculateFCS(buffer, 4, 6);
                    if (buffer[7] == expectedFCS) 
                    {
                        return ERR_OK;  // Valid acknowledgement received
                    }
                    else 
                    {
                        return ERR_INVALID_CHECKSUM;  // Invalid checksum
                    }
                }
                else 
                {
                    return ERR_COMMAND_RX_FRAME_DAMAGED;  // Invalid frame structure
                }
            }
            
            // Prevent buffer overflow by checking array bounds
            if (index > 9) 
            {
                return ERR_COMMAND_MAX_DATA_OVERFLOW;  // Buffer overflow error
            }
        }
    }
    
    return ERR_COMMAND_NO_DATA_RECEIVED; // Timeout error - no response received
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
    // Send the set output filter command to the radar sensor
    iSYSResult_t res = sendSetOutputFilterRequest(outputnumber, filter, destAddress);
    if (res != ERR_OK) 
    {
        return res;  // If sending failed, return error immediately
    }
    
    // Receive and verify the acknowledgement from the radar sensor
    res = receiveSetOutputFilterAcknowledgement(destAddress, timeout);
    if (res != ERR_OK) 
    {
        return res;  // If receiving/verification failed, return error
    }
    
    return ERR_OK;  // Success - filter type has been set
}

// Function to send set output filter request command to the radar sensor
// Parameters: outputnumber - specifies which output to use
//             filterType - the filter type to set
//             destAddress - destination address for the radar sensor
// Returns: iSYSResult_t - error code (always ERR_OK for this function)
iSYSResult_t iSYS4001::sendSetOutputFilterRequest(iSYSOutputNumber_t outputnumber, iSYSOutput_filter_t filter, uint8_t destAddress)
{
    // Based on the protocol: 68 07 07 68 80 01 D5 01 15 00 03 6F 16
    // Frame structure: SD2 LE LEr SD2 DA SA FC PDU1 PDU2 PDU3 PDU4 FCS ED
    // SD2 = Start Delimiter 2 (0x68), LE = Length, LEr = Length repeat, DA = Destination Address, 
    // SA = Source Address, FC = Function Code (0xD5 = write), PDU = Protocol Data Unit, FCS = Frame Check Sequence, ED = End Delimiter
    
    uint8_t command[13];  // Array to hold the complete command frame
    uint8_t index = 0;    // Index for building the command array
    
    // Build the command frame byte by byte according to iSYS protocol
    command[index++] = 0x68;  // SD2
    command[index++] = 0x07;  // LE
    command[index++] = 0x07;  // LEr
    command[index++] = 0x68;  // SD2
    command[index++] = destAddress;  // DA
    command[index++] = 0x01;  // SA
    command[index++] = 0xD5;  // FC - Write function code
    command[index++] = outputnumber;  // PDU1 - Output number
    command[index++] = 0x15;  // PDU2 - Sub-function code for output filter type
    command[index++] = 0x00;  // PDU3 - Filter type high byte
    command[index++] = (uint8_t)filter;  // PDU4 - Filter type low byte
    
    // Calculate FCS (Frame Check Sequence) - sum of bytes from DA to PDU4
    uint8_t fcs = calculateFCS(command, 4, index-1);
    command[index++] = fcs;  // FCS - Frame Check Sequence for error detection
    command[index++] = 0x16; // ED - End Delimiter
    
    // Debug: Print the command frame being sent to radar
    Serial.print("Setting output filter type command to radar: ");
    for (int i = 0; i < 13; i++) 
    {
        Serial.print("0x");
        if (command[i] < 0x10) 
        {
            Serial.print("0");  // Add leading zero for single digit hex
        }
        Serial.print(command[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    // Send the complete command frame over serial interface
    _serial.write(command, 13);
    _serial.flush();  // Ensure all data is transmitted before continuing
    return ERR_OK;
}

// Function to receive and verify set output filter acknowledgement from radar sensor
// Parameters: destAddress - destination address for the radar sensor
//             timeout - maximum time to wait for response in milliseconds
// Returns: iSYSResult_t - error code indicating success or failure
iSYSResult_t iSYS4001::receiveSetOutputFilterAcknowledgement(uint8_t destAddress, uint32_t timeout)
{
    if (timeout == 0) 
    {
        return ERR_TIMEOUT;
    }

    uint32_t startTime = millis();  // Record start time for timeout calculation
    uint8_t buffer[9];            // Buffer to store incoming response data
    uint16_t index = 0;             // Index for storing received bytes
    uint8_t byte;
    
    // Wait for response with timeout protection
    while ((millis() - startTime) < timeout) 
    {
        if (_serial.available()) 
        {  // Check if data is available to read
            byte = _serial.read();  // Read one byte from serial
            buffer[index++] = byte;         // Store byte in buffer
            
            // Check for end delimiter (0x16) which indicates end of frame
            if (byte == 0x16 && index >= 9) 
            {
                // Debug: Print the received acknowledgement frame
                Serial.print("Received output filter acknowledgement: ");
                for (int i = 0; i < index; i++)
                {
                    Serial.print("0x");
                    if (buffer[i] < 0x10) 
                    {
                        Serial.print("0");  // Add leading zero for single digit hex
                    }
                    Serial.print(buffer[i], HEX);
                    Serial.print(" ");
                }
                Serial.println();
                
                // Verify the acknowledgement frame structure
                // Expected: 68 03 03 68 01 destAddress D5 FCS 16
                if (index == 9 && 
                    buffer[0] == 0x68 && buffer[1] == 0x03 && buffer[2] == 0x03 &&
                    buffer[3] == 0x68 && buffer[4] == 0x01 && buffer[5] == destAddress &&
                    buffer[6] == 0xD5 && buffer[8] == 0x16) 
                {
                    uint8_t expectedFCS = calculateFCS(buffer, 4, 6);
                    if (buffer[7] == expectedFCS) 
                    {
                        return ERR_OK;  // Valid acknowledgement received
                    }
                    else 
                    {
                        return ERR_INVALID_CHECKSUM;  // Invalid checksum
                    }
                }
                else 
                {
                    return ERR_COMMAND_RX_FRAME_DAMAGED;  // Invalid frame structure
                }
            }
            
            // Prevent buffer overflow by checking array bounds
            if (index > 9) 
            {
                return ERR_COMMAND_MAX_DATA_OVERFLOW;  // Buffer overflow error
            }
        }
    }
    
    return ERR_COMMAND_NO_DATA_RECEIVED; // Timeout error - no response received
}

// Function to get the single target filter type from a selected output
// Parameters: outputnumber - specifies which output to use (1, 2, or 3)
//             filterType - pointer to store the retrieved filter type
//             destAddress - destination address for the radar sensor
//             timeout - maximum time to wait for response in milliseconds
// Returns: iSYSResult_t - error code indicating success or failure
iSYSResult_t iSYS4001::iSYS_getOutputFilter(iSYSOutputNumber_t outputnumber, iSYSOutput_filter_t *filter, uint8_t destAddress, uint32_t timeout)
{
    // Check for null pointer
    if (filter == nullptr) 
    {
        return ERR_NULL_POINTER;
    }
    
    // Send the get output filter command to the radar sensor
    iSYSResult_t res = sendGetOutputFilterRequest(outputnumber, destAddress);
    if (res != ERR_OK) 
    {
        return res;  // If sending failed, return error immediately
    }
    
    // Receive and decode the response from the radar sensor
    res = receiveGetOutputFilterResponse(filter, destAddress, timeout);
    if (res != ERR_OK) 
    {
        return res;  // If receiving/decoding failed, return error
    }
    
    return ERR_OK;  // Success - filter type has been retrieved
}

// Function to send get output filter request command to the radar sensor
// Parameters: outputnumber - specifies which output to use
//             destAddress - destination address for the radar sensor
// Returns: iSYSResult_t - error code (always ERR_OK for this function)
iSYSResult_t iSYS4001::sendGetOutputFilterRequest(iSYSOutputNumber_t outputnumber, uint8_t destAddress)
{
    // Based on the protocol: 68 05 05 68 80 01 D4 01 15 6B 16
    // Frame structure: SD2 LE LEr SD2 DA SA FC PDU1 PDU2 FCS ED
    // SD2 = Start Delimiter 2 (0x68), LE = Length, LEr = Length repeat, DA = Destination Address, 
    // SA = Source Address, FC = Function Code (0xD4 = read), PDU = Protocol Data Unit, FCS = Frame Check Sequence, ED = End Delimiter
    
    uint8_t command[11];  // Array to hold the complete command frame
    uint8_t index = 0;    // Index for building the command array
    
    // Build the command frame byte by byte according to iSYS protocol
    command[index++] = 0x68;  // SD2
    command[index++] = 0x05;  // LE
    command[index++] = 0x05;  // LEr
    command[index++] = 0x68;  // SD2
    command[index++] = destAddress;  // DA
    command[index++] = 0x01;  // SA
    command[index++] = 0xD4;  // FC - Read function code
    command[index++] = outputnumber;  // PDU1 - Output number
    command[index++] = 0x15;  // PDU2 - Sub-function code for output filter type
    
    // Calculate FCS (Frame Check Sequence) - sum of bytes from DA to PDU2
    uint8_t fcs = calculateFCS(command, 4, index-1);
    command[index++] = fcs;  // FCS - Frame Check Sequence for error detection
    command[index++] = 0x16; // ED - End Delimiter
    
    // Debug: Print the command frame being sent to radar
    Serial.print("Getting output filter type command to radar: ");
    for (int i = 0; i < 11; i++) 
    {
        Serial.print("0x");
        if (command[i] < 0x10) 
        {
            Serial.print("0");  // Add leading zero for single digit hex
        }
        Serial.print(command[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    // Send the complete command frame over serial interface
    _serial.write(command, 11);
    _serial.flush();  // Ensure all data is transmitted before continuing
    return ERR_OK;
}

// Function to receive and decode get output filter response from radar sensor
// Parameters: filterType - pointer to store the retrieved filter type
//             destAddress - destination address for the radar sensor
//             timeout - maximum time to wait for response in milliseconds
// Returns: iSYSResult_t - error code indicating success or failure
iSYSResult_t iSYS4001::receiveGetOutputFilterResponse(iSYSOutput_filter_t *filter, uint8_t destAddress, uint32_t timeout)
{
    if (timeout == 0) 
    {
        return ERR_TIMEOUT;
    }

    uint32_t startTime = millis();  // Record start time for timeout calculation
    uint8_t buffer[11];           // Buffer to store incoming response data
    uint16_t index = 0;             // Index for storing received bytes
    uint8_t byte;
    
    // Wait for response with timeout protection
    while ((millis() - startTime) < timeout) 
    {
        if (_serial.available()) 
        {  // Check if data is available to read
            byte = _serial.read();  // Read one byte from serial
            buffer[index++] = byte;         // Store byte in buffer
            
            // Check for end delimiter (0x16) which indicates end of frame
            if (byte == 0x16 && index >= 11) 
            {
                // Debug: Print the received response frame
                Serial.print("Received output filter response: ");
                for (int i = 0; i < index; i++)
                {
                    Serial.print("0x");
                    if (buffer[i] < 0x10) 
                    {
                        Serial.print("0");  // Add leading zero for single digit hex
                    }
                    Serial.print(buffer[i], HEX);
                    Serial.print(" ");
                }
                Serial.println();
                
                // Verify the response frame structure
                // Expected: 68 05 05 68 01 destAddress D4 00 filterType FCS 16
                if (index == 11 && 
                    buffer[0] == 0x68 && buffer[1] == 0x05 && buffer[2] == 0x05 &&
                    buffer[3] == 0x68 && buffer[4] == 0x01 && buffer[5] == destAddress &&
                    buffer[6] == 0xD4 && buffer[7] == 0x00 && buffer[10] == 0x16) 
                {
                    uint8_t expectedFCS = calculateFCS(buffer, 4, 8);
                    if (buffer[9] == expectedFCS) 
                    {
                        // Extract the filter type from the response
                        *filter = (iSYSOutput_filter_t)buffer[8];
                        return ERR_OK;  // Valid response received and decoded
                    }
                    else 
                    {
                        return ERR_INVALID_CHECKSUM;  // Invalid checksum
                    }
                }
                else 
                {
                    return ERR_COMMAND_RX_FRAME_DAMAGED;  // Invalid frame structure
                }
            }
            
            // Prevent buffer overflow by checking array bounds
            if (index > 11) 
            {
                return ERR_COMMAND_MAX_DATA_OVERFLOW;  // Buffer overflow error
            }
        }
    }
    
    return ERR_COMMAND_NO_DATA_RECEIVED; // Timeout error - no response received
}

// Function to set the single target filter signal for a selected output
// Parameters: outputnumber - specifies which output to use (1, 2, or 3)
//             filterSignal - the filter signal to set (off, velocity radial, range radial)
//             destAddress - destination address for the radar sensor
//             timeout - maximum time to wait for response in milliseconds
// Returns: iSYSResult_t - error code indicating success or failure
iSYSResult_t iSYS4001::iSYS_setOutputSignalFilter(iSYSOutputNumber_t outputnumber, iSYSFilter_signal_t signal, uint8_t destAddress, uint32_t timeout)
{
    // Send the set output signal filter command to the radar sensor
    iSYSResult_t res = sendSetOutputSignalFilterRequest(outputnumber, signal, destAddress);
    if (res != ERR_OK) 
    {
        return res;  // If sending failed, return error immediately
    }
    
    // Receive and verify the acknowledgement from the radar sensor
    res = receiveSetOutputSignalFilterAcknowledgement(destAddress, timeout);
    if (res != ERR_OK) 
    {
        return res;  // If receiving/verification failed, return error
    }
    
    return ERR_OK;  // Success - filter signal has been set
}

// Function to send set output signal filter request command to the radar sensor
// Parameters: outputnumber - specifies which output to use
//             filterSignal - the filter signal to set
//             destAddress - destination address for the radar sensor
// Returns: iSYSResult_t - error code (always ERR_OK for this function)
iSYSResult_t iSYS4001::sendSetOutputSignalFilterRequest(iSYSOutputNumber_t outputnumber, iSYSFilter_signal_t signal, uint8_t destAddress)
{
    // Based on the protocol: 68 07 07 68 80 01 D5 01 16 00 02 6F 16
    // Frame structure: SD2 LE LEr SD2 DA SA FC PDU1 PDU2 PDU3 PDU4 FCS ED
    // SD2 = Start Delimiter 2 (0x68), LE = Length, LEr = Length repeat, DA = Destination Address, 
    // SA = Source Address, FC = Function Code (0xD5 = write), PDU = Protocol Data Unit, FCS = Frame Check Sequence, ED = End Delimiter
    
    uint8_t command[13];  // Array to hold the complete command frame
    uint8_t index = 0;    // Index for building the command array
    
    // Build the command frame byte by byte according to iSYS protocol
    command[index++] = 0x68;  // SD2
    command[index++] = 0x07;  // LE
    command[index++] = 0x07;  // LEr
    command[index++] = 0x68;  // SD2
    command[index++] = destAddress;  // DA
    command[index++] = 0x01;  // SA
    command[index++] = 0xD5;  // FC - Write function code
    command[index++] = outputnumber;  // PDU1 - Output number
    command[index++] = 0x16;  // PDU2 - Sub-function code for output signal filter
    command[index++] = 0x00;  // PDU3 - Filter signal high byte
    command[index++] = (uint8_t)signal;  // PDU4 - Filter signal low byte
    
    // Calculate FCS (Frame Check Sequence) - sum of bytes from DA to PDU4
    uint8_t fcs = calculateFCS(command, 4, index-1);
    command[index++] = fcs;  // FCS - Frame Check Sequence for error detection
    command[index++] = 0x16; // ED - End Delimiter
    
    // Debug: Print the command frame being sent to radar
    Serial.print("Setting output signal filter command to radar: ");
    for (int i = 0; i < 13; i++) 
    {
        Serial.print("0x");
        if (command[i] < 0x10) 
        {
            Serial.print("0");  // Add leading zero for single digit hex
        }
        Serial.print(command[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    // Send the complete command frame over serial interface
    _serial.write(command, 13);
    _serial.flush();  // Ensure all data is transmitted before continuing
    return ERR_OK;
}

// Function to receive and verify set output signal filter acknowledgement from radar sensor
// Parameters: destAddress - destination address for the radar sensor
//             timeout - maximum time to wait for response in milliseconds
// Returns: iSYSResult_t - error code indicating success or failure
iSYSResult_t iSYS4001::receiveSetOutputSignalFilterAcknowledgement(uint8_t destAddress, uint32_t timeout)
{
    if (timeout == 0) 
    {
        return ERR_TIMEOUT;
    }

    uint32_t startTime = millis();  // Record start time for timeout calculation
    uint8_t buffer[9];            // Buffer to store incoming response data
    uint16_t index = 0;             // Index for storing received bytes
    uint8_t byte;
    
    // Wait for response with timeout protection
    while ((millis() - startTime) < timeout) 
    {
        if (_serial.available()) 
        {  // Check if data is available to read
            byte = _serial.read();  // Read one byte from serial
            buffer[index++] = byte;         // Store byte in buffer
            
            // Check for end delimiter (0x16) which indicates end of frame
            if (byte == 0x16 && index >= 9) 
            {
                // Debug: Print the received acknowledgement frame
                Serial.print("Received output signal filter acknowledgement: ");
                for (int i = 0; i < index; i++)
                {
                    Serial.print("0x");
                    if (buffer[i] < 0x10) 
                    {
                        Serial.print("0");  // Add leading zero for single digit hex
                    }
                    Serial.print(buffer[i], HEX);
                    Serial.print(" ");
                }
                Serial.println();
                
                // Verify the acknowledgement frame structure
                // Expected: 68 03 03 68 01 destAddress D5 FCS 16
                if (index == 9 && 
                    buffer[0] == 0x68 && buffer[1] == 0x03 && buffer[2] == 0x03 &&
                    buffer[3] == 0x68 && buffer[4] == 0x01 && buffer[5] == destAddress &&
                    buffer[6] == 0xD5 && buffer[8] == 0x16) 
                {
                    uint8_t expectedFCS = calculateFCS(buffer, 4, 6);
                    if (buffer[7] == expectedFCS) 
                    {
                        return ERR_OK;  // Valid acknowledgement received
                    }
                    else 
                    {
                        return ERR_INVALID_CHECKSUM;  // Invalid checksum
                    }
                }
                else 
                {
                    return ERR_COMMAND_RX_FRAME_DAMAGED;  // Invalid frame structure
                }
            }
            
            // Prevent buffer overflow by checking array bounds
            if (index > 9) 
            {
                return ERR_COMMAND_MAX_DATA_OVERFLOW;  // Buffer overflow error
            }
        }
    }
    
    return ERR_COMMAND_NO_DATA_RECEIVED; // Timeout error - no response received
}

// Function to get the single target filter signal from a selected output
// Parameters: outputnumber - specifies which output to use (1, 2, or 3)
//             filterSignal - pointer to store the retrieved filter signal
//             destAddress - destination address for the radar sensor
//             timeout - maximum time to wait for response in milliseconds
// Returns: iSYSResult_t - error code indicating success or failure
iSYSResult_t iSYS4001::iSYS_getOutputSignalFilter(iSYSOutputNumber_t outputnumber, iSYSFilter_signal_t *signal, uint8_t destAddress, uint32_t timeout)
{
    // Check for null pointer
    if (signal == nullptr) 
    {
        return ERR_NULL_POINTER;
    }
    
    // Send the get output signal filter command to the radar sensor
    iSYSResult_t res = sendGetOutputSignalFilterRequest(outputnumber, destAddress);
    if (res != ERR_OK) 
    {
        return res;  // If sending failed, return error immediately
    }
    
    // Receive and decode the response from the radar sensor
    res = receiveGetOutputSignalFilterResponse(signal, destAddress, timeout);
    if (res != ERR_OK) 
    {
        return res;  // If receiving/decoding failed, return error
    }
    
    return ERR_OK;  // Success - filter signal has been retrieved
}

// Function to send get output signal filter request command to the radar sensor
// Parameters: outputnumber - specifies which output to use
//             destAddress - destination address for the radar sensor
// Returns: iSYSResult_t - error code (always ERR_OK for this function)
iSYSResult_t iSYS4001::sendGetOutputSignalFilterRequest(iSYSOutputNumber_t outputnumber, uint8_t destAddress)
{
    // Based on the protocol: 68 05 05 68 80 01 D4 01 16 6C 16
    // Frame structure: SD2 LE LEr SD2 DA SA FC PDU1 PDU2 FCS ED
    // SD2 = Start Delimiter 2 (0x68), LE = Length, LEr = Length repeat, DA = Destination Address, 
    // SA = Source Address, FC = Function Code (0xD4 = read), PDU = Protocol Data Unit, FCS = Frame Check Sequence, ED = End Delimiter
    
    uint8_t command[11];  // Array to hold the complete command frame
    uint8_t index = 0;    // Index for building the command array
    
    // Build the command frame byte by byte according to iSYS protocol
    command[index++] = 0x68;  // SD2
    command[index++] = 0x05;  // LE
    command[index++] = 0x05;  // LEr
    command[index++] = 0x68;  // SD2
    command[index++] = destAddress;  // DA
    command[index++] = 0x01;  // SA
    command[index++] = 0xD4;  // FC - Read function code
    command[index++] = outputnumber;  // PDU1 - Output number
    command[index++] = 0x16;  // PDU2 - Sub-function code for output signal filter
    
    // Calculate FCS (Frame Check Sequence) - sum of bytes from DA to PDU2
    uint8_t fcs = calculateFCS(command, 4, index-1);
    command[index++] = fcs;  // FCS - Frame Check Sequence for error detection
    command[index++] = 0x16; // ED - End Delimiter
    
    // Debug: Print the command frame being sent to radar
    Serial.print("Getting output signal filter command to radar: ");
    for (int i = 0; i < 11; i++) 
    {
        Serial.print("0x");
        if (command[i] < 0x10) 
        {
            Serial.print("0");  // Add leading zero for single digit hex
        }
        Serial.print(command[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    // Send the complete command frame over serial interface
    _serial.write(command, 11);
    _serial.flush();  // Ensure all data is transmitted before continuing
    return ERR_OK;
}

// Function to receive and decode get output signal filter response from radar sensor
// Parameters: filterSignal - pointer to store the retrieved filter signal
//             destAddress - destination address for the radar sensor
//             timeout - maximum time to wait for response in milliseconds
// Returns: iSYSResult_t - error code indicating success or failure
iSYSResult_t iSYS4001::receiveGetOutputSignalFilterResponse(iSYSFilter_signal_t *signal, uint8_t destAddress, uint32_t timeout)
{
    if (timeout == 0) 
    {
        return ERR_TIMEOUT;
    }

    uint32_t startTime = millis();  // Record start time for timeout calculation
    uint8_t buffer[11];           // Buffer to store incoming response data
    uint16_t index = 0;             // Index for storing received bytes
    uint8_t byte;
    
    // Wait for response with timeout protection
    while ((millis() - startTime) < timeout) 
    {
        if (_serial.available()) 
        {  // Check if data is available to read
            byte = _serial.read();  // Read one byte from serial
            buffer[index++] = byte;         // Store byte in buffer
            
            // Check for end delimiter (0x16) which indicates end of frame
            if (byte == 0x16 && index >= 11) 
            {
                // Debug: Print the received response frame
                Serial.print("Received output signal filter response: ");
                for (int i = 0; i < index; i++)
                {
                    Serial.print("0x");
                    if (buffer[i] < 0x10) 
                    {
                        Serial.print("0");  // Add leading zero for single digit hex
                    }
                    Serial.print(buffer[i], HEX);
                    Serial.print(" ");
                }
                Serial.println();
                
                // Verify the response frame structure
                // Expected: 68 05 05 68 01 destAddress D4 00 filterSignal FCS 16
                if (index == 11 && 
                    buffer[0] == 0x68 && buffer[1] == 0x05 && buffer[2] == 0x05 &&
                    buffer[3] == 0x68 && buffer[4] == 0x01 && buffer[5] == destAddress &&
                    buffer[6] == 0xD4 && buffer[7] == 0x00 && buffer[10] == 0x16) 
                {
                    uint8_t expectedFCS = calculateFCS(buffer, 4, 8);
                    if (buffer[9] == expectedFCS) 
                    {
                        // Extract the filter signal from the response
                        *signal = (iSYSFilter_signal_t)buffer[8];
                        return ERR_OK;  // Valid response received and decoded
                    }
                    else 
                    {
                        return ERR_INVALID_CHECKSUM;  // Invalid checksum
                    }
                }
                else 
                {
                    return ERR_COMMAND_RX_FRAME_DAMAGED;  // Invalid frame structure
                }
            }
            
            // Prevent buffer overflow by checking array bounds
            if (index > 11) 
            {
                return ERR_COMMAND_MAX_DATA_OVERFLOW;  // Buffer overflow error
            }
        }
    }
    
    return ERR_COMMAND_NO_DATA_RECEIVED; // Timeout error - no response received
}