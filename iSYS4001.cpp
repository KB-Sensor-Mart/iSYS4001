#include "iSYS4001.h"

// Constructor for iSYS4001 class
// Parameters: serial - reference to HardwareSerial object for UART communication
//             baud - baud rate for serial communication
iSYS4001::iSYS4001(HardwareSerial& serial, uint32_t baud) : _serial(serial), _baud(baud){
    // UART will be initialized by the sketch (allows specifying pins on ESP32)
    // This constructor stores the serial interface and baud rate for later use
}

// Main function to get target list from iSYS4001 radar sensor
// Parameters: pTargetList - pointer to structure that will hold the target data
//             destAddress - destination address for the radar sensor
//             timeout - maximum time to wait for response in milliseconds
//             outputnumber - specifies which output to use (defaults to ISYS_OUTPUT_1 if not specified)
// Returns: iSYSResult_t - error code indicating success or failure
iSYSResult_t iSYS4001::getTargetList32(
    iSYSTargetList_t *pTargetList,
    uint8_t destAddress,
    uint32_t timeout,
    iSYSOutputNumber_t outputnumber
) {
    // Initialize target list structure with zeros to ensure clean state
    memset(pTargetList, 0, sizeof(iSYSTargetList_t));
    
    // Send the target list request command to the radar sensor
    iSYSResult_t res = sendTargetListRequest(outputnumber, destAddress);
    if (res != ERR_OK) return res;  // If sending failed, return error immediately
    
    // Receive and decode the response from the radar sensor
    res = receiveTargetListResponse(pTargetList, timeout);
    if (res != ERR_OK) return res;  // If receiving/decoding failed, return error
    
    return ERR_OK;  // Success - target list has been populated
}

// Function to send target list request command to the radar sensor
// Parameters: outputnumber - specifies which output to use
//             destAddress - destination address for the radar sensor
// Returns: iSYSResult_t - error code (always ERR_OK for this function)
iSYSResult_t iSYS4001::sendTargetListRequest(iSYSOutputNumber_t outputnumber, uint8_t destAddress) {
    // Based on the protocol from the image: 68 05 05 68 80 01 DA 01 20 7C 16    68 05 05 68 80 01 DA 01 20 7C 16
    // Frame structure: SD2 LE LEr SD2 DA SA FC PDU FCS ED
    // SD2 = Start Delimiter 2 (0x68), LE = Length, LEr = Length repeat, DA = Destination Address, 
    // SA = Source Address, FC = Function Code, PDU = Protocol Data Unit, FCS = Frame Check Sequence, ED = End Delimiter
    
    uint8_t command[11];  // Array to hold the complete command frame
    uint8_t index = 0;    // Index for building the command array
    
    // Build the command frame byte by byte according to iSYS protocol
    command[index++] = 0x68;  // SD2 - Start Delimiter 2
    command[index++] = 0x05;  // LE - Length (5 bytes from DA to PDU)
    command[index++] = 0x05;  // LEr - Length repeat (must match LE)
    command[index++] = 0x68;  // SD2 - Start Delimiter 2 (repeated)
    command[index++] = destAddress;  // DA - Destination Address (radar sensor address)
    command[index++] = 0x01;  // SA - Source Address (master address)
    command[index++] = 0xDA;  // FC - Function Code (target list request)
    command[index++] = outputnumber;  // PDU - Output number (specifies which output to use)
    command[index++] = 0x20;  // 32-bit resolution flag (0x20 = 32-bit mode)
    
    // Calculate FCS (Frame Check Sequence) - sum of bytes from DA to PDU
    uint8_t fcs = 0;
    for (int i = 4; i <= 8; i++) {
        fcs = (uint8_t)(fcs + command[i]); // sum DA..PDU only
    }

    command[index++] = fcs;  // FCS - Frame Check Sequence for error detection
    command[index++] = 0x16; // ED - End Delimiter
    
    // Debug: Print the command frame being sent to radar
    Serial.print("Sending command to radar: ");
    for (int i = 0; i < 11; i++) {
        Serial.print("0x");
        if (command[i] < 0x10) Serial.print("0");  // Add leading zero for single digit hex
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
          
            
            // Prevent buffer overflow by checking array bounds
            if (index >= 256) 
            {
                return ERR_COMMAND_MAX_DATA_OVERFLOW;  // Buffer overflow error
            }
        }

    }

    if (byte == 0x16 && index >= 11) 
    {
                // Debug: Print the received response frame from radar
                Serial.print("Received response from radar: ");
                for (int i = 0; i < index; i++)
                {
                    Serial.print("0x");
                    if (buffer[i] < 0x10) Serial.print("0");  // Add leading zero for single digit hex
                    Serial.print(buffer[i], HEX);
                    Serial.print(" ");
                }
                Serial.println();
                
                // Minimum valid response length reached (11 bytes minimum)
                // Decode the received frame using the decodeTargetFrame function
                iSYSResult_t res = decodeTargetFrame(buffer, index, 4001, 32, pTargetList);
                return res;  // Return the result of decoding
    }
    
    return ERR_COMMAND_NO_DATA_RECEIVED; // Timeout error - no response received
}

// Function to decode target frame data from radar sensor response
// Parameters: frame_array - array containing the received frame data
//             nrOfElements - number of bytes in the frame array
//             productcode - product code of the radar sensor (4001, 4004, 6003, etc.)
//             bitrate - resolution mode (16-bit or 32-bit)
//             targetList - pointer to structure that will hold decoded target data
// Returns: iSYSResult_t - error code indicating success or failure
iSYSResult_t iSYS4001::decodeTargetFrame(uint8_t *frame_array, uint16_t nrOfElements,
                                         uint16_t productcode, uint8_t bitrate,
                                         iSYSTargetList_t *targetList) 
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

    // Verify frame ends with proper end delimiter
    if (frame_array[nrOfElements - 1] != 0x16) {
        return ERR_COMMAND_NO_VALID_FRAME_FOUND;  // Invalid end delimiter
    }

    // Extract frame header information
    uint8_t output_number = static_cast<uint8_t>(frame_array[ui16_fc + 1] & 0xFF);  // Output number
    uint8_t nrOfTargets = static_cast<uint8_t>(frame_array[ui16_fc + 2] & 0xFF);    // Number of targets
    uint8_t* pData = &frame_array[ui16_fc + 3];  // Pointer to start of target data

    // Validate number of targets (0xFF indicates clipping, other values must be <= MAX_TARGETS)
    if ((nrOfTargets > MAX_TARGETS) && (nrOfTargets != 0xFF)) {
        return ERR_COMMAND_FAILURE;  // Too many targets
    }

    // Process target data if not in clipping mode
    if (nrOfTargets != 0xFF) {
        // Initialize all target structures with zeros
        for (uint8_t i = 0; i < MAX_TARGETS; i++) {
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
        if (bitrate == 32) {
            // 32-bit resolution mode - higher precision data
            for (uint8_t i = 0; i < nrOfTargets; i++) {
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
        // else if (bitrate == 16) {
        //     // 16-bit resolution mode - lower precision but faster processing
        //     for (uint8_t i = 0; i < nrOfTargets; i++) {
        //         // Decode signal strength (8-bit, no scaling)
        //         targetList->targets[i].signal = static_cast<float>(pData[0] & 0xFF);
        //         pData += 1;  // Move pointer to next data field

        //         // Decode velocity (16-bit, scaled by 0.01 m/s)
        //         int16_t tmp = (static_cast<int16_t>(pData[0]) << 8) | pData[1];
        //         pData += 2;  // Move pointer to next data field
        //         targetList->targets[i].velocity = static_cast<float>(tmp) * 0.01f;

        //         // Decode range (16-bit, scaling depends on product code)
        //         tmp = (static_cast<int16_t>(pData[0]) << 8) | pData[1];
        //         pData += 2;  // Move pointer to next data field
        //         if (productcode == 4004 || productcode == 6003) {
        //             targetList->targets[i].range = static_cast<float>(tmp) * 0.001f;  // mm to meters
        //         } else {
        //             targetList->targets[i].range = static_cast<float>(tmp) * 0.01f;   // cm to meters
        //         }

        //         // Decode angle (16-bit, scaled by 0.01 degrees)
        //         tmp = (static_cast<int16_t>(pData[0]) << 8) | pData[1];
        //         pData += 2;  // Move pointer to next data field
        //         targetList->targets[i].angle = static_cast<float>(tmp) * 0.01f;
        //     }
        //}
    } else {
        // Clipping mode detected (0xFF targets) - sensor is overloaded
        targetList->clippingFlag = 1;  // Set clipping flag to indicate overload
    }

    // Set error status based on number of targets
    if (nrOfTargets == MAX_TARGETS) {
        targetList->error.iSYSTargetListError = TARGET_LIST_FULL;  // Maximum targets reached
    } else {
        targetList->error.iSYSTargetListError = TARGET_LIST_OK;    // Normal operation
    }
    return ERR_OK;  // Success - target list has been decoded
}


