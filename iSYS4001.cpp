#include "iSYS4001.h"

iSYS4001::iSYS4001(HardwareSerial& serial, uint32_t baud) : _serial(serial), _baud(baud){
    // UART will be initialized by the sketch (allows specifying pins on ESP32)
}

iSYSResult_t iSYS4001::getTargetList32(
    iSYSTargetList_t *pTargetList,
    iSYSOutputNumber_t outputnumber,
    uint8_t destAddress,
    uint32_t timeout
) {
    // Initialize target list structure
    memset(pTargetList, 0, sizeof(iSYSTargetList_t));
    
    // Send the target list request
    iSYSResult_t res = sendTargetListRequest(outputnumber, destAddress);
    if (res != ERR_OK) return res;
    
    // Receive and decode the response
    res = receiveTargetListResponse(pTargetList, timeout);
    if (res != ERR_OK) return res;
    
    return ERR_OK;
}

iSYSResult_t iSYS4001::sendTargetListRequest(iSYSOutputNumber_t outputnumber, uint8_t destAddress) {
    // Based on the protocol from the image: 68 05 05 68 80 01 DA 01 20 7C 16    68 05 05 68 80 01 DA 01 20 7C 16
    // Frame structure: SD2 LE LEr SD2 DA SA FC PDU FCS ED
    
    uint8_t command[11];
    uint8_t index = 0;
    command[index++] = 0x68;
    command[index++] = 0x05;
    command[index++] = 0x05;
    command[index++] = 0x68;
    command[index++] = destAddress;
    command[index++] = 0x01;
    command[index++] = 0xDA;
    command[index++] = 0x01;  // Target list type
    command[index++] = 0x20;  // 32-bit resolution flag
    
    // Calculate FCS (Frame Check Sequence) - Byte 9
    uint8_t fcs = 0;
    for (int i = 4; i <= 8; i++) {
    fcs = (uint8_t)(fcs + command[i]); // sum DA..PDU only
}

    command[index++] = fcs; // FCS is the sum of bytes from DA to PDU
    command[index++] = 0x16;
    
    // Send the command
    _serial.write(command, 11);
    _serial.flush();
    return ERR_OK;
}

iSYSResult_t iSYS4001::receiveTargetListResponse(iSYSTargetList_t *pTargetList, uint32_t timeout) {
    uint32_t startTime = millis();
    uint8_t buffer[256];
    uint16_t index = 0;
    
    // Wait for response with timeout
    while ((millis() - startTime) < timeout) {
        if (_serial.available()) {
            uint8_t byte = _serial.read();
            buffer[index++] = byte;
            
            // Check for end delimiter
            if (byte == 0x16 && index >= 11) {
                // Minimum valid response length reached -> decode via iSYSResult_t API
                iSYSResult_t res = decodeTargetFrame(buffer, index, 4001, 32, pTargetList);
                return res;
            }
            
            // Prevent buffer overflow
            if (index >= 256) {
                return ERR_COMMAND_MAX_DATA_OVERFLOW;
            }
        }
    }
    
    return ERR_COMMAND_NO_DATA_RECEIVED; // Timeout
}

iSYSResult_t iSYS4001::decodeTargetFrame(uint8_t *frame_array, uint16_t nrOfElements,
                                         uint16_t productcode, uint8_t bitrate,
                                         iSYSTargetList_t *targetList) {
    if (frame_array == NULL || targetList == NULL) {
        return ERR_NULL_POINTER;
    }
    if (nrOfElements < 6) {
        return ERR_COMMAND_RX_FRAME_LENGTH;
    }

    uint16_t ui16_fc;
    if (frame_array[0] == 0x68) {
        ui16_fc = 6; // variable-length frame
    } else {
        ui16_fc = 3; // fixed-length frame
    }

    if (frame_array[nrOfElements - 1] != 0x16) {
        return ERR_COMMAND_NO_VALID_FRAME_FOUND;
    }

    uint8_t output_number = static_cast<uint8_t>(frame_array[ui16_fc + 1] & 0xFF);
    uint8_t nrOfTargets = static_cast<uint8_t>(frame_array[ui16_fc + 2] & 0xFF);
    uint8_t* pData = &frame_array[ui16_fc + 3];

    if ((nrOfTargets > MAX_TARGETS) && (nrOfTargets != 0xFF)) {
        return ERR_COMMAND_FAILURE;
    }

    if (nrOfTargets != 0xFF) {
        for (uint8_t i = 0; i < MAX_TARGETS; i++) {
            targetList->targets[i].angle = 0;
            targetList->targets[i].range = 0;
            targetList->targets[i].signal = 0;
            targetList->targets[i].velocity = 0;
        }

        targetList->nrOfTargets = nrOfTargets;
        targetList->clippingFlag = 0;
        targetList->outputNumber = output_number;

        if (bitrate == 32) {
            for (uint8_t i = 0; i < nrOfTargets; i++) {
                int16_t tmp = (static_cast<int16_t>(pData[0]) << 8) | pData[1];
                pData += 2;
                targetList->targets[i].signal = static_cast<float>(tmp) * 0.01f;

                int32_t tmp32 = (static_cast<int32_t>(pData[0]) << 24) |
                                 (static_cast<int32_t>(pData[1]) << 16) |
                                 (static_cast<int32_t>(pData[2]) << 8)  |
                                  static_cast<int32_t>(pData[3]);
                pData += 4;
                targetList->targets[i].velocity = static_cast<float>(tmp32) * 0.001f;

                tmp32 = (static_cast<int32_t>(pData[0]) << 24) |
                        (static_cast<int32_t>(pData[1]) << 16) |
                        (static_cast<int32_t>(pData[2]) << 8)  |
                         static_cast<int32_t>(pData[3]);
                pData += 4;
                targetList->targets[i].range = static_cast<float>(tmp32) * 1e-6f;

                tmp32 = (static_cast<int32_t>(pData[0]) << 24) |
                        (static_cast<int32_t>(pData[1]) << 16) |
                        (static_cast<int32_t>(pData[2]) << 8)  |
                         static_cast<int32_t>(pData[3]);
                pData += 4;
                targetList->targets[i].angle = static_cast<float>(tmp32) * 0.01f;
            }
        }
        else if (bitrate == 16) {
            for (uint8_t i = 0; i < nrOfTargets; i++) {
                targetList->targets[i].signal = static_cast<float>(pData[0] & 0xFF);
                pData += 1;

                int16_t tmp = (static_cast<int16_t>(pData[0]) << 8) | pData[1];
                pData += 2;
                targetList->targets[i].velocity = static_cast<float>(tmp) * 0.01f;

                tmp = (static_cast<int16_t>(pData[0]) << 8) | pData[1];
                pData += 2;
                if (productcode == 4004 || productcode == 6003) {
                    targetList->targets[i].range = static_cast<float>(tmp) * 0.001f;
                } else {
                    targetList->targets[i].range = static_cast<float>(tmp) * 0.01f;
                }

                tmp = (static_cast<int16_t>(pData[0]) << 8) | pData[1];
                pData += 2;
                targetList->targets[i].angle = static_cast<float>(tmp) * 0.01f;
            }
        }
    } else {
        targetList->clippingFlag = 1;
    }

    if (nrOfTargets == MAX_TARGETS) {
        targetList->error.iSYSTargetListError = TARGET_LIST_FULL;
    } else {
        targetList->error.iSYSTargetListError = TARGET_LIST_OK;
    }
    return ERR_OK;
}


