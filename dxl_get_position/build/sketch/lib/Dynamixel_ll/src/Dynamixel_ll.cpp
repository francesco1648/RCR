#line 1 "C:\\Users\\PC1\\Desktop\\ISAAC\\RCR\\RCR\\dxl_get_position\\lib\\Dynamixel_ll\\src\\Dynamixel_ll.cpp"
#include "Dynamixel_ll.h"
#define time_delay 0
DynamixelLL::DynamixelLL(HardwareSerial &serial, uint8_t servoID)
    : _serial(serial), _servoID(servoID) {}

DynamixelLL::~DynamixelLL()
{
    // Deallocate memory for motor IDs if they were allocated
    if (_motorIDs != nullptr)
    {
        delete[] _motorIDs;
        _motorIDs = nullptr;
    }
}

void DynamixelLL::begin_dxl(long baudrate)
{
    _serial.begin(baudrate);
    delay(time_delay); // Attendi che la seriale si stabilizzi
}
void DynamixelLL::ledOff()
{
    uint8_t packet[13] = {
        0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x06, 0x00, 0x03, 0x41, 0x00, 0x00

    };

    uint16_t crc = calculateCRC(packet, 11);
    packet[11] = crc & 0xFF;
    packet[12] = (crc >> 8) & 0xFF;

    sendPacket(packet, 13);
    delay(time_delay);
}



uint16_t DynamixelLL::calculateCRC(const uint8_t *data_blk_ptr, uint8_t data_blk_size)
{
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202};

    uint16_t crc_accum = 0;
    for (uint8_t j = 0; j < data_blk_size; j++)
    {
        uint16_t i = ((crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }
    return crc_accum;
}


void DynamixelLL::printResponse()
{
    // Pulisce eventuali byte rimasti nel buffer prima di leggere la risposta
    while (_serial.available())
    {
        _serial.read(); // Consuma i byte nel buffer
    }

    unsigned long startMillis = millis();
    unsigned long timeout = 5; // Timeout di 1 secondo per la lettura

        Serial.println("Inizio lettura risposta:");

    // Legge i byte dalla seriale
    while (millis() - startMillis < timeout)
    {
        if (_serial.available())
        {
            uint8_t byte = _serial.read();

                Serial.print("0x");
                if (byte < 0x10)
                    Serial.print("0");
                Serial.print(byte, HEX);
                Serial.print(" ");

        }
    }

        Serial.println("\nFine lettura risposta");

}


void DynamixelLL::setDebug(bool enable)
{
    _debug = enable;
}


void DynamixelLL::enableSync(const uint8_t* motorIDs, uint8_t numMotors)
{
    // Check if the number of motors is valid (at least two motors).
    if (numMotors < 2)
    {
        if (_debug)
            Serial.println("Invalid number of motors for sync mode. Must be at least 2.");
        return;
    }

    // If sync mode is already enabled, deallocate the previous array.
    if (_sync && _motorIDs != nullptr)
    {
        delete[] _motorIDs;
    }

    // Allocate memory for motor IDs and copy the contents of the input array.
    _motorIDs = new uint8_t[numMotors];
    for (uint8_t i = 0; i < numMotors; i++)
    {
        _motorIDs[i] = motorIDs[i];
    }

    // Update sync variables.
    _numMotors = numMotors;
    _sync = true;
}


void DynamixelLL::disableSync()
{
    if (_motorIDs != nullptr)
    {
        delete[] _motorIDs;
        _motorIDs = nullptr;
    }
    _numMotors = 1;
    _sync = false;
}


uint8_t DynamixelLL::checkArraySize(uint8_t arraySize) const
{
    if (arraySize != _numMotors)
    {
        if (_debug)
            Serial.println("Error: Array size does not match number of motors.");
        return 1;
    }
    return 0;
}


// ===============================
// ==   Instruction Functions   ==
// ===============================


uint8_t DynamixelLL::writeRegister(uint16_t address, uint32_t value, uint8_t size)
{
    if (_sync)
    {
        uint32_t buffer[_numMotors]; // a temporary buffer to hold the value for each motor
        for(uint8_t i = 0; i < _numMotors; i++)
            buffer[i] = value;
        return syncWrite(address, size, _motorIDs, buffer, _numMotors);
    }

    // length: Instruction (1) + Address (2) + CRC (2) + Data (size bytes) = 5 + size.
    uint16_t length = 5 + size;

    // Allocate the packet buffer.
    // Total packet size = Header (4) + ID (1) + Length (2) + Instruction (1) +
    //                      Address (2) + Data (size) + CRC (2) = 10 + size.
    uint8_t packet[10 + size];

    // Construct the Packet Header
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFD;
    packet[3] = 0x00;

    // Set Packet ID
    packet[4] = _servoID;

    // Insert the Length field (little-endian)
    packet[5] = length & 0xFF;
    packet[6] = (length >> 8) & 0xFF;

    // Set the Instruction byte: WRITE (0x03)
    packet[7] = 0x03;

    // Write the target register address (little-endian: LSB then MSB)
    packet[8] = address & 0xFF;
    packet[9] = (address >> 8) & 0xFF;

    // Insert the Data bytes in little-endian order
    for (uint8_t i = 0; i < size; i++)
    {
        packet[10 + i] = (value >> (8 * i)) & 0xFF;
    }

    // Compute and Append the CRC
    uint8_t lenNoCRC = 10 + size; // the packet length excluding the CRC field.
    uint16_t crc = calculateCRC(packet, lenNoCRC);
    packet[lenNoCRC]     = crc & 0xFF;         // Append CRC LSB.
    packet[lenNoCRC + 1] = (crc >> 8) & 0xFF;    // Append CRC MSB.

    // Send the Packet
    if (!sendPacket(packet, lenNoCRC + 2))
    {
        if (_debug)
            Serial.println("Error sending Write packet.");
        return 1;
    }
    delay(time_delay);

    // Receive and Process the Response
    StatusPacket response = receivePacket();
    if (_debug)
    {
        if (!response.valid)
            Serial.println("Invalid status packet received.");
        if (response.error != 0)
        {
            Serial.print("Error in status packet: ");
            Serial.println(response.error, HEX);
        }
    }

    return response.error;
}


bool DynamixelLL::sendPacket(const uint8_t *packet, uint8_t length)
{
    if (_debug)
    {
        Serial.print("Sent Packet: ");
        for (uint8_t i = 0; i < length; ++i)
        {
            Serial.print("0x");
            if (packet[i] < 0x10)
                Serial.print("0");
            Serial.print(packet[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
    while (_serial.available())
        _serial.read(); // Clear any pending data from the serial input buffer.

    size_t bytesWritten = _serial.write(packet, length);

    _serial.flush();

    return (bytesWritten == length);
}


StatusPacket DynamixelLL::receivePacket()
{
    StatusPacket result = {false, 0, 0, {0}, 0};

    const uint8_t maxPacketSize = 64;         // Maximum allowed packet size.
    uint8_t buffer[maxPacketSize];           // Buffer for incoming bytes.
    uint16_t index = 0;                      // Index into the buffer.
    uint32_t start = millis();
    const uint32_t timeout = 5;           // Timeout in milliseconds.
    bool headerFound = false;                // Flag to indicate header detection.

    // Step 1: Locate header (0xFF, 0xFF, 0xFD, 0x00)
    while ((millis() - start) < timeout && index < maxPacketSize)
    {
        if (_serial.available())
        {
            buffer[index++] = _serial.read();
            // When there is at least 4 bytes, check the last 4 bytes.
            if (index >= 4 &&
                buffer[index - 4] == 0xFF &&
                buffer[index - 3] == 0xFF &&
                buffer[index - 2] == 0xFD &&
                buffer[index - 1] == 0x00)
            {
                headerFound = true;
                break;
            }
        }
    }
    if (!headerFound)
    {
        if (_debug)
            Serial.println("Header not found within timeout");
        return result;
    }
    uint16_t headerStart = index - 4;

    // Step 2: Read header extension (ID and LENGTH fields; need 7 bytes total from header start)
    while ((millis() - start) < timeout && (index - headerStart) < 7 && index < maxPacketSize)
    {
        if (_serial.available())
            buffer[index++] = _serial.read();
    }
    if ((index - headerStart) < 7)
    {
        if (_debug)
            Serial.println("Timeout waiting for header extension");
        return result;
    }
    result.id = buffer[headerStart + 4];

    // Step 3: Determine total packet length
    uint16_t lengthField = buffer[headerStart + 5] | (buffer[headerStart + 6] << 8); // LSB | MSB
    uint16_t totalPacketLength = 7 + lengthField; // (header + ID + length field) + (Instruction + ERR + PARAM + CRC)

    // Step 4: Read remaining bytes until full packet is received
    while ((millis() - start) < timeout && (index - headerStart) < totalPacketLength && index < maxPacketSize)
    {
        if (_serial.available())
            buffer[index++] = _serial.read();
    }
    if ((index - headerStart) < totalPacketLength)
    {
        if (_debug)
            Serial.println("Incomplete packet received (timeout)");
        return result;
    }

    // Step 5: Debug print of the packet
    if (_debug) {
        Serial.print("Received Packet: ");
        for (uint16_t i = headerStart; i < headerStart + totalPacketLength; i++) {
            Serial.print("0x");
            if (buffer[i] < 0x10) Serial.print("0");
            Serial.print(buffer[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }

    // Step 6: Parse and validate the packet
    // [Header (4) | Packet ID (1) | Length (2) | Instruction (1) | Error (1) | Parameters (paramLength) | CRC (2)]
    if (buffer[headerStart + 7] != 0x55) // Verify instruction (expecting 0x55 for a status packet)
    {
        if (_debug)
            Serial.println("Invalid instruction; expected 0x55. Likely an echo; retrying...");
        return receivePacket();
    }
    result.error = buffer[headerStart + 8];
    uint8_t paramLength = lengthField - 4;
    result.dataLength = paramLength;
    for (uint8_t i = 0; i < paramLength && i < 4; i++)
    {
        result.data[i] = buffer[headerStart + 9 + i];
    }

    // Read the CRC from the packet.
    uint16_t receivedCRC = buffer[headerStart + 9 + paramLength] | (buffer[headerStart + 10 + paramLength] << 8);
    uint16_t computedCRC = calculateCRC(&buffer[headerStart], 9 + paramLength);
    if (receivedCRC != computedCRC)
    {
        if (_debug)
            Serial.println("CRC invalid");
        return result;
    }

    result.valid = true;
    return result;
}


uint8_t DynamixelLL::ping(uint32_t &value)
{
    // Packet format :
    //   [Header (4 bytes) | ID (1 byte) | Length (2 bytes) | Instruction (1 byte) | CRC (2 bytes)]
    uint8_t packet[10];

    // Header: fixed values for protocol 2.0.
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFD;
    packet[3] = 0x00;

    // Set the device ID.
    packet[4] = _servoID;

    // Length field (2 bytes, little-endian). For Ping, length=3 (No params, only Instruction + CRC)
    packet[5] = 0x03;   // LSB
    packet[6] = 0x00;   // MSB

    // Instruction: Ping command (0x01).
    packet[7] = 0x01;

    // Calculate and append the CRC.
    uint8_t lenNoCRC = 8;
    uint16_t crc = calculateCRC(packet, lenNoCRC);
    packet[lenNoCRC]     = crc & 0xFF;         // CRC LSB
    packet[lenNoCRC + 1] = (crc >> 8) & 0xFF;    // CRC MSB

    // Send the ping packet over the serial interface.
    if (!sendPacket(packet, lenNoCRC + 2))
    {
        if (_debug)
            Serial.println("Error sending Ping packet.");
        return 1;
    }
    delay(time_delay);

    // Receive the status packet in response.
    StatusPacket response = receivePacket();
    if (_debug)
    {
        if (!response.valid)
            Serial.println("Invalid status packet received.");
        if (response.error != 0)
        {
            Serial.print("Error in status packet: ");
            Serial.println(response.error, HEX);
        }
    }

    value = 0;
    for (uint8_t i = 0; i < response.dataLength; i++)
        value |= (response.data[i] >> (8 * i)); // Params = 3 (Model Number + Version of Firmware)

    return response.error;
}


bool DynamixelLL::syncWrite(uint16_t address, uint8_t dataLength, const uint8_t* ids, uint32_t* values, uint8_t count)
{
    // The fixed part of the parameter block: 4 bytes for starting address (2 bytes)
    // and data length (2 bytes)
    const uint16_t fixedParamLength = 4;

    // Each device block consists of 1 byte (Device ID) + dataLength bytes.
    const uint16_t deviceBlockLength = dataLength + 1;

    // Total parameter block: fixed part + one device block per motor.
    const uint16_t paramBlockLength = fixedParamLength + (deviceBlockLength * count);

    // Create a local buffer to hold the entire parameter block.
    uint8_t params[paramBlockLength];
    uint16_t idx = 0;

    // Fixed Parameters (4 bytes):
    // Parameter 1-2: Starting address (LSB, MSB)
    params[idx++] = address & 0xFF;
    params[idx++] = (address >> 8) & 0xFF;
    // Parameter 3-4: Data Length (LSB, MSB)
    params[idx++] = dataLength & 0xFF;
    params[idx++] = (dataLength >> 8) & 0xFF;

    // Append each device's parameter block.
    for (uint8_t i = 0; i < count; i++)
    {
        // Add device ID.
        params[idx++] = ids[i];
        // Add the data bytes in little-endian order.
        for (uint8_t j = 0; j < dataLength; j++)
            params[idx++] = (values[i] >> (8 * j)) & 0xFF;
    }

    return sendSyncWritePacket(params, paramBlockLength);
}


bool DynamixelLL::sendSyncWritePacket(const uint8_t* parameters, uint16_t parametersLength)
{
    // Calculate Length field = Parameter Block length + 3 (Instruction + CRC)
    uint16_t lengthField = parametersLength + 3;

    // Total packet size:
    //   Header (4) + Packet ID (1) + Length (2) + Instruction (1) +
    //   Parameter Block (parametersLength) + CRC (2)
    uint8_t packetSize = 10 + parametersLength;
    uint8_t packet[packetSize];
    uint16_t idx = 0;

    // Build Header (4 bytes)
    packet[idx++] = 0xFF;
    packet[idx++] = 0xFF;
    packet[idx++] = 0xFD;
    packet[idx++] = 0x00;

    // Packet ID (Broadcast ID: 0xFE)
    packet[idx++] = 0xFE;

    // Length (2 bytes): (Parameter Count + 3) in little-endian.
    packet[idx++] = lengthField & 0xFF;          // LSB
    packet[idx++] = (lengthField >> 8) & 0xFF;     // MSB

    // Instruction (1 byte): Sync Write (0x83)
    packet[idx++] = 0x83;

    // Copy the parameter block.
    memcpy(&packet[idx], parameters, parametersLength);
    idx += parametersLength;

    // Compute and append CRC (2 bytes).
    uint16_t crc = calculateCRC(packet, packetSize - 2);
    packet[idx++] = crc & 0xFF;          // CRC LSB
    packet[idx++] = (crc >> 8) & 0xFF;     // CRC MSB

    return sendPacket(packet, packetSize);
}


bool DynamixelLL::sendSyncReadPacket(uint16_t address, uint8_t dataLength, const uint8_t* ids, uint8_t count)
{
    // Fixed parameter block = 4 bytes (address (2) + dataLength (2))
    const uint16_t fixedParamLength = 4;
    const uint16_t paramBlockLength = fixedParamLength + count; // Add 1 byte per device.

    // LENGTH = (Parameter count + 3)
    uint16_t lengthField = paramBlockLength + 3;

    // Total packet size:
    //   Header (4) + Packet ID (1) + Length (2) + Instruction (1) +
    //   Parameter Block (paramBlockLength) + CRC (2)
    uint16_t packetSize = 10 + paramBlockLength;
    uint8_t packet[packetSize];
    uint16_t idx = 0;

    // Header (4 bytes)
    packet[idx++] = 0xFF;
    packet[idx++] = 0xFF;
    packet[idx++] = 0xFD;
    packet[idx++] = 0x00;

    // Packet ID (1 byte): Broadcast ID (0xFE)
    packet[idx++] = 0xFE;

    // Length (2 bytes) - little-endian.
    packet[idx++] = lengthField & 0xFF;
    packet[idx++] = (lengthField >> 8) & 0xFF;

    // Instruction (1 byte): Sync Read (0x82)
    packet[idx++] = 0x82;

    // Fixed Parameters (4 bytes):
    // Starting address (2 bytes, little-endian)
    packet[idx++] = address & 0xFF;
    packet[idx++] = (address >> 8) & 0xFF;
    // Data length (2 bytes, little-endian)
    packet[idx++] = dataLength & 0xFF;
    packet[idx++] = (dataLength >> 8) & 0xFF;

    // Device IDs (1 byte for each device)
    for (uint8_t i = 0; i < count; ++i)
        packet[idx++] = ids[i];

    // Compute CRC for the packet (excluding the final 2 CRC bytes)
    uint16_t crc = calculateCRC(packet, packetSize - 2);
    packet[idx++] = crc & 0xFF;       // CRC LSB
    packet[idx++] = (crc >> 8) & 0xFF;  // CRC MSB

    return sendPacket(packet, packetSize);
}


bool DynamixelLL::bulkWrite(const uint8_t* ids, uint16_t* addresses, uint8_t* dataLengths, uint32_t* values, uint8_t count)
{
    // Each device block consists of: 1 byte (Device ID) + 2 bytes (starting address)
    // + 2 bytes (data length) + data length bytes (data).
    uint16_t paramBlockLength = 0;
    for (uint8_t i = 0; i < count; i++)
        paramBlockLength += 5 + dataLengths[i];

    // Create a local buffer to hold the entire parameter block.
    uint8_t params[paramBlockLength];
    uint16_t idx = 0;

    // Append each device's parameter block.
    for (uint8_t i = 0; i < count; i++)
    {
        // Add device ID.
        params[idx++] = ids[i];
        // Add Starting address (LSB, MSB)
        params[idx++] = addresses[i] & 0xFF;
        params[idx++] = (addresses[i] >> 8) & 0xFF;
        // Add Data Length (LSB, MSB)
        params[idx++] = dataLengths[i] & 0xFF;
        params[idx++] = (dataLengths[i] >> 8) & 0xFF;
        // Add the data bytes in little-endian order.
        for (uint8_t j = 0; j < dataLengths[i]; j++)
            params[idx++] = (values[i] >> (8 * j)) & 0xFF;
    }

    return sendBulkWritePacket(params, paramBlockLength);
}


bool DynamixelLL::sendBulkWritePacket(const uint8_t* parameters, uint16_t parametersLength)
{
    // Calculate Length field = Parameter Block length + 3 (Instruction + CRC)
    uint16_t lengthField = parametersLength + 3;

    // Total packet size:
    //   Header (4) + Packet ID (1) + Length (2) + Instruction (1) +
    //   Parameter Block (parametersLength) + CRC (2)
    uint16_t packetSize = 10 + parametersLength;
    uint8_t packet[packetSize];
    uint16_t idx = 0;

    // Build Header (4 bytes)
    packet[idx++] = 0xFF;
    packet[idx++] = 0xFF;
    packet[idx++] = 0xFD;
    packet[idx++] = 0x00;

    // Packet ID (Broadcast ID: 0xFE)
    packet[idx++] = 0xFE;

    // Length (2 bytes): (Parameter Count + 3) in little-endian.
    packet[idx++] = lengthField & 0xFF;          // LSB
    packet[idx++] = (lengthField >> 8) & 0xFF;     // MSB

    // Instruction (1 byte): Bulk Write (0x93)
    packet[idx++] = 0x93;

    // Copy the parameter block.
    memcpy(&packet[idx], parameters, parametersLength);
    idx += parametersLength;

    // Compute and append CRC (2 bytes).
    uint16_t crc = calculateCRC(packet, packetSize - 2);
    packet[idx++] = crc & 0xFF;          // CRC LSB
    packet[idx++] = (crc >> 8) & 0xFF;     // CRC MSB

    return sendPacket(packet, packetSize);
}


bool DynamixelLL::sendBulkReadPacket(const uint8_t* ids, uint16_t* addresses, uint8_t* dataLengths, uint8_t count)
{
    // parameter length: 1 byte (Device ID) + 2 bytes (starting address) + 2 bytes (data length)
    const uint16_t paramBlockLength = 5 * count; // 5 bytes per device.

    // LENGTH = (Parameter count + 3)
    uint16_t lengthField = paramBlockLength + 3;

    // Total packet size:
    //   Header (4) + Packet ID (1) + Length (2) + Instruction (1) +
    //   Parameter Block (paramBlockLength) + CRC (2)
    uint16_t packetSize = 10 + paramBlockLength;
    uint8_t packet[packetSize];
    uint16_t idx = 0;

    // Header (4 bytes)
    packet[idx++] = 0xFF;
    packet[idx++] = 0xFF;
    packet[idx++] = 0xFD;
    packet[idx++] = 0x00;

    // Packet ID (1 byte): Broadcast ID (0xFE)
    packet[idx++] = 0xFE;

    // Length (2 bytes) - little-endian.
    packet[idx++] = lengthField & 0xFF;
    packet[idx++] = (lengthField >> 8) & 0xFF;

    // Instruction (1 byte): Bulk Read (0x92)
    packet[idx++] = 0x92;

    // Parameter Block (5 bytes per device):
    for (uint8_t i = 0; i < count; ++i)
    {
        // Device IDs (1 byte for each device)
        packet[idx++] = ids[i];
        // Starting address (2 bytes, little-endian)
        packet[idx++] = addresses[i] & 0xFF;
        packet[idx++] = (addresses[i] >> 8) & 0xFF;
        // Data length (2 bytes, little-endian)
        packet[idx++] = dataLengths[i] & 0xFF;
        packet[idx++] = (dataLengths[i] >> 8) & 0xFF;
    }

    // Compute CRC for the packet (excluding the final 2 CRC bytes)
    uint16_t crc = calculateCRC(packet, packetSize - 2);
    packet[idx++] = crc & 0xFF;       // CRC LSB
    packet[idx++] = (crc >> 8) & 0xFF;  // CRC MSB

    return sendPacket(packet, packetSize);
}


uint8_t DynamixelLL::bulkRead(const uint8_t* ids, uint16_t* addresses, uint8_t* dataLengths, uint32_t* values, uint8_t count)
{
    // Send Bulk Read Instruction Packet.
    if (!sendBulkReadPacket(ids, addresses, dataLengths, count))
    {
        if (_debug)
            Serial.println("Error sending Bulk Read packet.");
        return 1;
    }

    uint8_t retError = 0;
    // For each device, read its response.
    for (uint8_t i = 0; i < count; i++)
    {
        StatusPacket response = receivePacket();
        if (!response.valid || response.error != 0)
        {
            if (_debug)
            {
                Serial.print("Error in status packet from device ");
                Serial.print(ids[i]);
                Serial.print(": 0x");
                Serial.println(response.error, HEX);
            }
            retError = response.error;
        }
        values[i] = 0;
        for (uint8_t j = 0; j < response.dataLength; j++)
            values[i] |= (response.data[j] << (8 * j));
    }

    return retError;
}


uint8_t DynamixelLL::factoryReset(uint8_t level)
{
    // length: Instruction (1) + Data (1) + CRC(2) = 4.
    uint16_t length = 4;

    // Allocate the packet buffer.
    // Total packet size = Header (4) + ID (1) + Length (2) + Instruction (1) +
    //                      Data (1) + CRC (2) = 11.
    uint8_t packet[11];

    // Construct the Packet Header
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFD;
    packet[3] = 0x00;

    // Set Packet ID
    packet[4] = _servoID;

    // Insert the Length field (little-endian)
    packet[5] = length & 0xFF;
    packet[6] = (length >> 8) & 0xFF;

    // Set the Instruction byte: Factory Reset (0x06)
    packet[7] = 0x06;

    // Insert the factory reset level
    packet[8] = level;

    // Compute and Append the CRC
    uint8_t lenNoCRC = 9; // the packet length excluding the CRC field.
    uint16_t crc = calculateCRC(packet, lenNoCRC);
    packet[lenNoCRC]     = crc & 0xFF;         // Append CRC LSB.
    packet[lenNoCRC + 1] = (crc >> 8) & 0xFF;    // Append CRC MSB.

    // Send the Packet
    if (!sendPacket(packet, lenNoCRC + 2))
    {
        if (_debug)
            Serial.println("Error sending Factory Reset packet.");
        return 1;
    }
    delay(time_delay);

    // Receive and Process the Response
    StatusPacket response = receivePacket();
    if (_debug)
    {
        if (!response.valid)
            Serial.println("Invalid status packet received.");
        if (response.error != 0)
        {
            Serial.print("Error in status packet: ");
            Serial.println(response.error, HEX);
        }
    }

    return response.error;
}


uint8_t DynamixelLL::reboot()
{
    // length: Instruction (1) + CRC(2) = 3.
    uint16_t length = 3;

    // Allocate the packet buffer.
    // Total packet size = Header (4) + ID (1) + Length (2) + Instruction (1) +
    //                      CRC (2) = 10.
    uint8_t packet[10];

    // Construct the Packet Header
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFD;
    packet[3] = 0x00;

    // Set Packet ID
    packet[4] = _servoID;

    // Insert the Length field (little-endian)
    packet[5] = length & 0xFF;
    packet[6] = (length >> 8) & 0xFF;

    // Set the Instruction byte: Reboot (0x08)
    packet[7] = 0x08;

    // Compute and Append the CRC
    uint8_t lenNoCRC = 8; // the packet length excluding the CRC field.
    uint16_t crc = calculateCRC(packet, lenNoCRC);
    packet[lenNoCRC]     = crc & 0xFF;         // Append CRC LSB.
    packet[lenNoCRC + 1] = (crc >> 8) & 0xFF;    // Append CRC MSB.

    // Send the Packet
    if (!sendPacket(packet, lenNoCRC + 2))
    {
        if (_debug)
            Serial.println("Error sending Factory Reset packet.");
        return 1;
    }
    delay(time_delay);

    // Receive and Process the Response
    StatusPacket response = receivePacket();
    if (_debug)
    {
        if (!response.valid)
            Serial.println("Invalid status packet received.");
        if (response.error != 0)
        {
            Serial.print("Error in status packet: ");
            Serial.println(response.error, HEX);
        }
    }

    return response.error;
}


// ===============================
// ==  Control Table Functions  ==
// ===============================


uint8_t DynamixelLL::setOperatingMode(uint8_t mode)
{
    if (!(mode == 1 || mode == 3 || mode == 4 || mode == 16))
    {
        if (_debug)
                Serial.print("Error: Unsupported operating mode.");
        return 1;
    }
    return writeRegister(11, mode, 1); // EEPROM address 11, 1 byte
}


uint8_t DynamixelLL::setHomingOffset(int32_t offset)
{
    if (offset > 1044479)
    {
        offset = 1044479;
        if (_debug)
            Serial.println("Warning: Homing offset clamped to 1044479.");
    } else if (offset < -1044479) {
        offset = -1044479;
        if (_debug)
            Serial.println("Warning: Homing offset clamped to -1044479.");
    }
    return writeRegister(20, static_cast<uint32_t>(offset), 4); // EEPROM address 20, 4 bytes
}


uint8_t DynamixelLL::setHomingOffset_A(float offsetAngle)
{
    // Convert angle in degrees to pulses using conversion factor 0.088 [deg/pulse].
    uint32_t offset = offsetAngle / 0.088;
    if (offset > 1044479)
    {
        offset = 1044479;
        if (_debug)
            Serial.println("Warning: Homing offset clamped to 1044479.");
    } else if (offset < -1044479) {
        offset = -1044479;
        if (_debug)
            Serial.println("Warning: Homing offset clamped to -1044479.");
    }
    return writeRegister(20, static_cast<uint32_t>(offset), 4); // EEPROM address 20, 4 bytes
}


uint8_t DynamixelLL::setGoalPosition_PCM(uint16_t goalPosition)
{
    if (goalPosition > 4005)
    {
        goalPosition = 4005;
        if (_debug)
            Serial.println("Warning: Goal position clamped to 4005.");
    }
    return writeRegister(116, goalPosition, 4); // RAM address 116, 4 bytes
}


uint8_t DynamixelLL::setGoalPosition_A_PCM(float angleDegrees)
{
    // Convert angle in degrees to pulses using conversion factor 0.088 [deg/pulse].
    uint32_t goalPosition = static_cast<uint32_t>(angleDegrees / 0.088);
    if (goalPosition > 4095)
    {
        goalPosition = 4095;
        if (_debug)
            Serial.println("Warning: Angle conversion resulted in value exceeding 4095, clamped.");
    }
    return writeRegister(116, goalPosition, 4); // RAM address 116, 4 bytes
}


uint8_t DynamixelLL::setGoalPosition_EPCM(int32_t extendedPosition)
{
    if (extendedPosition > 1048575)
    {
        extendedPosition = 1048575;
        if (_debug)
            Serial.println("Warning: Extended position clamped to 1048575.");
    } else if (extendedPosition < -1048575) {
        extendedPosition = -1048575;
        if (_debug)
            Serial.println("Warning: Extended position clamped to -1048575.");
    }
    return writeRegister(116, static_cast<uint32_t>(extendedPosition), 4); // RAM address 116, 4 bytes
}


uint8_t DynamixelLL::setTorqueEnable(bool enable)
{
    return writeRegister(64, enable ? 1 : 0, 1); // RAM address 64, 1 byte
}


uint8_t DynamixelLL::setLED(bool enable)
{
    return writeRegister(65, enable ? 1 : 0, 1); // RAM address 65, 1 byte
}


uint8_t DynamixelLL::setStatusReturnLevel(uint8_t level)
{
    if (level > 2)
    {
        if (_debug)
            Serial.println("Error: Invalid status return level. Allowed values: 0, 1, or 2.");
        return 1;
    }
    return writeRegister(68, level, 1); // RAM address 68, 1 byte
}


uint8_t DynamixelLL::setID(uint8_t newID)
{
    if (newID > 253)
    {
        if (_debug)
            Serial.println("Error: Invalid ID. Valid IDs are 0 to 253.");
        return 1;
    }
    return writeRegister(7, newID, 1); // EEPROM address 7, 1 byte
}


uint8_t DynamixelLL::setBaudRate(uint8_t baudRate)
{
    const uint8_t allowed[] = {0, 1, 2, 3, 4, 5, 6, 7};
    bool valid = false;

    for (uint8_t i = 0; i < sizeof(allowed) / sizeof(allowed[0]); i++)
    {
        if (allowed[i] == baudRate)
        {
            valid = true;
            break;
        }
    }

    if (!valid)
    {
        if (_debug)
        {
            Serial.print("Error: Unrecognized baud rate code: ");
            Serial.println(baudRate);
        }
        return 1;
    }
    return writeRegister(8, baudRate, 1); // EEPROM address 8, 1 byte
}


uint8_t DynamixelLL::setReturnDelayTime(uint8_t delayTime)
{
    if (delayTime > 254)
    {
        delayTime = 254;
        if (_debug)
            Serial.println("Warning: setReturnDelayTime clamped to 254.");
    }
    return writeRegister(9, delayTime, 1); // EEPROM address 9, 1 byte
}


uint8_t DynamixelLL::setDriveMode(bool torqueOnByGoalUpdate, bool timeBasedProfile, bool reverseMode)
{
    uint8_t mode = 0;
    if (torqueOnByGoalUpdate)
        mode |= 0x08; // Set Bit 3.
    if (timeBasedProfile)
        mode |= 0x04; // Set Bit 2.
    if (reverseMode)
        mode |= 0x01; // Set Bit 0.
    return writeRegister(10, mode, 1); // EEPROM address 10, 1 byte
}


uint8_t DynamixelLL::setProfileVelocity(uint32_t profileVelocity)
{
    uint8_t driveMode = 0;
    uint8_t error = readRegister(10, driveMode, 1);
    // Bit 2 (0x04) set indicates time-based profile.
    bool timeBased = (error == 0) && ((driveMode & 0x04) != 0);

    // Select maximum allowed velocity based on profile type.
    const uint32_t maxProfileVelocity = timeBased ? 32737UL : 32767UL;

    if (profileVelocity > maxProfileVelocity)
    {
        if (_debug)
        {
            Serial.print("Profile velocity clamped to ");
            Serial.println(maxProfileVelocity);
        }
        profileVelocity = maxProfileVelocity;
    }
    return writeRegister(112, profileVelocity, 4); // RAM address 112, 4 bytes
}


uint8_t DynamixelLL::setProfileAcceleration(uint32_t profileAcceleration)
{
    // Read drive mode from register 10 to determine if a time-based profile is active.
    uint8_t driveMode = 0;
    uint8_t error = readRegister(10, driveMode, 1);
    bool timeBased = (error == 0) && ((driveMode & 0x04) != 0);

    // Choose the maximum allowed acceleration based on profile type.
    const uint32_t maxProfileAcceleration = timeBased ? 32737UL : 32767UL;

    if (profileAcceleration > maxProfileAcceleration)
    {
        if (_debug)
        {
            Serial.print("Profile acceleration clamped to ");
            Serial.println(maxProfileAcceleration);
        }
        profileAcceleration = maxProfileAcceleration;
    }

    // For time-based profiles, ensure that acceleration does not exceed half of the current profile velocity.
    uint32_t currentProfileVelocity = 0;
    error = readRegister(112, currentProfileVelocity, 4);
    if (timeBased && error == 0 && currentProfileVelocity > 0 && profileAcceleration > (currentProfileVelocity / 2))
    {
        uint32_t clampedValue = currentProfileVelocity / 2;
        if (_debug)
        {
            Serial.print("Profile acceleration clamped to half of current profile velocity: ");
            Serial.println(clampedValue);
        }
        profileAcceleration = clampedValue;
    } else if (error != 0 && _debug) {
        Serial.print("Error reading Profile Velocity: ");
        Serial.println(error);
    }
    return writeRegister(108, profileAcceleration, 4); // RAM address 108, 4 bytes
}


uint8_t DynamixelLL::setGoalVelocity_RPM(float rpm)
{
    const float maxRPM = 30.0f; // stimato per 12V
    if (rpm > maxRPM)
    {
        rpm = maxRPM;
        if (_debug) Serial.println("Warning: RPM clamped to 30 (12V limit).");
    }
    else if (rpm < -maxRPM)
    {
        rpm = -maxRPM;
        if (_debug) Serial.println("Warning: RPM clamped to -30 (12V limit).");
    }

    int16_t velocityValue = static_cast<int16_t>(rpm / 0.229f);
    return writeRegister(104, static_cast<uint32_t>(velocityValue), 4);
}


uint8_t DynamixelLL::getPresentVelocity_RPM(float &rpm)
{
    int16_t temp = 0;  // range : -Velocity Limit(44) ~ Velocity Limit(44)
    uint8_t error = readRegister(128, temp, 4); // address 128, 4 bytes
    if (error != 0)
    {
        if (_debug)
        {
            Serial.print("Error reading Present Velocity: ");
            Serial.println(error, HEX);
        }
    } else {
        rpm = temp * 0.229f;  // convert to RPM in float
    }
    return error;
}


uint8_t DynamixelLL::getPresentPosition(int32_t &presentPosition)
{
    uint8_t error = readRegister(132, presentPosition, 4); // RAM address 132, 4 bytes
    if (error != 0)
    {
        if (_debug)
        {
            Serial.print("Error reading Present Position: ");
            Serial.println(error, HEX);
        }
    }
    return error;
}


uint8_t DynamixelLL::getCurrentLoad(int16_t &currentLoad)
{
    uint8_t error = readRegister(126, currentLoad, 2); // RAM address 126, 2 bytes
    if (error != 0)
    {
        if (_debug)
        {
            Serial.print("Error reading Current Load: ");
            Serial.println(error, HEX);
        }
    }
    return error;
}


uint8_t DynamixelLL::getMovingStatus(MovingStatus &status)
{
    uint8_t error = readRegister(123, status.raw, 1); // RAM address 123, 1 byte
    if (error != 0)
    {
        if (_debug)
        {
            Serial.print("Error reading Moving Status, error code: ");
            Serial.println(error, HEX);
        }
    } else {
        // Decode bits 5 & 4 for Velocity Profile Type.
        uint8_t profileBits = (status.raw >> 4) & 0x03;
        status.profileType = static_cast<VelocityProfileType>(profileBits);

        // Decode bit 3 for Following Error.
        status.followingError = ((status.raw >> 3) & 0x01) != 0;
        // Decode bit 1 to check if a motion profile is ongoing.
        status.profileOngoing = ((status.raw >> 1) & 0x01) != 0;
        // Decode bit 0 to determine if target position is reached.
        status.inPosition = (status.raw & 0x01) != 0;
    }
    return error;
}
