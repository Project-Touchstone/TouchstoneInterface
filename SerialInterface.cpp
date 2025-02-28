#include "SerialInterface.h"

serialib SerialInterface::serial;
uint8_t SerialInterface::buffer[64];
uint8_t SerialInterface::bufferSize = 0;
uint8_t SerialInterface::header = 0;
bool SerialInterface::headerFlag = false;
bool SerialInterface::endFlag = true;

/// @brief Initializes the serial interface
/// @param baudRate Baud rate of serial communication
uint8_t SerialInterface::begin(const char* port, long baudRate) {
    return serial.openDevice(port, 115200);
}

void SerialInterface::end() {
    serial.closeDevice();
}

uint16_t SerialInterface::available() {
    return bufferSize;
}

/// @brief Blocks until serial data is available and checks for a header or end byte
/// @return true (packet to process), false (no packet to process)
bool SerialInterface::processPacket() {
    // Reads one byte of data with a 1 second timeout
    uint8_t byte;
    if (serial.readChar((char *)byte, 1000) != 1) {
        return false;
    }
    
    if (byte == END) {
        // Sets end flag
        endFlag = true;
        return false;
    } else if (endFlag) {
        // If end of data frame was already reached, starts new data frame
        endFlag = false;
        header = byte;
        // Sets header flag
        headerFlag = true;
    }
    else {
        //Otherwise is just a regular data byte and adds to buffer
        buffer[bufferSize++] = byte;
    }

    return true;
}

bool SerialInterface::isEnded() {
    return endFlag;
}

uint8_t SerialInterface::getHeader() {
    return header;
}

void SerialInterface::clearPacket() {
    headerFlag = false;
    endFlag = true;
}

void SerialInterface::sendByte(uint8_t data) {
    serial.writeChar(data);
}

void SerialInterface::sendBytes(uint8_t* buffer, uint8_t len) {
    serial.writeBytes(buffer, len);
}

void SerialInterface::sendFloat32(float data) {
    uint8_t buffer[sizeof(data)];
    std::memcpy(&buffer, &data, sizeof(data));
    SerialInterface::sendBytes(buffer, sizeof(data));
}

void SerialInterface::sendEnd() {
    SerialInterface::sendByte(END);
}

uint8_t SerialInterface::readByte() {
    if (SerialInterface::available() > 0) {
        return buffer[--bufferSize];
    }
    return 0;
}

bool SerialInterface::readBytes(uint8_t* buffer, uint8_t len) {
    if (SerialInterface::available() >= len) {
        for (uint8_t i = 0; i < len; i++) {
            buffer[i] = SerialInterface::readByte();
        }
        return true;
    }
    return false;
}