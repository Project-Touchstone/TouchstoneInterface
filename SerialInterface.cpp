#include "SerialInterface.h"

uint8_t SerialInterface::header = 0;
bool SerialInterface::headerFlag = false;
bool SerialInterface::endFlag = true;

/// @brief Initializes the serial interface
/// @param baudRate Baud rate of serial communication
void SerialInterface::begin(long baudRate) {
    Serial.begin(baudRate);
    while (!Serial) {
        // Wait for serial port to connect
    }
}

/// @brief Checks incoming serial data for a header or end byte
/// @return true (header to process), false (no header to process)
bool SerialInterface::processingHeader() {
    // Ensures last data frame and header have been processed
    if (Serial.available()) {
        // Reads one byte of data
        uint8_t byte = Serial.peek();
        
        if (byte == END) {
            // Sets end flag
            endFlag = true;
            return false;
        } else if (endFlag) {
            // If end of data frame was already reached, starts new data frame
            endFlag = false;
            header = Serial.read();
            // Sets header flag
            headerFlag = true;
        }

        return true;
    } else {
        return false;
    }
}

bool SerialInterface::isEnded() {
    return endFlag;
}

uint8_t SerialInterface::getHeader() {
    return header;
}

void SerialInterface::clearHeader() {
    headerFlag = false;
    endFlag = true;
}

void SerialInterface::sendByte(uint8_t data) {
    Serial.write(data);
}

void SerialInterface::sendEnd() {
    Serial.write(END);
}

uint8_t SerialInterface::readByte() {
    if (Serial.available() > 0) {
        return Serial.read();
    }
    return 0;
}

float SerialInterface::readFloat() {
    float value;
    if (Serial.available() >= sizeof(value)) {
        value = Serial.parseFloat();
    }
    return value;
}