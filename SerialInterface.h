#ifndef SERIAL_INTERFACE_H
#define SERIAL_INTERFACE_H

#include <serialib.h>
#include <stdint.h>
#include <string>

// Byte signifying end of data frame
#define END 0x0

class SerialInterface {
    private:
        // Serial object
        static serialib serial;
        // Incoming data buffer
        static uint8_t buffer[64];
        // Incoming data buffer size
        static uint8_t bufferSize;
        // Current header
        static uint8_t header;
        // Whether new header has been received
        static bool headerFlag;
        // Whether current data frame has ended
        static bool endFlag;
    public:
        // Initializes the serial interface
        static uint8_t begin(const char* port, long baudRate);

        // Closes the serial interface
        static void end();

        static uint16_t available();

        // Checks for an incoming header or end byte
        static bool processPacket();

        // Gets the current header
        static uint8_t getHeader();

        // Sends a byte of data
        static void sendByte(uint8_t data);

        static void sendBytes(uint8_t* buffer, uint8_t len);

        // Sends a floating point number
        static void sendFloat32(float data);

        // Sends the end of data frame
        static void sendEnd();

        // Checks if the packet has ended
        static bool isEnded();

        // Reads a byte of data
        static uint8_t readByte();

        static bool readBytes(uint8_t* buffer, uint8_t len);

        template <typename T>
        static T readData();

        // Clears the current packet
        static void clearPacket();
};

template <typename T>
T SerialInterface::readData() {
    T data;
    uint8_t buffer[sizeof(data)];
    SerialInterface::readBytes(buffer, sizeof(data));
    
    std::memcpy(&data, buffer, sizeof(data));

    return data;
}

#endif