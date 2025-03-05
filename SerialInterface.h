#ifndef SERIAL_INTERFACE_H
#define SERIAL_INTERFACE_H

#include <iostream>
#include <stdint.h>
#include <cstring>
#include <boost/asio.hpp>

// Byte signifying end of data frame
#define END 0x0

using namespace std;
using namespace boost;

class SerialInterface {
    private:
        //Serial port object
        asio::serial_port* serialPort;
        // Incoming data buffer
        uint8_t readBuffer[64];
        // Incoming data buffer size
        uint8_t bufferSize = 0;
        // Current header
        uint8_t header = 0;
        // Whether new header has been received
        bool headerFlag = false;
        // Whether current data frame has ended
        bool endFlag = true;
    public:
        // Initializes the serial interface
        bool begin(asio::any_io_executor ioExecutor, const char* port, long baudRate);

        // Closes the serial interface
        void end();

        uint16_t available();

        // Checks for an incoming header or end byte
        bool processPacket();

        // Gets the current header
        uint8_t getHeader();

        // Sends a byte of data
        void sendByte(uint8_t data);

        void sendBytes(uint8_t* buffer, uint8_t len);

        // Sends a floating point number
        void sendFloat32(float data);

        // Sends the end of data frame
        void sendEnd();

        // Checks if the packet has ended
        bool isEnded();

        // Reads a byte of data
        uint8_t readByte();

        bool readBytes(uint8_t* buffer, uint8_t len);

        template <typename T>
        T readData();

        // Clears the current packet
        void clearPacket();
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