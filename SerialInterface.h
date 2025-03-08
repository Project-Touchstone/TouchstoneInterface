#ifndef SERIAL_INTERFACE_H
#define SERIAL_INTERFACE_H

#include <iostream>
#include <stdint.h>
#include <cstring>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/bind/bind.hpp>
#include <optional>

// Byte signifying end of data frame
#define END 0x0

using namespace std;
using namespace boost;

class SerialInterface {
private:
    // Boost io executor object
    asio::io_context ioContext;
    // Serial port object
    asio::serial_port serialPort;
    // Timeout time
    int32_t timeout = 1000;
    // Timeout timer
    boost::asio::system_timer readTimeoutTimer;
    // Incoming data byte
    std::array<std::byte, 1> byteBuffer;
    //Incoming data byffer
    uint8_t readBuffer[64];
    // Incoming data buffer size
    uint8_t bufferSize = 0;
    // Current header
    uint8_t header = 0;
    // Whether new header has been received
    volatile bool headerFlag = false;
    // Whether current data frame has ended
    volatile bool endFlag = true;
    // Whether asynchronous read has timed out
    volatile bool timeoutFlag = false;
    // Ansychronous read handler function
    void readHandler(const boost::system::error_code& error, std::size_t bytes_transferred);
    // Timeout handler function
    void timeoutHandler(const boost::system::error_code& error);
public:
    SerialInterface();

    // Initializes the serial interface
    bool begin(const char* port, long baudRate, uint16_t timeout);

    // Closes the serial interface
    void end();

    uint16_t available();

    bool timedout();

    bool headerReady();

    void update(int32_t timeout = -1);

    void flush();

    // Reads from serial port with timeout
    bool readAsync(int32_t timeout = -1);

    // Gets the current header
    uint8_t getHeader();

    // Sends a byte of data
    void sendByte(uint8_t data);

    void sendBytes(uint8_t* buffer, uint8_t len);

    // Sends a 16 bit integer
	void sendInt16(int16_t data);

    // Sends a floating point number
    void sendFloat(float data);

    // Sends the end of data frame
    void sendEnd();

    // Checks if the packet has ended
    bool isPacketEnded();

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

#endif // SERIAL_INTERFACE_H