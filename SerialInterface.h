#ifndef SERIAL_INTERFACE_H
#define SERIAL_INTERFACE_H

#include <iostream>
#include <stdint.h>
#include <cstring>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <optional>

// Byte signifying end of data frame
#define END 0x0

using namespace std;
using namespace boost;

class SerialInterface {
private:
    // Boost io executor object
    asio::any_io_executor ioExecutor;
    // Serial port object
    asio::serial_port* serialPort;
    // Serial port timeout
    asio::steady_timer::duration readTimeout;
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
    // Reads from serial port with timeout
    template <typename SyncReadStream, typename MutableBufferSequence>
    bool readWithTimeout(SyncReadStream& s, const MutableBufferSequence& buffers, const asio::steady_timer::duration& expiry_time);
public:
    // Initializes the serial interface
    bool begin(asio::any_io_executor ioExecutor, const char* port, long baudRate, uint16_t timeout);

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

template <typename SyncReadStream, typename MutableBufferSequence>
bool SerialInterface::readWithTimeout(SyncReadStream& s, const MutableBufferSequence& buffers, const asio::steady_timer::duration& expiry_time)
{
    std::optional<boost::system::error_code> timer_result;
    asio::steady_timer timer(ioExecutor);
    timer.expires_after(expiry_time);
    timer.async_wait([&timer_result](const boost::system::error_code& error) { timer_result = error; });

    std::optional<boost::system::error_code> read_result;
    asio::async_read(s, buffers, [&read_result](const boost::system::error_code& error, size_t) { read_result = error; });

    ((boost::asio::io_context&)(ioExecutor).context()).restart();
    while (((boost::asio::io_context&)(ioExecutor).context()).run_one())
    {
        if (read_result) {
            timer.cancel();
            break;
        }
        else if (timer_result) {
            return false;
        }
    }

    if (*read_result)
        std::cerr << "Exception: " << read_result->what() << std::endl;
    return true;
}

template <typename T>
T SerialInterface::readData() {
    T data;
    uint8_t buffer[sizeof(data)];
    SerialInterface::readBytes(buffer, sizeof(data));

    std::memcpy(&data, buffer, sizeof(data));

    return data;
}

#endif // SERIAL_INTERFACE_H