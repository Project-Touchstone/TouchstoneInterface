#include "SerialInterface.h"

using namespace std;
using namespace boost;


SerialInterface::SerialInterface() : ioContext(), serialPort(ioContext), readTimeoutTimer(ioContext) {};

/// @brief Initializes the serial interface
/// @param port Serial port file path
/// @param baudRate Baud rate of serial communication
bool SerialInterface::begin(const char* port, long baudRate, uint16_t timeout) {
    try {
        serialPort.open(port);
        serialPort.set_option(asio::serial_port_base::baud_rate(baudRate));
        serialPort.set_option(asio::serial_port_base::character_size(8));
        serialPort.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
        serialPort.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
        serialPort.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
    }
    catch (boost::system::system_error& e) {
        cerr << "Error opening serial port: " << e.what() << endl;
        return false; // Error opening the port
    }
    return true; // Success
}

void SerialInterface::end() {
    if (serialPort.is_open()) {
        serialPort.close();
    }
}

uint16_t SerialInterface::available() {
    return readQueue.size();
}

bool SerialInterface::timedout() {
    return timeoutFlag;
}

bool SerialInterface::headerReady() {
    return headerFlag;
}

void SerialInterface::update(int32_t timeout) {
    if (!ioContext.run_one()) {
        ioContext.restart();
        readAsync(timeout);
        ioContext.run_one();
    }
}

void SerialInterface::flush() {
	while (!timeoutFlag) {
        clearPacket();
        update();
	}
}

bool SerialInterface::readAsync(int32_t timeout)
{
    try
    {
        if (timeout not_eq -1)
        {
            this->timeout = timeout;//If read_timeout is not set to ignore_timeout, update the read_timeout else use old read_timeout
        }
        serialPort.async_read_some(
            boost::asio::buffer(
                byteBuffer.data(),
                1
            ),
            boost::bind(
                &SerialInterface::readHandler,
                this,
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred
            )
        );
        readTimeoutTimer.expires_after(boost::asio::chrono::milliseconds(this->timeout));   // Reset timer to current timestamp + timeout time
        readTimeoutTimer.async_wait(boost::bind(&SerialInterface::timeoutHandler, this, boost::asio::placeholders::error));
        return true;
    }
    catch (const std::exception& ex)
    {
        return false;
    }
}

void SerialInterface::readHandler(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    try
    {
        if (error not_eq boost::system::errc::success)  //Error in serial port read
        {
            return;
        }

        // Cancels timer
        readTimeoutTimer.cancel();

        // Adds byte to read buffer
        uint8_t currByte = static_cast<uint8_t>(byteBuffer[0]);
        
		if (checkEndFlag && currByte == END) {
            checkEndFlag = false;
			endFlag = true;
		}
        else if (endFlag) {
            endFlag = false;
            header = currByte;
            headerFlag = true;
        }
        else {
            readQueue.push(currByte);
        }
    }
    catch (const std::exception& ex)
    {
    }
}

void SerialInterface::timeoutHandler(const boost::system::error_code& error)
{
    try
    {
        if (error != boost::asio::error::operation_aborted)  // Check if the timer was not cancelled
        {
            timeoutFlag = true;
            serialPort.cancel();
        }
    }
    catch (const std::exception& ex)
    {
    }
}

bool SerialInterface::isPacketEnded() {
    return endFlag;
}

void SerialInterface::checkEnd() {
    checkEndFlag = true;
}

uint8_t SerialInterface::getHeader() {
    return header;
}

void SerialInterface::clearPacket() {
    headerFlag = false;
    timeoutFlag = false;
    endFlag = true;
    while (!readQueue.empty()) {
		readQueue.pop();
    }
}

void SerialInterface::sendByte(uint8_t data) {
    try {
        if (serialPort.is_open()) {
            uint8_t bytes[1] = { data };
            asio::write(serialPort, asio::buffer(bytes, 1));
        }
    }
    catch (system::system_error& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
}

void SerialInterface::sendBytes(uint8_t* buffer, uint8_t len) {
    if (serialPort.is_open()) {
        asio::write(serialPort, asio::buffer(buffer, len));
    }
}

void SerialInterface::sendInt16(int16_t data) {
    uint8_t buffer[sizeof(data)];
    memcpy(buffer, &data, sizeof(data));
    sendBytes(buffer, sizeof(data));
}

void SerialInterface::sendFloat(float data) {
    uint8_t buffer[sizeof(data)];
    memcpy(buffer, &data, sizeof(data));
    sendBytes(buffer, sizeof(data));
}

void SerialInterface::sendEnd() {
    SerialInterface::sendByte(END);
}

uint8_t SerialInterface::readByte() {
    if (available() > 0) {
		uint8_t byte = readQueue.front();
        readQueue.pop();
		return byte;
    }
    return 0;
}

bool SerialInterface::readBytes(uint8_t* buffer, uint8_t len) {
    if (available() >= len) {
        for (uint8_t i = 0; i < len; i++) {
            buffer[i] = readByte();
        }
        return true;
    }
    return false;
}