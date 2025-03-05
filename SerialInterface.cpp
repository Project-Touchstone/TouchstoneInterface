#include "SerialInterface.h"

using namespace std;
using namespace boost;

/// @brief Initializes the serial interface
/// @param port Serial port file path
/// @param baudRate Baud rate of serial communication
bool SerialInterface::begin(asio::any_io_executor ioExecutor, const char* port, long baudRate) {
    try {
        serialPort = new asio::serial_port(ioExecutor);
        serialPort->open(port);
        serialPort->set_option(asio::serial_port_base::baud_rate(baudRate));
        serialPort->set_option(asio::serial_port_base::character_size(8));
        serialPort->set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
        serialPort->set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
        serialPort->set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
    }
    catch (boost::system::system_error& e) {
        cerr << "Error opening serial port: " << e.what() << endl;
        return false; // Error opening the port
    }
    return true; // Success
}

void SerialInterface::end() {
    if (serialPort->is_open()) {
        serialPort->close();
    }
}

uint16_t SerialInterface::available() {
    return bufferSize;
}

/// @brief Blocks until serial data is available and checks for a header or end byte
/// @return true (packet to process), false (no packet to process)
bool SerialInterface::processPacket() {
    if (!serialPort->is_open()) {
        return false;
    }

    uint8_t buffer[1];
    system::error_code error;

    try {
        size_t bytes_read = asio::read(*serialPort, boost::asio::buffer(buffer, 1), error);
        if (error || bytes_read == 0) {
            return false;
        }
    } catch (system::system_error& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return false;
    }

    if (buffer[0] == END) {
        endFlag = true;
        return false;
    }
    else if (endFlag) {
        endFlag = false;
        header = buffer[0];
        headerFlag = true;
    }
    else {
        readBuffer[bufferSize++] = buffer[0];
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
    try {
        if (serialPort->is_open()) {
            uint8_t bytes[1] = { data };
            string buffer(reinterpret_cast<char*>(bytes), 1);
            asio::write(*serialPort, asio::buffer(buffer, 1));
        }
    }
    catch (system::system_error& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
}

void SerialInterface::sendBytes(uint8_t* buffer, uint8_t len) {
    if (serialPort->is_open()) {
        asio::write(*serialPort, asio::buffer(buffer, len));
    }
}

void SerialInterface::sendFloat32(float data) {
    uint8_t buffer[sizeof(data)];
    memcpy(buffer, &data, sizeof(data));
    sendBytes(buffer, sizeof(data));
}

void SerialInterface::sendEnd() {
    SerialInterface::sendByte(END);
}

uint8_t SerialInterface::readByte() {
    if (available() > 0) {
        return readBuffer[--bufferSize];
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