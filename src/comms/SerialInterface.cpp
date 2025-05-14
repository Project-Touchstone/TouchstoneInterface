#include "SerialInterface.h"

using namespace std;
using namespace boost;

SerialInterface::SerialInterface()
    : ioContext(),
      serialStream(std::make_shared<SerialStream>(std::make_shared<asio::serial_port>(ioContext))),
      readTimeoutTimer(ioContext),
      dataProtocol(serialStream) {
    dataProtocol.setEndianness(DataProtocol::Endianness::LittleEndian); // Set to LittleEndian
}

DataProtocol* SerialInterface::getDataProtocol() {
	return &dataProtocol;
}

void SerialInterface::setDataHandler(std::function<void(DataProtocol*)> handler) {
	dataHandler = std::move(handler);
}

void SerialInterface::setTimeoutHandler(std::function<void(DataProtocol*)> handler) {
    timeoutHandler = std::move(handler);
}

/// @brief Initializes the serial interface
/// @param port Serial port file path
/// @param baudRate Baud rate of serial communication
bool SerialInterface::begin(const char* port, long baudRate, uint16_t timeout, size_t bufferSize) {
    try {
        auto serialPort = serialStream->getSerialPort();
        serialPort->open(port);
        serialPort->set_option(asio::serial_port_base::baud_rate(baudRate));
        serialPort->set_option(asio::serial_port_base::character_size(8));
        serialPort->set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
        serialPort->set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
        serialPort->set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
		this->timeout = timeout;
        // Starts io thread
		ioThread = std::thread([this]() {
            ioContext.run();
		});
		// Begins asynchronous read
        readAsync(bufferSize);
    } catch (boost::system::system_error& e) {
        cerr << "Error opening serial port: " << e.what() << endl;
        return false; // Error opening the port
    }
    return true; // Success
}

void SerialInterface::end() {
    ioContext.stop();
    ioThread.join();
    serialStream->close();
}

bool SerialInterface::timedout() {
    return timeoutFlag;
}

void SerialInterface::flushUntilTimeout() {
    flushFlag = true;
    while (!timedout()) {
        Utils::sleep(10);
	}
    resetTimeout();
    flushFlag = false;
}

void SerialInterface::readAsync(std::size_t bufferSize) {
    dataProtocol.setReadHandler([this, bufferSize](DataProtocol* data, const system::error_code& error, std::size_t bytesTransferred) {
        if (!error) {
            if (bytesTransferred > 0) {
                if (flushFlag) {
                    // If flush is active, clear the buffer
                    data->flush();
                }
                else {
                    if (dataHandler) {
                        dataHandler(data);
                    }
                }

                //Resets timeout timer
                resetTimeout();
            }

            // Continue reading from the serial port
            data->asyncReadBytes(bufferSize);
        }
        else {
            std::cerr << "Error reading from serial port: " << error.message() << std::endl;
        }
    });
    dataProtocol.asyncReadBytes(bufferSize);
}

void SerialInterface::resetTimeout() {
	timeoutFlag = false;
	readTimeoutTimer.cancel();
	readTimeoutTimer.expires_after(boost::asio::chrono::milliseconds(this->timeout));
	readTimeoutTimer.async_wait([this](const boost::system::error_code& error) {
		if (error != boost::asio::error::operation_aborted) {
			timeoutFlag = true;
			if (!flushFlag && timeoutHandler) {
				timeoutHandler(&dataProtocol);
			}
		}
	});
}