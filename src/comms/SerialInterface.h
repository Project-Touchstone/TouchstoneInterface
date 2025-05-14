#ifndef SERIAL_INTERFACE_H
#define SERIAL_INTERFACE_H

//External imports
#include <iostream>
#include <stdint.h>
#include <cstring>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/bind/bind.hpp>
#include <optional>
#include <queue>

//Local imports
#include "DataProtocol.h"
#include "SerialStream.h" // Include SerialStream

// Byte signifying end of data frame
#define END 0x0

using namespace std;
using namespace boost;

class SerialInterface {
private:
    // Boost io executor object
    asio::io_context ioContext;
    // Serial stream object
    std::shared_ptr<SerialStream> serialStream; // Use SerialStream
    // Data protocol object
	DataProtocol dataProtocol;
    // Data handler function
	std::function<void(DataProtocol*)> dataHandler;
    // IO execution thread
	thread ioThread;
    // Timeout time
    int32_t timeout;
    // Timeout timer
    boost::asio::system_timer readTimeoutTimer;
    // Whether data is currently being flushed
    bool flushFlag = false;
    // Whether asynchronous read has timed out
    bool timeoutFlag = false;

    // Reads from serial port with timeout
    void readAsync(std::size_t bufferSize);
public:
    SerialInterface();

    // Gets data protocol pointer
	DataProtocol* getDataProtocol() {
		return &dataProtocol;
	}

    // Sets data handler
    void setDataHandler(std::function<void(DataProtocol*)> handler);

    // Initializes the serial interface
    bool begin(const char* port, long baudRate, uint16_t timeout, size_t bufferSize);

    // Closes the serial interface
    void end();

    bool timedout();

    void flushUntilTimeout();
};

#endif // SERIAL_INTERFACE_H