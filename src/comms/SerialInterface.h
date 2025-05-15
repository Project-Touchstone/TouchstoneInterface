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
#include "../utils/Utils.h"

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
    std::unique_ptr<DataProtocol> dataProtocol; // Use unique_ptr for ownership
    // Data handler function
	std::function<void(DataProtocol*)> dataHandler;
	// Timeout handler function
    std::function<void(DataProtocol*)> timeoutHandler;
    // IO execution thread
	std::thread ioThread;
    // Timeout time
    int32_t timeout = 1000;
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
    ~SerialInterface(); // Add destructor

    // Gets data protocol pointer
    DataProtocol* getDataProtocol();

    // Sets data handler
    void setDataHandler(std::function<void(DataProtocol*)> handler);

	// Sets timeout handler
	void setTimeoutHandler(std::function<void(DataProtocol*)> handler);

    // Initializes the serial interface
    bool begin(const char* port, long baudRate, uint16_t timeout, size_t bufferSize);

    // Closes the serial interface
    void end();

    // Checks for timeout
    bool timedout();
    // Resets timeout
    void resetTimeout();
    // Flushes buffer until timeout is reached
    void flushUntilTimeout();
};

#endif // SERIAL_INTERFACE_H