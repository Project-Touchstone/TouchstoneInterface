#ifndef MINBIT_SERIAL_CLIENT_H
#define MINBIT_SERIAL_CLIENT_H

#include <memory>
#include <string>
#include <boost/asio.hpp>
#include "SerialStream.h"
#include "MinBiTCore.h"

class MinBiTSerialClient {
public:
    using ReadHandler = std::function<void(std::shared_ptr<MinBiTCore>, std::shared_ptr<MinBiTCore::Request>)>;

    MinBiTSerialClient(std::string name);
    ~MinBiTSerialClient();

    // Initialize and open the serial port
    bool begin(const std::string& port, unsigned int baudRate);

    // Sets read handler
    void setReadHandler(ReadHandler readHandler);

    // Close the serial port
    void end();

    // Get the MinBiTCore protocol object
    std::shared_ptr<MinBiTCore> getProtocol();

    // Check if the serial port is open
    bool isOpen() const;

private:
    boost::asio::io_context ioContext;
    std::shared_ptr<SerialStream> serialStream;
    std::shared_ptr<MinBiTCore> protocol;
    std::thread ioThread;
    bool running = false;

    ReadHandler readHandler;

    // Attaches protocol
    void attachProtocol();
};

#endif // MINBIT_SERIAL_CLIENT_H