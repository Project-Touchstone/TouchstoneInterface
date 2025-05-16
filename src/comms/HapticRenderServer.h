#ifndef HAPTIC_RENDER_SERVER_H
#define HAPTIC_RENDER_SERVER_H

//External imports
#include <boost/asio.hpp>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <functional>

//Local imports
#include "DataProtocol.h"
#include "TcpStream.h" // Include TcpStream

using namespace boost;

class HapticRenderServer {
public:
    HapticRenderServer(uint16_t port);
    ~HapticRenderServer();

    void start();
    void stop();

    void setRequestHandler(std::function<void(std::shared_ptr<DataProtocol>)> handler);

private:
    void acceptConnection();
    void handleClient(std::shared_ptr<DataProtocol> client);

    // Boost io executor object
    asio::io_context ioContext;
    asio::ip::tcp::acceptor acceptor;
    std::vector<std::thread> workerThreads;
    std::mutex clientsMutex;
    // Vector to hold shared_ptr<DataProtocol> objects for each client
    std::vector<std::shared_ptr<DataProtocol>> clients; // Use shared_ptr for DataProtocol
    bool isRunning;

	// Request handler
    std::function<void(std::shared_ptr<DataProtocol>)> requestHandler;
};

#endif // HAPTIC_RENDER_SERVER_H