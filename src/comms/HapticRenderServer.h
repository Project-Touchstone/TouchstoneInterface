#ifndef HAPTIC_RENDER_SERVER_H
#define HAPTIC_RENDER_SERVER_H

#include <boost/asio.hpp>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <functional>

using namespace boost;

class HapticRenderServer {
public:
    HapticRenderServer(uint16_t port);
    ~HapticRenderServer();

    void start();
    void stop();

private:
    void acceptConnection();
    void handleClient(std::shared_ptr<boost::asio::ip::tcp::socket> clientSocket);

    // Boost io executor object
    asio::io_context ioContext;
    asio::ip::tcp::acceptor acceptor;
    std::vector<std::thread> workerThreads;
    std::mutex clientsMutex;
    std::vector<std::shared_ptr<boost::asio::ip::tcp::socket>> clients;
    bool isRunning;
};

#endif // RENDER_SERVER_H