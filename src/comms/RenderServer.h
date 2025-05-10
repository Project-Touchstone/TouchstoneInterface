#ifndef RENDER_SERVER_H
#define RENDER_SERVER_H

#include <boost/asio.hpp>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <functional>

class RenderServer {
public:
    RenderServer(boost::asio::io_context& ioContext, uint16_t port);
    ~RenderServer();

    void start();
    void stop();

private:
    void acceptConnection();
    void handleClient(std::shared_ptr<boost::asio::ip::tcp::socket> clientSocket);

    boost::asio::io_context& ioContext_;
    boost::asio::ip::tcp::acceptor acceptor_;
    std::vector<std::thread> workerThreads_;
    std::mutex clientsMutex_;
    std::vector<std::shared_ptr<boost::asio::ip::tcp::socket>> clients_;
    bool isRunning_;
};

#endif // RENDER_SERVER_H