#include "HapticRenderServer.h"
#include <iostream>

using namespace boost;

HapticRenderServer::HapticRenderServer(uint16_t port)
    : ioContext(),
    acceptor(ioContext, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port)),
    isRunning(false) {
}

HapticRenderServer::~HapticRenderServer() {
    stop();
}

void HapticRenderServer::start() {
    isRunning = true;
    acceptConnection();

    // Start worker threads to handle asynchronous operations
    for (size_t i = 0; i < std::thread::hardware_concurrency(); ++i) {
        workerThreads.emplace_back([this]() { ioContext.run(); });
    }

    std::cout << "HapticRenderServer started and listening for connections..." << std::endl;
}

void HapticRenderServer::stop() {
    if (!isRunning) return;

    isRunning = false;
    ioContext.stop();

    // Join all worker threads
    for (auto& thread : workerThreads) {
        if (thread.joinable()) {
            thread.join();
        }
    }

    std::lock_guard<std::mutex> lock(clientsMutex);
    for (auto& client : clients) {
        if (client->is_open()) {
            client->close();
        }
    }

    clients.clear();
    std::cout << "HapticRenderServer stopped." << std::endl;
}

void HapticRenderServer::acceptConnection() {
    auto clientSocket = std::make_shared<boost::asio::ip::tcp::socket>(ioContext);
    acceptor.async_accept(*clientSocket, [this, clientSocket](const boost::system::error_code& error) {
        if (!error) {
            {
                std::lock_guard<std::mutex> lock(clientsMutex);
                clients.push_back(clientSocket);
            }

            std::cout << "New client connected: " << clientSocket->remote_endpoint() << std::endl;
            handleClient(clientSocket);
        }
        else {
            std::cerr << "Error accepting connection: " << error.message() << std::endl;
        }

        if (isRunning) {
            acceptConnection();
        }
        });
}

void HapticRenderServer::handleClient(std::shared_ptr<boost::asio::ip::tcp::socket> clientSocket) {
    auto buffer = std::make_shared<std::vector<char>>(1024);

    clientSocket->async_read_some(boost::asio::buffer(*buffer),
        [this, clientSocket, buffer](const boost::system::error_code& error, std::size_t bytesTransferred) {
            if (!error) {
                std::string message(buffer->data(), bytesTransferred);
                std::cout << "Received message: " << message << std::endl;

                // Echo the message back to the client
                boost::asio::async_write(*clientSocket, boost::asio::buffer(message),
                    [this, clientSocket](const boost::system::error_code& writeError, std::size_t) {
                        if (writeError) {
                            std::cerr << "Error sending response: " << writeError.message() << std::endl;
                        }
                    });

                // Continue reading from the client
                handleClient(clientSocket);
            }
            else {
                std::cerr << "Error reading from client: " << error.message() << std::endl;

                std::lock_guard<std::mutex> lock(clientsMutex);
                clients.erase(std::remove(clients.begin(), clients.end(), clientSocket), clients.end());
            }
        });
}