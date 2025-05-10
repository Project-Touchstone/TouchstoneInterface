#include "RenderServer.h"
#include <iostream>

RenderServer::RenderServer(boost::asio::io_context& ioContext, uint16_t port)
    : ioContext_(ioContext),
    acceptor_(ioContext, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port)),
    isRunning_(false) {
}

RenderServer::~RenderServer() {
    stop();
}

void RenderServer::start() {
    isRunning_ = true;
    acceptConnection();

    // Start worker threads to handle asynchronous operations
    for (size_t i = 0; i < std::thread::hardware_concurrency(); ++i) {
        workerThreads_.emplace_back([this]() { ioContext_.run(); });
    }

    std::cout << "RenderServer started and listening for connections..." << std::endl;
}

void RenderServer::stop() {
    if (!isRunning_) return;

    isRunning_ = false;
    ioContext_.stop();

    // Join all worker threads
    for (auto& thread : workerThreads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }

    std::lock_guard<std::mutex> lock(clientsMutex_);
    for (auto& client : clients_) {
        if (client->is_open()) {
            client->close();
        }
    }

    clients_.clear();
    std::cout << "RenderServer stopped." << std::endl;
}

void RenderServer::acceptConnection() {
    auto clientSocket = std::make_shared<boost::asio::ip::tcp::socket>(ioContext_);
    acceptor_.async_accept(*clientSocket, [this, clientSocket](const boost::system::error_code& error) {
        if (!error) {
            {
                std::lock_guard<std::mutex> lock(clientsMutex_);
                clients_.push_back(clientSocket);
            }

            std::cout << "New client connected: " << clientSocket->remote_endpoint() << std::endl;
            handleClient(clientSocket);
        }
        else {
            std::cerr << "Error accepting connection: " << error.message() << std::endl;
        }

        if (isRunning_) {
            acceptConnection();
        }
        });
}

void RenderServer::handleClient(std::shared_ptr<boost::asio::ip::tcp::socket> clientSocket) {
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

                std::lock_guard<std::mutex> lock(clientsMutex_);
                clients_.erase(std::remove(clients_.begin(), clients_.end(), clientSocket), clients_.end());
            }
        });
}