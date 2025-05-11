#include "HapticRenderServer.h"
#include <iostream>

using namespace boost;

HapticRenderServer::HapticRenderServer(uint16_t port)
    : ioContext(),
    acceptor(ioContext, asio::ip::tcp::endpoint(asio::ip::tcp::v4(), port)),
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
    auto clientSocket = std::make_shared<asio::ip::tcp::socket>(ioContext);
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

void HapticRenderServer::handleClient(std::shared_ptr<asio::ip::tcp::socket> clientSocket) {
    auto buffer = std::make_shared<std::vector<char>>(1024);

    clientSocket->async_read_some(asio::buffer(*buffer),
        [this, clientSocket, buffer](const system::error_code& error, std::size_t bytesTransferred) {
            if (!error) {
                std::string message(buffer->data(), bytesTransferred);
                std::cout << "Received message: " << message << std::endl;

                // Echo the message back to the client
                /*asio::async_write(*clientSocket, asio::buffer(message),
                    [this, clientSocket](const system::error_code& writeError, std::size_t) {
                        if (writeError) {
                            std::cerr << "Error sending response: " << writeError.message() << std::endl;
                        }
                    });*/
                float test = 123.456;
                sendFloat(clientSocket, test);

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

void HapticRenderServer::sendFloat(std::shared_ptr<asio::ip::tcp::socket> clientSocket, float value) {
    // Convert float to network byte order
    uint32_t networkValue = htonl(*reinterpret_cast<uint32_t*>(&value));
    uint8_t buffer[sizeof(networkValue)];
    memcpy(buffer, &networkValue, sizeof(networkValue));

    // Asynchronously write the buffer to the socket
    asio::async_write(*clientSocket, asio::buffer(buffer, sizeof(networkValue)),
        [this, clientSocket](const boost::system::error_code& error, std::size_t bytesTransferred) {
            if (error) {
                std::cerr << "Error sending float: " << error.message() << std::endl;
            }
            else if (bytesTransferred < sizeof(uint32_t)) {
                std::cerr << "Partial write detected. Ensure all bytes are sent." << std::endl;
            }
        });
}