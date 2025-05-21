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
    for (size_t i = 0; i < 1; ++i) {
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
    clients.clear();
    std::cout << "HapticRenderServer stopped." << std::endl;
}

void HapticRenderServer::acceptConnection() {
    auto clientSocket = std::make_shared<asio::ip::tcp::socket>(ioContext);
    auto tcpStream = std::make_shared<TcpStream>(clientSocket);
    auto client = std::make_shared<DataProtocol>(tcpStream);
    client->setEndianness(DataProtocol::Endianness::BigEndian); // Set to BigEndian

    acceptor.async_accept(*clientSocket, [this, client, tcpStream](const boost::system::error_code& error) {
        if (!error) {
            {
                std::lock_guard<std::mutex> lock(clientsMutex);
                clients.push_back(client);
            }

            std::cout << "New client connected: " << tcpStream->getSocket()->remote_endpoint() << std::endl;
            handleClient(client);
        } else {
            std::cerr << "Error accepting connection: " << error.message() << std::endl;
        }

        if (isRunning) {
            acceptConnection();
        }
    });
}

void HapticRenderServer::setRequestHandler(std::function<void(std::shared_ptr<DataProtocol>)> handler) {
	requestHandler = handler;
}

void HapticRenderServer::handleClient(std::shared_ptr<DataProtocol> client) {
    client->setReadHandler([this, client](const system::error_code& error, std::size_t bytesTransferred) {
        if (!error && requestHandler) {
            requestHandler(client);
        }
        else {
            std::cerr << "Error reading from client: " << error.message() << std::endl;
        }

        // Continue reading from the client
        if (isRunning) {
            client->asyncReadBytes();
        }
    });
	client->asyncReadBytes();
}