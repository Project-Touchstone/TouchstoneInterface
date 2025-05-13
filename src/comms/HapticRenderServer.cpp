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
        if (client.socket->is_open()) {
            client.socket->close();
        }
    }

    clients.clear();
    std::cout << "HapticRenderServer stopped." << std::endl;
}

void HapticRenderServer::acceptConnection() {
    auto clientSocket = std::make_shared<asio::ip::tcp::socket>(ioContext);
    auto buffer = std::make_shared<std::vector<char>>(1024);
    acceptor.async_accept(*clientSocket, [this, clientSocket, buffer](const boost::system::error_code& error) {
        if (!error) {
            clientType client = { clientSocket, buffer };
            {
                std::lock_guard<std::mutex> lock(clientsMutex);
                clients.push_back(client);
            }

            std::cout << "New client connected: " << clientSocket->remote_endpoint() << std::endl;
            handleClient(client);
        }
        else {
            std::cerr << "Error accepting connection: " << error.message() << std::endl;
        }

        if (isRunning) {
            acceptConnection();
        }
        });
}

void HapticRenderServer::setRequestHandler(std::function<void(clientType)> handler) {
	requestHandler = handler;
}

void HapticRenderServer::handleClient(clientType client) {
    auto buffer = std::make_shared<std::vector<char>>(1024); // Allocate a new buffer for this read
    client.socket->async_read_some(asio::buffer(*buffer),
        [this, client, buffer](const system::error_code& error, std::size_t bytesTransferred) mutable {
            if (!error) {
                client.buffer->insert(client.buffer->end(), buffer->begin(), buffer->begin() + bytesTransferred);
                client.bufferSize += bytesTransferred;

                if (client.endFlag) {
                    client.endFlag = false;
                    client.header = readByte(client);
                }
                requestHandler(client);

                // Continue reading from the client
                handleClient(client);
            }
            else {
                std::cerr << "Error reading from client: " << error.message() << std::endl;

                std::lock_guard<std::mutex> lock(clientsMutex);
                clients.erase(std::remove(clients.begin(), clients.end(), client), clients.end());
            }
        });
}

void HapticRenderServer::sendByte(clientType client, uint8_t value) {
	sendBytes(client, &value, sizeof(value));
}

void HapticRenderServer::sendBytes(clientType client, uint8_t* buffer, uint8_t len) {
	// Asynchronously write the buffer to the socket
	asio::async_write(*client.socket, asio::buffer(buffer, len),
		[len](const boost::system::error_code& error, std::size_t bytesTransferred) {
			if (error) {
				std::cerr << "Error sending bytes: " << error.message() << std::endl;
			}
			else if (bytesTransferred < len) {
				std::cerr << "Partial write detected. Ensure all bytes are sent." << std::endl;
			}
		});
}

void HapticRenderServer::sendFloat(clientType client, float value) {
    // Convert float to network byte order
    uint32_t networkValue = htonl(*reinterpret_cast<uint32_t*>(&value));
    uint8_t buffer[sizeof(networkValue)];
    memcpy(buffer, &networkValue, sizeof(networkValue));

    // Asynchronously write the buffer to the socket
	sendBytes(client, buffer, sizeof(networkValue));
}

uint8_t HapticRenderServer::readByte(clientType client) {
	uint8_t value;
	readBytes(client, &value, sizeof(value));
    return value;
}

void HapticRenderServer::readBytes(clientType client, uint8_t* buffer, uint8_t len) {
	//Copies len bytes of buffer into an array
	memcpy(buffer, client.buffer->data(), len);

	// Removes the read bytes from the buffer
	client.buffer->erase(client.buffer->begin(), client.buffer->begin() + len);
	client.bufferSize -= len;
}

float HapticRenderServer::readFloat(clientType client) {
	uint32_t networkValue;
    uint8_t buffer[sizeof(networkValue)];
	readBytes(client, buffer, sizeof(networkValue));
    memcpy(&networkValue, buffer, sizeof(networkValue));
	return ntohl(*reinterpret_cast<float*>(&networkValue));
}

void HapticRenderServer::clearPacket(clientType client) {
	client.endFlag = true;
	client.header = 0;
}