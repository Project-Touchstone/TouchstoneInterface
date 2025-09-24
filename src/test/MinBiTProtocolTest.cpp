#include "../comms/MinBiTTcpServer.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <memory>

#define SERVER_PORT 8080

int main() {
    MinBiTTcpServer server("TestServer", 8080);
    
    std::cout << "Waiting for client connection on port " << SERVER_PORT << "..." << std::endl;
    if (!server.begin()) {
        std::cerr << "Failed to start MinBiTTcpServer on port " << SERVER_PORT << std::endl;
        return 1;
    }

    auto proto = server.getProtocol();
    proto->loadPacketLengthsFromJson("test_packet_lengths.json");

    std::cout << "Client connected!" << std::endl;

    // Aynchronous test: send a request and wait for response
    std::cout << "[ASYNC] Sending request with header 1..." << std::endl;
    auto asyncRequest = proto->writeRequest(1);
    proto->sendAll();
    int timeoutMs = 2000;
    auto future = asyncRequest->WaitAsync();
    if (future.get() == MinBiTCore::Request::Status::COMPLETE) {
        std::cout << "[ASYNC] Response received for header 1." << std::endl;
        // Prints response data
        while (proto->getReservedBytes() > 0) {
            std::cout << proto->readByte() << " ";
        }
		std::cout << std::endl;
    }
    else {
        std::cout << "[ASYNC] Timeout or error waiting for response." << std::endl;
    }

    // Read handler test: set a read handler and send a request
    bool handlerReceived = false;
    proto->setReadHandler([&](std::shared_ptr<MinBiTCore::Request> req) {
        if (req && req->GetHeader() == 2) {
            std::cout << "[HANDLER] Response received for header 2." << std::endl;
            // Prints response data
            while (proto->getReservedBytes() > 0) {
                std::cout << proto->readByte() << " ";
            }
            std::cout << std::endl;
            handlerReceived = true;
        }
        });
    std::cout << "[HANDLER] Sending request with header 2..." << std::endl;
    auto handlerRequest = proto->writeRequest(2);
    proto->sendAll();
    // Wait for read handler to be called
    int handlerTimeoutMs = 2000;
    auto start = std::chrono::steady_clock::now();
    while (!handlerReceived && std::chrono::steady_clock::now() - start < std::chrono::milliseconds(handlerTimeoutMs)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!handlerReceived) {
        std::cout << "[HANDLER] Timeout or error waiting for async response." << std::endl;
    }

    server.end();
    std::cout << "Test complete." << std::endl;
    return 0;
}