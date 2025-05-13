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

    struct clientType {
        std::shared_ptr<asio::ip::tcp::socket> socket;
        std::shared_ptr<std::vector<char>> buffer;
        uint8_t header = 0;
        int bufferSize = 0;
        bool endFlag = true;
    };

	void setRequestHandler(std::function<void(clientType)> handler);
	void sendByte(clientType client, uint8_t value);
	void sendBytes(clientType client, uint8_t* buffer, uint8_t len);
    void sendFloat(clientType client, float value);

	uint8_t readByte(clientType client);
	void readBytes(clientType client, uint8_t* buffer, uint8_t len);
	float readFloat(clientType client);

    void clearPacket(clientType client);
private:
    void acceptConnection();
    void handleClient(clientType client);

    // Boost io executor object
    asio::io_context ioContext;
    asio::ip::tcp::acceptor acceptor;
    std::vector<std::thread> workerThreads;
    std::mutex clientsMutex;
    //Vector to hold client objects
    std::vector<clientType> clients;
    bool isRunning;

	// Request handler
    std::function<void(clientType)> requestHandler;
};

#endif // RENDER_SERVER_H