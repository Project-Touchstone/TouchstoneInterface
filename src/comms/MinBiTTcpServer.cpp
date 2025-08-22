#include "MinBiTTcpServer.h"

MinBiTTcpServer::MinBiTTcpServer(unsigned short port)
    : listenPort(port)
{
}

MinBiTTcpServer::~MinBiTTcpServer() {
    stop();
}

bool MinBiTTcpServer::start() {
    try {
        acceptor = std::make_unique<boost::asio::ip::tcp::acceptor>(
            ioContext,
            boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), listenPort)
        );
        auto socket = std::make_shared<boost::asio::ip::tcp::socket>(ioContext);
        acceptor->accept(*socket);
        tcpStream = std::make_shared<TcpStream>(socket);
        protocol = std::make_shared<MinBiTCore>(tcpStream);
        protocol->setNodeType(MinBiTCore::NodeType::SERVER);
        protocol->setEndianness(MinBiTCore::Endianness::LittleEndian);
        protocol->setWriteMode(MinBiTCore::WriteMode::IMMEDIATE);
        connected = true;
        ioThread = std::thread([this]() { ioContext.run(); });
        return true;
    }
    catch (const std::exception& e) {
        connected = false;
        return false;
    }
}

void MinBiTTcpServer::setReadHandler(ReadHandler handler) {
    this->readHandler = handler;
}

void MinBiTTcpServer::attachProtocol() {
    protocol->setReadHandler([this](std::shared_ptr<MinBiTCore::Request> request) {
        if (readHandler) {
            readHandler(protocol, request);
        }
        if (connected) {
            protocol->asyncReadByte();
        }
        });
    protocol->asyncReadByte();
}

void MinBiTTcpServer::stop() {
    if (connected) {
        connected = false;
        if (tcpStream && tcpStream->isOpen()) {
            tcpStream->close();
        }
        ioContext.stop();
        if (ioThread.joinable()) {
            ioThread.join();
        }
    }
}

std::shared_ptr<MinBiTCore> MinBiTTcpServer::getCore() {
    return protocol;
}

bool MinBiTTcpServer::isConnected() const {
    return tcpStream && tcpStream->isOpen();
}