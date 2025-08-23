#include "MinBiTSerialClient.h"

MinBiTSerialClient::MinBiTSerialClient(std::string name)
    : serialStream(std::make_shared<SerialStream>(std::make_shared<boost::asio::serial_port>(ioContext))),
    protocol(std::make_shared<MinBiTCore>(name, serialStream))
{
    protocol->setNodeType(MinBiTCore::NodeType::CLIENT);
    protocol->setEndianness(MinBiTCore::Endianness::LittleEndian);
    protocol->setWriteMode(MinBiTCore::WriteMode::PACKET);
    protocol->setRequestTimeout(1000);
}

MinBiTSerialClient::~MinBiTSerialClient() {
    end();
}

bool MinBiTSerialClient::begin(const std::string& port, unsigned int baudRate) {
    try {
        auto serialPort = serialStream->getSerialPort();
        serialPort->open(port);
        serialPort->set_option(boost::asio::serial_port_base::baud_rate(baudRate));
        serialPort->set_option(boost::asio::serial_port_base::character_size(8));
        serialPort->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serialPort->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serialPort->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        running = true;
        ioThread = std::thread([this]() { ioContext.run(); });
        attachProtocol();
        return true;
    }
    catch (boost::system::system_error& e) {
        std::cerr << "Error opening serial port: " << e.what() << std::endl;
        running = false;
        return false;
    }
}

void MinBiTSerialClient::setReadHandler(ReadHandler readHander) {
    this->readHandler = readHandler;
}

void MinBiTSerialClient::attachProtocol() {
    protocol->setReadHandler([this](std::shared_ptr<MinBiTCore::Request> request) {
        if (readHandler) {
            readHandler(protocol, request);
        }
        if (running) {
            protocol->asyncReadByte();
        }
    });
    protocol->asyncReadByte();
}

void MinBiTSerialClient::end() {
    if (running) {
        running = false;
        if (serialStream && serialStream->isOpen()) {
            serialStream->close();
        }
        ioContext.stop();
        if (ioThread.joinable()) {
            ioThread.join();
        }
    }
}

std::shared_ptr<MinBiTCore> MinBiTSerialClient::getProtocol() {
    return protocol;
}

bool MinBiTSerialClient::isOpen() const {
    return serialStream && serialStream->isOpen();
}