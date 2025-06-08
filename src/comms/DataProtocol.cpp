#include "DataProtocol.h"
#include <boost/endian/conversion.hpp>
#include <cstring>
#include <iostream>

using namespace boost;

// DataProtocol handles serialization and communication of data packets over an IStream.
DataProtocol::DataProtocol(std::shared_ptr<IStream> stream)
    : stream(std::move(stream)) {}

DataProtocol::~DataProtocol() {
    // Ensure the stream is closed on destruction.
    if (stream && stream->isOpen()) {
        stream->close();
    }
    // No dynamic allocations to clean up, but destructor ensures stream is closed.
}

void DataProtocol::setReadHandler(ReadHandler handler) {
    // Set the callback to be invoked when data is read.
    readHandler = handler;
}

std::shared_ptr<IStream> DataProtocol::getStream() {
    // Return the underlying stream.
    return stream;
}

void DataProtocol::setEndianness(Endianness endianness) {
    // Set the byte order for serialization.
    this->endianness = endianness;
}

void DataProtocol::setSendMode(SendMode mode) {
    // Set the mode for sending packets (immediate or buffered).
    this->sendMode = mode;
}

void DataProtocol::sendBytes(const uint8_t* buffer, std::size_t length) {
    // Appends data to send buffer
    appendToSendBuffer(buffer, length);

    // Sends packet immediately if in immediate mode
    if (sendMode == SendMode::IMMEDIATE) {
        sendPacket();
    }
}

void DataProtocol::sendByte(uint8_t value) {
    // Send a single byte.
    sendBytes(&value, sizeof(value));
}

void DataProtocol::sendFloat(float value) {
    // Serialize a float with correct endianness and send.
    uint32_t networkValue = *reinterpret_cast<uint32_t*>(&value);
    if (endianness == Endianness::BigEndian) {
        networkValue = boost::endian::native_to_big(networkValue);
    } else {
        networkValue = boost::endian::native_to_little(networkValue);
    }
    sendBytes(reinterpret_cast<uint8_t*>(&networkValue), sizeof(networkValue));
}

void DataProtocol::sendInt16(int16_t data) {
    // Serialize and send a 16-bit integer.
    uint8_t buffer[sizeof(data)];
    memcpy(buffer, &data, sizeof(data));
    sendBytes(buffer, sizeof(data));
}

void DataProtocol::sendVector3d(const Eigen::Vector3d& vector) {
    // Send a 3D vector as three floats.
    for (int i = 0; i < 3; ++i) {
        sendFloat(static_cast<float>(vector(i)));
    }
}

void DataProtocol::sendQuaterniond(const Eigen::Quaterniond& quaternion) {
    // Send a quaternion as four floats (coefficients order).
    for (int i = 0; i < 4; ++i) {
        sendFloat(static_cast<float>(quaternion.coeffs()(i)));
    }
}

void DataProtocol::sendPacket() {
    // Send the contents of the send buffer as a packet.
    std::lock_guard<std::mutex> lock(dataMutex);
    if (!stream || !stream->isOpen()) return;

    size_t trueBufferSize = sendBuffer.size();
    stream->asyncWrite(sendBuffer.data(), trueBufferSize,
        [trueBufferSize](const boost::system::error_code& error, std::size_t bytesTransferred) {
            if (error) {
                std::cerr << "Error sending bytes: " << error.message() << std::endl;
            }
            else if (bytesTransferred < trueBufferSize) {
                std::cerr << "Partial write detected. Ensure all bytes are sent." << std::endl;
            }
        });

    // Clears send buffer
    sendBuffer.clear();
}

void DataProtocol::asyncReadBytes() {
    // Begin asynchronous read of bytes from the stream.
    if (!stream || !stream->isOpen()) return;

    auto tempBuffer = std::make_shared<std::vector<uint8_t>>(1);
    stream->asyncRead(tempBuffer->data(), 1,
        [this, tempBuffer](const boost::system::error_code& error, std::size_t bytesTransferred) {
            if (!error) {
                appendToReadBuffer(tempBuffer->data(), bytesTransferred);
                if (readHandler) {
                    do {
                        if (endFlag) {
                            endFlag = false;
                            uint8_t newHeader = readByte();
                            {
                                std::lock_guard<std::mutex> lock(dataMutex);
                                header = newHeader;
                                headerFlag = true;
                            }
                        }
                        readHandler(error, bytesTransferred);
                    } while (endFlag && getReadBufferSize() > 0);
                }
            } else {
                std::cerr << "Error reading from stream: " << error.message() << std::endl;
            }
        });
}

uint8_t DataProtocol::readByte() {
    // Read a single byte from the read buffer.
    uint8_t value;
    readBytes(&value, sizeof(value));
    return value;
}

void DataProtocol::readBytes(uint8_t* buffer, std::size_t len) {
    // Read a sequence of bytes from the read buffer.
    std::lock_guard<std::mutex> lock(dataMutex);
    if (readBuffer.size() < len) throw std::runtime_error("Buffer underflow");
    std::memcpy(buffer, readBuffer.data(), len);
    readBuffer.erase(readBuffer.begin(), readBuffer.begin() + len);
}

float DataProtocol::readFloat() {
    // Read a float from the read buffer, handling endianness.
    uint8_t buffer[sizeof(float)];
    readBytes(buffer, sizeof(float));
    uint32_t networkValue;
    std::memcpy(&networkValue, buffer, sizeof(uint32_t));
    if (endianness == Endianness::BigEndian) {
        networkValue = boost::endian::big_to_native(networkValue);
    } else {
        networkValue = boost::endian::little_to_native(networkValue);
    }
    return *reinterpret_cast<float*>(&networkValue);
}

Eigen::Vector3d DataProtocol::readVector3d() {
    // Read a 3D vector (three floats) from the read buffer.
    Eigen::Vector3d vector;
    for (int i = 0; i < 3; ++i) {
        vector(i) = readFloat();
    }
    return vector;
}

Eigen::Quaterniond DataProtocol::readQuaterniond() {
    // Read a quaternion (four floats) from the read buffer.
    Eigen::Vector<double, 4> coeffs;
    for (int i = 0; i < 4; ++i) {
        coeffs(i) = readFloat();
    }
    return Eigen::Quaterniond(coeffs[3], coeffs[0], coeffs[1], coeffs[2]);
}

void DataProtocol::clearReadPacket() {
    // Reset flags to start a new packet.
    endFlag = true;
    headerFlag = false;
}

bool DataProtocol::isReadPacketPending() {
    // Check if there is a pending packet to be read.
    return (getReadBufferSize() > 0) || headerFlag;
}

void DataProtocol::flush() {
    // Clear the read buffer and reset packet state.
    clearReadPacket();
    std::lock_guard<std::mutex> lock(dataMutex);
    readBuffer.clear();
}

uint8_t DataProtocol::getHeader() {
    // Get the current packet header (thread-safe).
    std::lock_guard<std::mutex> lock(dataMutex); // Ensure thread-safe access
    return header;
}

std::size_t DataProtocol::getReadBufferSize() {
    // Get the size of the read buffer (thread-safe).
    std::lock_guard<std::mutex> lock(dataMutex); // Ensure thread-safe access
    return readBuffer.size();
}

std::size_t DataProtocol::getSendBufferSize() {
    // Get the size of the send buffer (thread-safe).
    std::lock_guard<std::mutex> lock(dataMutex); // Ensure thread-safe access
    return sendBuffer.size();
}

void DataProtocol::appendToReadBuffer(const uint8_t* data, std::size_t length) {
    // Append data to the read buffer (thread-safe).
    std::lock_guard<std::mutex> lock(dataMutex);
    readBuffer.insert(readBuffer.end(), data, data + length);
}

void DataProtocol::appendToSendBuffer(const uint8_t* data, std::size_t length) {
    // Append data to the send buffer (thread-safe).
    std::lock_guard<std::mutex> lock(dataMutex);
    sendBuffer.insert(sendBuffer.end(), data, data + length);
}