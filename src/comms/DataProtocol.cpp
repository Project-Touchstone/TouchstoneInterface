#include "DataProtocol.h"
#include <boost/endian/conversion.hpp>
#include <cstring>
#include <iostream>

using namespace boost;

DataProtocol::DataProtocol(std::shared_ptr<IStream> stream)
    : stream(std::move(stream)) {}

DataProtocol::~DataProtocol() {
    if (stream && stream->isOpen()) {
        stream->close();
    }
    // No dynamic allocations to clean up, but destructor ensures stream is closed.
}

void DataProtocol::setReadHandler(ReadHandler handler) {
	readHandler = handler;
}

std::shared_ptr<IStream> DataProtocol::getStream() {
    return stream;
}

void DataProtocol::setEndianness(Endianness endianness) {
    currentEndianness = endianness;
}

void DataProtocol::sendBytes(const uint8_t* buffer, std::size_t length) {
    if (!stream || !stream->isOpen()) return;

    stream->asyncWrite(buffer, length,
        [length](const boost::system::error_code& error, std::size_t bytesTransferred) {
            if (error) {
                std::cerr << "Error sending bytes: " << error.message() << std::endl;
            } else if (bytesTransferred < length) {
                std::cerr << "Partial write detected. Ensure all bytes are sent." << std::endl;
            }
        });
}

void DataProtocol::sendByte(uint8_t value) {
    sendBytes(&value, sizeof(value));
}

void DataProtocol::sendFloat(float value) {
    uint32_t networkValue = *reinterpret_cast<uint32_t*>(&value);
    if (currentEndianness == Endianness::BigEndian) {
        networkValue = boost::endian::native_to_big(networkValue);
    } else {
        networkValue = boost::endian::native_to_little(networkValue);
    }
    sendBytes(reinterpret_cast<uint8_t*>(&networkValue), sizeof(networkValue));
}

void DataProtocol::sendInt16(int16_t data) {
    uint8_t buffer[sizeof(data)];
    memcpy(buffer, &data, sizeof(data));
    sendBytes(buffer, sizeof(data));
}

void DataProtocol::sendVector3d(const Eigen::Vector3d& vector) {
	for (int i = 0; i < 3; ++i) {
		sendFloat(static_cast<float>(vector(i)));
	}
}

void DataProtocol::sendQuaterniond(const Eigen::Quaterniond& quaternion) {
	for (int i = 0; i < 4; ++i) {
		sendFloat(static_cast<float>(quaternion.coeffs()(i)));
	}
}

void DataProtocol::asyncReadBytes() {
    if (!stream || !stream->isOpen()) return;

    auto tempBuffer = std::make_shared<std::vector<uint8_t>>(1);
    stream->asyncRead(tempBuffer->data(), 1,
        [this, tempBuffer](const boost::system::error_code& error, std::size_t bytesTransferred) {
            if (!error) {
                appendToBuffer(tempBuffer->data(), bytesTransferred);
                {
                    std::lock_guard<std::mutex> lock(dataMutex);
                    bufferSize += bytesTransferred;
                    if (bufferSize > 256) {
						std::cerr << "Buffer overflow detected. Consider increasing buffer size." << std::endl;
                    }
                }
                if (readHandler) {
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
                }
            } else {
                std::cerr << "Error reading from stream: " << error.message() << std::endl;
            }
        });
}

uint8_t DataProtocol::readByte() {
    uint8_t value;
	readBytes(&value, sizeof(value));
    return value;
}

void DataProtocol::readBytes(uint8_t* buffer, std::size_t len) {
    std::lock_guard<std::mutex> lock(dataMutex);
    if (readBuffer.size() < len) throw std::runtime_error("Buffer underflow");
    std::memcpy(buffer, readBuffer.data(), len);
    readBuffer.erase(readBuffer.begin(), readBuffer.begin() + len);
	bufferSize -= len;
}

float DataProtocol::readFloat() {
    uint8_t buffer[sizeof(float)];
    readBytes(buffer, sizeof(float));
    uint32_t networkValue;
    std::memcpy(&networkValue, buffer, sizeof(uint32_t));
    if (currentEndianness == Endianness::BigEndian) {
        networkValue = boost::endian::big_to_native(networkValue);
    } else {
        networkValue = boost::endian::little_to_native(networkValue);
    }
    return *reinterpret_cast<float*>(&networkValue);
}

Eigen::Vector3d DataProtocol::readVector3d() {
	Eigen::Vector3d vector;
	for (int i = 0; i < 3; ++i) {
		vector(i) = readFloat();
	}
	return vector;
}

Eigen::Quaterniond DataProtocol::readQuaterniond() {
	Eigen::Quaterniond quaternion;
	for (int i = 0; i < 4; ++i) {
		quaternion.coeffs()(i) = readFloat();
	}
	return quaternion;
}

void DataProtocol::clearPacket() {
    std::lock_guard<std::mutex> lock(dataMutex);
    endFlag = true;
    headerFlag = false;
}

bool DataProtocol::isPacketPending() {
    std::lock_guard<std::mutex> lock(dataMutex);
	return (bufferSize > 0) || headerFlag;
}

void DataProtocol::flush() {
    clearPacket();
    std::lock_guard<std::mutex> lock(dataMutex);
    size_t numBytes = readBuffer.size();
    if (numBytes > static_cast<int8_t>(readBuffer.size())) {
        numBytes = readBuffer.size();
    }
    readBuffer.erase(readBuffer.begin(), readBuffer.begin() + numBytes);
    bufferSize -= numBytes;
}

uint8_t DataProtocol::getHeader() {
    std::lock_guard<std::mutex> lock(dataMutex); // Ensure thread-safe access
    return header;
}

std::size_t DataProtocol::getBufferSize() {
    std::lock_guard<std::mutex> lock(dataMutex); // Ensure thread-safe access
    return bufferSize;
}

void DataProtocol::appendToBuffer(const uint8_t* data, std::size_t length) {
    std::lock_guard<std::mutex> lock(dataMutex);
    readBuffer.insert(readBuffer.end(), data, data + length);
}