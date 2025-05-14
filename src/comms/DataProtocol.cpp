#include "DataProtocol.h"
#include <boost/endian/conversion.hpp>
#include <cstring>
#include <iostream>

using namespace boost;

DataProtocol::DataProtocol(std::shared_ptr<IStream> stream)
    : stream(std::move(stream)) {}

DataProtocol::~DataProtocol() {
    if (stream->isOpen()) {
        stream->close();
    }
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

void DataProtocol::asyncReadBytes(std::size_t length, ReadHandler handler) {
    if (!stream || !stream->isOpen()) return;

    auto tempBuffer = std::make_shared<std::vector<uint8_t>>(length);
    stream->asyncRead(tempBuffer->data(), length,
        [this, tempBuffer, handler](const boost::system::error_code& error, std::size_t bytesTransferred) {
            if (!error) {
                appendToBuffer(tempBuffer->data(), bytesTransferred);
                {
                    std::lock_guard<std::mutex> lock(varsMutex);
                    bufferSize += bytesTransferred;

                    if (endFlag) {
                        endFlag = false;
                        header = readByte();
                    }
                }
                handler(this, error, bytesTransferred);
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
	std::lock_guard<std::mutex> lock(bufferMutex);
	if (readBuffer.size() < len) throw std::runtime_error("Buffer underflow");
	std::memcpy(buffer, readBuffer.data(), len);
	readBuffer.erase(readBuffer.begin(), readBuffer.begin() + len);\
    std::lock_guard<std::mutex> lock(varsMutex);
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

void DataProtocol::clearPacket() {
    std::lock_guard<std::mutex> lock(varsMutex);
    endFlag = true;
    header = 0;
}

bool DataProtocol::isPacketPending() {
    std::lock_guard<std::mutex> lock(bufferMutex);
	return !readBuffer.empty();
}

void DataProtocol::flush(int8_t numBytes) {
    std::lock_guard<std::mutex> lock(bufferMutex);
    if (numBytes < 0) {
        numBytes = readBuffer.size();
    }
	if (numBytes > static_cast<int8_t>(readBuffer.size())) {
		numBytes = readBuffer.size();
	}
	readBuffer.erase(readBuffer.begin(), readBuffer.begin() + numBytes);
    std::lock_guard<std::mutex> lock(varsMutex);
    bufferSize -= numBytes;
}

uint8_t DataProtocol::getHeader() {
    std::lock_guard<std::mutex> lock(varsMutex); // Ensure thread-safe access
    return header;
}

std::size_t DataProtocol::getBufferSize() {
    std::lock_guard<std::mutex> lock(varsMutex); // Ensure thread-safe access
    return bufferSize;
}

void DataProtocol::appendToBuffer(const uint8_t* data, std::size_t length) {
    std::lock_guard<std::mutex> lock(bufferMutex);
    readBuffer.insert(readBuffer.end(), data, data + length);
}