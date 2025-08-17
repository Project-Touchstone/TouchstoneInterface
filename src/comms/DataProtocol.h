#ifndef DATA_PROTOCOL_H
#define DATA_PROTOCOL_H

#include <vector>
#include <mutex>
#include <boost/endian/conversion.hpp>
#include <Eigen/Dense>

#include "IStream.h"

class DataProtocol {
    public:
		// Alias for IStream::StreamHandler
        using ReadHandler = IStream::StreamHandler;

        enum class Endianness {
            BigEndian,
            LittleEndian
        };

        enum class WriteMode {
            IMMEDIATE,
            PACKET
        };

        DataProtocol(std::shared_ptr<IStream> stream);
        ~DataProtocol();

        //Sets read handler
        void setReadHandler(ReadHandler handler);

        // Gets stream object
        std::shared_ptr<IStream> getStream();

        // Set endianness
        void setEndianness(Endianness endianness);

        // Set writing mode
        void setWriteMode(WriteMode mode);

        // Writeing functions
        void writeBytes(const uint8_t* buffer, std::size_t length);
        void writeByte(uint8_t value);
        void writeFloat(float value);
        // Writes a 16 bit integer
        void writeInt16(int16_t data);
        // Writes 3d vector
		void writeVector3d(const Eigen::Vector3d& vector);
		// Writes quaterniond
		void writeQuaterniond(const Eigen::Quaterniond& quaternion);
        // Writes packet
        void writePacket();

        // Receiving functions
        void asyncReadByte();
        uint8_t readByte();
        void readBytes(uint8_t* buffer, std::size_t len);
        float readFloat();
        int16_t readInt16();
        // Reads 3d vector
		Eigen::Vector3d readVector3d();
        // Reads quaterniond
		Eigen::Quaterniond readQuaterniond();

        template <typename T>
        T readData();

        // Packet management
        void clearReadPacket();
        bool isReadPacketPending();
        // Flushes the read buffer
        void flush();
        uint8_t getHeader();
        std::size_t getReadBufferSize();
        std::size_t getWriteBufferSize();

    private:
        std::shared_ptr<IStream> stream;
        std::vector<uint8_t> readBuffer;
        std::vector<uint8_t> writeBuffer;
        std::mutex dataMutex;

        uint8_t header = 0;
        bool headerFlag = false;
        bool endFlag = true;
        Endianness endianness = Endianness::BigEndian; // Default to BigEndian
        WriteMode writeMode = WriteMode::IMMEDIATE;

        //Read handler
		ReadHandler readHandler;

        // Buffer management
        void appendToReadBuffer(const uint8_t* data, std::size_t length);
        void appendToWriteBuffer(const uint8_t* data, std::size_t length);
};

template <typename T>
T DataProtocol::readData() {
    T data;
    uint8_t buffer[sizeof(data)];
    DataProtocol::readBytes(buffer, sizeof(data));

    std::memcpy(&data, buffer, sizeof(data));

    return data;
}

#endif // DATA_PROTOCOL_H