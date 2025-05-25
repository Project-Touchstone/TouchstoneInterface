#ifndef DATA_PROTOCOL_H
#define DATA_PROTOCOL_H

#include <vector>
#include <mutex>
#include <boost/endian/conversion.hpp>
#include <Eigen/Dense>

#include "IStream.h"

class DataProtocol {
    public:
        using ReadHandler = std::function<void(const boost::system::error_code&, std::size_t)>;

        enum class Endianness {
            BigEndian,
            LittleEndian
        };

        enum class SendMode {
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

        // Set sending mode
        void setSendMode(SendMode mode);

        // Sending functions
        void sendBytes(const uint8_t* buffer, std::size_t length);
        void sendByte(uint8_t value);
        void sendFloat(float value);
        // Sends a 16 bit integer
        void sendInt16(int16_t data);
        // Sends 3d vector
		void sendVector3d(const Eigen::Vector3d& vector);
		// Sends quaterniond
		void sendQuaterniond(const Eigen::Quaterniond& quaternion);
        // Sends packet
        void sendPacket();

        // Receiving functions
        void asyncReadBytes();
        uint8_t readByte();
        void readBytes(uint8_t* buffer, std::size_t len);
        float readFloat();
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
        std::size_t getSendBufferSize();

    private:
        std::shared_ptr<IStream> stream;
        std::vector<uint8_t> readBuffer;
        std::vector<uint8_t> sendBuffer;
        std::mutex dataMutex;

        uint8_t header = 0;
        bool headerFlag = false;
        bool endFlag = true;
        Endianness endianness = Endianness::BigEndian; // Default to BigEndian
        SendMode sendMode = SendMode::IMMEDIATE;

        //Read handler
		ReadHandler readHandler;

        // Buffer management
        void appendToReadBuffer(const uint8_t* data, std::size_t length);
        void appendToSendBuffer(const uint8_t* data, std::size_t length);
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