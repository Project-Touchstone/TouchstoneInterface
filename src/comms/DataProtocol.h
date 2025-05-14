#ifndef DATA_PROTOCOL_H
#define DATA_PROTOCOL_H

#include <vector>
#include <mutex>
#include <boost/endian/conversion.hpp>

#include "IStream.h"

class DataProtocol {
    public:
        using ReadHandler = std::function<void(DataProtocol*, const boost::system::error_code&, std::size_t)>;

        enum class Endianness {
            BigEndian,
            LittleEndian
        };

        DataProtocol(std::shared_ptr<IStream> stream);
        ~DataProtocol();

        //Sets read handler
        void setReadHandler(ReadHandler handler);

        // Gets stream object
        std::shared_ptr<IStream> getStream();

        // Set endianness
        void setEndianness(Endianness endianness);

        // Sending functions
        void sendBytes(const uint8_t* buffer, std::size_t length);
        void sendByte(uint8_t value);
        void sendFloat(float value);
        // Sends a 16 bit integer
        void sendInt16(int16_t data);

        // Receiving functions
        void asyncReadBytes(std::size_t length);
        uint8_t readByte();
        void readBytes(uint8_t* buffer, std::size_t len);
        float readFloat();

        template <typename T>
        T readData();

        // Packet management
        void clearPacket();
        bool isPacketPending();
        // Flushes the read buffer
        void flush();
        uint8_t getHeader();
        std::size_t getBufferSize();

    private:
        std::shared_ptr<IStream> stream;
        std::vector<uint8_t> readBuffer;
        std::mutex dataMutex;

        uint8_t header = 0;
        std::size_t bufferSize = 0;
        bool headerFlag = false;
        bool endFlag = true;
        Endianness currentEndianness = Endianness::BigEndian; // Default to BigEndian

        //Read handler
		ReadHandler readHandler;

        // Buffer management
        void appendToBuffer(const uint8_t* data, std::size_t length);
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