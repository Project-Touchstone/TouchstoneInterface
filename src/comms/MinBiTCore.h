#ifndef MINBIT_CORE_H
#define MINBIT_CORE_H

#include <vector>
#include <mutex>
#include <boost/endian/conversion.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>
#include <queue>
#include <nlohmann/json.hpp>

#include "IStream.h"

class MinBiTCore {
    public:
        enum class NodeType {
            SERVER,
            CLIENT
        };

        enum class Endianness {
            BigEndian,
            LittleEndian
        };

        enum class WriteMode {
            IMMEDIATE,
            PACKET
        };

        struct PacketLengthEntry {
            uint8_t header;
            int length;
        };

        class Request {
        public:
            enum class Status {
                UNSENT,
                WAITING,
                FUFILLED,
                TIMEDOUT
            };

            Request(uint8_t header);

            void Start();
            void SetStatus(Status newStatus);
            void SetResponseHeader(uint8_t responseHeader);
            void SetPayloadLength(int responseLength);

            Status GetStatus();
            int64_t GetId() const;
            uint8_t GetHeader() const;
            uint8_t GetResponseHeader();
            int GetResponseLength();
            bool IsWaiting();
            bool IsTimedOut();

            std::chrono::steady_clock::time_point GetSentTime();

            std::future<Status> WaitAsync(int pollIntervalMs = 5);

        private:
            static std::atomic<int64_t> nextId;
            int64_t id;
            uint8_t header;
            uint8_t responseHeader;
            int responseLength;
            Status status;
            std::chrono::steady_clock::time_point sentTime;
            mutable std::mutex requestMutex;
        };

        using ReadHandler = std::function<void(std::shared_ptr<MinBiTCore::Request>)>;

        MinBiTCore(std::string name, std::shared_ptr<IStream> stream);
        ~MinBiTCore();

        //Sets read handler
        void setReadHandler(ReadHandler handler);

        // Gets stream object
        std::shared_ptr<IStream> getStream();

        // Sets node type (server or client)
        void setNodeType(NodeType type);

        // Gets whether node is client or server
        bool isClient() const;
        bool isServer() const;

        // Set endianness
        void setEndianness(Endianness endianness);

        // Set writing mode
        void setWriteMode(WriteMode mode);

        // Set request timeout
        void setRequestTimeout(uint16_t timeoutMs);

        // Loads packet length information from JSON
        bool loadPacketLengthsFromJson(const std::string& json);

        // Gets the expected data length for a given header
        bool getExpectedPacketLength(uint8_t header, std::size_t& length);

        // Writing functions
        std::shared_ptr<MinBiTCore::Request> writeHeader(uint8_t header);
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

        // Reading functions
        void asyncReadByte();
        uint8_t readByte();
        uint8_t peekByte();
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
        // Loads response lengths from a JSON string
        bool loadFromJson(const std::string& json);

        // Gets the expected length for a header, returns false if not found
        bool getExpectedPacketLength(std::shared_ptr<Request> request, std::size_t& length) const;
        bool getPacketParameters(int expectedLength, std::size_t& payloadLength, std::size_t& totalPacketLength);
        bool getCurrentRequest(std::shared_ptr<Request>& request);
        bool isPacketPending();
        // Flushes the read buffer
        void flush();
        bool clearRequest();
        std::size_t getReadBufferSize();
        std::size_t getWriteBufferSize();
        std::size_t getRequestQueueSize();

    private:
        std::string name;
        std::shared_ptr<IStream> stream;
        std::vector<uint8_t> readBuffer;
        std::vector<uint8_t> writeBuffer;
        std::queue<std::shared_ptr<MinBiTCore::Request>> requestQueue;
        uint16_t requestTimeoutMs = 1000; // or make this configurable
        std::mutex dataMutex;

        bool packetFlag = false;
        // Packet lengths by request header
        std::unordered_map<uint8_t, std::size_t> lengthsByRequest;
        // Packet lengths by response header
        std::unordered_map<uint8_t, std::size_t> lengthsByResponse;

        NodeType nodeType = NodeType::CLIENT; // Default to CLIENT
        Endianness endianness = Endianness::BigEndian; // Default to BigEndian
        WriteMode writeMode = WriteMode::IMMEDIATE;

        //Read handler
		ReadHandler readHandler;

        // Buffer management
        void appendToReadBuffer(const uint8_t* data, std::size_t length);
        void appendToWriteBuffer(const uint8_t* data, std::size_t length);

        // Processing loop
        void checkForTimeouts();
        bool characterizePacket(std::shared_ptr<MinBiTCore::Request>& request, bool& variableLength, std::size_t payloadLength);
};

template <typename T>
T MinBiTCore::readData() {
    T data;
    uint8_t buffer[sizeof(data)];
    MinBiTCore::readBytes(buffer, sizeof(data));

    std::memcpy(&data, buffer, sizeof(data));

    return data;
}

#endif // MINBIT_CORE_H