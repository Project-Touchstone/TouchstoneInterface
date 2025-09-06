// GoogleTest unit tests for MinBiTCore
#include "gtest/gtest.h"
#include "../comms/IStream.h"
#include "../comms/MinBiTCore.h"
#include <vector>
#include <cstring>
#include <thread>
#include <memory>
#include <Eigen/Dense>

using Request = MinBiTCore::Request;
using RequestPtr = std::shared_ptr<MinBiTCore::Request>;

// Mock IStream for testing
class MockStream : public IStream {
public:
    std::vector<uint8_t> writeBuffer;
    std::vector<uint8_t> readBuffer;
    bool open = true;

    void asyncWrite(const uint8_t* buffer, std::size_t length, StreamHandler handler) override {
        writeBuffer.insert(writeBuffer.end(), buffer, buffer + length);
        // Simulate successful write
        handler(boost::system::error_code(), length);
    }
    void asyncRead(uint8_t* buffer, std::size_t length, StreamHandler handler) override {
        std::size_t toRead = std::min(length, readBuffer.size());
        std::memcpy(buffer, readBuffer.data(), toRead);
        readBuffer.erase(readBuffer.begin(), readBuffer.begin() + toRead);
        handler(boost::system::error_code(), toRead);
    }
    bool isOpen() const override { return open; }
    void close() override { open = false; }
};

TEST(MinBiTCoreTest, ImmediateWriteMode) {
    auto stream = std::make_shared<MockStream>();
    MinBiTCore proto("Test", stream);
    // Test IMMEDIATE mode
    proto.setWriteMode(MinBiTCore::WriteMode::IMMEDIATE);
    proto.writeByte(0xAA);
    proto.writeByte(0xBB);
    // The bytes should already be sent
    EXPECT_EQ(stream->writeBuffer.size(), 2);
}

TEST(MinBiTCoreTest, BulkWriteMode) {
    auto stream = std::make_shared<MockStream>();
    MinBiTCore proto("Test", stream);
    // Test BULK mode
    proto.setWriteMode(MinBiTCore::WriteMode::BULK);
    proto.writeByte(0xAA);
    proto.writeByte(0xBB);
    // No bytes sent yet, since packet mode buffers data
    EXPECT_EQ(stream->writeBuffer.size(), 0);
    proto.sendAll();
    // Now the packet should be sent
    EXPECT_EQ(stream->writeBuffer.size(), 2);
}

TEST(MinBiTCoreTest, ParsePacketLengths) {
    auto stream = std::make_shared<MockStream>();
    MinBiTCore proto("Test", stream);
    proto.loadPacketLengthsFromJson("test_packet_lengths.json");
    RequestPtr request;
    int16_t length = 0;

    // Checks outgoing by response
    request = std::make_shared<Request>(1, Request::Type::OUTGOING);
    request->SetResponseHeader(1);
    // Checks expected packet length
    EXPECT_TRUE(proto.getExpectedPacketLength(request, length));
    EXPECT_EQ(length, 2);

    // Checks outgoing by request
    request = std::make_shared<Request>(1, Request::Type::OUTGOING);
    request->SetResponseHeader(2);
    // Checks expected packet length
    EXPECT_TRUE(proto.getExpectedPacketLength(request, length));
    EXPECT_EQ(length, 1);

    // Checks incoming by request
    request = std::make_shared<Request>(6, Request::Type::INCOMING);
    // Checks expected packet length
    EXPECT_TRUE(proto.getExpectedPacketLength(request, length));
    EXPECT_EQ(length, 3);

    // Checks unknown header
    request = std::make_shared<Request>(46, Request::Type::INCOMING);
    // Checks that get expected length fails
    EXPECT_FALSE(proto.getExpectedPacketLength(request, length));
}

TEST(MinBiTCoreTest, WriteAndReceiveByte) {
    auto stream = std::make_shared<MockStream>();
    MinBiTCore proto("Test", stream);
    proto.loadPacketLengthsFromJson("test_packet_lengths.json");
    uint8_t value = 0x42;
    RequestPtr request = proto.writeRequest(1);
    // Ensures handle is created
    auto future = request->WaitAsync();
    proto.writeByte(value);
    ASSERT_EQ(stream->writeBuffer.size(), 2);
    ASSERT_EQ(stream->writeBuffer[1], value);
    // Simulate receiving the same byte
    stream->readBuffer.push_back(2); // Random response header
    stream->readBuffer.push_back(value);
    // Fills protocol read buffer from stream read buffer
    while (stream->readBuffer.size() > 0) {
        proto.asyncFetchByte();
    }
    future.get();
    uint8_t received = proto.readByte();
    proto.clearRequest();
    EXPECT_EQ(received, value);
}

TEST(MinBiTCoreTest, WriteAndReceiveInt16) {
    auto stream = std::make_shared<MockStream>();
    MinBiTCore proto("Test", stream);
    proto.loadPacketLengthsFromJson("test_packet_lengths.json");
    int16_t val = -12345;
    RequestPtr request = proto.writeRequest(2);
    auto future = request->WaitAsync();
    proto.writeInt16(val);
    ASSERT_EQ(stream->writeBuffer.size(), sizeof(int16_t) + 1);
    // Simulate receiving the same int16
    stream->readBuffer.push_back(2); // Random response header
    for (size_t i = 1; i < sizeof(int16_t) + 1; ++i) {
        stream->readBuffer.push_back(stream->writeBuffer[i]);
    }
    while (stream->readBuffer.size() > 0) {
        proto.asyncFetchByte();
    }
    future.get();
    int16_t received = proto.readInt16();
    proto.clearRequest();
    EXPECT_EQ(received, val);
}

TEST(MinBiTCoreTest, WriteAndReceiveFloat) {
    auto stream = std::make_shared<MockStream>();
    MinBiTCore proto("Test", stream);
    proto.loadPacketLengthsFromJson("test_packet_lengths.json");
    float f = 3.14159f;
    // Test BigEndian
    stream->writeBuffer.clear();
    proto.setEndianness(MinBiTCore::Endianness::BigEndian);
    RequestPtr request = proto.writeRequest(3);
    auto future = request->WaitAsync();
    proto.writeFloat(f);
    ASSERT_EQ(stream->writeBuffer.size(), sizeof(float) + 1);
    stream->readBuffer.clear();
    // Simulate receiving the same float
    stream->readBuffer.push_back(2); // Random response header
    for (size_t i = 1; i < sizeof(float) + 1; ++i) {
        stream->readBuffer.push_back(stream->writeBuffer[i]);
    }
    // Fills protocol read buffer from stream read buffer
    while (stream->readBuffer.size() > 0) {
        proto.asyncFetchByte();
    }
    future.get();
    float receivedBig = proto.readFloat();
    proto.clearRequest();
    EXPECT_FLOAT_EQ(receivedBig, f);
    // Test LittleEndian
    stream->writeBuffer.clear();
    proto.setEndianness(MinBiTCore::Endianness::LittleEndian);
    request = proto.writeRequest(3);
    auto otherFuture = request->WaitAsync();
    proto.writeFloat(f);
    ASSERT_EQ(stream->writeBuffer.size(), sizeof(float) + 1);
    stream->readBuffer.clear();
    // Simulate receiving the same float
    stream->readBuffer.push_back(2); // Random response header
    for (size_t i = 1; i < sizeof(float) + 1; ++i) {
        stream->readBuffer.push_back(stream->writeBuffer[i]);
    }
    // Fills protocol read buffer from stream read buffer
    while (stream->readBuffer.size() > 0) {
        proto.asyncFetchByte();
    }
    otherFuture.get();
    float receivedLittle = proto.readFloat();
    proto.clearRequest();
    EXPECT_FLOAT_EQ(receivedLittle, f);
}

TEST(MinBiTCoreTest, WriteAndReceiveVector3d) {
    auto stream = std::make_shared<MockStream>();
    MinBiTCore proto("Test", stream);
    proto.loadPacketLengthsFromJson("test_packet_lengths.json");
    Eigen::Vector3d v(1.1, 2.2, 3.3);
    proto.setEndianness(MinBiTCore::Endianness::LittleEndian);
    RequestPtr request = proto.writeRequest(4);
    auto future = request->WaitAsync();
    proto.writeVector3d(v);
    ASSERT_EQ(stream->writeBuffer.size(), sizeof(float) * 3 + 1);
    // Simulate receiving the same vector
    stream->readBuffer.push_back(2); // Random response header
    for (size_t i = 1; i < stream->writeBuffer.size(); ++i) {
        stream->readBuffer.push_back(stream->writeBuffer[i]);
    }
    // Fills protocol read buffer from stream read buffer
    while (stream->readBuffer.size() > 0) {
        proto.asyncFetchByte();
    }
    future.get();
    Eigen::Vector3d received = proto.readVector3d();
    proto.clearRequest();
    EXPECT_NEAR((received - v).norm(), 0, 1e-5);
}

TEST(MinBiTCoreTest, WriteAndReceiveQuaterniond) {
    auto stream = std::make_shared<MockStream>();
    MinBiTCore proto("Test", stream);
    proto.loadPacketLengthsFromJson("test_packet_lengths.json");
    Eigen::Quaterniond q(1, 2, 3, 4);
    proto.setEndianness(MinBiTCore::Endianness::LittleEndian);
    RequestPtr request = proto.writeRequest(5);
    auto future = request->WaitAsync();
    proto.writeQuaterniond(q);
    ASSERT_EQ(stream->writeBuffer.size(), sizeof(float) * 4 + 1);
    // Simulate receiving the same quaternion
    stream->readBuffer.push_back(2); // Random response header
    for (size_t i = 1; i < stream->writeBuffer.size(); ++i) {
        stream->readBuffer.push_back(stream->writeBuffer[i]);
    }
    // Fills protocol read buffer from stream read buffer
    while (stream->readBuffer.size() > 0) {
        proto.asyncFetchByte();
    }
    future.get();
    Eigen::Quaterniond received = proto.readQuaterniond();
    proto.clearRequest();
    for (int i = 0; i < 4; ++i) {
        EXPECT_NEAR(received.coeffs()(i), float(q.coeffs()(i)), 1e-5);
    }
}


/*
TEST(MinBiTCoreTest, GetPacketParameters) {

}

TEST(MinBiTCoreTest, RequestCreation) {

}

TEST(MinBiTCoreTest, RequestTimeout) {

}

TEST(MinBiTCoreTest, AsyncRequestResponse) {

}*/

/*
TEST(MinBiTCoreTest, PacketReadMode) {
    auto stream = std::make_shared<MockStream>();
    MinBiTCore proto("Test", stream);
    // Set up a mock packet: header byte 0xAB, followed by 3 bytes of data 0x01, 0x02, 0x03
    std::vector<uint8_t> packet = {0xAB, 0x01, 0x02, 0x03};
    for (auto b : packet) stream->readBuffer.push_back(b);
    // Set up a handler to process the packet
    bool handlerCalled = false;
    std::vector<uint8_t> receivedData;
    proto.setReadHandler([&](const boost::system::error_code&, std::size_t) {
        handlerCalled = true;
        // After header is processed, the rest should be in the read buffer
        while (proto.getReadBufferSize() > 0) {
            receivedData.push_back(proto.readByte());
        }
    });
    // Fills protocol read buffer from stream read buffer
    while (stream->readBuffer.size() > 0) {
        proto.asyncFetchByte();
    }
    // Handler should have been called
    EXPECT_TRUE(handlerCalled);
    // Header should be set
    EXPECT_EQ(proto.getHeader(), 0xAB);
    // Data should match
    ASSERT_EQ(receivedData.size(), 3u);
    EXPECT_EQ(receivedData[0], 0x01);
    EXPECT_EQ(receivedData[1], 0x02);
    EXPECT_EQ(receivedData[2], 0x03);

    // isReadPacketPending should still be true
    EXPECT_TRUE(proto.isReadPacketPending());

    // Test clearReadPacket resets headerFlag and endFlag
    proto.clearReadPacket();
    EXPECT_FALSE(proto.isReadPacketPending());

    // Add another packet to buffer
    std::vector<uint8_t> extra = { 0x55, 0xAA};
    for (auto b : extra) stream->readBuffer.push_back(b);

    proto.setReadHandler([&](const boost::system::error_code&, std::size_t) {
        //This handler does nothing
    });

    // Fills protocol read buffer from stream read buffer
    while (stream->readBuffer.size() > 0) {
        proto.asyncFetchByte();
    }

	// Check that the extra packet was correctly processed
	EXPECT_EQ(proto.getHeader(), 0x55);
	EXPECT_EQ(proto.getReadBufferSize(), 1u); // One byte left in buffer

	// Tests that clear packet does not clear buffer
	proto.clearReadPacket();
    EXPECT_TRUE(proto.isReadPacketPending());
	EXPECT_EQ(proto.getReadBufferSize(), 1u);

    // Test flush clears buffer and resets state
    proto.flush();
    EXPECT_FALSE(proto.isReadPacketPending());
	EXPECT_EQ(proto.getReadBufferSize(), 0u);
}
*/
