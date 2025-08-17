// GoogleTest unit tests for DataProtocol
#include "gtest/gtest.h"
#include "../comms/DataProtocol.h"
#include <vector>
#include <cstring>
#include <memory>
#include <Eigen/Dense>

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

TEST(DataProtocolTest, SendAndReceiveByte) {
    auto stream = std::make_shared<MockStream>();
    DataProtocol proto(stream);
    uint8_t value = 0x42;
    proto.sendByte(value);
    ASSERT_EQ(stream->writeBuffer.size(), 1);
    EXPECT_EQ(stream->writeBuffer[0], value);
    // Simulate receiving the same byte
    stream->readBuffer.push_back(value);
    // Fills protocol read buffer from stream read buffer
    while (stream->readBuffer.size() > 0) {
        proto.asyncReadByte();
    }
    uint8_t received = proto.readByte();
    EXPECT_EQ(received, value);
}

TEST(DataProtocolTest, SendAndReceiveInt16) {
    auto stream = std::make_shared<MockStream>();
    DataProtocol proto(stream);
    int16_t val = -12345;
    proto.sendInt16(val);
    ASSERT_EQ(stream->writeBuffer.size(), sizeof(int16_t));
    // Simulate receiving the same int16
    for (size_t i = 0; i < sizeof(int16_t); ++i) {
        stream->readBuffer.push_back(stream->writeBuffer[i]);
    }
    while (stream->readBuffer.size() > 0) {
        proto.asyncReadByte();
    }
    int16_t received = proto.readInt16();
    EXPECT_EQ(received, val);
}

TEST(DataProtocolTest, SendAndReceiveFloat) {
    auto stream = std::make_shared<MockStream>();
    DataProtocol proto(stream);
    float f = 3.14159f;
    // Test LittleEndian
    proto.setEndianness(DataProtocol::Endianness::LittleEndian);
    proto.sendFloat(f);
    ASSERT_EQ(stream->writeBuffer.size(), sizeof(float));
    for (size_t i = 0; i < sizeof(float); ++i) {
        stream->readBuffer.push_back(stream->writeBuffer[i]);
    }
    // Fills protocol read buffer from stream read buffer
    while (stream->readBuffer.size() > 0) {
        proto.asyncReadByte();
    }
    float receivedLittle = proto.readFloat();
    EXPECT_FLOAT_EQ(receivedLittle, f);
    // Test BigEndian
    stream->writeBuffer.clear();
    proto.setEndianness(DataProtocol::Endianness::BigEndian);
    proto.sendFloat(f);
    ASSERT_EQ(stream->writeBuffer.size(), sizeof(float));
    stream->readBuffer.clear();
    for (size_t i = 0; i < sizeof(float); ++i) {
        stream->readBuffer.push_back(stream->writeBuffer[i]);
    }
    // Fills protocol read buffer from stream read buffer
    while (stream->readBuffer.size() > 0) {
        proto.asyncReadByte();
    }
    float receivedBig = proto.readFloat();
    EXPECT_FLOAT_EQ(receivedBig, f);
}

TEST(DataProtocolTest, SendAndReceiveVector3d) {
    auto stream = std::make_shared<MockStream>();
    DataProtocol proto(stream);
    Eigen::Vector3d v(1.1, 2.2, 3.3);
    proto.setEndianness(DataProtocol::Endianness::LittleEndian);
    proto.sendVector3d(v);
    ASSERT_EQ(stream->writeBuffer.size(), sizeof(float) * 3);
    // Simulate receiving the same vector
    for (size_t i = 0; i < stream->writeBuffer.size(); ++i) {
        stream->readBuffer.push_back(stream->writeBuffer[i]);
    }
    // Fills protocol read buffer from stream read buffer
    while (stream->readBuffer.size() > 0) {
        proto.asyncReadByte();
    }
    Eigen::Vector3d received = proto.readVector3d();
    EXPECT_NEAR((received - v).norm(), 0, 1e-5);
}

TEST(DataProtocolTest, SendAndReceiveQuaterniond) {
    auto stream = std::make_shared<MockStream>();
    DataProtocol proto(stream);
    Eigen::Quaterniond q(1, 2, 3, 4);
    proto.setEndianness(DataProtocol::Endianness::LittleEndian);
    proto.sendQuaterniond(q);
    ASSERT_EQ(stream->writeBuffer.size(), sizeof(float) * 4);
    // Simulate receiving the same quaternion
    for (size_t i = 0; i < stream->writeBuffer.size(); ++i) {
        stream->readBuffer.push_back(stream->writeBuffer[i]);
    }
    // Fills protocol read buffer from stream read buffer
    while (stream->readBuffer.size() > 0) {
        proto.asyncReadByte();
    }
    Eigen::Quaterniond received = proto.readQuaterniond();
    for (int i = 0; i < 4; ++i) {
        EXPECT_NEAR(received.coeffs()(i), float(q.coeffs()(i)), 1e-5);
    }
}

TEST(DataProtocolTest, PacketSendMode) {
    auto stream = std::make_shared<MockStream>();
    DataProtocol proto(stream);
	// Test PACKET mode
	proto.setSendMode(DataProtocol::SendMode::PACKET);
    proto.sendByte(0xAA);
    proto.sendByte(0xBB);
	// No bytes sent yet, since packet mode buffers data
    EXPECT_EQ(stream->writeBuffer.size(), 0);
    proto.sendPacket();
	// Now the packet should be sent
    EXPECT_EQ(stream->writeBuffer.size(), 2);
}

TEST(DataProtocolTest, PacketReadMode) {
    auto stream = std::make_shared<MockStream>();
    DataProtocol proto(stream);
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
        proto.asyncReadByte();
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
        proto.asyncReadByte();
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
