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
    request = std::make_shared<Request>(7, Request::Type::INCOMING);
    // Checks expected packet length
    EXPECT_TRUE(proto.getExpectedPacketLength(request, length));
    EXPECT_EQ(length, 3);

    // Checks unknown header
    request = std::make_shared<Request>(46, Request::Type::INCOMING);
    // Checks that get expected length fails
    EXPECT_FALSE(proto.getExpectedPacketLength(request, length));
}

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

TEST(MinBiTCoreTest, ClearRequest) {
    auto stream = std::make_shared<MockStream>();
    MinBiTCore proto("Test", stream);
    proto.loadPacketLengthsFromJson("test_packet_lengths.json");
    auto request = proto.writeRequest(1);
    // Simulate receiving a response
    stream->readBuffer.push_back(2); // Random response header
	stream->readBuffer.push_back(0x10); // Dummy payload
	
    proto.asyncFetchByte(); // Process first byte

	EXPECT_TRUE(proto.clearRequest()); // Clear current request
	EXPECT_FALSE(proto.clearRequest()); // No current request to clear
	EXPECT_EQ(proto.getNumOutgoingRequests(), 0); // No outgoing requests should remain
}

TEST(MinBiTCoreTest, FlushBuffer) {
    auto stream = std::make_shared<MockStream>();
    MinBiTCore proto("Test", stream);
    proto.loadPacketLengthsFromJson("test_packet_lengths.json");
    // Simulate receiving some data
    stream->readBuffer.push_back(7);
	stream->readBuffer.push_back(2);
    // Fills protocol read buffer from stream read buffer
    while (stream->readBuffer.size() > 0) {
        proto.asyncFetchByte();
    }
    EXPECT_GT(proto.getReadBufferSize(), 0);
    proto.flush();
    EXPECT_EQ(proto.getReadBufferSize(), 0);
}

TEST(MinBiTCoreTest, FlushRequest) {
    auto stream = std::make_shared<MockStream>();
    MinBiTCore proto("Test", stream);
    proto.loadPacketLengthsFromJson("test_packet_lengths.json");
    auto request = proto.writeRequest(1);
    // Simulate receiving a response
    stream->readBuffer.push_back(2); // Random response header
    stream->readBuffer.push_back(0x10); // Dummy payload
    
    // Fills protocol read buffer from stream read buffer
    while (stream->readBuffer.size() > 0) {
        proto.asyncFetchByte();
    }

    EXPECT_EQ(proto.getNumOutgoingRequests(), 0); // No outgoing requests should remain
	EXPECT_EQ(proto.getReservedBytes(), 1);

	proto.flushRequest();

	EXPECT_EQ(proto.getReservedBytes(), 0);
}

TEST(MinBiTCoreTest, GetPacketParameters) {
    auto stream = std::make_shared<MockStream>();
    MinBiTCore proto("Test", stream);
    proto.loadPacketLengthsFromJson("test_packet_lengths.json");
    
    // Fixed length packet test (header 1)
    int16_t expectedLength = 0;
    EXPECT_TRUE(proto.getExpectedPacketLength(std::make_shared<Request>(1, Request::Type::OUTGOING), expectedLength));
    std::size_t payloadLength = 0, totalPacketLength = 0;
    EXPECT_TRUE(proto.getPacketParameters(expectedLength, payloadLength, totalPacketLength));
    EXPECT_EQ(payloadLength, expectedLength);
    EXPECT_EQ(totalPacketLength, expectedLength + 1);

    // Variable length packet test (header 7)
    expectedLength = -1;
    // Simulate read buffer with header and length byte
    stream->readBuffer.push_back(7); // header
    stream->readBuffer.push_back(5); // length byte
    // Fills protocol read buffer from stream read buffer
    while (stream->readBuffer.size() > 0) {
        proto.asyncFetchByte();
    }
    payloadLength = 0;
    totalPacketLength = 0;
    EXPECT_TRUE(proto.getPacketParameters(expectedLength, payloadLength, totalPacketLength));
    EXPECT_EQ(payloadLength, 5);
    EXPECT_EQ(totalPacketLength, 7);
}

TEST(MinBiTCoreTest, IncomingRequestLifecycle) {
    auto stream = std::make_shared<MockStream>();
    MinBiTCore proto("Test", stream);
    proto.loadPacketLengthsFromJson("test_packet_lengths.json");
    // Simulate receiving a packet for header 7
    stream->readBuffer.push_back(7); // header
    for (int i = 0; i < 3; ++i) {
        stream->readBuffer.push_back(0x10 + i); // dummy payload
    }
    // First read: should create the request
    proto.asyncFetchByte();
    // Find the created request (assuming proto has a method to get current request, or you can check via internal state)
    // For this test, we assume proto exposes a method getCurrentRequest() or similar
    auto currentRequest = proto.getCurrentRequest();
    EXPECT_TRUE(currentRequest != nullptr);
    EXPECT_EQ(currentRequest->GetHeader(), 7);
    EXPECT_TRUE(currentRequest->IsIncoming());
    EXPECT_TRUE(currentRequest->IsCharacterized());
    
    // Read remaining bytes
    while (stream->readBuffer.size() > 0) {
        proto.asyncFetchByte();
    }
    EXPECT_TRUE(currentRequest->IsComplete());
    EXPECT_EQ(proto.getReservedBytes(), 3);
}

TEST(MinBiTCoreTest, OutgoingRequestLifecycle) {
    auto stream = std::make_shared<MockStream>();
    MinBiTCore proto("Test", stream);
    proto.loadPacketLengthsFromJson("test_packet_lengths.json");
    auto request = proto.writeRequest(1);
    
    // Check request creation and header
    EXPECT_TRUE(request != nullptr);
    EXPECT_EQ(request->GetHeader(), 1);
    EXPECT_TRUE(request->IsOutgoing());
    EXPECT_TRUE(request->IsWaiting());
    
    // Simulate receiving response header and payload
    stream->readBuffer.push_back(1); // response header
    for (uint8_t i = 0; i < 2; ++i) {
        stream->readBuffer.push_back(0x30 + i); // dummy payload
	}
    // First read: should update response header
    proto.asyncFetchByte();
    EXPECT_EQ(request->GetResponseHeader(), 1);
    // Read remaining bytes
    while (stream->readBuffer.size() > 0) {
        proto.asyncFetchByte();
    }
    EXPECT_TRUE(request->IsComplete());
    EXPECT_EQ(proto.getReservedBytes(), 2);
}

TEST(MinBiTCoreTest, VariableLengthPacket) {
    auto stream = std::make_shared<MockStream>();
    MinBiTCore proto("Test", stream);
    proto.loadPacketLengthsFromJson("test_packet_lengths.json");
    auto request = proto.writeRequest(6);

    // Check request creation and header
    EXPECT_TRUE(request != nullptr);
    EXPECT_EQ(request->GetHeader(), 6);
    EXPECT_TRUE(request->IsOutgoing());
    EXPECT_TRUE(request->IsWaiting());

    // Simulate receiving response header and payload
    stream->readBuffer.push_back(2); // response header
	stream->readBuffer.push_back(2); // length byte
    for (uint8_t i = 0; i < 2; ++i) {
        stream->readBuffer.push_back(0x30 + i); // dummy payload
    }
    // First read: should update response header
    proto.asyncFetchByte();
	EXPECT_FALSE(request->IsCharacterized());
    EXPECT_EQ(request->GetResponseHeader(), 2);
	// Second read: should read length byte
    proto.asyncFetchByte();
	EXPECT_TRUE(request->IsCharacterized());
	EXPECT_EQ(request->GetPayloadLength(), 2);
    // Read remaining bytes
    while (stream->readBuffer.size() > 0) {
        proto.asyncFetchByte();
    }
    EXPECT_TRUE(request->IsComplete());
    EXPECT_EQ(proto.getReservedBytes(), 2);
}

TEST(MinBiTCoreTest, ReadHandlerTest) {
    auto stream = std::make_shared<MockStream>();
    MinBiTCore proto("Test", stream);
    proto.loadPacketLengthsFromJson("test_packet_lengths.json");
    bool handlerCalled = false;
    proto.setReadHandler([&](std::shared_ptr<Request> req) {
        handlerCalled = true;
        EXPECT_TRUE(req != nullptr);
        });
    auto request = proto.writeRequest(1);
    stream->readBuffer.push_back(2);
    stream->readBuffer.push_back(0xAA);
    // Read remaining bytes
    while (stream->readBuffer.size() > 0) {
        proto.asyncFetchByte();
    }
    EXPECT_TRUE(handlerCalled);
    EXPECT_EQ(proto.getReservedBytes(), 1);
}

TEST(MinBiTCoreTest, AsyncHandlerTest) {
    auto stream = std::make_shared<MockStream>();
    MinBiTCore proto("Test", stream);
    proto.loadPacketLengthsFromJson("test_packet_lengths.json");
    auto request = proto.writeRequest(1);
    auto future = request->WaitAsync();
    stream->readBuffer.push_back(2);
    stream->readBuffer.push_back(0xAA);
    // Fills protocol read buffer from stream read buffer
    while (stream->readBuffer.size() > 0) {
        proto.asyncFetchByte();
    }
    EXPECT_EQ(future.get(), Request::Status::COMPLETE);
    EXPECT_EQ(proto.getReservedBytes(), 1);
}

TEST(MinBiTCoreTest, CombinedHandlersTest) {
    auto stream = std::make_shared<MockStream>();
    MinBiTCore proto("Test", stream);
    proto.loadPacketLengthsFromJson("test_packet_lengths.json");
    bool handlerCalled = false;
    auto request = proto.writeRequest(1);
    auto future = request->WaitAsync();
    proto.setReadHandler([&](std::shared_ptr<Request> req) {
        handlerCalled = true;
        EXPECT_TRUE(req != nullptr);
        });
    proto.sendAll();
    stream->readBuffer.push_back(2);
    stream->readBuffer.push_back(0xAA);
    // Fills protocol read buffer from stream read buffer
    while (stream->readBuffer.size() > 0) {
        proto.asyncFetchByte();
    }
    EXPECT_TRUE(handlerCalled);
    EXPECT_EQ(future.get(), Request::Status::COMPLETE);
    EXPECT_EQ(proto.getReservedBytes(), 1);
}

TEST(MinBiTCoreTest, RequestTimeout) {
    auto stream = std::make_shared<MockStream>();
    MinBiTCore proto("Test", stream);
    proto.loadPacketLengthsFromJson("test_packet_lengths.json");
    proto.setRequestTimeout(1); // 1 ms
    auto request = proto.writeRequest(1);
    proto.sendAll();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    proto.checkForTimeouts();
    EXPECT_TRUE(request->IsTimedOut());
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
    EXPECT_EQ(received, val);
}

TEST(MinBiTCoreTest, WriteAndReceiveFloatBigEndian) {
    auto stream = std::make_shared<MockStream>();
    MinBiTCore proto("Test", stream);
    proto.loadPacketLengthsFromJson("test_packet_lengths.json");
    float f = 3.14159f;
    // Test BigEndian
    proto.setEndianness(MinBiTCore::Endianness::BigEndian);
    RequestPtr request = proto.writeRequest(3);
    auto future = request->WaitAsync();
    proto.writeFloat(f);
    ASSERT_EQ(stream->writeBuffer.size(), sizeof(float) + 1);
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
    EXPECT_FLOAT_EQ(receivedBig, f);
}

TEST(MinBiTCoreTest, WriteAndReceiveFloatLittleEndian) {
    auto stream = std::make_shared<MockStream>();
    MinBiTCore proto("Test", stream);
    proto.loadPacketLengthsFromJson("test_packet_lengths.json");
    float f = 3.14159f;
    // Test LittleEndian
    proto.setEndianness(MinBiTCore::Endianness::LittleEndian);
    RequestPtr request = proto.writeRequest(3);
    auto otherFuture = request->WaitAsync();
    proto.writeFloat(f);
    ASSERT_EQ(stream->writeBuffer.size(), sizeof(float) + 1);
    
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
    for (int i = 0; i < 4; ++i) {
        EXPECT_NEAR(received.coeffs()(i), float(q.coeffs()(i)), 1e-5);
    }
}









