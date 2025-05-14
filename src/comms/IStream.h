#ifndef ISTREAM_H
#define ISTREAM_H

#include <boost/asio.hpp>
#include <functional>
#include <memory>

#include "DataProtocol.h"

class IStream {
public:
    using ReadHandler = std::function<void(const boost::system::error_code&, std::size_t)>;

    virtual ~IStream() = default;

    virtual void asyncWrite(const uint8_t* buffer, std::size_t length, ReadHandler handler) = 0;
    virtual void asyncRead(uint8_t* buffer, std::size_t length, ReadHandler handler) = 0;
    virtual bool isOpen() const = 0;
    virtual void close() = 0;
};

#endif // ISTREAM_H
