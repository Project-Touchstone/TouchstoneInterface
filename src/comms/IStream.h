#ifndef ISTREAM_H
#define ISTREAM_H

#include <boost/asio.hpp>
#include <functional>
#include <memory>

class IStream {
    public:
        using StreamHandler = std::function<void(const boost::system::error_code&, std::size_t)>;

        virtual ~IStream() = default;

        virtual void asyncWrite(const uint8_t* buffer, std::size_t length, StreamHandler handler);
        virtual void asyncRead(uint8_t* buffer, std::size_t length, StreamHandler handler);
        virtual bool isOpen() const;
        virtual void close();
};

#endif // ISTREAM_H
