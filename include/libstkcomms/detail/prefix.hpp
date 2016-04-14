#ifndef LIBSTKCOMMS_DETAIL_PREFIX_HPP
#define LIBSTKCOMMS_DETAIL_PREFIX_HPP

#include <util/asio/composed.hpp>

#include <boost/asio/buffer.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/write.hpp>
#include <boost/asio/streambuf.hpp>

#include <boost/endian/arithmetic.hpp>

#include <boost/regex.hpp>

#include <chrono>
#include <string>
#include <utility>

namespace stk {
namespace detail {
namespace {
    using util::composed::makeOperation;
    using util::composed::Operation;

    using std::move;

    using boost::regex;
    using boost::system::error_code;
    using boost::endian::little_uint16_t;
    using boost::endian::big_uint16_t;

    namespace asio = boost::asio;
    using asio::buffer;
    using asio::const_buffer;
    using asio::async_read_until;
    using asio::async_write;
    using StreamBufIter = asio::buffers_iterator<asio::streambuf::const_buffers_type>;

    const std::chrono::seconds kStartDelay { 2 }; // time between setting baud rate and syncing
    const std::chrono::milliseconds kRetryTimeout { 500 }; // time between sync messages
    const unsigned kSyncMaxAttempts { 3 };
    using UString = std::basic_string<uint8_t>;
    const UString kMobotILSignature { 0x1e, 0xa7, 0x01 };
    const UString kMobotASignature { 0x1e, 0x95, 0x0f };
} // <anonymous>
} // detail
} // stk

#endif
