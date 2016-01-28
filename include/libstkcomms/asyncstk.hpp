#ifndef LIBSTKCOMMS_ASYNCSTK_HPP
#define LIBSTKCOMMS_ASYNCSTK_HPP

#include <libstkcomms/fullregexmatch.hpp>
#include <libstkcomms/messages.hpp>
#include <libstkcomms/system_error.hpp>

#include <util/composed.hpp>

#include <boost/asio/async_result.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/write.hpp>

#include <boost/endian/arithmetic.hpp>

#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

#include <boost/regex.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <iomanip>
#include <iterator>
#include <limits>
#include <sstream>
#include <tuple>
#include <utility>

#include <boost/asio/yield.hpp>

namespace stk {
namespace {
    using util::composed::makeOperation;
    using util::composed::Operation;

    using std::move;
    using boost::regex;
    using boost::system::error_code;

    namespace asio = boost::asio;
    using asio::buffer;
    using asio::async_read_until;
    using asio::async_write;

    const std::chrono::milliseconds kSyncTimeout { 200 };
    const unsigned kSyncMaxAttempts { 10 };
    using UString = std::basic_string<uint8_t>;
    const UString kMobotILSignature { 0x1e, 0xa7, 0x01 };
    const UString kMobotASignature { 0x1e, 0x95, 0x0f };
} // <anonymous>

namespace detail {
template <class Iter>
std::string toHexString (Iter b, Iter e) {
    auto ss = std::ostringstream();
    ss << std::hex << std::setw(2);
    if (b != e) { ss << static_cast<int>(*b++); }
    for (; b != e; ++b) {
        ss << " " << static_cast<int>(*b);
    }
    return ss.str();
}
} // detail

namespace operations {

template <class BufIter>
Status handleSyncReply (size_t n, asio::streambuf& buf,
    const boost::match_results<BufIter>& what)
{
    boost::log::sources::logger lg;

    auto b = BufIter::begin(buf.data());
    auto e = BufIter::end(buf.data());

    auto hexdump = detail::toHexString(b, e);
    BOOST_LOG(lg) << "Received " << n << " bytes: " << hexdump;

    if (what[0].matched) {
        buf.consume(n);
        return Status::OK;
    }
    return Status::PROTOCOL_ERROR;
}

template <class BufIter>
Status handleReadSignReply (size_t n, asio::streambuf& buf,
    const boost::match_results<BufIter>& what, uint16_t& pageSize)
{
    boost::log::sources::logger lg;

    auto b = BufIter::begin(buf.data());
    auto e = BufIter::end(buf.data());

    auto hexdump = detail::toHexString(b, e);
    BOOST_LOG(lg) << "Received " << n << " bytes: " << hexdump;

    if (what[0].matched) {
        auto& sigMatch = what[1];
        auto sigMatchStr = sigMatch.str();
        auto signature = UString(sigMatchStr.begin(), sigMatchStr.end());
        if (!signature.compare(kMobotILSignature)) {
            pageSize = 256;
        }
        else if (!signature.compare(kMobotASignature)) {
            pageSize = 128;
        }
        else {
            auto sigHexdump = detail::toHexString(sigMatch.first, sigMatch.second);
            return Status::UNKNOWN_SIGNATURE;
        }
        buf.consume(n);
        return Status::OK;
    }
    return Status::PROTOCOL_ERROR;
}

struct ProgramAll {
    ProgramAll (asio::serial_port& sp,
        uint32_t flashBase, asio::const_buffer flash,
        uint32_t eepromBase, asio::const_buffer eeprom)
        : sp_(sp)
        , timer_(sp.get_io_service())
        , flashBase_(flashBase)
        , flash_(flash)
        , eepromBase_(eepromBase)
        , eeprom_(eeprom)
    {}

    asio::serial_port& sp_;
    asio::streambuf buf_;
    using BufIter = asio::buffers_iterator<decltype(buf_)::const_buffers_type>;
    boost::match_results<BufIter> what_;

    asio::steady_timer timer_;
    int syncAttempts_ = 0;

    uint32_t flashBase_;
    asio::const_buffer flash_;
    uint32_t eepromBase_;
    asio::const_buffer eeprom_;
    uint16_t pageSize_ = 0;

    // These two values get sent over the wire in differing endianness (wtf),
    // so it's most maintainable to just store them as such.
    boost::endian::big_uint16_t txPageSize_;
    boost::endian::little_uint16_t txAddress_;

    boost::log::sources::logger log_;
    error_code rc_;

    std::tuple<error_code> result () {
        return std::make_tuple(rc_);
    }

    template <class Op>
    void operator() (Op& op, error_code ec = {}, size_t n = 0) {
        if (!ec) reenter (op) {
            if (blobTooBig()) {
                rc_ = Status::BLOB_TOO_BIG;
                return;
            }
            // Spawn a child to bang the serial port with sync messages.
            // We can kill it later with timer_.cancel().
            fork Op{op}();
            if (op.is_child()) {
                while (++syncAttempts_ <= kSyncMaxAttempts) {
                    BOOST_LOG(log_) << "Writing sync message (" << syncAttempts_ << "/" << kSyncMaxAttempts << ")";
                    yield async_write(sp_, buffer(kSyncMessage), move(op));
                    timer_.expires_from_now(kSyncTimeout);
                    yield timer_.async_wait(move(op));
                }
                BOOST_LOG(log_) << "Too many sync attempts";
                rc_ = Status::TIMEOUT;
                sp_.close();
                return;
            }

            // Parent
            BOOST_LOG(log_) << "Reading sync reply";
            yield async_read_until(sp_, buf_, weReceive(kSyncReply), move(op));
            // Kill the child we forked
            timer_.cancel();
            if ((rc_ = handleSyncReply(n, buf_, what_))) return;
            BOOST_LOG(log_) << "Handshake success";

            BOOST_LOG(log_) << "Setting device parameters";
            yield async_write(sp_, buffer(kSetDeviceMessage), move(op));
            yield async_read_until(sp_, buf_, weReceive(kSyncReply), move(op));
            if ((rc_ = handleSyncReply(n, buf_, what_))) return;

            BOOST_LOG(log_) << "Setting extended device parameters";
            yield async_write(sp_, buffer(kSetDeviceExtMessage), move(op));
            yield async_read_until(sp_, buf_, weReceive(kSyncReply), move(op));
            if ((rc_ = handleSyncReply(n, buf_, what_))) return;

            BOOST_LOG(log_) << "Entering programming mode";
            yield async_write(sp_, buffer(kEnterProgmodeMessage), move(op));
            yield async_read_until(sp_, buf_, weReceive(kSyncReply), move(op));
            if ((rc_ = handleSyncReply(n, buf_, what_))) return;

            BOOST_LOG(log_) << "Reading signature";
            yield async_write(sp_, buffer(kReadSignMessage), move(op));
            yield async_read_until(sp_, buf_, weReceive(kReadSignReply), move(op));
            if ((rc_ = handleReadSignReply(n, buf_, what_, pageSize_))) return;
            BOOST_LOG(log_) << "Page size " << pageSize_ << " detected";

            txAddress_ = static_cast<uint16_t>(flashBase_ / 2);
            while (asio::buffer_size(flash_)) {
                BOOST_LOG(log_) << "Programming flash page at address " << std::hex << txAddress_;
                yield async_write(sp_, loadAddressMessage(buffer(txAddress_.data(), 2)), move(op));
                yield async_read_until(sp_, buf_, weReceive(kSyncReply), move(op));
                if ((rc_ = handleSyncReply(n, buf_, what_))) return;

                txPageSize_ = static_cast<uint16_t>(
                    std::min(asio::buffer_size(flash_), static_cast<size_t>(pageSize_)));
                yield {
                    auto message = progPageMessage(
                        buffer(txPageSize_.data(), 2),
                        buffer(kFlashType),
                        buffer(flash_, txPageSize_));
                    async_write(sp_, message, move(op));
                }
                yield async_read_until(sp_, buf_, weReceive(kSyncReply), move(op));
                if ((rc_ = handleSyncReply(n, buf_, what_))) return;

                flash_ = flash_ + pageSize_;
                txAddress_ += pageSize_ / 2;
            }

            // TODO read back flash to verify?

            txAddress_ = static_cast<uint16_t>(eepromBase_ / 2);
            while (asio::buffer_size(eeprom_)) {
                BOOST_LOG(log_) << "Programming EEPROM page at address " << std::hex << txAddress_;
                yield async_write(sp_, loadAddressMessage(buffer(txAddress_.data(), 2)), move(op));
                yield async_read_until(sp_, buf_, weReceive(kSyncReply), move(op));
                if ((rc_ = handleSyncReply(n, buf_, what_))) return;

                txPageSize_ = static_cast<uint16_t>(
                    std::min(asio::buffer_size(eeprom_), static_cast<size_t>(pageSize_)));
                yield {
                    auto message = progPageMessage(
                        buffer(txPageSize_.data(), 2),
                        buffer(kEepromType),
                        buffer(eeprom_, txPageSize_));
                    async_write(sp_, message, move(op));
                }
                yield async_read_until(sp_, buf_, weReceive(kSyncReply), move(op));
                if ((rc_ = handleSyncReply(n, buf_, what_))) return;

                eeprom_ = eeprom_ + pageSize_;
                txAddress_ += pageSize_ / 2;
            }

            // TODO read back EEPROM to verify?

            BOOST_LOG(log_) << "Leaving programming mode";
            yield async_write(sp_, buffer(kLeaveProgmodeMessage), move(op));
            yield async_read_until(sp_, buf_, weReceive(kSyncReply), move(op));
            if ((rc_ = handleSyncReply(n, buf_, what_))) return;

            BOOST_LOG(log_) << "Programming successful";
        }
        else if (asio::error::operation_aborted != ec) {
            BOOST_LOG(log_) << "Sync I/O error: " << ec.message();
            timer_.cancel();
            sp_.close();
            rc_ = ec;
        }
    }

private:
    bool blobTooBig () const {
        auto flashMax = flashBase_ + asio::buffer_size(flash_);
        auto eepromMax = eepromBase_ + asio::buffer_size(eeprom_);
        auto maxAddress = std::numeric_limits<uint16_t>::max();
        return flashMax / 2 > maxAddress
            || eepromMax / 2 > maxAddress;
    }

    detail::FullRegexMatch<BufIter> weReceive (regex re) {
        return detail::FullRegexMatch<BufIter>{what_, re};
    }
};

} // operations

template <class Handler>
BOOST_ASIO_INITFN_RESULT_TYPE(Handler, void(error_code))
asyncProgramAll (asio::serial_port& sp,
    uint32_t flashBase,
    asio::const_buffer flash,
    uint32_t eepromBase,
    asio::const_buffer eeprom,
    Handler&& handler)
{
    asio::detail::async_result_init<
        Handler, void(error_code)
    > init { std::forward<Handler>(handler) };

    makeOperation<operations::ProgramAll>(move(init.handler), sp,
        flashBase, flash,
        eepromBase, eeprom)();

    return init.result.get();
}

} // stk

#include <boost/asio/unyield.hpp>

#endif