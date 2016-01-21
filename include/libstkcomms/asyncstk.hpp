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

#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

#include <boost/regex.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <iomanip>
#include <iterator>
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

struct ProgramAll {
    ProgramAll (asio::serial_port& sp)
        : sp_(sp)
        , timer_(sp.get_io_service())
    {}

    asio::serial_port& sp_;
    asio::steady_timer timer_;
    int syncAttempts_ = 0;
    size_t pageSize_ = 0;

    asio::streambuf buf_;
    using BufIter = asio::buffers_iterator<decltype(buf_)::const_buffers_type>;
    boost::match_results<BufIter> what_;

    boost::log::sources::logger log_;
    error_code rc_;

    std::tuple<error_code> result () {
        return std::make_tuple(rc_);
    }

    template <class Op>
    void operator() (Op& op, error_code ec = {}, size_t bytesTransferred = 0) {
        if (!ec) {
            reenter (op) {
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
                if (!handleSyncReply(bytesTransferred)) return;
                BOOST_LOG(log_) << "Handshake success";

                BOOST_LOG(log_) << "Setting device parameters";
                yield async_write(sp_, buffer(kSetDeviceMessage), move(op));
                yield async_read_until(sp_, buf_, weReceive(kSyncReply), move(op));
                if (!handleSyncReply(bytesTransferred)) return;

                BOOST_LOG(log_) << "Setting extended device parameters";
                yield async_write(sp_, buffer(kSetDeviceExtMessage), move(op));
                yield async_read_until(sp_, buf_, weReceive(kSyncReply), move(op));
                if (!handleSyncReply(bytesTransferred)) return;

                BOOST_LOG(log_) << "Entering programming mode";
                yield async_write(sp_, buffer(kEnterProgmodeMessage), move(op));
                yield async_read_until(sp_, buf_, weReceive(kSyncReply), move(op));
                if (!handleSyncReply(bytesTransferred)) return;

                BOOST_LOG(log_) << "Reading signature";
                yield async_write(sp_, buffer(kReadSignMessage), move(op));
                yield async_read_until(sp_, buf_, weReceive(kReadSignReply), move(op));
                if (!handleReadSignReply(bytesTransferred)) return;

                BOOST_LOG(log_) << "Leaving programming mode";
                yield async_write(sp_, buffer(kLeaveProgmodeMessage), move(op));
                yield async_read_until(sp_, buf_, weReceive(kSyncReply), move(op));
                if (!handleSyncReply(bytesTransferred)) return;                

                BOOST_LOG(log_) << "Programming successful";
            }
        }
        else if (asio::error::operation_aborted != ec) {
            BOOST_LOG(log_) << "Sync I/O error: " << ec.message();
            timer_.cancel();
            sp_.close();
            rc_ = ec;
        }
    }

private:
    detail::FullRegexMatch<BufIter> weReceive (regex re) {
        return detail::FullRegexMatch<BufIter>{what_, re};
    }

    bool handleSyncReply (size_t n) {
        auto b = BufIter::begin(buf_.data());
        auto e = BufIter::end(buf_.data());

        auto hexdump = detail::toHexString(b, e);
        BOOST_LOG(log_) << "Received " << n << " bytes: " << hexdump;

        if (what_[0].matched) {
            buf_.consume(n);
            return true;
        }
        rc_ = Status::PROTOCOL_ERROR;
        return false;
    }

    bool handleReadSignReply (size_t n) {
        auto b = BufIter::begin(buf_.data());
        auto e = BufIter::end(buf_.data());

        auto hexdump = detail::toHexString(b, e);
        BOOST_LOG(log_) << "Received " << n << " bytes: " << hexdump;

        if (what_[0].matched) {
            auto& sigMatch = what_[1];
            auto sigMatchStr = sigMatch.str();
            auto signature = UString(sigMatchStr.begin(), sigMatchStr.end());
            if (!signature.compare(kMobotILSignature)) {
                pageSize_ = 256;
                BOOST_LOG(log_) << "Setting page size to 256";
            }
            else if (!signature.compare(kMobotASignature)) {
                pageSize_ = 128;
                BOOST_LOG(log_) << "Setting page size to 128";
            }
            else {
                auto sigHexdump = detail::toHexString(sigMatch.first, sigMatch.second);
                BOOST_LOG(log_) << "Unknown signature: " << sigHexdump;
                rc_ = Status::UNKNOWN_SIGNATURE;
                return false;
            }
            buf_.consume(n);
            BOOST_LOG(log_) << "Page size " << pageSize_ << " detected";
            return true;
        }
        rc_ = Status::PROTOCOL_ERROR;
        return false;
    }
};

} // operations

template <class Handler>
BOOST_ASIO_INITFN_RESULT_TYPE(Handler, void(error_code))
asyncProgramAll (asio::serial_port& sp, Handler&& handler) {
    asio::detail::async_result_init<
        Handler, void(error_code)
    > init { std::forward<Handler>(handler) };

    makeOperation<operations::ProgramAll>(move(init.handler), sp)();

    return init.result.get();
}

} // stk

#include <boost/asio/unyield.hpp>

#endif