#ifndef LIBSTKCOMMS_ASYNCSTK_HPP
#define LIBSTKCOMMS_ASYNCSTK_HPP

#include <libstkcomms/fullregexmatch.hpp>
#include <libstkcomms/messages.hpp>
#include <libstkcomms/system_error.hpp>

#include <util/composed.hpp>
#include <util/setserialportoptions.hpp>

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
    using namespace std::placeholders;

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

    const std::chrono::milliseconds kSyncRetryTimeout { 500 };
    const std::chrono::seconds kSyncTimeout { 2 };
    const unsigned kSyncMaxAttempts { 3 };
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
namespace {

inline bool handleSyncReply (size_t n, asio::streambuf& buf,
    const boost::match_results<StreamBufIter>& what, error_code& ec)
{
    auto b = StreamBufIter::begin(buf.data());
    auto e = StreamBufIter::end(buf.data());

    auto hexdump = detail::toHexString(b, e);

    if (what[0].matched) {
        buf.consume(n);
        return true;
    }
    ec = Status::PROTOCOL_ERROR;
    return false;
}

inline bool handleReadSignReply (size_t n, asio::streambuf& buf,
    const boost::match_results<StreamBufIter>& what, uint16_t& pageSize, error_code& ec)
{
    auto b = StreamBufIter::begin(buf.data());
    auto e = StreamBufIter::end(buf.data());

    auto hexdump = detail::toHexString(b, e);

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
            ec = Status::UNKNOWN_SIGNATURE;
            return false;
        }
        buf.consume(n);
        return true;
    }
    ec = Status::PROTOCOL_ERROR;
    return false;
}

inline detail::FullRegexMatch<StreamBufIter>
weReceive (boost::match_results<StreamBufIter>& what, regex re) {
    return detail::FullRegexMatch<StreamBufIter>{what, re};
}

} // <anonymous>

// Upload a buffer of contiguous code to the device, starting at txAddress.
// txAddress must be a 2-byte word address (i.e., byte address / 2). The code
// is uploaded in pages of pageSize bytes. type should be either a buffer with
// 'F' for flash, or 'E' for EEPROM.
template <class Progress>
struct ProgramPages {
    ProgramPages (asio::serial_port& sp,
        little_uint16_t txAddress, big_uint16_t pageSize,
        const_buffer type, const_buffer code, Progress&& progress)
        : sp_(sp)
        , txAddress_(txAddress)
        , pageSize_(pageSize)
        , type_(type)
        , code_(code)
        , progress_(std::forward<Progress>(progress))
    {}

    asio::serial_port& sp_;
    asio::streambuf buf_;
    boost::match_results<StreamBufIter> what_;

    // These two values get sent over the wire in differing endianness (wtf),
    // so it's most maintainable to just store them as such.
    little_uint16_t txAddress_;
    big_uint16_t pageSize_;

    const_buffer type_; // 'F' or 'E' for flash or EEPROM
    const_buffer code_;

    Progress progress_;

    boost::log::sources::logger log_;
    error_code rc_ = asio::error::operation_aborted;

    std::tuple<error_code> result () {
        return std::make_tuple(rc_);
    }

    template <class Op>
    void operator() (Op&& op, error_code ec = {}, size_t n = 0) {
        if (!ec) reenter (op) {
            while (asio::buffer_size(code_)) {
                yield async_write(sp_, loadAddressMessage(buffer(txAddress_.data(), 2)), move(op));
                yield async_read_until(sp_, buf_, weReceive(what_, kSyncReply), move(op));
                if (!handleSyncReply(n, buf_, what_, rc_)) return;

                if (asio::buffer_size(code_) < pageSize_) {
                    pageSize_ = static_cast<uint16_t>(asio::buffer_size(code_));
                }
                yield {
                    auto message = progPageMessage(
                        buffer(pageSize_.data(), 2),
                        type_,
                        buffer(code_, pageSize_));
                    async_write(sp_, message, move(op));
                }
                yield async_read_until(sp_, buf_, weReceive(what_, kSyncReply), move(op));
                if (!handleSyncReply(n, buf_, what_, rc_)) return;

                code_ = code_ + pageSize_;
                txAddress_ += pageSize_ / 2;

                // Update client code on progress, i.e., bytes written
                progress_(pageSize_);
            }
            // TODO read back pages to verify?
            rc_ = Status::OK;
        }
        else if (asio::error::operation_aborted != ec) {
            BOOST_LOG(log_) << "ProgramPages I/O error: " << ec.message();
            sp_.close();
            rc_ = ec;
        }
    }
};

template <class FlashProgress, class EepromProgress>
struct ProgramAll {
    ProgramAll (asio::serial_port& sp, const std::string& path, asio::steady_timer& timer,
        uint32_t flashBase, const_buffer flash, FlashProgress&& flashProgress,
        uint32_t eepromBase, const_buffer eeprom, EepromProgress&& eepromProgress)
        : sp_(sp)
        , path_(path)
        , timer_(timer)
        , flashBase_(flashBase)
        , flash_(flash)
        , flashProgress_(std::forward<FlashProgress>(flashProgress))
        , eepromBase_(eepromBase)
        , eeprom_(eeprom)
        , eepromProgress_(std::forward<EepromProgress>(eepromProgress))
    {}

    asio::serial_port& sp_;
    std::string path_;
    asio::streambuf buf_;
    boost::match_results<StreamBufIter> what_;

    asio::steady_timer& timer_;
    int syncAttempts_ = 0;

    uint32_t flashBase_;
    const_buffer flash_;
    FlashProgress flashProgress_;

    uint32_t eepromBase_;
    const_buffer eeprom_;
    EepromProgress eepromProgress_;

    uint16_t pageSize_ = 0;

    boost::log::sources::logger log_;
    error_code rc_ = asio::error::operation_aborted;

    std::tuple<error_code> result () {
        return std::make_tuple(rc_);
    }

    template <class Op>
    void operator() (Op&& op, error_code ec = {}, size_t n = 0) {
        if (!ec) reenter (op) {
            // Guarantee that the blobs we got passed will actually fit on a Linkbot
            if (blobTooBig()) {
                rc_ = Status::BLOB_TOO_BIG;
                return;
            }

            sp_.open(path_, ec);
            if (ec) {
                rc_ = ec;
                return;
            }

            timer_.expires_from_now(util::kSerialSettleTimeAfterOpen);
            yield timer_.async_wait(move(op));
            util::setSerialPortOptions(sp_, 57600, ec);
            if (ec) {
                rc_ = ec;
                return;
            }

            // Spawn a child to bang the serial port with sync messages.
            // We can kill it later with:
            //   timer_.expires_at(decltype(timer_)::time_point::min())
            // Simply calling timer_.cancel() wouldn't do the trick without a
            // separate boolean flag in case this coroutine child were
            // suspended on the async_write() call.
            if (timer_.expires_at() == asio::steady_timer::time_point::min()) {
                BOOST_LOG(log_) << "Sync spammer cancelled mid-write";
                return;
            }
            timer_.expires_from_now(kSyncTimeout);
            yield timer_.async_wait(move(op));
            fork Op{op}();
            if (op.is_child()) {
                while (++syncAttempts_ <= kSyncMaxAttempts) {
                    BOOST_LOG(log_) << "Writing sync message (" << syncAttempts_ << "/" << kSyncMaxAttempts << ")";
                    yield async_write(sp_, buffer(kSyncMessage), move(op));
                    if (timer_.expires_at() == asio::steady_timer::time_point::min()) {
                        BOOST_LOG(log_) << "Sync spammer cancelled mid-write";
                        return;
                    }
                    timer_.expires_from_now(kSyncRetryTimeout);
                    yield timer_.async_wait(move(op));
                }
                BOOST_LOG(log_) << "Too many sync attempts";
                rc_ = Status::TIMEOUT;
                sp_.close();
                return;
            }

            // Parent
            BOOST_LOG(log_) << "Reading sync reply";
            yield async_read_until(sp_, buf_, weReceive(what_, kSyncReply), move(op));
            // Kill the child we forked
            timer_.expires_at(asio::steady_timer::time_point::min());
            if (!handleSyncReply(n, buf_, what_, rc_)) return;
            BOOST_LOG(log_) << "Handshake success";

            BOOST_LOG(log_) << "Setting device parameters";
            yield async_write(sp_, buffer(kSetDeviceMessage), move(op));
            yield async_read_until(sp_, buf_, weReceive(what_, kSyncReply), move(op));
            if (!handleSyncReply(n, buf_, what_, rc_)) return;

            BOOST_LOG(log_) << "Setting extended device parameters";
            yield async_write(sp_, buffer(kSetDeviceExtMessage), move(op));
            yield async_read_until(sp_, buf_, weReceive(what_, kSyncReply), move(op));
            if (!handleSyncReply(n, buf_, what_, rc_)) return;

            BOOST_LOG(log_) << "Entering programming mode";
            yield async_write(sp_, buffer(kEnterProgmodeMessage), move(op));
            yield async_read_until(sp_, buf_, weReceive(what_, kSyncReply), move(op));
            if (!handleSyncReply(n, buf_, what_, rc_)) return;

            BOOST_LOG(log_) << "Reading signature";
            yield async_write(sp_, buffer(kReadSignMessage), move(op));
            yield async_read_until(sp_, buf_, weReceive(what_, kReadSignReply), move(op));
            if (!handleReadSignReply(n, buf_, what_, pageSize_, rc_)) return;
            BOOST_LOG(log_) << "Page size " << pageSize_ << " detected";

            BOOST_LOG(log_) << "Programming flash";
            yield {
                // Guaranteed to be safe because we called blobTooBig() earlier
                auto txAddress = static_cast<uint16_t>(flashBase_ / 2);
                makeOperation<ProgramPages<FlashProgress>>(move(op), sp_,
                    txAddress, pageSize_,
                    buffer(kFlashType), flash_,
                    move(flashProgress_))();
            }

            BOOST_LOG(log_) << "Programming EEPROM";
            yield {
                // Guaranteed to be safe because we called blobTooBig() earlier
                auto txAddress = static_cast<uint16_t>(eepromBase_ / 2);
                makeOperation<ProgramPages<EepromProgress>>(move(op), sp_,
                    txAddress, pageSize_,
                    buffer(kEepromType), eeprom_,
                    move(eepromProgress_))();
            }

            BOOST_LOG(log_) << "Leaving programming mode";
            yield async_write(sp_, buffer(kLeaveProgmodeMessage), move(op));
            yield async_read_until(sp_, buf_, weReceive(what_, kSyncReply), move(op));
            if (!handleSyncReply(n, buf_, what_, rc_)) return;

            rc_ = Status::OK;
        }
        else if (asio::error::operation_aborted != ec) {
            BOOST_LOG(log_) << "ProgramAll I/O error: " << ec.message();
            timer_.expires_at(asio::steady_timer::time_point::min());
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
};

} // operations

template <class FlashProgress, class EepromProgress, class Handler>
BOOST_ASIO_INITFN_RESULT_TYPE(Handler, void(error_code))
asyncProgramAllImpl (asio::serial_port& sp,
    const std::string& path,
    asio::steady_timer& timer,
    uint32_t flashBase,
    const_buffer flash,
    FlashProgress&& flashProgress,
    uint32_t eepromBase,
    const_buffer eeprom,
    EepromProgress&& eepromProgress,
    Handler&& handler)
{
    asio::detail::async_result_init<
        Handler, void(error_code)
    > init { std::forward<Handler>(handler) };

    using ProgramAll = operations::ProgramAll<FlashProgress, EepromProgress>;
    makeOperation<ProgramAll>(move(init.handler), sp, path, timer,
        flashBase, flash, std::forward<FlashProgress>(flashProgress),
        eepromBase, eeprom, std::forward<EepromProgress>(eepromProgress))();

    return init.result.get();
}

} // stk

#include <boost/asio/unyield.hpp>

#endif
