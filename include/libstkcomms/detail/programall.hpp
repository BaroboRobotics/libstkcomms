#ifndef LIBSTKCOMMS_ASYNCSTK_HPP
#define LIBSTKCOMMS_ASYNCSTK_HPP

#include <libstkcomms/system_error.hpp>

#include <libstkcomms/detail/fullregexmatch.hpp>
#include <libstkcomms/detail/messages.hpp>
#include <libstkcomms/detail/prefix.hpp>
#include <libstkcomms/detail/transaction.hpp>

#include <util/setserialportoptions.hpp>

#include <boost/asio/async_result.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/steady_timer.hpp>

#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

#include <algorithm>
#include <array>
#include <iterator>
#include <limits>
#include <tuple>

#include <boost/asio/yield.hpp>

namespace stk {
namespace detail {
namespace {

inline bool handleSyncReply (size_t n, asio::streambuf& buf,
    const boost::match_results<StreamBufIter>& what, error_code& ec)
{
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
    if (what[0].matched) {
        const auto& sigMatch = what[1].str();
        auto signature = UString(sigMatch.begin(), sigMatch.end());
        if (!signature.compare(kMobotILSignature)) {
            pageSize = 256;
        }
        else if (!signature.compare(kMobotASignature)) {
            pageSize = 128;
        }
        else {
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
    ProgramPages (asio::serial_port& sp, asio::steady_timer& timer,
        little_uint16_t txAddress, big_uint16_t pageSize,
        const_buffer type, const_buffer code, Progress&& progress)
        : sp_(sp)
        , timer_(timer)
        , txAddress_(txAddress)
        , pageSize_(pageSize)
        , type_(type)
        , code_(code)
        , progress_(std::forward<Progress>(progress))
    {}

    asio::serial_port& sp_;
    asio::steady_timer& timer_;
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
                yield asyncTransaction(sp_, timer_, loadAddressMessage(buffer(txAddress_.data(), 2)),
                    buf_, weReceive(what_, kSyncReply), move(op));
                if (!handleSyncReply(n, buf_, what_, rc_)) return;

                if (asio::buffer_size(code_) < pageSize_) {
                    pageSize_ = static_cast<uint16_t>(asio::buffer_size(code_));
                }
                yield {
                    auto message = progPageMessage(
                        buffer(pageSize_.data(), 2),
                        type_,
                        buffer(code_, pageSize_));
                    asyncTransaction(sp_, timer_, message,
                        buf_, weReceive(what_, kSyncReply), move(op));
                }
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
            timer_.expires_at(asio::steady_timer::time_point::min());
            sp_.close();
            rc_ = ec;
        }
    }
};

template <class Progress, class Handler>
BOOST_ASIO_INITFN_RESULT_TYPE(Handler, void(error_code))
asyncProgramPages (asio::serial_port& sp,
    asio::steady_timer& timer,
    uint16_t txAddress,
    uint16_t pageSize,
    const_buffer type,
    const_buffer code,
    Progress&& progress,
    Handler&& handler)
{
    asio::detail::async_result_init<
        Handler, void(error_code)
    > init { std::forward<Handler>(handler) };

    using Op = ProgramPages<Progress>;
    makeOperation<Op>(move(init.handler), sp, timer,
        txAddress, pageSize, type, code, std::forward<Progress>(progress))();

    return init.result.get();
}

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

    uint32_t flashBase_;
    const_buffer flash_;
    FlashProgress flashProgress_;

    uint32_t eepromBase_;
    const_buffer eeprom_;
    EepromProgress eepromProgress_;

    uint16_t pageSize_ = 0;

    error_code rc_ = asio::error::operation_aborted;

    boost::log::sources::logger log_;

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

            if (timer_.expires_at() == asio::steady_timer::time_point::min()) {
                BOOST_LOG(log_) << "ProgramAll cancelled before setting options";
                return;
            }
            timer_.expires_from_now(util::kSerialSettleTimeAfterOpen);
            yield timer_.async_wait(move(op));

            util::setSerialPortOptions(sp_, 57600, ec);
            if (ec) {
                rc_ = ec;
                return;
            }

            if (timer_.expires_at() == asio::steady_timer::time_point::min()) {
                BOOST_LOG(log_) << "ProgramAll cancelled before sync";
                return;
            }
            timer_.expires_from_now(kStartDelay);
            yield timer_.async_wait(move(op));

            BOOST_LOG(log_) << "Syncing with device";
            yield asyncTransaction(sp_, timer_, kSyncMaxAttempts, buffer(kSyncMessage),
                buf_, weReceive(what_, kSyncReply), move(op));
            if (!handleSyncReply(n, buf_, what_, rc_)) return;

            BOOST_LOG(log_) << "Setting device parameters";
            yield asyncTransaction(sp_, timer_, buffer(kSetDeviceMessage),
                buf_, weReceive(what_, kSyncReply), move(op));
            if (!handleSyncReply(n, buf_, what_, rc_)) return;

            BOOST_LOG(log_) << "Setting extended device parameters";
            yield asyncTransaction(sp_, timer_, buffer(kSetDeviceExtMessage),
                buf_, weReceive(what_, kSyncReply), move(op));
            if (!handleSyncReply(n, buf_, what_, rc_)) return;

            BOOST_LOG(log_) << "Entering programming mode";
            yield asyncTransaction(sp_, timer_, buffer(kEnterProgmodeMessage),
                buf_, weReceive(what_, kSyncReply), move(op));
            if (!handleSyncReply(n, buf_, what_, rc_)) return;

            BOOST_LOG(log_) << "Reading signature";
            yield asyncTransaction(sp_, timer_, buffer(kReadSignMessage),
                buf_, weReceive(what_, kReadSignReply), move(op));
            if (!handleReadSignReply(n, buf_, what_, pageSize_, rc_)) return;
            BOOST_LOG(log_) << "Page size " << pageSize_ << " detected";

            BOOST_LOG(log_) << "Programming flash";
            yield {
                // Guaranteed to be safe because we called blobTooBig() earlier
                auto txAddress = static_cast<uint16_t>(flashBase_ / 2);
                asyncProgramPages(sp_, timer_,
                    txAddress, pageSize_,
                    buffer(kFlashType), flash_,
                    move(flashProgress_), move(op));
            }

            BOOST_LOG(log_) << "Programming EEPROM";
            yield {
                // Guaranteed to be safe because we called blobTooBig() earlier
                auto txAddress = static_cast<uint16_t>(eepromBase_ / 2);
                asyncProgramPages(sp_, timer_,
                    txAddress, pageSize_,
                    buffer(kEepromType), eeprom_,
                    move(eepromProgress_), move(op));
            }

            BOOST_LOG(log_) << "Leaving programming mode";
            yield asyncTransaction(sp_, timer_, buffer(kLeaveProgmodeMessage),
                buf_, weReceive(what_, kSyncReply), move(op));
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

template <class FlashProgress, class EepromProgress, class Handler>
BOOST_ASIO_INITFN_RESULT_TYPE(Handler, void(error_code))
asyncProgramAll (asio::serial_port& sp,
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

    using Op = ProgramAll<FlashProgress, EepromProgress>;
    makeOperation<Op>(move(init.handler), sp, path, timer,
        flashBase, flash, std::forward<FlashProgress>(flashProgress),
        eepromBase, eeprom, std::forward<EepromProgress>(eepromProgress))();

    return init.result.get();
}

} // detail
} // stk

#include <boost/asio/unyield.hpp>

#endif