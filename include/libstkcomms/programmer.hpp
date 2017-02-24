#ifndef LIBSTKCOMMS_PROGRAMMER_HPP
#define LIBSTKCOMMS_PROGRAMMER_HPP

#include <libstkcomms/system_error.hpp>

#include <libstkcomms/detail/fullregexmatch.hpp>
#include <libstkcomms/detail/messages.hpp>

#include <util/asio/asynccompletion.hpp>
#include <util/asio/operation.hpp>
#include <util/asio/setserialportoptions.hpp>
#include <util/asio/transparentservice.hpp>
#include <util/log.hpp>

#include <boost/asio/serial_port.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/write.hpp>

#include <boost/endian/arithmetic.hpp>

#include <boost/regex.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <iterator>
#include <limits>
#include <string>
#include <tuple>
#include <utility>

#include <sched.h>

#include <boost/asio/yield.hpp>

namespace stk {

typedef void ProgramAllHandlerSignature(boost::system::error_code);

class ProgrammerImpl : public std::enable_shared_from_this<ProgrammerImpl> {
public:
    explicit ProgrammerImpl (boost::asio::io_service& context)
        : mContext(context)
        , mTimer(mContext)
        , mSerialPort(mContext)
        , mTransactionTimeout(std::chrono::milliseconds(500))
    {}

    void init (boost::asio::serial_port&& serialPort) {
        mSerialPort = std::move(serialPort);
    }

    void close (boost::system::error_code& ec) {
        auto self = this->shared_from_this();
        mContext.post([self, this, ec]() mutable {
            mTimer.expires_at(decltype(mTimer)::time_point::min());
            mSerialPort.close(ec);
        });
    }

    boost::asio::serial_port& stream () { return mSerialPort; }

    template <class Duration>
    void transactionTimeout (Duration&& timeout) {
        mTransactionTimeout = std::forward<Duration>(timeout);
    }

    auto transactionTimeout () const {
        return mTransactionTimeout;
    }

    template <class CompletionToken>
    auto asyncSync (unsigned maxAttempts, CompletionToken&& token) {
        return asyncTransaction(
                maxAttempts,
                boost::asio::buffer(detail::kSyncMessage),
                detail::kSyncReply,
                detail::syncReplyValidator(),
                std::forward<CompletionToken>(token));
    }

    // Upload buffers of contiguous code and data to Flash and EEPROM on the device, each starting
    // at given base addresses.
    template <class FlashProgress, class EepromProgress, class CompletionToken>
    auto asyncProgramAll (
        uint32_t flashBase,
        boost::asio::const_buffer flash,
        FlashProgress&& flashProgress,
        uint32_t eepromBase,
        boost::asio::const_buffer eeprom,
        EepromProgress&& eepromProgress,
        CompletionToken&& token);

private:
    // Upload a buffer of contiguous code to the device, starting at txAddress.
    // txAddress must be a 2-byte word address (i.e., byte address / 2). The code
    // is uploaded in pages of pageSize bytes. type should be either a buffer with
    // 'F' for flash, or 'E' for EEPROM.
    template <class Progress, class CompletionToken>
    auto asyncProgramPages (
            uint16_t txAddress,
            uint16_t pageSize,
            boost::asio::const_buffer type,
            boost::asio::const_buffer code,
            Progress&& progress,
            CompletionToken&& token);

    // Perform one write -> read transaction on the given serial port. The given
    // timer is used to ensure that the write and read operations don't take too
    // long. If the write takes too long, the serial port is simply closed. If the
    // read operation takes too long, the write operation is retried, up to a
    // maximum of maxAttempts retries. If zero is passed as maxAttempts,
    // asyncTransaction is a no-op.
    template <class ConstBufferSequence, class Validator, class CompletionToken>
    auto asyncTransaction (unsigned maxAttempts,
            const ConstBufferSequence& outBuf, boost::regex re, Validator&& validator,
            CompletionToken&& token);

    // Same as above, with a default 1 maxAttempts.
    template <class ConstBufferSequence, class Validator, class CompletionToken>
    auto asyncTransaction (
            const ConstBufferSequence& outBuf, boost::regex re, Validator&& validator,
            CompletionToken&& token)
    {
        return asyncTransaction(1, outBuf, re, std::forward<Validator>(validator),
            std::forward<CompletionToken>(token));
    }

    boost::asio::io_service& mContext;
    boost::asio::steady_timer mTimer;
    boost::asio::serial_port mSerialPort;

    std::chrono::milliseconds mTransactionTimeout;

    mutable util::log::Logger mLog;
};

template <class FlashProgress, class EepromProgress, class CompletionToken>
inline auto ProgrammerImpl::asyncProgramAll (
        uint32_t flashBase, boost::asio::const_buffer flash, FlashProgress&& flashProgress,
        uint32_t eepromBase, boost::asio::const_buffer eeprom, EepromProgress&& eepromProgress,
        CompletionToken&& token) {

    auto blobTooBig = [&] {
        auto flashMax = flashBase + boost::asio::buffer_size(flash);
        auto eepromMax = eepromBase + boost::asio::buffer_size(eeprom);
        auto maxAddress = std::numeric_limits<uint16_t>::max();
        return flashMax / 2 > maxAddress
            || eepromMax / 2 > maxAddress;
    }();

    auto coroutine =
    [ =
    , this
    , pageSize = uint16_t(0)
    ]
    (auto&& op, boost::system::error_code ec = {}) mutable {
        if (!ec) reenter (op) {
            // Guarantee that the blobs we got passed will actually fit on a Linkbot
            if (blobTooBig) {
                op.complete(make_error_code(Status::BLOB_TOO_BIG));
                return;
            }

            BOOST_LOG(op.log()) << "Setting device parameters";
            yield this->asyncTransaction(
                boost::asio::buffer(detail::kSetDeviceMessage),
                detail::kSyncReply, detail::syncReplyValidator(), std::move(op));

            BOOST_LOG(op.log()) << "Setting extended device parameters";
            yield this->asyncTransaction(
                boost::asio::buffer(detail::kSetDeviceExtMessage),
                detail::kSyncReply, detail::syncReplyValidator(), std::move(op));

            BOOST_LOG(op.log()) << "Entering programming mode";
            yield this->asyncTransaction(
                boost::asio::buffer(detail::kEnterProgmodeMessage),
                detail::kSyncReply, detail::syncReplyValidator(), std::move(op));

            BOOST_LOG(op.log()) << "Reading signature";
            yield this->asyncTransaction(
                boost::asio::buffer(detail::kReadSignMessage),
                detail::kReadSignReply, detail::readSignReplyValidator(pageSize), std::move(op));
            BOOST_LOG(op.log()) << "Page size " << pageSize << " detected";

            BOOST_LOG(op.log()) << "Programming flash";
            yield {
                // Guaranteed to be safe because we calculated blobTooBig earlier
                auto txAddress = static_cast<uint16_t>(flashBase / 2);
                this->asyncProgramPages(
                    txAddress, pageSize,
                    boost::asio::buffer(detail::kFlashType), flash,
                    flashProgress, std::move(op));
            }

            BOOST_LOG(op.log()) << "Programming EEPROM";
            yield {
                // Guaranteed to be safe because we called blobTooBig() earlier
                auto txAddress = static_cast<uint16_t>(eepromBase / 2);
                this->asyncProgramPages(
                    txAddress, pageSize,
                    boost::asio::buffer(detail::kEepromType), eeprom,
                    eepromProgress, std::move(op));
            }

            BOOST_LOG(op.log()) << "Leaving programming mode";
            yield this->asyncTransaction(
                boost::asio::buffer(detail::kLeaveProgmodeMessage),
                detail::kSyncReply, detail::syncReplyValidator(), std::move(op));

            op.complete(make_error_code(Status::OK));
        }
        else if (boost::asio::error::operation_aborted != ec) {
            op.complete(ec);
            BOOST_LOG(op.log()) << "ProgramAll: " << ec.message();
        }
    };

    return util::asio::asyncDispatch(
        mContext,
        std::make_tuple(make_error_code(boost::asio::error::operation_aborted)),
        std::move(coroutine),
        std::forward<CompletionToken>(token)
    );
}

template <class Progress, class CompletionToken>
inline auto ProgrammerImpl::asyncProgramPages (
        uint16_t txAddress,
        uint16_t pageSize,
        boost::asio::const_buffer type,
        boost::asio::const_buffer code,
        Progress&& progress,
        CompletionToken&& token) {
    auto coroutine =
    [ this
    , type
    , code
    , progress = std::forward<Progress>(progress)
    // txAddress and pageSize get sent over the wire in differing endianness (wtf),
    // so it's most maintainable to just store them as such.
    , txAddress = boost::endian::little_uint16_t(txAddress)
    , pageSize = boost::endian::big_uint16_t(pageSize)
    ]
    (auto&& op, boost::system::error_code ec = {}) mutable {
        if (!ec) reenter (op) {
            while (boost::asio::buffer_size(code)) {
                yield {
                    auto message = detail::loadAddressMessage(
                        boost::asio::buffer(txAddress.data(), 2));
                    this->asyncTransaction(message, detail::kSyncReply,
                            detail::syncReplyValidator(), std::move(op));
                }

                if (boost::asio::buffer_size(code) < pageSize) {
                    pageSize = static_cast<uint16_t>(boost::asio::buffer_size(code));
                }
                yield {
                    auto message = detail::progPageMessage(
                        boost::asio::buffer(pageSize.data(), 2),
                        type,
                        boost::asio::buffer(code, pageSize));
                    this->asyncTransaction(message, detail::kSyncReply,
                            detail::syncReplyValidator(), std::move(op));
                }

                code = code + pageSize;
                txAddress += pageSize / 2;

                // Update client code on progress, i.e., bytes written
                progress(static_cast<uint16_t>(pageSize));
            }
            // TODO read back pages to verify?
            op.complete(make_error_code(Status::OK));
        }
        else if (boost::asio::error::operation_aborted != ec) {
            op.complete(ec);
            BOOST_LOG(op.log()) << "ProgramPages: " << ec.message();
        }
    };

    return util::asio::asyncDispatch(
        mContext,
        std::make_tuple(make_error_code(boost::asio::error::operation_aborted)),
        std::move(coroutine),
        std::forward<CompletionToken>(token)
    );
}

template <class ConstBufferSequence, class Validator, class CompletionToken>
inline auto ProgrammerImpl::asyncTransaction (unsigned maxAttempts,
        const ConstBufferSequence& outBuf, boost::regex re, Validator&& validator,
        CompletionToken&& token) {
    auto coroutine =
    [ this
    , maxAttempts
    , outBuf
    , re
    , validator = std::forward<Validator>(validator)
    , inBuf = std::make_unique<detail::StreamBuf>()  // StreamBuf is immovable
    , what = boost::match_results<detail::StreamBufIter>{}
    , attempts = 0u
    , readDone = false
    , writeDone = false
    ]
    (auto&& op, boost::system::error_code ec = {}, size_t n = 0) mutable {
        BOOST_LOG(op.log()) << "asyncTransaction: reenter on CPU=[" << sched_getcpu()
                << "] ec=[" << ec.message() << "]";
        if (!ec) reenter (op) {
            if (!maxAttempts) {
                BOOST_LOG(op.log()) << "asyncTransaction: passed 0 maxAttempts";
                op.complete(make_error_code(Status::OK));
                return;
            }
            // Spawn a child to continuously spawn write ops if the read op
            // takes forever. If the write op runs out of retries, the child
            // will close the serial port.
            fork op.runChild();
            if (op.is_child()) {
                BOOST_LOG(op.log()) << "asyncTransaction: write child Entering write loop";
                while (!readDone && ++attempts <= maxAttempts) {
                    writeDone = false;

                    // Spawn a child to close the serial port if the write op
                    // takes forever.
                    fork op.runChild();
                    if (op.is_child()) {
                        BOOST_LOG(op.log()) << "asyncTransaction: write child beginning write watchdog";
                        if (mTimer.expires_at() ==
                                boost::asio::steady_timer::time_point::min()) {
                            BOOST_LOG(op.log()) << "asyncTransaction: write child already canceled";
                            return;
                        }
                        mTimer.expires_from_now(this->transactionTimeout());
                        yield mTimer.async_wait(std::move(op));

                        // Rely on writeDone to close the serial port, in case
                        // the async_write and async_wait continuations were
                        // scheduled back-to-back.
                        if (!writeDone) {
                            op.complete(make_error_code(Status::TIMEOUT));
                            BOOST_LOG(op.log()) << "Transaction timed out on write";
                            this->close(ec);
                        }
                        BOOST_LOG(op.log()) << "asyncTransaction: write child write watchdog done";
                        return;
                    }

                    if (attempts > 1) {
                        BOOST_LOG(op.log()) << "Retrying Transaction write attempt ("
                            << attempts << "/" << maxAttempts << ")";
                    }
                    BOOST_LOG(op.log()) << "asyncTransaction: write child writing";
                    yield boost::asio::async_write(mSerialPort, outBuf, std::move(op));
                    writeDone = true;

                    BOOST_LOG(op.log()) << "asyncTransaction: write child beginning read watchdog";
                    if (mTimer.expires_at() ==
                            boost::asio::steady_timer::time_point::min()) {
                        return;
                    }
                    mTimer.expires_from_now(this->transactionTimeout());
                    yield mTimer.async_wait(std::move(op));
                }
                // Similar to writeDone, we need to check the return code, in
                // case the async_read and async_wait continuations were
                // scheduled back-to-back.
                if (!readDone) {
                    BOOST_LOG(op.log()) << "Transaction timed out on read";
                    op.complete(make_error_code(Status::TIMEOUT));
                    this->close(ec);
                }
                BOOST_LOG(op.log()) << "asyncTransaction: write child read watchdog done";
                return;
            }

            BOOST_LOG(op.log()) << "asyncTransaction: reading";
            yield {
                auto matchCond = detail::FullRegexMatch<detail::StreamBufIter>{what, re};
                boost::asio::async_read_until(mSerialPort, *inBuf, matchCond, std::move(op));
            }
            readDone = true;
            mTimer.expires_at(boost::asio::steady_timer::time_point::min());

            BOOST_LOG(op.log()) << "asyncTransaction: validating input";
            ec = Status::OK;  // ensure `ec` is 0 before calling `validator`
            validator(n, *inBuf, what, ec);
            op.complete(ec);
            BOOST_LOG(op.log()) << "asyncTransaction: done";
        }
        else if (ec != boost::asio::error::operation_aborted) {
            BOOST_LOG(op.log()) << "TransactionOperation error: " << ec.message();
            op.complete(ec);
            this->close(ec);
        }
    };

    mTimer.expires_from_now(std::chrono::seconds(0));
    // We use mTimer.expires_at(time_point::min()) to break the transaction attempt loop. Make sure
    // it's in an "uncanceled" state before spawning the operation.

    return util::asio::asyncDispatch(mContext,
        std::make_tuple(make_error_code(boost::asio::error::operation_aborted)),
        std::move(coroutine),
        std::forward<CompletionToken>(token)
    );
}

class Programmer : public util::asio::TransparentIoObject<ProgrammerImpl> {
public:
    explicit Programmer (boost::asio::io_service& context)
        : util::asio::TransparentIoObject<ProgrammerImpl>(context)
    {}

    explicit Programmer (boost::asio::serial_port&& serialPort)
        : util::asio::TransparentIoObject<ProgrammerImpl>(serialPort.get_io_service())
    {
        this->get_implementation()->init(std::move(serialPort));
    }

    boost::asio::serial_port& stream () {
        return this->get_implementation()->stream();
    }

    template <class Duration>
    void transactionTimeout (Duration&& timeout) {
        this->get_implementation()->transactionTimeout(std::forward<Duration>(timeout));
    }

    auto transactionTimeout () const {
        return this->get_implementation()->transactionTimeout();
    }

    UTIL_ASIO_DECL_ASYNC_METHOD(asyncSync)
    UTIL_ASIO_DECL_ASYNC_METHOD(asyncProgramAll)
};

} // namespace stk

#include <boost/asio/unyield.hpp>

#endif
