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
#include <boost/asio/streambuf.hpp>

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

#include <boost/asio/yield.hpp>

namespace stk {

typedef void ProgramAllHandlerSignature(boost::system::error_code);

class ProgrammerImpl : public std::enable_shared_from_this<ProgrammerImpl> {
public:
    using StreamBufIter = boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type>;

    explicit ProgrammerImpl (boost::asio::io_service& context)
        : mContext(context)
        , mTimer(mContext)
        , mSerialPort(mContext)
        , mTransactionTimeout(std::chrono::milliseconds(500))
    {}

    void close (boost::system::error_code& ec) {
        auto self = this->shared_from_this();
        mContext.post([self, this, ec]() mutable {
            mTimer.expires_at(decltype(mTimer)::time_point::min());
            mSerialPort.close(ec);
        });
    }

    template <class Duration>
    void transactionTimeout (Duration&& timeout) {
        mTransactionTimeout = std::forward<Duration>(timeout);
    }

    auto transactionTimeout () const {
        return mTransactionTimeout;
    }

    template <class Dur1, class Dur2>
    struct OpenDeviceOperation;
    template <class Dur1, class Dur2>
    friend struct OpenDeviceOperation;

    template <class Dur1, class Dur2, class CompletionToken>
    auto asyncOpenDevice (const std::string& path,
            Dur1&& settleDelay, Dur2&& writeDelay, CompletionToken&& token) {
        util::asio::AsyncCompletion<
            CompletionToken, void(boost::system::error_code)
        > init { std::forward<CompletionToken>(token) };

        using Op = OpenDeviceOperation<Dur1, Dur2>;
        util::asio::makeOperation<Op>(std::move(init.handler),
            this->shared_from_this(), path,
            std::forward<Dur1>(settleDelay), std::forward<Dur2>(writeDelay)
        )();

        return init.result.get();
    }

    template <class CompletionToken>
    auto asyncSync (unsigned maxAttempts, CompletionToken&& token) {
        util::asio::AsyncCompletion<
            CompletionToken, void(boost::system::error_code)
        > init { std::forward<CompletionToken>(token) };

        asyncTransaction(maxAttempts,
            boost::asio::buffer(detail::kSyncMessage), detail::kSyncReply,
            [handler = std::move(init.handler), this, self = this->shared_from_this()]
            (boost::system::error_code ec, size_t n) {
                if (!ec) {
                    checkSyncReply(n, ec);
                    mContext.post(std::bind(handler, ec));
                }
                else {
                    mContext.post(std::bind(handler, ec));
                    close(ec);
                }
            });

        return init.result.get();
    }

    template <class FlashProgress, class EepromProgress>
    struct ProgramAllOperation;
    template <class FlashProgress, class EepromProgress>
    friend struct ProgramAllOperation;

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
        CompletionToken&& token)
    {
        util::asio::AsyncCompletion<
            CompletionToken, ProgramAllHandlerSignature
        > init { std::forward<CompletionToken>(token) };

        using Op = ProgramAllOperation<FlashProgress, EepromProgress>;
        util::asio::makeOperation<Op>(std::move(init.handler),
            this->shared_from_this(),
            flashBase, flash, std::forward<FlashProgress>(flashProgress),
            eepromBase, eeprom, std::forward<EepromProgress>(eepromProgress)
        )();

        return init.result.get();
    }

private:
    template <class Progress>
    struct ProgramPagesOperation;
    template <class Progress>
    friend struct ProgramPagesOperation;

    // Upload a buffer of contiguous code to the device, starting at txAddress.
    // txAddress must be a 2-byte word address (i.e., byte address / 2). The code
    // is uploaded in pages of pageSize bytes. type should be either a buffer with
    // 'F' for flash, or 'E' for EEPROM.
    template <class Progress, class CompletionToken>
    BOOST_ASIO_INITFN_RESULT_TYPE(CompletionToken, void(boost::system::error_code))
    asyncProgramPages (
        uint16_t txAddress,
        uint16_t pageSize,
        boost::asio::const_buffer type,
        boost::asio::const_buffer code,
        Progress&& progress,
        CompletionToken&& token)
    {
        util::asio::AsyncCompletion<
            CompletionToken, void(boost::system::error_code)
        > init { std::forward<CompletionToken>(token) };

        using Op = ProgramPagesOperation<Progress>;
        util::asio::makeOperation<Op>(std::move(init.handler),
            this->shared_from_this(),
            txAddress, pageSize, type, code, std::forward<Progress>(progress))();

        return init.result.get();
    }

    template <class ConstBufferSequence>
    struct TransactionOperation;
    template <class ConstBufferSequence>
    friend struct TransactionOperation;

    // Perform one write -> read transaction on the given serial port. The given
    // timer is used to ensure that the write and read operations don't take too
    // long. If the write takes too long, the serial port is simply closed. If the
    // read operation takes too long, the write operation is retried, up to a
    // maximum of maxAttempts retries. If zero is passed as maxAttempts,
    // asyncTransaction is a no-op.
    template <class ConstBufferSequence, class CompletionToken>
    auto asyncTransaction (unsigned maxAttempts,
        const ConstBufferSequence& outBuf, boost::regex re,
        CompletionToken&& token)
    {
        util::asio::AsyncCompletion<
            CompletionToken, void(boost::system::error_code, size_t)
        > init { std::forward<CompletionToken>(token) };

        using Op = TransactionOperation<ConstBufferSequence>;
        util::asio::makeOperation<Op>(std::move(init.handler),
            this->shared_from_this(), maxAttempts, outBuf, re)();

        return init.result.get();
    }

    // Same as above, with a default 1 maxAttempts.
    template <class ConstBufferSequence, class CompletionToken>
    auto asyncTransaction (
        const ConstBufferSequence& outBuf, boost::regex re,
        CompletionToken&& token)
    {
        return asyncTransaction(1, outBuf, re,
            std::forward<CompletionToken>(token));
    }

    bool checkSyncReply (size_t n, boost::system::error_code& ec) {
        if (mWhat[0].matched) {
            mBuf.consume(n);
            return true;
        }
        ec = Status::PROTOCOL_ERROR;
        return false;
    }

    bool checkReadSignReply (size_t n, uint16_t& pageSize, boost::system::error_code& ec) {
        if (mWhat[0].matched) {
            const auto& sigMatch = mWhat[1].str();
            auto signature = detail::UString(sigMatch.begin(), sigMatch.end());
            if (!signature.compare(detail::kMobotILSignature)) {
                pageSize = 256;
            }
            else if (!signature.compare(detail::kMobotASignature)) {
                pageSize = 128;
            }
            else {
                ec = Status::UNKNOWN_SIGNATURE;
                return false;
            }
            mBuf.consume(n);
            return true;
        }
        ec = Status::PROTOCOL_ERROR;
        return false;
    }

    auto weReceive (boost::regex re) {
        return detail::FullRegexMatch<StreamBufIter>{mWhat, re};
    }

    boost::asio::io_service& mContext;
    boost::asio::steady_timer mTimer;
    boost::asio::serial_port mSerialPort;
    boost::asio::streambuf mBuf;
    boost::match_results<StreamBufIter> mWhat;

    std::chrono::milliseconds mTransactionTimeout;

    mutable util::log::Logger mLog;
};

class Programmer : public util::asio::TransparentIoObject<ProgrammerImpl> {
public:
    explicit Programmer (boost::asio::io_service& context)
        : util::asio::TransparentIoObject<ProgrammerImpl>(context)
    {}

    template <class Duration>
    void transactionTimeout (Duration&& timeout) {
        this->get_implementation()->transactionTimeout(std::forward<Duration>(timeout));
    }

    auto transactionTimeout () const {
        return this->get_implementation()->transactionTimeout();
    }

    UTIL_ASIO_DECL_ASYNC_METHOD(asyncOpenDevice)
    UTIL_ASIO_DECL_ASYNC_METHOD(asyncSync)
    UTIL_ASIO_DECL_ASYNC_METHOD(asyncProgramAll)
};

template <class Dur1, class Dur2>
struct ProgrammerImpl::OpenDeviceOperation {
    using Nest = ProgrammerImpl;

    OpenDeviceOperation (std::shared_ptr<Nest> nest, const std::string& path,
        Dur1&& settleDelay, Dur2&& writeDelay)
        : nest_(std::move(nest))
        , path_(path)
        , settleDelay_(std::forward<Dur1>(settleDelay))
        , writeDelay_(std::forward<Dur2>(writeDelay))
    {}

    std::shared_ptr<Nest> nest_;
    std::string path_;
    Dur1 settleDelay_;
    Dur2 writeDelay_;

    boost::system::error_code rc_ = boost::asio::error::operation_aborted;

    auto result () const {
        return std::make_tuple(rc_);
    }

    template <class Op>
    void operator() (Op&& op, boost::system::error_code ec = {}) {
        if (!ec) reenter (op) {
            nest_->mSerialPort.open(path_, ec);
            if (ec) {
                rc_ = ec;
                return;
            }

            if (nest_->mTimer.expires_at() == boost::asio::steady_timer::time_point::min()) {
                BOOST_LOG(nest_->mLog) << "OpenDeviceOperation cancelled before settle delay";
                return;
            }

            nest_->mTimer.expires_from_now(settleDelay_);
            yield nest_->mTimer.async_wait(std::move(op));

            BOOST_LOG(nest_->mLog) << "Setting baud rate";
            util::asio::setSerialPortOptions(nest_->mSerialPort, 57600, ec);
            if (ec) {
                rc_ = ec;
                return;
            }

            if (nest_->mTimer.expires_at() == boost::asio::steady_timer::time_point::min()) {
                BOOST_LOG(nest_->mLog) << "OpenDeviceOperation cancelled before write delay";
                return;
            }
            nest_->mTimer.expires_from_now(writeDelay_);
            yield nest_->mTimer.async_wait(std::move(op));

            rc_ = Status::OK;
        }
        else if (boost::asio::error::operation_aborted != ec) {
            rc_ = ec;
            BOOST_LOG(nest_->mLog) << "OpenDeviceOperation: " << ec.message();
            nest_->close(ec);
        }
    }
};

template <class FlashProgress, class EepromProgress>
struct ProgrammerImpl::ProgramAllOperation {
    using Nest = ProgrammerImpl;

    ProgramAllOperation (std::shared_ptr<Nest> nest,
        uint32_t flashBase, boost::asio::const_buffer flash, FlashProgress&& flashProgress,
        uint32_t eepromBase, boost::asio::const_buffer eeprom, EepromProgress&& eepromProgress)
        : nest_(std::move(nest))
        , flashBase_(flashBase)
        , flash_(flash)
        , flashProgress_(std::forward<FlashProgress>(flashProgress))
        , eepromBase_(eepromBase)
        , eeprom_(eeprom)
        , eepromProgress_(std::forward<EepromProgress>(eepromProgress))
    {}

    std::shared_ptr<Nest> nest_;

    uint32_t flashBase_;
    boost::asio::const_buffer flash_;
    FlashProgress flashProgress_;

    uint32_t eepromBase_;
    boost::asio::const_buffer eeprom_;
    EepromProgress eepromProgress_;

    uint16_t pageSize_ = 0;

    boost::system::error_code rc_ = boost::asio::error::operation_aborted;

    auto result () const {
        return std::make_tuple(rc_);
    }

    template <class Op>
    void operator() (Op&& op, boost::system::error_code ec = {}, size_t n = 0) {
        if (!ec) reenter (op) {
            // Guarantee that the blobs we got passed will actually fit on a Linkbot
            if (blobTooBig()) {
                rc_ = Status::BLOB_TOO_BIG;
                return;
            }

            BOOST_LOG(nest_->mLog) << "Setting device parameters";
            yield nest_->asyncTransaction(
                boost::asio::buffer(detail::kSetDeviceMessage),
                detail::kSyncReply, std::move(op));
            if (!nest_->checkSyncReply(n, rc_)) return;

            BOOST_LOG(nest_->mLog) << "Setting extended device parameters";
            yield nest_->asyncTransaction(
                boost::asio::buffer(detail::kSetDeviceExtMessage),
                detail::kSyncReply, std::move(op));
            if (!nest_->checkSyncReply(n, rc_)) return;

            BOOST_LOG(nest_->mLog) << "Entering programming mode";
            yield nest_->asyncTransaction(
                boost::asio::buffer(detail::kEnterProgmodeMessage),
                detail::kSyncReply, std::move(op));
            if (!nest_->checkSyncReply(n, rc_)) return;

            BOOST_LOG(nest_->mLog) << "Reading signature";
            yield nest_->asyncTransaction(
                boost::asio::buffer(detail::kReadSignMessage),
                detail::kReadSignReply, std::move(op));
            if (!nest_->checkReadSignReply(n, pageSize_, rc_)) return;
            BOOST_LOG(nest_->mLog) << "Page size " << pageSize_ << " detected";

            BOOST_LOG(nest_->mLog) << "Programming flash";
            yield {
                // Guaranteed to be safe because we called blobTooBig() earlier
                auto txAddress = static_cast<uint16_t>(flashBase_ / 2);
                nest_->asyncProgramPages(
                    txAddress, pageSize_,
                    boost::asio::buffer(detail::kFlashType), flash_,
                    flashProgress_, std::move(op));
            }

            BOOST_LOG(nest_->mLog) << "Programming EEPROM";
            yield {
                // Guaranteed to be safe because we called blobTooBig() earlier
                auto txAddress = static_cast<uint16_t>(eepromBase_ / 2);
                nest_->asyncProgramPages(
                    txAddress, pageSize_,
                    boost::asio::buffer(detail::kEepromType), eeprom_,
                    eepromProgress_, std::move(op));
            }

            BOOST_LOG(nest_->mLog) << "Leaving programming mode";
            yield nest_->asyncTransaction(
                boost::asio::buffer(detail::kLeaveProgmodeMessage),
                detail::kSyncReply, std::move(op));
            if (!nest_->checkSyncReply(n, rc_)) return;

            rc_ = Status::OK;
        }
        else if (boost::asio::error::operation_aborted != ec) {
            rc_ = ec;
            BOOST_LOG(nest_->mLog) << "ProgramAllOperation: " << ec.message();
            nest_->close(ec);
        }
    }

    bool blobTooBig () const {
        auto flashMax = flashBase_ + boost::asio::buffer_size(flash_);
        auto eepromMax = eepromBase_ + boost::asio::buffer_size(eeprom_);
        auto maxAddress = std::numeric_limits<uint16_t>::max();
        return flashMax / 2 > maxAddress
            || eepromMax / 2 > maxAddress;
    }
};

template <class Progress>
struct ProgrammerImpl::ProgramPagesOperation {
    using Nest = ProgrammerImpl;

    ProgramPagesOperation (std::shared_ptr<Nest> nest,
        boost::endian::little_uint16_t txAddress, boost::endian::big_uint16_t pageSize,
        boost::asio::const_buffer type, boost::asio::const_buffer code, Progress&& progress)
        : nest_(std::move(nest))
        , txAddress_(txAddress)
        , pageSize_(pageSize)
        , type_(type)
        , code_(code)
        , progress_(std::forward<Progress>(progress))
    {}

    std::shared_ptr<Nest> nest_;

    // These two values get sent over the wire in differing endianness (wtf),
    // so it's most maintainable to just store them as such.
    boost::endian::little_uint16_t txAddress_;
    boost::endian::big_uint16_t pageSize_;

    boost::asio::const_buffer type_; // 'F' or 'E' for flash or EEPROM
    boost::asio::const_buffer code_;

    Progress progress_;

    boost::system::error_code rc_ = boost::asio::error::operation_aborted;

    std::tuple<boost::system::error_code> result () {
        return std::make_tuple(rc_);
    }

    template <class Op>
    void operator() (Op&& op, boost::system::error_code ec = {}, size_t n = 0) {
        if (!ec) reenter (op) {
            while (boost::asio::buffer_size(code_)) {
                yield {
                    auto message = detail::loadAddressMessage(
                        boost::asio::buffer(txAddress_.data(), 2));
                    nest_->asyncTransaction(message, detail::kSyncReply, std::move(op));
                }
                if (!nest_->checkSyncReply(n, rc_)) return;

                if (boost::asio::buffer_size(code_) < pageSize_) {
                    pageSize_ = static_cast<uint16_t>(boost::asio::buffer_size(code_));
                }
                yield {
                    auto message = detail::progPageMessage(
                        boost::asio::buffer(pageSize_.data(), 2),
                        type_,
                        boost::asio::buffer(code_, pageSize_));
                    nest_->asyncTransaction(message, detail::kSyncReply, std::move(op));
                }
                if (!nest_->checkSyncReply(n, rc_)) return;

                code_ = code_ + pageSize_;
                txAddress_ += pageSize_ / 2;

                // Update client code on progress, i.e., bytes written
                nest_->mContext.post(std::bind(progress_, pageSize_));
            }
            // TODO read back pages to verify?
            rc_ = Status::OK;
        }
        else if (boost::asio::error::operation_aborted != ec) {
            rc_ = ec;
            BOOST_LOG(nest_->mLog) << "ProgramPagesOperation error: " << ec.message();
            nest_->close(ec);
        }
    }
};

template <class ConstBufferSequence>
struct ProgrammerImpl::TransactionOperation {
    using Nest = ProgrammerImpl;

    TransactionOperation (std::shared_ptr<Nest> nest,
        unsigned maxAttempts, const ConstBufferSequence& outBuf,
        boost::regex re)
        : nest_(std::move(nest))
        , maxAttempts_(maxAttempts)
        , outBuf_(outBuf)
        , re_(re)
    {}

    std::shared_ptr<Nest> nest_;
    unsigned maxAttempts_;
    ConstBufferSequence outBuf_;
    boost::regex re_;

    unsigned attempts_ = 0;
    bool writeDone_ = false;

    boost::system::error_code rc_ = boost::asio::error::operation_aborted;
    size_t n_ = 0;

    std::tuple<boost::system::error_code, size_t> result () const {
        return std::make_tuple(rc_, n_);
    }

    template <class Op>
    void operator() (Op&& op, boost::system::error_code ec = {}, size_t n = 0) {
        if (!ec) reenter (op) {
            if (!maxAttempts_) {
                rc_ = Status::OK;
                n_ = 0;
                return;
            }
            // Spawn a child to continuously spawn write ops if the read op
            // takes forever. If the write op runs out of retries, the child
            // will close the serial port.
            fork Op{op}();
            if (op.is_child()) {
                while (rc_ && ++attempts_ <= maxAttempts_) {
                    writeDone_ = false;

                    // Spawn a child to close the serial port if the write op
                    // takes forever.
                    fork Op{op}();
                    if (op.is_child()) {
                        if (nest_->mTimer.expires_at() ==
                                boost::asio::steady_timer::time_point::min()) {
                            return;
                        }
                        nest_->mTimer.expires_from_now(nest_->transactionTimeout());
                        yield nest_->mTimer.async_wait(std::move(op));

                        // Rely on writeDone_ to close the serial port, in case
                        // the async_write and async_wait continuations were
                        // scheduled back-to-back.
                        if (!writeDone_) {
                            rc_ = Status::TIMEOUT;
                            BOOST_LOG(nest_->mLog) << "Transaction timed out on write";
                            nest_->close(ec);
                        }
                        return;
                    }

                    if (attempts_ > 1) {
                        BOOST_LOG(nest_->mLog) << "Retrying Transaction write attempt ("
                            << attempts_ << "/" << maxAttempts_ << ")";
                    }
                    yield boost::asio::async_write(nest_->mSerialPort, outBuf_, std::move(op));
                    writeDone_ = true;

                    if (nest_->mTimer.expires_at() ==
                            boost::asio::steady_timer::time_point::min()) {
                        return;
                    }
                    nest_->mTimer.expires_from_now(nest_->transactionTimeout());
                    yield nest_->mTimer.async_wait(std::move(op));
                }
                // Similar to writeDone_, we need to check the return code, in
                // case the async_read and async_wait continuations were
                // scheduled back-to-back.
                if (rc_) {
                    rc_ = Status::TIMEOUT;
                    BOOST_LOG(nest_->mLog) << "Transaction timed out on read";
                    nest_->mSerialPort.close();
                }
                return;
            }

            yield boost::asio::async_read_until(
                nest_->mSerialPort, nest_->mBuf, nest_->weReceive(re_), std::move(op));
            nest_->mTimer.cancel();
            rc_ = Status::OK;
            n_ = n;
        }
        else if (ec != boost::asio::error::operation_aborted) {
            rc_ = ec;
            BOOST_LOG(nest_->mLog) << "TransactionOperation error: " << ec.message();
            nest_->close(ec);
        }
    }
};

} // namespace stk

#include <boost/asio/unyield.hpp>

#endif
