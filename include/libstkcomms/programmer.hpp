#ifndef LIBSTKCOMMS_PROGRAMMER_HPP
#define LIBSTKCOMMS_PROGRAMMER_HPP

#include <libstkcomms/system_error.hpp>

#include <libstkcomms/detail/fullregexmatch.hpp>
#include <libstkcomms/detail/messages.hpp>

#include <composed/op.hpp>
#include <composed/timed.hpp>
#include <composed/handler_executor.hpp>

#include <util/log.hpp>

#include <beast/core/handler_alloc.hpp>

#include <boost/asio/buffer.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/write.hpp>

#include <boost/endian/arithmetic.hpp>

#include <boost/regex.hpp>

#include <chrono>
#include <limits>
#include <utility>

#include <sched.h>

#include <boost/asio/yield.hpp>

namespace stk {

template <class AsyncStream>
class Programmer {
    AsyncStream next_layer_;

    template <class FlashProgress, class EepromProgress, class Handler = void(boost::system::error_code)>
    struct ProgramAllOp;

    template <class Handler = void(boost::system::error_code)>
    struct SyncOp;

    template <class Progress, class Handler = void(boost::system::error_code)>
    struct ProgramPagesOp;

    template <class ConstBufferSequence, class Validator, class Handler = void(boost::system::error_code)>
    struct TransactionOp;

public:
    template <class... Args>
    explicit Programmer(Args&&... args): next_layer_(std::forward<Args>(args)...) {}

    boost::asio::io_service& get_io_service() { return next_layer_.get_io_service(); }
    AsyncStream& next_layer() { return next_layer_; }

    // Upload buffers of contiguous code and data to Flash and EEPROM on the device, each starting
    // at given base addresses.
    template <class FlashProgress, class EepromProgress, class Token>
    auto asyncProgramAll(
            uint32_t flashBase,
            boost::asio::const_buffer flash,
            FlashProgress&& flashProgress,
            uint32_t eepromBase,
            boost::asio::const_buffer eeprom,
            EepromProgress&& eepromProgress,
            Token&& token) {
        return composed::operation<ProgramAllOp<std::decay_t<FlashProgress>, std::decay_t<EepromProgress>>>{}(
                *this,
                flashBase, flash, std::forward<FlashProgress>(flashProgress),
                eepromBase, eeprom, std::forward<EepromProgress>(eepromProgress),
                std::forward<Token>(token));
    }

private:
    template <class Token>
    auto asyncSync(Token&& token) {
        return composed::operation<SyncOp<>>{}(*this, std::forward<Token>(token));
    }

    // Upload a buffer of contiguous code to the device, starting at txAddress.
    // txAddress must be a 2-byte word address (i.e., byte address / 2). The code
    // is uploaded in pages of pageSize bytes. type should be either a buffer with
    // 'F' for flash, or 'E' for EEPROM.
    template <class Progress, class Token>
    auto asyncProgramPages(
            uint16_t txAddress,
            uint16_t pageSize,
            boost::asio::const_buffer type,
            boost::asio::const_buffer code,
            Progress& progress,
            Token&& token) {
        return composed::operation<ProgramPagesOp<Progress>>{}(
                *this, txAddress, pageSize, type, code,
                progress, std::forward<Token>(token));
    }

    // Perform one write -> read transaction on the given serial port.
    template <class ConstBufferSequence, class Validator, class Token>
    auto asyncTransaction(
            const ConstBufferSequence& outBuf, boost::regex re, Validator&& validator,
            Token&& token) {
        return composed::operation<TransactionOp<ConstBufferSequence, std::decay_t<Validator>>>{}(
                *this, outBuf, std::move(re),
                std::forward<Validator>(validator), std::forward<Token>(token));
    }
};

template <class AsyncStream>
template <class FlashProgress, class EepromProgress, class Handler>
struct Programmer<AsyncStream>::ProgramAllOp: boost::asio::coroutine {
    using handler_type = Handler;
    using allocator_type = beast::handler_alloc<char, handler_type>;
    using executor_type = composed::handler_executor<handler_type>;

    using logger_type = composed::logger;
    logger_type get_logger() const { return &lg; }

    Programmer& self;

    uint32_t flashBase;
    boost::asio::const_buffer flash;
    FlashProgress flashProgress;
    uint32_t eepromBase;
    boost::asio::const_buffer eeprom;
    EepromProgress eepromProgress;

    uint16_t pageSize = 0;

    mutable util::log::Logger lg;
    boost::system::error_code ec;

    template <class DeducedFlashProgress, class DeducedEepromProgress>
    ProgramAllOp(handler_type& h, Programmer& s,
            uint32_t flBase, boost::asio::const_buffer fl, DeducedFlashProgress&& flProgress,
            uint32_t eeBase, boost::asio::const_buffer ee, DeducedEepromProgress&& eeProgress)
        : self(s)
        , flashBase(flBase)
        , flash(fl)
        , flashProgress(std::forward<DeducedFlashProgress>(flProgress))
        , eepromBase(eeBase)
        , eeprom(ee)
        , eepromProgress(std::forward<DeducedEepromProgress>(eeProgress))
        , lg(composed::get_associated_logger(h).clone())
    {
        lg.add_attribute("Protocol", boost::log::attributes::make_constant("STK"));
    }

    void operator()(composed::op<ProgramAllOp>&);
};

static inline yourBlobIsTooBig(uint32_t base, boost::asio::const_buffer data) {
    // The Linkbot bootloader can address up to 2^16 16-bit words.
    auto maxAddress = base + boost::asio::buffer_size(data);
    return maxAddress / 2 > std::numeric_limits<uint16_t>::max();
}

template <class AsyncStream>
template <class FlashProgress, class EepromProgress, class Handler>
void Programmer<AsyncStream>::ProgramAllOp<FlashProgress, EepromProgress, Handler>::
operator()(composed::op<ProgramAllOp>& op) {
    if (!ec) reenter(this) {
        // Guarantee that the blobs we got passed will actually fit on a Linkbot
        if (yourBlobIsTooBig(flashBase, flash) || yourBlobIsTooBig(eepromBase, eeprom)) {
            ec = Status::BLOB_TOO_BIG;
            self.next_layer_.get_io_service().post(op());
        }

        yield return self.asyncSync(op(ec));

        BOOST_LOG(lg) << "Setting device parameters";
        yield return self.asyncTransaction(
            boost::asio::buffer(detail::kSetDeviceMessage),
            detail::kSyncReply, detail::syncReplyValidator(), op(ec));

        BOOST_LOG(lg) << "Setting extended device parameters";
        yield return self.asyncTransaction(
            boost::asio::buffer(detail::kSetDeviceExtMessage),
            detail::kSyncReply, detail::syncReplyValidator(), op(ec));

        BOOST_LOG(lg) << "Entering programming mode";
        yield return self.asyncTransaction(
            boost::asio::buffer(detail::kEnterProgmodeMessage),
            detail::kSyncReply, detail::syncReplyValidator(), op(ec));

        BOOST_LOG(lg) << "Reading signature";
        yield return self.asyncTransaction(
            boost::asio::buffer(detail::kReadSignMessage),
            detail::kReadSignReply, detail::readSignReplyValidator(pageSize), op(ec));
        BOOST_LOG(lg) << "Page size " << pageSize << " detected";

        BOOST_LOG(lg) << "Programming flash";
        yield {
            // Guaranteed to be safe because we calculated blobTooBig earlier
            auto txAddress = static_cast<uint16_t>(flashBase / 2);
            return self.asyncProgramPages(
                txAddress, pageSize,
                boost::asio::buffer(detail::kFlashType), flash,
                flashProgress, op(ec));
        }

        BOOST_LOG(lg) << "Programming EEPROM";
        yield {
            // Guaranteed to be safe because we called blobTooBig() earlier
            auto txAddress = static_cast<uint16_t>(eepromBase / 2);
            return self.asyncProgramPages(
                txAddress, pageSize,
                boost::asio::buffer(detail::kEepromType), eeprom,
                eepromProgress, op(ec));
        }

        BOOST_LOG(lg) << "Leaving programming mode";
        yield return self.asyncTransaction(
            boost::asio::buffer(detail::kLeaveProgmodeMessage),
            detail::kSyncReply, detail::syncReplyValidator(), op(ec));
    }
    op.complete(ec);
}

template <class AsyncStream>
template <class Handler>
struct Programmer<AsyncStream>::SyncOp: boost::asio::coroutine {
    using handler_type = Handler;
    using allocator_type = beast::handler_alloc<char, handler_type>;
    using executor_type = composed::handler_executor<handler_type>;

    Programmer& self;

    composed::associated_logger_t<Handler> lg;
    boost::system::error_code ec;

    unsigned attempts = 0;

    SyncOp(handler_type& h, Programmer& s)
        : self(s)
        , lg(composed::get_associated_logger(h))
    {
    }

    void operator()(composed::op<SyncOp>&);
};

template <class AsyncStream>
template <class Handler>
void Programmer<AsyncStream>::SyncOp<Handler>::operator()(composed::op<SyncOp>& op) {
    constexpr unsigned kMaxAttempts = 3;

    // We don't check !ec, because this is simply a loop on asyncTransaction, retrying on timeout.
    // We complete the operation as soon as asyncTransaction succeeds or fails without a timeout.
    reenter(this) {
        do {
            BOOST_LOG(lg) << "Sync attempt (" << attempts << "/" << kMaxAttempts << ")";
            yield return self.asyncTransaction(
                    boost::asio::buffer(detail::kSyncMessage),
                    detail::kSyncReply,
                    detail::syncReplyValidator(),
                    op(ec));
        } while (++attempts < kMaxAttempts && ec == boost::asio::error::timed_out);
    }
    op.complete(ec);
}

template <class AsyncStream>
template <class Progress, class Handler>
struct Programmer<AsyncStream>::ProgramPagesOp: boost::asio::coroutine {
    using handler_type = Handler;
    using allocator_type = beast::handler_alloc<char, handler_type>;
    using executor_type = composed::handler_executor<handler_type>;

    Programmer& self;

    boost::endian::little_uint16_t txAddress;
    boost::endian::big_uint16_t pageSize;
    // txAddress and pageSize get sent over the wire in differing endianness (wtf),
    // so it's most maintainable to just store them as such.

    boost::asio::const_buffer type;
    boost::asio::const_buffer code;
    Progress& progress;

    composed::associated_logger_t<Handler> lg;
    boost::system::error_code ec;

    ProgramPagesOp(handler_type& h, Programmer& s,
            uint16_t txAddr,
            uint16_t pgSize,
            boost::asio::const_buffer t,
            boost::asio::const_buffer c,
            Progress& pr)
        : self(s)
        , txAddress(txAddr)
        , pageSize(pgSize)
        , type(t)
        , code(c)
        , progress(pr)
        , lg(composed::get_associated_logger(h))
    {
    }

    void operator()(composed::op<ProgramPagesOp>&);
};

template <class AsyncStream>
template <class Progress, class Handler>
void Programmer<AsyncStream>::ProgramPagesOp<Progress, Handler>::
operator()(composed::op<ProgramPagesOp>& op) {
    if (!ec) reenter(this) {
        while (boost::asio::buffer_size(code)) {
            yield {
                auto message = detail::loadAddressMessage(
                    boost::asio::buffer(txAddress.data(), 2));
                return self.asyncTransaction(message, detail::kSyncReply,
                        detail::syncReplyValidator(), op(ec));
            }

            if (boost::asio::buffer_size(code) < pageSize) {
                pageSize = static_cast<uint16_t>(boost::asio::buffer_size(code));
            }
            yield {
                auto message = detail::progPageMessage(
                    boost::asio::buffer(pageSize.data(), 2),
                    type,
                    boost::asio::buffer(code, pageSize));
                return self.asyncTransaction(message, detail::kSyncReply,
                        detail::syncReplyValidator(), op(ec));
            }

            code = code + pageSize;
            txAddress += pageSize / 2;

            // Update client code on progress, i.e., bytes written
            progress(static_cast<uint16_t>(pageSize));
        }
        // TODO read back pages to verify?
    }
    op.complete(ec);
}

template <class AsyncStream>
template <class ConstBufferSequence, class Validator, class Handler>
struct Programmer<AsyncStream>::TransactionOp: boost::asio::coroutine {
    using handler_type = Handler;
    using allocator_type = beast::handler_alloc<char, handler_type>;
    using executor_type = composed::handler_executor<handler_type>;

    Programmer& self;

    ConstBufferSequence outBuf;
    boost::regex re;
    Validator validator;

    detail::StreamBuf inBuf;
    boost::match_results<detail::StreamBufIter> what;
    bool readDone = false;
    bool writeDone = false;

    composed::associated_logger_t<Handler> lg;
    boost::system::error_code ec;

    size_t n;

    template <class DeducedValidator>
    TransactionOp(handler_type& h, Programmer& s,
            const ConstBufferSequence& b, boost::regex r, DeducedValidator&& v)
        : self(s)
        , outBuf(b)
        , re(std::move(r))
        , validator(std::forward<DeducedValidator>(v))
        , lg(composed::get_associated_logger(h))
    {
    }

    void operator()(composed::op<TransactionOp>&);
};

template <class AsyncStream>
template <class ConstBufferSequence, class Validator, class Handler>
void Programmer<AsyncStream>::TransactionOp<ConstBufferSequence, Validator, Handler>::
operator()(composed::op<TransactionOp>& op) {
    constexpr std::chrono::milliseconds kTransactionTimeout{500};

    BOOST_LOG(lg) << "asyncTransaction: reenter on CPU=[" << sched_getcpu()
            << "] ec=[" << ec.message() << "]";

    if (!ec) reenter(this) {
        BOOST_LOG(lg) << "asyncTransaction: writing";
        yield return boost::asio::async_write(self.next_layer_, outBuf,
                composed::timed(self.next_layer_, kTransactionTimeout, op(ec, std::ignore)));

        BOOST_LOG(lg) << "asyncTransaction: reading";
        yield {
            auto matchCond = detail::FullRegexMatch<detail::StreamBufIter>{what, re};
            return boost::asio::async_read_until(self.next_layer_, inBuf, matchCond,
                    composed::timed(self.next_layer_, kTransactionTimeout, op(ec, n)));
        }

        BOOST_LOG(lg) << "asyncTransaction: validating input";
        ec = Status::OK;  // ensure `ec` is 0 before calling `validator`
        validator(n, inBuf, what, ec);
        BOOST_LOG(lg) << "asyncTransaction: done";
    }
    op.complete(ec);
}

} // namespace stk

#include <boost/asio/unyield.hpp>

#endif
