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

#include <boost/asio/yield.hpp>

namespace stk {

template <class AsyncStream, class Token>
auto asyncSync(AsyncStream& stream, Token&& token);
// Synchronize with the STK-speaking bootloader on the given `stream`. This operation results in
// boost::asio::error::timed_out if the bootloader takes longer than a hard-coded timeout.

template <class AsyncStream, class FlashProgress, class EepromProgress, class Token>
auto asyncProgramAll(AsyncStream& stream,
        uint32_t flashBase, boost::asio::const_buffer flash, FlashProgress&& flashProgress,
        uint32_t eepromBase, boost::asio::const_buffer eeprom, EepromProgress&& eepromProgress,
        Token&& token);
// Update the firmware on a Barobo Linkbot or Z-Link dongle attached via the given stream.
//
// This function uploads buffers of contiguous code and data to Flash and EEPROM on the device, each
// starting at given base addresses.
//
// This function will fail with Status::BLOB_TOO_BIG if the sum of the base address and the size of
// the code to load are greater than 128k.
//
// The two progress callbacks will be called for every successful page write. They will be passed
// the size of the most recently written page. If every page write is successful, the sum of these
// arguments will equal the size of the `flash` and `eeprom` buffers, respectively.

// =======================================================================================
// Inline implementation

namespace _ {

static inline yourBlobIsTooBig(uint32_t base, boost::asio::const_buffer data) {
    // The Linkbot bootloader can address up to 2^16 16-bit words.
    auto maxAddress = base + boost::asio::buffer_size(data);
    return maxAddress / 2 > std::numeric_limits<uint16_t>::max();
}

// =======================================================================================
// Transaction op implementation

template <class AsyncStream, class ConstBufferSequence, class Validator,
          class Handler = void(boost::system::error_code)>
struct TransactionOp: boost::asio::coroutine {
    using handler_type = Handler;
    using allocator_type = beast::handler_alloc<char, handler_type>;
    using executor_type = composed::handler_executor<handler_type>;

    AsyncStream& stream;

    ConstBufferSequence outBuf;
    boost::regex re;
    Validator validator;

    StreamBuf inBuf;
    boost::match_results<StreamBufIter> what;
    bool readDone = false;
    bool writeDone = false;

    composed::associated_logger_t<Handler> lg;
    boost::system::error_code ec;

    size_t n;

    template <class DeducedValidator>
    TransactionOp(handler_type& h, AsyncStream& s,
            const ConstBufferSequence& b, boost::regex r, DeducedValidator&& v)
        : stream(s)
        , outBuf(b)
        , re(std::move(r))
        , validator(std::forward<DeducedValidator>(v))
        , lg(composed::get_associated_logger(h))
    {}

    void operator()(composed::op<TransactionOp>&);
};

template <class AsyncStream, class ConstBufferSequence, class Validator, class Handler>
void TransactionOp<AsyncStream, ConstBufferSequence, Validator, Handler>::
operator()(composed::op<TransactionOp>& op) {
    constexpr std::chrono::milliseconds kTransactionTimeout{500};

    if (!ec) reenter(this) {
        yield return boost::asio::async_write(stream, outBuf,
                composed::timed(stream, kTransactionTimeout, op(ec, std::ignore)));

        yield {
            auto matchCond = FullRegexMatch<StreamBufIter>{what, re};
            return boost::asio::async_read_until(stream, inBuf, matchCond,
                    composed::timed(stream, kTransactionTimeout, op(ec, n)));
        }

        ec = Status::OK;  // ensure `ec` is 0 before calling `validator`
        validator(n, inBuf, what, ec);
    }
    op.complete(ec);
}

// Perform one write -> read transaction on the given serial port.
template <class AsyncStream, class ConstBufferSequence, class Validator, class Token>
auto asyncTransaction(AsyncStream& stream,
        const ConstBufferSequence& outBuf, boost::regex re, Validator&& validator,
        Token&& token) {
    using Op = TransactionOp<AsyncStream, ConstBufferSequence, std::decay_t<Validator>>;
    return composed::operation<Op>{}(
            stream, outBuf, std::move(re),
            std::forward<Validator>(validator), std::forward<Token>(token));
}

// =======================================================================================
// Program pages op implementation

template <class AsyncStream, class Progress, class Handler = void(boost::system::error_code)>
struct ProgramPagesOp: boost::asio::coroutine {
    using handler_type = Handler;
    using allocator_type = beast::handler_alloc<char, handler_type>;
    using executor_type = composed::handler_executor<handler_type>;

    AsyncStream& stream;

    boost::endian::little_uint16_t txAddress;
    boost::endian::big_uint16_t pageSize;
    // txAddress and pageSize get sent over the wire in differing endianness (wtf), so it's most
    // maintainable to just store them as such.

    boost::asio::const_buffer type;
    boost::asio::const_buffer code;
    Progress& progress;

    composed::associated_logger_t<Handler> lg;
    boost::system::error_code ec;

    ProgramPagesOp(handler_type& h, AsyncStream& s,
            uint16_t txAddr,
            uint16_t pgSize,
            boost::asio::const_buffer t,
            boost::asio::const_buffer c,
            Progress& pr)
        : stream(s)
        , txAddress(txAddr)
        , pageSize(pgSize)
        , type(t)
        , code(c)
        , progress(pr)
        , lg(composed::get_associated_logger(h))
    {}

    void operator()(composed::op<ProgramPagesOp>&);
};

template <class AsyncStream, class Progress, class Handler>
void ProgramPagesOp<AsyncStream, Progress, Handler>::
operator()(composed::op<ProgramPagesOp>& op) {
    if (!ec) reenter(this) {
        while (boost::asio::buffer_size(code)) {
            yield {
                auto message = loadAddressMessage(
                    boost::asio::buffer(txAddress.data(), 2));
                return asyncTransaction(stream, message, kSyncReply,
                        syncReplyValidator(), op(ec));
            }

            if (boost::asio::buffer_size(code) < pageSize) {
                pageSize = static_cast<uint16_t>(boost::asio::buffer_size(code));
            }
            yield {
                auto message = progPageMessage(
                    boost::asio::buffer(pageSize.data(), 2),
                    type,
                    boost::asio::buffer(code, pageSize));
                return asyncTransaction(stream, message, kSyncReply,
                        syncReplyValidator(), op(ec));
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

// Upload a buffer of contiguous code to the device, starting at txAddress.
// txAddress must be a 2-byte word address (i.e., byte address / 2). The code
// is uploaded in pages of pageSize bytes. type should be either a buffer with
// 'F' for flash, or 'E' for EEPROM.
template <class AsyncStream, class Progress, class Token>
auto asyncProgramPages(AsyncStream& stream,
        uint16_t txAddress,
        uint16_t pageSize,
        boost::asio::const_buffer type,
        boost::asio::const_buffer code,
        Progress& progress,
        Token&& token) {
    return composed::operation<ProgramPagesOp<AsyncStream, Progress>>{}(
            stream, txAddress, pageSize, type, code,
            progress, std::forward<Token>(token));
}

// =======================================================================================
// Program all op implementation

template <class AsyncStream, class FlashProgress, class EepromProgress,
          class Handler = void(boost::system::error_code)>
struct ProgramAllOp: boost::asio::coroutine {
    using handler_type = Handler;
    using allocator_type = beast::handler_alloc<char, handler_type>;
    using executor_type = composed::handler_executor<handler_type>;

    using logger_type = composed::logger;
    logger_type get_logger() const { return &lg; }

    AsyncStream& stream;

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
    ProgramAllOp(handler_type& h, AsyncStream& s,
            uint32_t flBase, boost::asio::const_buffer fl, DeducedFlashProgress&& flProgress,
            uint32_t eeBase, boost::asio::const_buffer ee, DeducedEepromProgress&& eeProgress)
        : stream(s)
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

template <class AsyncStream, class FlashProgress, class EepromProgress, class Handler>
void ProgramAllOp<AsyncStream, FlashProgress, EepromProgress, Handler>::
operator()(composed::op<ProgramAllOp>& op) {
    if (!ec) reenter(this) {
        // Guarantee that the blobs we got passed will actually fit on a Linkbot
        if (yourBlobIsTooBig(flashBase, flash) || yourBlobIsTooBig(eepromBase, eeprom)) {
            ec = Status::BLOB_TOO_BIG;
            yield return stream.get_io_service().post(op());
        }

        BOOST_LOG(lg) << "Setting device parameters";
        yield return asyncTransaction(stream,
            boost::asio::buffer(kSetDeviceMessage),
            kSyncReply, syncReplyValidator(), op(ec));

        BOOST_LOG(lg) << "Setting extended device parameters";
        yield return asyncTransaction(stream,
            boost::asio::buffer(kSetDeviceExtMessage),
            kSyncReply, syncReplyValidator(), op(ec));

        BOOST_LOG(lg) << "Entering programming mode";
        yield return asyncTransaction(stream,
            boost::asio::buffer(kEnterProgmodeMessage),
            kSyncReply, syncReplyValidator(), op(ec));

        BOOST_LOG(lg) << "Reading signature";
        yield return asyncTransaction(stream,
            boost::asio::buffer(kReadSignMessage),
            kReadSignReply, readSignReplyValidator(pageSize), op(ec));
        BOOST_LOG(lg) << "Page size " << pageSize << " detected";

        BOOST_LOG(lg) << "Programming flash";
        yield {
            // Guaranteed to be safe because we calculated blobTooBig earlier
            auto txAddress = static_cast<uint16_t>(flashBase / 2);
            return asyncProgramPages(stream,
                txAddress, pageSize,
                boost::asio::buffer(kFlashType), flash,
                flashProgress, op(ec));
        }

        BOOST_LOG(lg) << "Programming EEPROM";
        yield {
            // Guaranteed to be safe because we called blobTooBig() earlier
            auto txAddress = static_cast<uint16_t>(eepromBase / 2);
            return asyncProgramPages(stream,
                txAddress, pageSize,
                boost::asio::buffer(kEepromType), eeprom,
                eepromProgress, op(ec));
        }

        BOOST_LOG(lg) << "Leaving programming mode";
        yield return asyncTransaction(stream,
            boost::asio::buffer(kLeaveProgmodeMessage),
            kSyncReply, syncReplyValidator(), op(ec));
    }
    op.complete(ec);
}

} // _

template <class AsyncStream, class Token>
auto asyncSync(AsyncStream& stream, Token&& token) {
    return _::asyncTransaction(stream,
            boost::asio::buffer(_::kSyncMessage), _::kSyncReply, _::syncReplyValidator(),
            std::forward<Token>(token));
}

template <class AsyncStream, class FlashProgress, class EepromProgress, class Token>
auto asyncProgramAll(AsyncStream& stream,
        uint32_t flashBase, boost::asio::const_buffer flash, FlashProgress&& flashProgress,
        uint32_t eepromBase, boost::asio::const_buffer eeprom, EepromProgress&& eepromProgress,
        Token&& token) {
    using Op = _::ProgramAllOp<
            AsyncStream, std::decay_t<FlashProgress>, std::decay_t<EepromProgress>>;
    return composed::operation<Op>{}(stream,
            flashBase, flash, std::forward<FlashProgress>(flashProgress),
            eepromBase, eeprom, std::forward<EepromProgress>(eepromProgress),
            std::forward<Token>(token));
}

} // stk

#include <boost/asio/unyield.hpp>

#endif
