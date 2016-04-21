#ifndef LIBSTKCOMMS_DETAIL_TRANSACTION_HPP
#define LIBSTKCOMMS_DETAIL_TRANSACTION_HPP

#include <util/asio/asynccompletion.hpp>

#include <libstkcomms/detail/prefix.hpp>

#include <boost/asio/async_result.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/steady_timer.hpp>

#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

#include <boost/asio/yield.hpp>

namespace stk {
namespace detail {

template <class ConstBufferSequence, class MatchCondition>
struct Transaction {
    Transaction (asio::serial_port& sp, asio::steady_timer& timer,
        unsigned maxAttempts, const ConstBufferSequence& outBuf,
        asio::streambuf& inBuf, MatchCondition matchCond)
        : sp_(sp)
        , timer_(timer)
        , maxAttempts_(maxAttempts)
        , outBuf_(outBuf)
        , inBuf_(inBuf)
        , matchCond_(matchCond)
    {}

    asio::serial_port& sp_;
    asio::steady_timer& timer_;
    unsigned maxAttempts_;
    ConstBufferSequence outBuf_;
    asio::streambuf& inBuf_;
    MatchCondition matchCond_;

    unsigned attempts_ = 0;
    bool writeDone_ = false;

    error_code rc_ = asio::error::operation_aborted;
    size_t n_ = 0;

    boost::log::sources::logger log_;

    std::tuple<error_code, size_t> result () const {
        return std::make_tuple(rc_, n_);
    }

    template <class Op>
    void operator() (Op&& op, error_code ec = {}, size_t n = 0) {
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
                        if (timer_.expires_at() == asio::steady_timer::time_point::min()) {
                            return;
                        }
                        timer_.expires_from_now(kRetryTimeout);
                        yield timer_.async_wait(move(op));

                        // Rely on writeDone_ to close the serial port, in case
                        // the async_write and async_wait continuations were
                        // scheduled back-to-back.
                        if (!writeDone_) {
                            BOOST_LOG(log_) << "Transaction timed out on write";
                            sp_.close();
                            rc_ = Status::TIMEOUT;
                        }
                        return;
                    }

                    if (attempts_ > 1) {
                        BOOST_LOG(log_) << "Retrying Transaction write attempt (" << attempts_ << "/" << maxAttempts_ << ")";
                    }
                    yield async_write(sp_, outBuf_, move(op));
                    writeDone_ = true;

                    if (timer_.expires_at() == asio::steady_timer::time_point::min()) {
                        return;
                    }
                    timer_.expires_from_now(kRetryTimeout);
                    yield timer_.async_wait(move(op));
                }
                // Similar to writeDone_, we need to check the return code, in
                // case the async_read and async_wait continuations were
                // scheduled back-to-back.
                if (rc_) {
                    BOOST_LOG(log_) << "Transaction timed out on read";
                    sp_.close();
                    rc_ = Status::TIMEOUT;
                }
                return;
            }

            yield async_read_until(sp_, inBuf_, matchCond_, move(op));
            timer_.cancel();
            rc_ = Status::OK;
            n_ = n;
        }
        else if (ec != asio::error::operation_aborted) {
            BOOST_LOG(log_) << "Transaction I/O error: " << ec.message();
            timer_.expires_at(asio::steady_timer::time_point::min());
            sp_.close();
            rc_ = ec;
        }
    }
};

// Perform one write -> read transaction on the given serial port. The given
// timer is used to ensure that the write and read operations don't take too
// long. If the write takes too long, the serial port is simply closed. If the
// read operation takes too long, the write operation is retried, up to a
// maximum of maxAttempts retries. If zero is passed as maxAttempts,
// asyncTransaction is a no-op.
template <class ConstBufferSequence, class MatchCondition, class Handler>
BOOST_ASIO_INITFN_RESULT_TYPE(Handler, void(error_code, size_t))
asyncTransaction (asio::serial_port& sp,
    asio::steady_timer& timer,
    unsigned maxAttempts,
    const ConstBufferSequence& outBuf,
    asio::streambuf& inBuf,
    MatchCondition matchCond,
    Handler&& handler)
{
    util::asio::AsyncCompletion<
        Handler, void(error_code, size_t)
    > init { std::forward<Handler>(handler) };

    using Op = Transaction<ConstBufferSequence, MatchCondition>;
    makeOperation<Op>(move(init.handler), sp, timer,
        maxAttempts, outBuf,
        inBuf, matchCond)();

    return init.result.get();
}

// Same as above, with a default 1 maxAttempts.
template <class ConstBufferSequence, class MatchCondition, class Handler>
BOOST_ASIO_INITFN_RESULT_TYPE(Handler, void(error_code, size_t))
asyncTransaction (asio::serial_port& sp,
    asio::steady_timer& timer,
    const ConstBufferSequence& outBuf,
    asio::streambuf& inBuf,
    MatchCondition matchCond,
    Handler&& handler)
{
    return asyncTransaction(sp, timer, 1, outBuf, inBuf, matchCond, std::forward<Handler>(handler));
}

} // detail
} // stk

#include <boost/asio/unyield.hpp>

#endif
