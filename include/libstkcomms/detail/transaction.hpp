#ifndef LIBSTKCOMMS_DETAIL_TRANSACTION_HPP
#define LIBSTKCOMMS_DETAIL_TRANSACTION_HPP

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
        const ConstBufferSequence& outBuf, asio::streambuf& inBuf, MatchCondition matchCond)
        : sp_(sp)
        , timer_(timer)
        , outBuf_(outBuf)
        , inBuf_(inBuf)
        , matchCond_(matchCond)
    {}

    asio::serial_port& sp_;
    asio::steady_timer& timer_;
    ConstBufferSequence outBuf_;
    asio::streambuf& inBuf_;
    MatchCondition matchCond_;

    error_code rc_ = asio::error::operation_aborted;
    size_t n_;

    boost::log::sources::logger log_;

    std::tuple<error_code, size_t> result () const {
        return std::make_tuple(rc_, n_);
    }

    template <class Op>
    void operator() (Op&& op, error_code ec = {}, size_t n = 0) {
        if (!ec) reenter (op) {
            yield async_write(sp_, outBuf_, move(op));
            yield async_read_until(sp_, inBuf_, matchCond_, move(op));

            rc_ = Status::OK;
            n_ = n;
        }
        else if (ec != asio::error::operation_aborted) {
            BOOST_LOG(log_) << "Transaction I/O error: " << ec.message();
            sp_.close();
            rc_ = ec;
        }
    }
};

template <class ConstBufferSequence, class MatchCondition, class Handler>
BOOST_ASIO_INITFN_RESULT_TYPE(Handler, void(error_code, size_t))
asyncTransaction (asio::serial_port& sp,
    asio::steady_timer& timer,
    const ConstBufferSequence& outBuf,
    asio::streambuf& inBuf,
    MatchCondition matchCond,
    Handler&& handler)
{
    asio::detail::async_result_init<
        Handler, void(error_code, size_t)
    > init { std::forward<Handler>(handler) };

    using Op = Transaction<ConstBufferSequence, MatchCondition>;
    makeOperation<Op>(move(init.handler), sp, timer,
        outBuf, inBuf, matchCond)();

    return init.result.get();
}

} // detail
} // stk

#include <boost/asio/unyield.hpp>

#endif