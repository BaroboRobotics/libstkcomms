#ifndef LIBSTKCOMMS_PROGRAMMER_HPP
#define LIBSTKCOMMS_PROGRAMMER_HPP

#include <libstkcomms/asyncstk.hpp>

#include <util/iothread.hpp>

#include <utility>

namespace stk {

typedef void ProgramAllHandlerSignature(error_code);
    
class ProgrammerImpl : public std::enable_shared_from_this<ProgrammerImpl> {
public:
    explicit ProgrammerImpl (asio::io_service& ios)
        : mContext(ios)
        , mTimer(mContext)
        , mSerialPort(mContext)
    {}

    void close (error_code& ec) {
        auto self = this->shared_from_this();
        mContext.post([self, this, ec]() mutable {
            mTimer.expires_at(decltype(mTimer)::time_point::min());
            mSerialPort.close(ec);
        });
    }

    template <class FlashProgress, class EepromProgress, class CompletionToken>
    BOOST_ASIO_INITFN_RESULT_TYPE(CompletionToken, ProgramAllHandlerSignature)
    asyncProgramAll (asio::io_service::work work,
        const std::string& path,
        uint32_t flashBase,
        const_buffer flash,
        FlashProgress&& flashProgress,
        uint32_t eepromBase,
        const_buffer eeprom,
        EepromProgress&& eepromProgress,
        CompletionToken&& token)
{
        asio::detail::async_result_init<
            CompletionToken, ProgramAllHandlerSignature
        > init { std::forward<CompletionToken>(token) };

        auto self = this->shared_from_this();
        asyncProgramAllImpl(mSerialPort, path, mTimer,
            flashBase, flash,
            std::bind(&ProgrammerImpl::callProgress<FlashProgress>, self, work, flashProgress, _1),
            eepromBase, eeprom,
            std::bind(&ProgrammerImpl::callProgress<EepromProgress>, self, work, eepromProgress, _1),
            std::bind(&ProgrammerImpl::callHandler<decltype(init.handler)>, self, work, init.handler, _1));

        return init.result.get();
    }

private:
    template <class Progress>
    void callProgress (asio::io_service::work work, Progress progress, uint16_t n) {
        work.get_io_service().post(std::bind(progress, n));
    }

    template <class Handler>
    void callHandler (asio::io_service::work work, Handler handler, error_code ec) {
        work.get_io_service().post(std::bind(handler, ec));
    }

    asio::io_service& mContext;
    asio::steady_timer mTimer;
    asio::serial_port mSerialPort;
};

template <class Impl>
class ProgrammerService : public boost::asio::io_service::service {
public:
    static boost::asio::io_service::id id;

    explicit ProgrammerService (boost::asio::io_service& ios)
        : boost::asio::io_service::service(ios)
    {}

    ~ProgrammerService () {
        mIoThread.join();
    }

    using implementation_type = std::shared_ptr<Impl>;

    void construct (implementation_type& impl) {
        impl = std::make_shared<Impl>(mIoThread.context());
    }

    void move_construct (implementation_type& impl, implementation_type& other) {
        impl = std::move(other);
    }

    void destroy (implementation_type& impl) {
        auto ec = error_code{};
        close(impl, ec);
        impl.reset();
    }

    void close (implementation_type& impl, boost::system::error_code& ec) {
        impl->close(ec);
    }

#if 0
    template <class Handler>
    BOOST_ASIO_INITFN_RESULT_TYPE(Handler, ProgramAllHandlerSignature)
    asyncProgramAll (implementation_type& impl, Handler&& handler) {
        boost::asio::io_service::work work { this->get_io_service() };
        return impl->asyncProgramAll(work, std::forward<Handler>(handler));
    }
#endif

    template <class... Args>
    auto asyncProgramAll (implementation_type& impl, Args&&... args) -> decltype(
        impl->asyncProgramAll(std::declval<asio::io_service::work>(), std::forward<Args>(args)...))
    {
        asio::io_service::work work { this->get_io_service() };
        return impl->asyncProgramAll(work, std::forward<Args>(args)...);
    }

private:
    void shutdown_service () {}
    util::IoThread mIoThread;
};

template <class Impl>
boost::asio::io_service::id ProgrammerService<Impl>::id;

template <class Service>
class BasicProgrammer : public asio::basic_io_object<Service> {
public:
    explicit BasicProgrammer (asio::io_service& ios)
        : boost::asio::basic_io_object<Service>(ios)
    {}

    BasicProgrammer (const BasicProgrammer&) = delete;
    BasicProgrammer& operator= (const BasicProgrammer&) = delete;

#if 0
    BasicProgrammer (BasicProgrammer&&) = default;
    BasicProgrammer& operator= (BasicProgrammer&&) = default;
#endif

    void close () {
        error_code ec;
        close(ec);
        if (ec) {
            throw boost::system::system_error(ec);
        }
    }

    void close (boost::system::error_code& ec) {
        this->get_service().close(this->get_implementation(), ec);
    }

    template <class... Args>
    auto asyncProgramAll (Args&&... args) -> decltype(
        std::declval<BasicProgrammer>().get_service().asyncProgramAll(
            std::declval<BasicProgrammer>().get_implementation(),
            std::forward<Args>(args)...))
    {
        return this->get_service().asyncProgramAll(this->get_implementation(),
            std::forward<Args>(args)...);
    }
};

using Programmer = BasicProgrammer<ProgrammerService<ProgrammerImpl>>;

} // namespace stk

#endif
