#include <libstkcomms/asyncstk.hpp>

#include <usbcdc/devices.hpp>

#include <util/setserialportoptions.hpp>
#include <util/iothread.hpp>

namespace asio = boost::asio;

int main () try {
    boost::log::sources::logger lg;

    util::IoThread io;

    auto sp = asio::serial_port{io.context()};
    BOOST_LOG(lg) << "Waiting for USB CDC device";
    while (!sp.is_open()) {
        for (auto d : usbcdc::devices()) {
            BOOST_LOG(lg) << "Opening " << d.productString() << " at " << d.path();
            try {
                sp.open(d.path());
                std::this_thread::sleep_for(util::kSerialSettleTimeAfterOpen);
                util::setSerialPortOptions(sp, 57600);
                break;
            }
            catch (std::exception& e) {
                BOOST_LOG(lg) << "Failed: " << e.what();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    BOOST_LOG(lg) << "Programming";
    stk::asyncProgramAll(sp, [=] (stk::error_code ec) mutable {
        BOOST_LOG(lg) << "Programming complete: " << ec.message();
    });

    BOOST_LOG(lg) << "IO thread ran " << io.join() << " handlers";
}
catch (std::exception& e) {
    boost::log::sources::logger lg;
    BOOST_LOG(lg) << "Exception in main: " << e.what();
}