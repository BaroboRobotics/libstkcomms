#include <libstkcomms/programmer.hpp>

#include <usbcdc/devices.hpp>

#include <util/blob.hpp>
#include <util/asio/iothread.hpp>
#include <util/readfile.hpp>

#include <boost/asio/signal_set.hpp>
#include <boost/asio/use_future.hpp>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include <sstream>

namespace asio = boost::asio;
namespace fs = boost::filesystem;
using boost::system::error_code;

int main (int argc, char** argv) try {
    boost::log::sources::logger lg;

    if (argc < 3) {
        BOOST_LOG(lg) << "Usage: " << argv[0] << " <file.hex> <file.eeprom>\n";
        return 1;
    }

    auto flashName = fs::path{argv[1]};
    auto eepromName = fs::path{argv[2]};
    if (flashName.extension() != ".hex") { swap(flashName, eepromName); }
    if (flashName.extension() != ".hex" || eepromName.extension() != ".eeprom") {
        BOOST_LOG(lg) << "One argument must have a .hex extension and the "
            << "other must have a .eeprom extension";
        return 1;
    }

    BOOST_LOG(lg) << "Loading flash blob";
    auto flash = util::makeBlobFromIntelHex(util::readFile(flashName));
    BOOST_LOG(lg) << "Flash is " << flash.code().size()
        << " bytes starting at address " << flash.address();

    BOOST_LOG(lg) << "Loading EEPROM blob";
    auto eeprom = util::makeBlobFromIntelHex(util::readFile(eepromName));
    BOOST_LOG(lg) << "EEPROM is " << eeprom.code().size()
        << " bytes starting at address " << eeprom.address();

    util::asio::IoThread io;

    BOOST_LOG(lg) << "Waiting for USB CDC device";
    auto done = false;
    while (!done) {
        for (auto&& d : usbcdc::devices()) {
            BOOST_LOG(lg) << "Opening " << d.productString() << " at " << d.path();
            auto progressBytes = size_t(0);
            auto totalBytes = size_t(flash.code().size() + eeprom.code().size());
            auto progress = [lg, &progressBytes, totalBytes](uint16_t n) mutable {
                progressBytes += n;
                BOOST_LOG(lg) << "Progress: " << double(progressBytes) / double(totalBytes);
            };

            stk::Programmer programmer{io.context()};
            asio::signal_set sigSet{io.context(), SIGTERM, SIGINT};
            sigSet.async_wait([&done, &programmer, lg](error_code ec, int sigNo) mutable {
                if (!ec) {
                    programmer.close(ec);
                    done = true;
                    BOOST_LOG(lg) << "Signal number: " << sigNo
                        << ", closing programmer resulted in " << ec.message();
                }
            });
            try {
                programmer.asyncProgramAll(d.path(),
                    flash.address(), asio::buffer(flash.code()),
                    progress,
                    eeprom.address(), asio::buffer(eeprom.code()),
                    progress,
                    asio::use_future).get();
                BOOST_LOG(lg) << "Programming successful";
                done = true;
            }
            catch (std::exception& e) {
                BOOST_LOG(lg) << "Failed: " << e.what();
            }
            sigSet.cancel();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    BOOST_LOG(lg) << "IO thread ran " << io.join() << " handlers";
}
catch (std::exception& e) {
    boost::log::sources::logger lg;
    BOOST_LOG(lg) << "Exception in main: " << e.what();
}
