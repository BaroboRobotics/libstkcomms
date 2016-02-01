#include <libstkcomms/asyncstk.hpp>

#include <usbcdc/devices.hpp>

#include <util/blob.hpp>
#include <util/iothread.hpp>
#include <util/readfile.hpp>
#include <util/setserialportoptions.hpp>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include <sstream>

namespace asio = boost::asio;
namespace fs = boost::filesystem;

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
        if (!sp.is_open()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    auto progressBytes = size_t(0);
    auto totalBytes = size_t(flash.code().size() + eeprom.code().size());
    auto progress = [lg, &progressBytes, totalBytes](uint16_t n) mutable {
        progressBytes += n;
        BOOST_LOG(lg) << "Progress: " << double(progressBytes) / double(totalBytes);
    };

    BOOST_LOG(lg) << "Programming";
    stk::asyncProgramAll(sp,
        flash.address(), asio::buffer(flash.code()),
        progress,
        eeprom.address(), asio::buffer(eeprom.code()),
        progress,
        [=] (stk::error_code ec) mutable
    {
        BOOST_LOG(lg) << "Programming complete: " << ec.message();
    });

    BOOST_LOG(lg) << "IO thread ran " << io.join() << " handlers";
}
catch (std::exception& e) {
    boost::log::sources::logger lg;
    BOOST_LOG(lg) << "Exception in main: " << e.what();
}