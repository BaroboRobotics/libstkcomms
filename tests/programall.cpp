#include <libstkcomms/asyncstk.hpp>

#include <usbcdc/devices.hpp>

#include <util/blob.hpp>
#include <util/iothread.hpp>
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
    auto ss = std::stringstream{};
    fs::ifstream flashFile{flashName};
    ss << flashFile.rdbuf();
    auto flashString = ss.str();
    auto flash = util::makeBlobFromIntelHex(flashString.data(), flashString.data() + flashString.size());
    BOOST_LOG(lg) << "Flash is " << flash.code().size()
        << " bytes starting at address " << flash.address();

    BOOST_LOG(lg) << "Loading EEPROM blob";
    ss = std::stringstream{};
    fs::ifstream eepromFile{eepromName};
    ss << eepromFile.rdbuf();
    auto eepromString = ss.str();
    auto eeprom = util::makeBlobFromIntelHex(eepromString.data(), eepromString.data() + eepromString.size());
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
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    BOOST_LOG(lg) << "Programming";
    stk::asyncProgramAll(sp, flash.address(), asio::buffer(flash.code()),
        eeprom.address(), asio::buffer(eeprom.code()),
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