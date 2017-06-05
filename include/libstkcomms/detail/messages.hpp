#ifndef LIBSTKCOMMS_MESSAGES_HPP
#define LIBSTKCOMMS_MESSAGES_HPP

#include <libstkcomms/detail/command.h>

#include <boost/regex.hpp>

#include <boost/asio/buffer.hpp>
#include <boost/asio/streambuf.hpp>

#include <array>
#include <string>

#include <cstdint>

namespace stk { namespace _ { namespace {

using StreamBuf = boost::asio::streambuf;
using StreamBufIter = boost::asio::buffers_iterator<StreamBuf::const_buffers_type>;

using UString = std::basic_string<uint8_t>;
const UString kMobotILSignature { 0x1e, 0xa7, 0x01 };
const UString kMobotASignature { 0x1e, 0x95, 0x0f };

const uint8_t kSyncMessage [] = { Cmnd_STK_GET_SYNC, Sync_CRC_EOP };
const boost::regex kSyncReply {"\x14\x10"};

inline auto syncReplyValidator () {
    return [](size_t n, StreamBuf& buf,
            const boost::match_results<StreamBufIter>& what, boost::system::error_code& ec) {
        if (what[0].matched) {
            buf.consume(n);
        }
        else {
            ec = Status::PROTOCOL_ERROR;
        }
    };
}

const uint8_t kSetDeviceMessage [] = {
    Cmnd_STK_SET_DEVICE,
    /*DeviceCode*/     0x86,
    /*Revision*/       0x00,
    /*progtype*/       0x00,
    /*parmode*/        0x01,
    /*polling*/        0x01,
    /*selftimed*/      0x01,
    /*lockbytes*/      0x01,
    /*fusebytes*/      0x03,
    /*flashpollval1*/  0xff,
    /*flashpollval2*/  0xff,
    /*eeprompollval1*/ 0xff,
    /*eeprompollval2*/ 0xff,
    /*pagesizehigh*/   0x00,
    /*pagesizelow*/    0x80,
    /*eepromsizehigh*/ 0x04,
    /*eepromsizelow*/  0x00,
    /*flashsize4*/     0x00,
    /*flashsize3*/     0x00,
    /*flashsize2*/     0x80,
    /*flashsize1*/     0x00,
    Sync_CRC_EOP
};

const uint8_t kSetDeviceExtMessage [] = {
    Cmnd_STK_SET_DEVICE_EXT,
    /*commandsize*/    0x05,
    /*eeprompagesize*/ 0x04,
    /*signalpagel*/    0xd7,
    /*signalbs2*/      0xc2,
    /*resetdisable*/   0x00,
    Sync_CRC_EOP
};

const uint8_t kEnterProgmodeMessage [] = { Cmnd_STK_ENTER_PROGMODE, Sync_CRC_EOP };
const uint8_t kLeaveProgmodeMessage [] = { Cmnd_STK_LEAVE_PROGMODE, Sync_CRC_EOP };

const uint8_t kReadSignMessage [] = { Cmnd_STK_READ_SIGN, Sync_CRC_EOP };
const boost::regex kReadSignReply {"\x14(.{3})\x10"};

inline auto readSignReplyValidator (uint16_t& pageSize) {
    return [&pageSize](size_t n, StreamBuf& buf,
            const boost::match_results<StreamBufIter>& what, boost::system::error_code& ec) {
        if (what[0].matched) {
            const auto& sigMatch = what[1].str();
            auto signature = _::UString(sigMatch.begin(), sigMatch.end());
            if (!signature.compare(_::kMobotILSignature)) {
                pageSize = 256;
            }
            else if (!signature.compare(_::kMobotASignature)) {
                pageSize = 128;
            }
            else {
                ec = Status::UNKNOWN_SIGNATURE;
            }
            if (!ec) {
                buf.consume(n);
            }
        }
        else {
            ec = Status::PROTOCOL_ERROR;
        }
    };
}

const uint8_t kFlashType [] = { 'F' };
const uint8_t kEepromType [] = { 'E' };

const uint8_t kLoadAddressPrefix [] = { Cmnd_STK_LOAD_ADDRESS };
const uint8_t kCrcEop [] = { Sync_CRC_EOP };

inline std::array<boost::asio::const_buffer, 3>
loadAddressMessage (boost::asio::const_buffer address) {
    return {
        boost::asio::buffer(kLoadAddressPrefix),
        address,
        boost::asio::buffer(kCrcEop)
    };
}

const uint8_t kProgPagePrefix [] = { Cmnd_STK_PROG_PAGE };

inline std::array<boost::asio::const_buffer, 5>
progPageMessage (boost::asio::const_buffer size, boost::asio::const_buffer type,
        boost::asio::const_buffer data) {
    return {
        boost::asio::buffer(kProgPagePrefix),
        size, type, data,
        boost::asio::buffer(kCrcEop)
    };
}

}}} // stk::_::<anonymous>

#endif
