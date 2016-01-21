#ifndef LIBSTKCOMMS_STATUS_HPP
#define LIBSTKCOMMS_STATUS_HPP

namespace stk {

enum class Status {
    OK,
    TIMEOUT,
    PROTOCOL_ERROR,
    UNKNOWN_SIGNATURE
};

} // namespace stk

#endif