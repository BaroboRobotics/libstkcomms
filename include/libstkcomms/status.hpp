#ifndef LIBSTKCOMMS_STATUS_HPP
#define LIBSTKCOMMS_STATUS_HPP

namespace stk {

enum class Status {
    OK,
    PROTOCOL_ERROR,
    UNKNOWN_SIGNATURE,
    BLOB_TOO_BIG
};

} // namespace stk

#endif