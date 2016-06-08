#ifndef LIBSTKCOMMS_BLOB_HPP
#define LIBSTKCOMMS_BLOB_HPP

#include <exception>
#include <string>
#include <utility>
#include <vector>
#include <stdexcept>

#include <cstdint>

namespace stk {

// Dumb container for a firmware blob. Encapsulates a 32-bit base address and
// a vector full of binary machine code.
class Blob {
public:
    using Address = uint32_t;
    using Code = std::vector<uint8_t>;

    Blob () = default;
    Blob (Address a, const Code& c) : mAddress(a), mCode(c) {}
    Blob (Address a, Code&& c) : mAddress(a), mCode(std::move(c)) {}

    Address address () const { return mAddress; }
    const Code& code () const { return mCode; }

    void address (Address a) { mAddress = a; }
    void code (const Code& c) { mCode = c; }
    void code (Code&& c) { mCode = std::move(c); }

private:
    Address mAddress = 0;
    Code mCode;
};

// Construct a Blob from a string containing Intel HEX records.
Blob makeBlobFromIntelHex (const std::string&);

struct BlobError : std::runtime_error {
    BlobError (std::string w) : std::runtime_error(w) {}
};

} // stk

#endif
