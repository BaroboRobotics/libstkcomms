#ifndef LIBSTKCOMMS_INTELHEX_HPP
#define LIBSTKCOMMS_INTELHEX_HPP

#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_fusion.hpp>
#include <boost/spirit/include/phoenix_stl.hpp>
#include <boost/spirit/include/qi.hpp>

#include <boost/fusion/adapted.hpp>

namespace stk {
namespace intelhex {
using Address = uint32_t;
struct DataRecord {
    Address address;
    std::vector<uint8_t> payload;
};
} // intelhex
} // stk

BOOST_FUSION_ADAPT_STRUCT(stk::intelhex::DataRecord, address, payload)

namespace stk {
namespace {
    namespace qi = boost::spirit::qi;
} // <anonymous>

namespace intelhex {

template <class Iter>
struct Grammar : qi::grammar<Iter, qi::locals<Address>, std::list<DataRecord>()> {
    // The start rule needs a local to track the segment base address. It
    // synthesizes a list of data records.
    qi::rule<Iter, qi::locals<Address>, std::list<DataRecord>()> start;

    // A data record needs a local byte to track the length of the record
    // payload. It inherits a base address and synthesizes a data record.
    qi::rule<Iter, qi::locals<uint8_t>, DataRecord(Address)> data;

    qi::rule<Iter> end;

    // An extended segment address record synthesizes a base address.
    qi::rule<Iter, Address()> extendedSegmentAddress;

    Grammar () : Grammar::base_type(start) {
        // Parse a 4-digit hexadecimal number into an Address. Addresses are
        // always 16-bits wide in a hex file, but they can be up to 20 bits
        // wide after adding the segment base address from an extended segment
        // address record.
        auto address = qi::uint_parser<Address, 16, 4, 4>();

        // Parse a 2-digit hexadecimal number into a uint8_t.
        auto byte = qi::uint_parser<uint8_t, 16, 2, 2>();

        using qi::_a;
        using qi::_1;
        using qi::_r1;
        using qi::_val;
        using qi::repeat;
        using boost::phoenix::at_c;
        using boost::phoenix::push_back;

        // An Intel HEX file consists of multiple data and extended segment
        // address records followed by an end record. We begin the start rule
        // with an epsilon (empty) parser so we can initialize _a, the base
        // address.
        start = qi::eps[(_a = 0) || true]
            >> *(data(_a)[push_back(_val, _1)]
                | extendedSegmentAddress[_a = _1])
            >> end;

        // Every record begins with a : and ends with a CRLF.
        data = ':'
            >> byte[_a = _1]                        // length of payload
            >> address[at_c<0>(_val) = _1 + _r1]    // destination address of payload
            >> byte(0)                              // record type (data)
            > repeat(_a)[byte][at_c<1>(_val) = _1]  // payload, _a bytes long
            > byte                                  // checksum
            > qi::eol;

        end = ':'
            >> byte(0)     // length of payload
            >> address(0)  // ignored
            >> byte(1)     // record type (end)
            > byte(0xff)   // checksum
            > qi::eol;

        extendedSegmentAddress = ':'
            >> byte(2)                  // length of payload
            >> address(0)               // ignored
            >> byte(2)                  // record type (extended address segment)
            > address[_val = _1 << 4]   // bits 19-4 of the base address
            > byte                      // checksum
            > qi::eol;
    }
};

} // intelhex
} // stk

#endif