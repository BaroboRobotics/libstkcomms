#ifndef LIBSTKCOMMS_INTELHEX_HPP
#define LIBSTKCOMMS_INTELHEX_HPP

#include <libstkcomms/blob.hpp>

#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_fusion.hpp>
#include <boost/spirit/include/phoenix_stl.hpp>
#include <boost/spirit/include/qi.hpp>

#include <boost/fusion/adapted.hpp>

#include <iostream>

// Adapt Blob to be a Fusion sequence so we can use the Phoenix at_c lazy
// accessor in Spirit.Qi semantic actions.
BOOST_FUSION_ADAPT_ADT(stk::Blob,
    (obj.address(), obj.address(val))
    (obj.code(), obj.code(val)))

namespace stk { namespace intelhex {

namespace {
    namespace qi = boost::spirit::qi;
} // <anonymous>

// A Spirit.Qi grammar for parsing an Intel HEX file. Currently it only
// recognizes three records: data, end, and extended segment address. These are
// the only three present in Linkbot firmware HEX files.
template <class Iter>
struct Grammar : qi::grammar<Iter, qi::locals<Blob::Address>, std::list<Blob>()> {
    // The start rule needs a local to track the segment base address. It
    // synthesizes a list of data records.
    qi::rule<Iter, qi::locals<Blob::Address>, std::list<Blob>()> start;

    // A data record needs a local byte to track the length of the record
    // payload. It inherits a base address and synthesizes a data record.
    qi::rule<Iter, qi::locals<uint8_t>, Blob(Blob::Address)> data;

    qi::rule<Iter> end;

    // An extended segment address record synthesizes a base address.
    qi::rule<Iter, Blob::Address()> extendedSegmentAddress;

    Grammar () : Grammar::base_type(start, "intelhex") {
        // Parse a 4-digit hexadecimal number into an Address. Addresses are
        // always 16-bits wide in a hex file, but they can be up to 20 bits
        // wide after adding the segment base address from an extended segment
        // address record.
        auto address = qi::uint_parser<Blob::Address, 16, 4, 4>();

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
        // address. An epsilon parser's semantic action must return true or
        // else the match fails.
        start.name("start");
        start = qi::eps[(_a = 0), true]
            >> *(data(_a)[push_back(_val, _1)]
                | extendedSegmentAddress[_a = _1])
            >> end
            > qi::eoi;

        // Every record begins with a : and ends with a CRLF.
        data.name("data");
        data = ':'
            >> byte[_a = _1]                        // length of payload
            >> address[at_c<0>(_val) = _1 + _r1]    // destination address of payload
            >> byte(0)                              // record type (data)
            > repeat(_a)[byte][at_c<1>(_val) = _1]  // payload, _a bytes long
            > byte                                  // checksum
            > qi::eol;

        end.name("end");
        end = ':'
            >> byte(0)     // length of payload
            >> address(0)  // ignored
            >> byte(1)     // record type (end)
            > byte(0xff)   // checksum
            > qi::eol;

        extendedSegmentAddress.name("extendedSegmentAddress");
        extendedSegmentAddress = ':'
            >> byte(2)                  // length of payload
            >> address(0)               // ignored
            >> byte(2)                  // record type (extended address segment)
            > address[_val = _1 << 4]   // bits 19-4 of the base address
            > byte                      // checksum
            > qi::eol;

        using ErrorHandlerArgs = boost::fusion::vector<
            Iter&, const Iter&, const Iter&, const qi::info&>;

        qi::on_error<qi::fail>(start,
        [](ErrorHandlerArgs args, auto&, qi::error_handler_result&) {
            std::cout << "Expected '" << at_c<3>(args) << "' here: '"
                << std::string(at_c<2>(args), at_c<1>(args)) << "'\n";
        });
        qi::on_error<qi::fail>(data,
        [](ErrorHandlerArgs args, auto&, qi::error_handler_result&) {
            std::cout << "Expected '" << at_c<3>(args) << "' here: '"
                << std::string(at_c<2>(args), at_c<1>(args)) << "'\n";
        });
        qi::on_error<qi::fail>(end,
        [](ErrorHandlerArgs args, auto&, qi::error_handler_result&) {
            std::cout << "Expected '" << at_c<3>(args) << "' here: '"
                << std::string(at_c<2>(args), at_c<1>(args)) << "'\n";
        });
        qi::on_error<qi::fail>(extendedSegmentAddress,
        [](ErrorHandlerArgs args, auto&, qi::error_handler_result&) {
            std::cout << "Expected '" << at_c<3>(args) << "' here: '"
                << std::string(at_c<2>(args), at_c<1>(args)) << "'\n";
        });
    }


};

}} // stk::intelhex

#endif
