#ifndef LIBSTKCOMMS_FULLREGEXMATCH_HPP
#define LIBSTKCOMMS_FULLREGEXMATCH_HPP

#include <boost/asio/read_until.hpp>
#include <boost/regex.hpp>

#include <type_traits>

namespace stk { namespace _ {

// FullRegexMatch is an Asio MatchCondition to be used in conjunction with
// async_read_until. Given a regular expression, FullRegexMatch causes
// async_read_until to read until:
//   - the RE matches the beginning of the input; or
//   - the input so far guarantees that the RE cannot match
// Note: this requires that the given regular expression is well-behaved in
// partial matches.
template <class Iter>
struct FullRegexMatch {
    boost::match_results<Iter>& what_;
    boost::regex re_;
    std::pair<Iter, bool> operator() (Iter b, Iter e) const {
        if (b == e) {
            // no input yet, continue reading
            return std::make_pair(b, false);
        }
        const auto flags = boost::match_default
            | boost::match_partial
            | boost::match_continuous;
        if (boost::regex_search(b, e, what_, re_, flags)) {
            // stop reading if this was a full match, continue if partial
            auto stop = what_[0].matched;
            return std::make_pair(stop ? what_[0].second : b, stop);
        }
        // no match, stop reading
        return std::make_pair(b, true);
    }
};

}}  // stk::_

// Asio demands that we explicitly tell it FullRegexMatch is a MatchCondition.
namespace boost { namespace asio {
template <class Iter>
struct is_match_condition<::stk::_::FullRegexMatch<Iter>> : std::true_type {};
}}  // boost::asio

#endif