#ifndef LIBSTKCOMMS_SYSTEM_ERROR_HPP
#define LIBSTKCOMMS_SYSTEM_ERROR_HPP

#include <libstkcomms/status.hpp>

#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>

#include <string>

namespace stk {

class ErrorCategory : public boost::system::error_category {
public:
    virtual const char* name () const BOOST_NOEXCEPT override;
    virtual std::string message (int ev) const BOOST_NOEXCEPT override;
};

const boost::system::error_category& errorCategory ();
boost::system::error_code make_error_code (Status status);
boost::system::error_condition make_error_condition (Status status);

} // namespace stk

namespace boost {
namespace system {

template <>
struct is_error_code_enum< ::stk::Status> : public std::true_type { };

} // namespace system
} // namespace boost

#endif