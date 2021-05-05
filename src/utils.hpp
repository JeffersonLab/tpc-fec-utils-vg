#pragma once

#include <memory>
#include <sstream>
#include <regex>
#include <boost/program_options.hpp>

//------------------------------------------------------------------------------
// ...as long as there's no make_unique in C++
template<typename T, typename ...Args>
std::unique_ptr<T> make_unique(Args&& ...args)
{
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}


//------------------------------------------------------------------------------
/** Utility class for boost::program_options command line input
 *    to accept numbers in decimal or hex form with validation
 *
 *  Accepts input of the form\n
 *    - HEX: 0x123, 0Xabcd, 0xABcad, ...
 *    - DECIMAL: 123456
 *    - BINARY: b1101 b000110 B101
 *
 *  Corresponding validator will throw if input does not meet these
 *    formats
 *
 *  Stored value can accessed via member v or operator()
 */
class cl_uint_t
{
  public:
    cl_uint_t(unsigned long long n = 0) : v(n) {}
    unsigned long long v;                                  /**< Stored value */
    unsigned long long operator()() const { return v; }    /**< For member v retrival */

    friend std::ostream& operator<<(std::ostream &os, const cl_uint_t& c)
      {
        os << c.v;
        return os;
      }
};


namespace boost {

/** Custom boost::program_options validator for
 *    cl_uint_t
 */
void
validate(any& v, const std::vector<std::string>& values,
         cl_uint_t* target, int);


/** Lexical cast for cl_uint_t to allow for printing of the
 *    stored value
 */
template<>
std::string lexical_cast(const cl_uint_t& arg);

} // namespace boost
