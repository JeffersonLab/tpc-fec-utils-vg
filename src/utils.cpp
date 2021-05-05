#include "utils.hpp"

namespace boost {

void
validate(any& v,
         const std::vector<std::string>& values,
         cl_uint_t*, int)
{
  using namespace boost::program_options;
  validators::check_first_occurrence(v);
  const std::string& s = validators::get_single_string(values);

  std::regex r("^(0x|0X|b|B)?([0-9a-fA-F]+$)", std::regex::ECMAScript);
  std::smatch m;
  if (std::regex_match(s, m, r)) {
    uint8_t base(10);
    if (m[1].length() == 2) {
      base = 16;
    }
    else if (m[1].length() == 1) {
      base = 2;
      std::regex r2(".*[2-9a-fA-F].*");
      std::smatch m2;
      const std::string s2(m[2]);
      if (std::regex_match(s2, m2, r2))
        throw validation_error(validation_error::invalid_option_value);
    }
    else {
      base = 10;
      std::regex r2(".*[a-fA-F]+.*");
      std::smatch m2;
      const std::string s2(m[2]);
      if (std::regex_match(s2, m2, r2))
        throw validation_error(validation_error::invalid_option_value);
    }
    v = boost::any(cl_uint_t(std::stoul(m[2], nullptr, base)));
  } 
  else
    throw validation_error(validation_error::invalid_option_value);
}

template<>
std::string lexical_cast(const cl_uint_t& arg)
{
  std::stringstream s;
  s << arg.v;
  return s.str();
}

} // namespace boost
