#pragma once
#include <ostream>
#include <string>

class GitInfo
{
  public:
    static const uint32_t    hash_{0x6ddac21};
    static const bool        clean_{false};
    static const std::string branch_;
    static const std::string user_;
    static const std::string hostname_;

    friend std::ostream&
    operator<<(std::ostream& os, const GitInfo& g)
      {
        os << "Version information\n"
           << "  branch : " << g.branch_ << "\n"
           << "  hash   : " << std::hex << g.hash_ << std::dec << "\n"
           << "  clean  : " << (g.clean_ ? "yes" : "no") << "\n"
           << "  build  : " << g.user_ << "@" << g.hostname_ << "\n";
        return os;
      }
};

const std::string GitInfo::branch_ = {"master"};
const std::string GitInfo::user_ = {"gurjyan"};
const std::string GitInfo::hostname_ = {"vapapi.jlab.org"};
