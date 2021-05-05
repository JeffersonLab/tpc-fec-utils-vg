#include <sstream>
#include <thread>
#include <bitset>

#include "hdlc_core.hpp"

namespace common {

//------------------------------------------------------------------------------
const std::string
HdlcCore::hdlcPayloadErrors_[] =
{
  "Unknown/Undocumented",
  "Invalid channel request",
  "Invalid command request",
  "Invalid transaction number request",
  "Invalid request length",
  "Channel disabled",
  "Channel busy",
  "Command in treatment"
};

std::string
HdlcCore::getPayloadErrorMessage(uint8_t error_code) const
{
  std::stringstream os;
  for(uint8_t i = 0; i < 8; i++)
    if ((1 << i) & error_code)
      os << " +" << hdlcPayloadErrors_[i];
  return os.str();
}

void
HdlcCore::sendFrame(HdlcEcFramePayload& r)
{
  adjustRequestTrid(r);
  stat_total_requests_++;
  this->sendFrame_(r);
//  hdlcdebug("  HDLC | Sent frame [" << std::setw(5) << std::dec << stat_total_requests_ << "] :       " << r)
}

} // namespace common
