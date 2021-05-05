#include "hdlc_cern_me.hpp"

namespace cru {

void
HdlcCernMe::sendFrame_(HdlcEcFramePayload& r) const
{
  uint32_t wdata, wcmd;
  assembleRegisters(r, wcmd, wdata);
  bar_.write(base_addr_ + REG_WR_DATA, wdata);
  bar_.write(base_addr_ + REG_WR_CMD, wcmd);
  startFrameTx();
}

uint32_t
HdlcCernMe::getFrame(HdlcEcFramePayload& r) const
{
  disassembleRegisters(r, bar_.read(base_addr_ + REG_RD_CMD), bar_.read(base_addr_ + REG_RD_DATA));
  return r.error;
}

void
HdlcCernMe::transmitCommand(HdlcEcFramePayload& request)
{
  std::chrono::high_resolution_clock::time_point ts = std::chrono::high_resolution_clock::now();
  while (!txReady()) {
    if ((std::chrono::high_resolution_clock::now() - ts) > transceive_timeout_) {
      throw HdlcException("Rx timeout - SCA core busy flag stuck?");
    }
  }

  sendFrame(request);
}

uint32_t
HdlcCernMe::executeCommand(HdlcEcFramePayload& request, HdlcEcFramePayload& reply)
{
  transmitCommand(request);

  std::chrono::high_resolution_clock::time_point ts = std::chrono::high_resolution_clock::now();
  while (!rxReady()) {
    if ((std::chrono::high_resolution_clock::now() - ts) > transceive_timeout_) {
      throw HdlcException("Rx timeout");
    }
  }

  getFrame(reply);

  return 0;  // retry not implemented, number of retries -> 0
}

} // namespace cru
