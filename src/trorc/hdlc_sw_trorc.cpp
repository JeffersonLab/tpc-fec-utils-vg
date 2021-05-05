#include "hdlc_sw_trorc.hpp"


// Normal debug messaging
#ifdef HDLC_DEBUG
  #define hdlcdebug(s)  std::cout << __func__ << " : " << s << std::endl;
#else
  #define hdlcdebug(...)
#endif

// hdlc_debug1 has a lot of details
#ifdef HDLC_DEBUG1
  #define hdlcdebug1(s)  std::cout << __func__ << " : " << s << std::endl;
#else
  #define hdlcdebug1(...)
#endif


namespace trorc {

//------------------------------------------------------------------------------
void
HdlcSw::sendFrame_(HdlcEcFramePayload& r) const
{
  hdlc_sw_ec_256_t fb;
  hdlc_.iFrame(fb, r, txSwseqCnt(), rxSwseqCnt());

  txShregLoad(fb);
  rxAck();    // Allow Rx of a new frame
  txStart();  // Start Tx
  txSwseqCntInc();
}


uint32_t
HdlcSw::getFrame(HdlcEcFrame& r) const
{
  hdlc_sw_ec_256_t fb(0);
  rxShregFetch(fb);
  std::vector<size_t> flag_pos(hdlc_.getStartFlagPositions(fb));

  // hdlcdebug("Found " << flag_pos.size() << " frames in Rx buffer");

  // Only process the first frame found in the buffer, ignore others
  uint32_t bif(0);
  if (flag_pos.size()) {
    bif = hdlc_.deFrame(r, fb);
    // std::cout << r << " (" << bif << ")" << std::endl;

    rxSwseqCntInc();

    hdlcdebug("  HDLC | Got " << r)
  }
  return ((bif > 16) && r.crcMatch()) ? bif : 0;
}


void
HdlcSw::transmitCommand(HdlcEcFramePayload& request)
{
  hdlcdebug1("  HDLC | Waiting for TX ready")
  while (!txReady()) {};
  hdlcdebug1("  HDLC | TX ready, sending request")
  sendFrame(request);
}


uint32_t
HdlcSw::executeCommand(HdlcEcFramePayload& request, HdlcEcFramePayload& reply)
{
  uint8_t retries(0);
  bool    do_retry(true);
  HdlcEcFrame f;

  while (do_retry) {
    try {
      // Transmit request frame
      transmitCommand(request);

      // Wait for frame receive, detect Rx timeout
      std::chrono::high_resolution_clock::time_point ts = std::chrono::high_resolution_clock::now();
      hdlcdebug1("  HDLC | Waiting for RX done")
      while (!rxReady()) {
        if ((std::chrono::high_resolution_clock::now() - ts) > transceive_timeout_) {
          throw HdlcException("Rx timeout");
        }
        // Keeping this commented causes significantly more CPU load, but also speeds up executeCommand
        //   std::this_thread::sleep_for(std::chrono::microseconds(50));
      }
      hdlcdebug1("  HDLC | RX done, frame received, decoding")

      // Decode frame, only accept I-Frames
      if (getFrame(f) && f.isIFrame()) {
        reply = f.payload;
        do_retry = false;
      }
    }
    catch (HdlcException& e) {
      // Handle Rx exceptions
      retries++;
      if (print_hdlc_dbg_ || (retries >= transceive_max_retries_))
        std::cerr << e.what() << " -> retry " << static_cast<uint32_t>(retries) << "/"
                  << static_cast<uint32_t>(transceive_max_retries_) << std::endl;

      if (retries >= transceive_max_retries_)
        throw;
    }
  }

  return retries;
}


std::ostream&
HdlcSw::printStatus(std::ostream& os) const
{
  uint32_t tx_ctrl = bar_.read(tx_base_addr_ + TRORC_REG_GBTTX_SCA_SW_CTRL);
  uint32_t rx_ctrl = bar_.read(rx_base_addr_ + TRORC_REG_GBTRX_SCA_SW_CTRL);

  os << "SCA core " << static_cast<uint32_t>(sca_idx_) << " status (gbt_hdlc_sw) :"
     << std::hex << std::setfill('0') << std::right
     << "\n  txBaseAddr 0x" << std::setw(8) << tx_base_addr_
     << "  | rxBaseAddr 0x" << std::setw(8) << rx_base_addr_
     << "\n  txCtrl     0x" << std::setw(8) << tx_ctrl
     << "  | rxCtrl     0x" << std::setw(8) << rx_ctrl
     << std::dec << std::left << std::setfill(' ')
     << "\n    start      "   << std::setw(8) << (bar_.getBit(tx_ctrl, B_TX_START) ? "1" : "0") << "  "
     <<   "|   ack        "   << std::setw(8) << (bar_.getBit(rx_ctrl, B_RX_ACK)   ? "1" : "0")
     << "\n    done       "   << std::setw(8) << (bar_.getBit(tx_ctrl, B_TX_DONE)  ? "1" : "0") << "  "
     <<   "|   done       "   << std::setw(8) << (bar_.getBit(rx_ctrl, B_RX_DONE)  ? "1" : "0")
     << "\n    clr        "   << std::setw(8) << (bar_.getBit(tx_ctrl, B_TX_CLR)   ? "1" : "0") << "  "
     <<   "|   clr        "   << std::setw(8) << (bar_.getBit(rx_ctrl, B_RX_CLR)   ? "1" : "0")
     << "\n    bitswap    "   << std::setw(8) << (bar_.getBit(tx_ctrl, B_TX_BITSWAP_SEL)  ? "1" : "0") << "  "
     <<   "|   bitswap    "   << std::setw(8) << (bar_.getBit(rx_ctrl, B_RX_BITSWAP_SEL)  ? "1" : "0")
     << "\n    seqCnt     "   << std::setw(8) << static_cast<uint32_t>(bar_.getField(tx_ctrl, F_TX_SEQ_CNT_LSB, F_TX_SEQ_CNT_WDT)) << "  "
     <<   "|   seqCnt     "   << std::setw(8) << static_cast<uint32_t>(bar_.getField(rx_ctrl, F_RX_SEQ_CNT_LSB, F_RX_SEQ_CNT_WDT))
     << "\n    swInc      "   << std::setw(8) << (bar_.getBit(tx_ctrl, B_TX_SWSEQ_CNT_INC)   ? "1" : "0") << "  "
     <<   "|   swInc      "   << std::setw(8) << (bar_.getBit(rx_ctrl, B_RX_SWSEQ_CNT_INC)   ? "1" : "0")
     << "\n    swSeqCnt   "   << std::setw(8) << static_cast<uint32_t>(txSwseqCnt()) << "  "
     <<   "|   swSeqCnt   "   << std::setw(8) << static_cast<uint32_t>(rxSwseqCnt())
     << "\n";
  for (uint32_t i = 0; i < 8; i++)
    os << std::hex << std::setfill('0') << std::right
       <<   "  txData" << i << "    0x" << std::setw(8) << bar_.read(tx_base_addr_ + TRORC_REG_GBTTX_SCA_SW_DATA_0 + i)
       << "  | rxData" << i << "    0x" << std::setw(8) << bar_.read(rx_base_addr_ + TRORC_REG_GBTRX_SCA_SW_DATA_0 + i)
       << "\n" << std::dec;
  return os;
}



/**
 * SOME DEBUGGING BELOW
 */
void
HdlcSw::dbg0()
{
  // DATA=0x7f7f7f7f7f7f7f7f7f7f7f7f7e70b8000000000000addeabab210402ab08007e
  uint32_t d[] = {
      0xab08007e,
      0xab210402,
      0x00addeab,
      0x00000000,
      0x7e70b800,
      0x7f7f7f7f,
      0x7f7f7f7f,
      0x7f7f7f7f
    };
  for (uint8_t i = 0; i < 8; i++)
    bar_.write(tx_base_addr_ + TRORC_REG_GBTTX_SCA_SW_DATA_0 + i, d[i]);

  bar_.writeBit(rx_base_addr_ + TRORC_REG_GBTRX_SCA_SW_CTRL, B_RX_ACK, 0x1);
  bar_.writeBit(rx_base_addr_ + TRORC_REG_GBTRX_SCA_SW_CTRL, B_RX_ACK, 0x0);
  bar_.writeBit(tx_base_addr_ + TRORC_REG_GBTTX_SCA_SW_CTRL, B_TX_START, 0x1);
  bar_.writeBit(tx_base_addr_ + TRORC_REG_GBTTX_SCA_SW_CTRL, B_TX_START, 0x0);
}

void
HdlcSw::dbg1()
{
  HdlcEcFrame r;
  getFrame(r);
}

} // namespace trorc

#undef hdlcdebug
#undef hdlcdebug1
