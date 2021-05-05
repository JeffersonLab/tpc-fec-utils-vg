#pragma once

#include <iostream>
#include <iomanip>

#include "bar_common.hpp"
#include "hdlc_core.hpp"

namespace cru {

/** Interface the HDLC core provided by CERN Microelectronics Group
 *    (also controls the SCA core multiplexer implemented by the CRU core team)
 */
class HdlcCernMe : public common::HdlcCore
{
  public:
    /** Constructor
     *  @warn Do not instantiate 2 of these objects, as the constructor sets the
     *        SCA core mux to reflect the SCA index. Instead, only use one object
     *        and potentially use setScaCoreMux to adjust which link/SCA you're talking to.
     *        (For compatibility reasons, it is not forbidden to construct 2 instances of this
     *        object, neither is the structure of the HdlcCore classes changed to properly
     *        reflect these changes)
     *  @see  setScaCoreMux
     */
    HdlcCernMe(const common::Bar& bar, uint8_t sca_idx = 0x0, bool request_trid_auto = false) :
        HdlcCore(request_trid_auto, false),
        bar_(bar),
        base_addr_(SCA_CORE_BASE_INDEX)
      {
        // Adjust valid request trid stored in HDLC core with last know valid value
        request_trid_ = getLastValidRequestTrid();

        // Set SCA core mux
        setScaCoreMux(sca_idx);
      }

    void     transmitCommand(HdlcEcFramePayload& request) final override;
    uint32_t executeCommand(HdlcEcFramePayload& request, HdlcEcFramePayload& reply) final override;

    /** Control SCA core multiplexer (new as of CRU firmware v2.0.0.)
     */
    void setScaCoreMux(uint8_t sca_idx) const
      { bar_.writeField(base_addr_ + REG_SCA_CORE_MUX, 0, 5, sca_idx); }

    /** Get SCA core multiplexer setting
     */
    uint8_t getScaCoreMux() const
      { return bar_.getField(base_addr_ + REG_SCA_CORE_MUX, 0, 5); }

    /** Send supervisory level command "CONNECT" to SCA (see SCA manual p.11)
     */
    void sendSvlConnect() const final override { startFrameTx(B_SEND_CONNECT); }

    /** Send supervisory level command "TEST" to SCA (see SCA manual p.11)
     */
    void sendSvlTest() const final override { std::cout << __func__ << " not implemented for HdlcCernMe core\n"; }

    /** Send supervisory level command "RESET" to SCA (see SCA manual p.11)
     */
    void sendSvlReset() const final override { startFrameTx(B_SEND_RST); }

    /** Reset SCA core
     */
    void rst() const final override
      {
        // This is taken from the SCA __init__() routine of the CRU team python tools (in SCA.py)
        bar_.writeBit(base_addr_ + REG_SC_RST, B_SC_RST, 1);
        bar_.writeBit(base_addr_ + REG_SC_RST, B_SC_RST, 0);

        // This is taken from the reset() routine in SCA.py
        //   What is the difference here to the reset above? ...no idea
        bar_.writeField(base_addr_ + REG_CTRL, F_SEND_CTRL_BITS_LSB, F_SEND_CTRL_BITS_WDT, 0x0);
        bar_.writeBit(base_addr_+ REG_CTRL, B_CORE_RST, 1);
        bar_.writeBit(base_addr_+ REG_CTRL, B_CORE_RST, 0);

        // FIXME: This is a workaround for the broken busy flag of the core:
        //   Send a connect frame to the SCA core, so that the busy flag gets cleared
        //   on reply
        startFrameTx(B_SEND_CONNECT, true);
      }


    /** Print status information
     */
    std::ostream& printStatus(std::ostream& os) const final override
      {
        os << "SCA core\n"
           << std::hex << std::setfill('0')
           << "  Base address 0x" << base_addr_*4 << "\n"
           << "  [0x" << REG_SCA_CORE_MUX*4 << "]  core mux  0x" << static_cast<uint32_t>(getScaCoreMux()) << "\n"
           << "  [0x" << REG_WR_DATA*4      << "]  wr_data   0x" << bar_.read(base_addr_ + REG_WR_DATA) << "\n"
           << "  [0x" << REG_WR_CMD*4       << "]  wr_cmd    0x" << bar_.read(base_addr_ + REG_WR_CMD) << "\n"
           << "  [0x" << REG_CTRL*4         << "]  ctrl      0x" << bar_.read(base_addr_ + REG_CTRL) << "\n"
           << "  [0x" << REG_RD_DATA*4      << "]  rd_data   0x" << bar_.read(base_addr_ + REG_RD_DATA) << "\n"
           << "  [0x" << REG_RD_CMD*4       << "]  rd_cmd    0x" << bar_.read(base_addr_ + REG_RD_CMD) << "\n"
           << "  [0x" << REG_CTRL2*4        << "]  ctrl2     0x" << bar_.read(base_addr_ + REG_CTRL2) << "\n"
           << "  [0x" << REG_MON*4          << "]  mon       0x" << bar_.read(base_addr_ + REG_MON) << "\n"
           << std::dec << std::setfill(' ')
           << "";
        return os;
      }

  protected:
    void     sendFrame_(HdlcEcFramePayload& r) const final override;

  private:
    const common::Bar& bar_;
    const uint32_t base_addr_;

    static constexpr uint32_t SCA_CORE_BASE_INDEX {0x00f00000/4};

    static const uint32_t REG_WR_DATA    {0x00/4}; // {0x20/4};    // HDLC payload write data part

    static const uint32_t REG_WR_CMD     {0x04/4}; // {0x24/4};    // HDLC payload write cmd part
      static const uint8_t REQ_TRID_LSB {16};
      static const uint8_t REQ_TRID_WDT {8};

    static const uint32_t REG_CTRL        {0x08/4}; // {0x28/4};    // Ctrl, write-related
      static const uint8_t B_SEND_RST     {0};
      static const uint8_t B_SEND_CONNECT {1};
      static const uint8_t B_SEND_CMD     {2};
      static const uint8_t F_SEND_CTRL_BITS_LSB {0};         // Field for 3 bits controlling the sending of
      static const uint8_t F_SEND_CTRL_BITS_WDT {3};        //   frames
      static const uint8_t B_CORE_RST     {23};

    static const uint32_t REG_RD_DATA     {0x10/4}; // {0x30/4};    // HDLC payload read data part

    static const uint32_t REG_RD_CMD      {0x14/4}; // {0x34/4};    // HDLC payload read cmd part

    static const uint32_t REG_CTRL2       {0x18/4}; // {0x38/4};    // Ctrl, read-related
      static const uint8_t B_CORE_TX_BUSY {30};             // TX busy    FIXME: Add support for this
      static const uint8_t B_CORE_BUSY    {31};             // RX busy

    static const uint32_t REG_MON         {0x1c/4}; // {0x3c/4};    // Monitoring
      static const uint8_t F_TX_TO_RX_CYCLES_LSB {0};       // Time from sending of a command (raise busy) to
      static const uint8_t F_TX_TO_RX_CYCLES_WDT {28};      //   reception of the reply (release busy) FIXME: careful, might change in hardware!

    static const uint32_t REG_SCA_CORE_MUX {0x60/4};

    static const uint32_t REG_SC_RST {0x64/4};        // This is access in the CRU team python tool in the init() of the SCA object only
      static const uint8_t B_SC_RST  {0};

    /** Start transmission of a normal I-frame (after frame cmd/data regs were loaded)
     *  @param bit_idx SEND bit to drive (B_SEND_RST, B_SEND_CONNECT, B_SEND_CMD)
     *  @param ignore_busy_flag Set to true if tx busy flag should not be checked before initiating frame send
     */
    inline void startFrameTx(uint8_t bit_idx = B_SEND_CMD, bool ignore_busy_flag = false) const
      {
        while (!ignore_busy_flag && !txReady()) {}
        bar_.writeBit(base_addr_+ REG_CTRL, bit_idx, 1);
        bar_.writeField(base_addr_ + REG_CTRL, F_SEND_CTRL_BITS_LSB, F_SEND_CTRL_BITS_WDT, 0x0);
      }

    /**  Get HDLC frame payload content from SCA core registers and disassemble it
     *   @note Just pulls register content and disassembles it, but does not check whether
     *         received frame is avaible
     *   @param r Reference to store disabled frame data
     *   @return SCA error code (0x0 means no error)
     */
    inline uint32_t getFrame(HdlcEcFramePayload& r) const;

    /** Test SCA core RX busy bit
     *    (bit gets set at the beginning of any send request (I-frame or S-frames),
     *    and released with the reception of the reply frame)
     */
    inline bool rxReady() const
      {
        return bar_.readBit(base_addr_ + REG_CTRL2, B_CORE_BUSY) == 0x0;
      }

    /** Test SCA core busy bit
     *    (bit gets set at the beginning of any send request (I-frame or S-frames),
     *    and released with the reception of the reply frame)
     *
     *  FIXME: At the moment, this is the same as the RX busy bit - wait for CERN group
     *         to supply proper bit in EC core and then update
     */
    inline bool txReady() const
      {
        return bar_.readBit(base_addr_ + REG_CTRL2, B_CORE_BUSY) == 0x0;
      }

    /** Read previous request transaction ID from request register
     *    (and recover from potential illegal values)
     *
     *  @return Request TRID in the range [1..254] and different from the current register
     *            value, typically reg_value+1
     */
    inline uint8_t getLastValidRequestTrid()
      {
        uint8_t req_trid = bar_.readField(REG_WR_CMD, REQ_TRID_LSB, REQ_TRID_WDT);
        if ((req_trid == 0xfe) || (req_trid == 0xff))
          req_trid = 0x0;
        return req_trid++;
      }

    /** Transfer payload struct content to write register
     *    and adjust to correct format for this core
     */
    inline void
    assembleRegisters(const HdlcEcFramePayload& r, uint32_t& w2, uint32_t& w3) const {
        w2 = ((r.channel << 24) |
              (r.trid    << 16) |
              (r.length  << 8)  |
              (r.command << 0));
        w3 = ((r.data & 0xff)       << 24) |
             ((r.data & 0xff00)     << 8) |
             ((r.data & 0xff0000)   >> 8) |
             ((r.data & 0xff000000) >> 24);
      }

    /** Pack register read values into payload struct
     *    and adjust to correct format for this core
     */
    inline void
    disassembleRegisters(HdlcEcFramePayload& r, uint32_t w2, uint32_t w3) const {
        r.channel = (w2 >> 24) & 0xff;
        r.trid    = (w2 >> 16) & 0xff;
        r.length  = (w2 >> 8) & 0xff;
        r.error   = (w2 >> 0) & 0xff;
        r.data = ((w3 & 0xff)       << 24) |
                 ((w3 & 0xff00)     << 8) |
                 ((w3 & 0xff0000)   >> 8) |
                 ((w3 & 0xff000000) >> 24);
      }

};

} // namespace cru
