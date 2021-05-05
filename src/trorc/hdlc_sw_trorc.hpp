#pragma once

#include "trorc_registers.h"
#include "trorc.hpp"

#include "hdlc_sw.hpp"
#include "hdlc_core.hpp"


namespace trorc {

class HdlcSw : public common::HdlcCore
{
  public:
    /** Create interface for gbt_hdlc_sw core on T-RORC for communication with the GBT-SCA
     *  @param bar               Bar the core registers are found in
     *  @param request_trid_auto Enable automatic transaction ID handling
     *  @see ScaComm
     */
    HdlcSw(const common::Bar& bar,
           uint8_t sca_idx = 0x0,
           bool request_trid_auto = false,
           bool print_hdlc_dbg = true) :
        HdlcCore(request_trid_auto, print_hdlc_dbg),
        bar_(bar),
        sca_idx_(sca_idx)
      {}

    void     transmitCommand(HdlcEcFramePayload& request) final override;
    uint32_t executeCommand(HdlcEcFramePayload& request, HdlcEcFramePayload& reply) final override;

    void     sendSvlConnect() const override { sendSFrame(HdlcSwEc256::SFT_CONNECT); }
    void     sendSvlTest() const override { sendSFrame(HdlcSwEc256::SFT_TEST); }
    void     sendSvlReset() const override { sendSFrame(HdlcSwEc256::SFT_RESET); }

    /** Clear statistics, counters, ...
     */
    inline void clr() const final override
      {
        bar_.writeBit(rx_base_addr_ + TRORC_REG_GBTRX_SCA_SW_CTRL, B_RX_CLR, 0x1);
        bar_.writeBit(rx_base_addr_ + TRORC_REG_GBTRX_SCA_SW_CTRL, B_RX_CLR, 0x0);
        bar_.writeBit(tx_base_addr_ + TRORC_REG_GBTTX_SCA_SW_CTRL, B_TX_CLR, 0x1);
        bar_.writeBit(tx_base_addr_ + TRORC_REG_GBTTX_SCA_SW_CTRL, B_TX_CLR, 0x0);
      }

    /** Trigger reset of hdlc_sw core
     */
    inline void rst() const final override
      {
        bar_.writeBit(rx_base_addr_ + TRORC_REG_GBTRX_SCA_SW_CTRL, B_RX_USR_RST, 0x1);
        bar_.writeBit(tx_base_addr_ + TRORC_REG_GBTTX_SCA_SW_CTRL, B_TX_USR_RST, 0x1);
        bar_.writeBit(rx_base_addr_ + TRORC_REG_GBTRX_SCA_SW_CTRL, B_RX_USR_RST, 0x0);
        bar_.writeBit(tx_base_addr_ + TRORC_REG_GBTTX_SCA_SW_CTRL, B_TX_USR_RST, 0x0);
      }

    /** Setup special functions of the SCA core, if available
     *
     *  Sets up tx-bitswap, rx-bitswap, and tx-mux options of the HDLC SW core
     *    as required on the T-RORC
     */
    inline void init() const final override
      {
        txBitswap(1);
        rxBitswap(1);
        txMuxSelect(1);
      }

    /** Print core status
     */
    std::ostream& printStatus(std::ostream& os) const override;

    /** Swap Tx output bits\n
     *  @param val Multiplexer value\n
     *         0: tx_2b[1..0] <= tx_2b_i[1..0]\n
     *         1: tx_2b[1..0] <= tx_2b_i[0..1]\n
     *  @return Current bit value
     */
    inline uint32_t txBitswap(uint8_t val) const
      { return bar_.writeBitRb(tx_base_addr_ + TRORC_REG_GBTTX_SCA_SW_CTRL, B_TX_BITSWAP_SEL, val); }


    /** Control T-RORC Tx MUX setting (1 -> gbt_hdlc_sw, 0 -> gbt_hdlc_light)
     *  @warning This is T-RORC specific and will be removed again
     *  @param val MUX setting
     *  @return Current MUX setting
     */
    inline uint32_t txMuxSelect(uint8_t val) const
      { return bar_.writeBitRb(tx_base_addr_ + TRORC_REG_GBTTX_SCA_SW_CTRL, B_TX_MUX_SEL_DBG, val); }

    /** Swap Rx input bits\n
     *  @param val Multiplexer value\n
     *         0:  rx_2b_i[1..0] <= rx_2b[1..0]\n
     *         1:  rx_2b_i[1..0] <= rx_2b[0..1]\n
     *  @return Current bit value
     */
    inline uint32_t rxBitswap(uint8_t val) const
      { return bar_.writeBitRb(rx_base_addr_ + TRORC_REG_GBTRX_SCA_SW_CTRL, B_RX_BITSWAP_SEL, val); }

    void dbg0();
    void dbg1();

  protected:
    void sendFrame_(HdlcEcFramePayload& r) const final override;

    /** Get full SCA HDLC reply frame as detected by HDLC SW core
     *  @param r Buffer for the reply frame
     *  @return Number of bits in frame, if a valid frame was received
     */
    uint32_t getFrame(HdlcEcFrame& r) const;

  private:
    const common::Bar& bar_;
    const uint8_t sca_idx_;
    HdlcSwEc256 hdlc_;

    /** Test txReady bit
     */
    inline bool txReady() const
      { return bar_.readBit(tx_base_addr_ + TRORC_REG_GBTTX_SCA_SW_CTRL, B_TX_DONE) == 1; }

    /** Test rxReady bit
     */
    inline bool rxReady() const
      { return bar_.readBit(rx_base_addr_ + TRORC_REG_GBTRX_SCA_SW_CTRL, B_RX_DONE) == 1; }

    /** Pulse Tx start bit
     */
    void txStart() const
      {
        bar_.writeBit(tx_base_addr_ + TRORC_REG_GBTTX_SCA_SW_CTRL, B_TX_START, 0x1);
        bar_.writeBit(tx_base_addr_ + TRORC_REG_GBTTX_SCA_SW_CTRL, B_TX_START, 0x0);
      }

    /** Pulse Rx acknowledge bit
     */
    void rxAck() const
      {
        bar_.writeBit(rx_base_addr_ + TRORC_REG_GBTRX_SCA_SW_CTRL, B_RX_ACK, 0x1);
        bar_.writeBit(rx_base_addr_ + TRORC_REG_GBTRX_SCA_SW_CTRL, B_RX_ACK, 0x0);
      }

    /** Trigger transmission of supervisory-level frame
     */
    void
    sendSFrame(HdlcSwEc256::sFrameType s) const
      {
        hdlc_sw_ec_256_t fb;
        hdlc_.sFrame(fb, s);
        txShregLoad(fb);
        txStart();
      }

    /** Get Rx sequence counter
     *  @return Rx SW sequence count
     */
    inline uint8_t rxSwseqCnt() const
      {
        return static_cast<uint8_t>(bar_.readField(rx_base_addr_ + TRORC_REG_GBTRX_SCA_SW_CTRL,
                             F_RX_SWSEQ_CNT_LSB, F_RX_SWSEQ_CNT_WDT));
      }

    /** Get Tx sequence counter
     *  @return Tx SW sequence count
     */
    inline uint8_t txSwseqCnt() const
      {
        return static_cast<uint8_t>(bar_.readField(tx_base_addr_ + TRORC_REG_GBTTX_SCA_SW_CTRL,
                             F_TX_SWSEQ_CNT_LSB, F_TX_SWSEQ_CNT_WDT));
      }

    /** Load Tx shift register with frame buffer data
     */
    void
    txShregLoad(const hdlc_sw_ec_256_t& fb) const
      {
        for (uint32_t i = 0; i < 8; i++)
          bar_.write(tx_base_addr_ + TRORC_REG_GBTTX_SCA_SW_DATA_0 + i,
                       static_cast<uint32_t>((fb >> i*32) & 0xffffffff));
      }

    /** Fetch rx shift register content to frame buffer
     */
    void
    rxShregFetch(hdlc_sw_ec_256_t& fb) const
      {
        fb = 0x0;
        for (uint32_t i = 0; i < 8; i++)
          fb |= (hdlc_sw_ec_256_t(bar_.read(rx_base_addr_ + TRORC_REG_GBTRX_SCA_SW_DATA_0 + i))
                 << (i * 32));
      }

    /** Increment software-controlled Rx sequence counter
     *  @return New SW Rx sequence count
     */
    inline uint8_t rxSwseqCntInc() const
      {
        bar_.writeBit(rx_base_addr_ + TRORC_REG_GBTRX_SCA_SW_CTRL, B_RX_SWSEQ_CNT_INC, 0x1);
        bar_.writeBit(rx_base_addr_ + TRORC_REG_GBTRX_SCA_SW_CTRL, B_RX_SWSEQ_CNT_INC, 0x0);
        return rxSwseqCnt();
      }

    /** Increment software-controlled Tx sequence counter
     *  @return New SW Tx sequence count
     */
    inline uint8_t txSwseqCntInc() const
      {
        bar_.writeBit(tx_base_addr_ + TRORC_REG_GBTTX_SCA_SW_CTRL, B_TX_SWSEQ_CNT_INC, 0x1);
        bar_.writeBit(tx_base_addr_ + TRORC_REG_GBTTX_SCA_SW_CTRL, B_TX_SWSEQ_CNT_INC, 0x0);
        return txSwseqCnt();
      }

    /** Base addresses in the T-RORC\n
     *    Assumes one SCA per FEC on GBTx0\n
     *    With multiple GBTx connected to the T-RORC, GBTx0s must be connected as link 0, 2, 4, ...
     */
    const uint32_t rx_base_addr_ = (2 * sca_idx_ + 1) * TRORC_CHANNEL_OFFSET + (1 << TRORC_REGFILE_GTXRX_SEL);
    const uint32_t tx_base_addr_ = (2 * sca_idx_ + 1) * TRORC_CHANNEL_OFFSET + (2 << TRORC_REGFILE_GTXRX_SEL);

    // in TRORC_REG_GBTTX_SCA_SW_CTRL
      const uint8_t B_TX_USR_RST       =  0;  // B -> Bit, F -> Field
      const uint8_t B_TX_CLR           =  1;
      const uint8_t B_TX_START         =  2;
      const uint8_t B_TX_DONE          =  3;
      const uint8_t B_TX_SWSEQ_CNT_INC =  4;
      const uint8_t B_TX_BITSWAP_SEL   =  8;
      const uint8_t F_TX_SEQ_CNT_LSB   = 16;
      const uint8_t F_TX_SEQ_CNT_WDT   =  4;
      const uint8_t F_TX_SWSEQ_CNT_LSB = 20;
      const uint8_t F_TX_SWSEQ_CNT_WDT =  4;
      const uint8_t B_TX_MUX_SEL_DBG   = 31;

    // in TRORC_REG_GBTRX_SCA_SW_CTRL
      const uint8_t B_RX_USR_RST       =  0;  // B -> Bit, F -> Field
      const uint8_t B_RX_CLR           =  1;
      const uint8_t B_RX_ACK           =  2;
      const uint8_t B_RX_DONE          =  3;
      const uint8_t B_RX_SWSEQ_CNT_INC =  4;
      const uint8_t B_RX_BITSWAP_SEL   =  8;
      const uint8_t F_RX_SEQ_CNT_LSB   = 16;
      const uint8_t F_RX_SEQ_CNT_WDT   =  4;
      const uint8_t F_RX_SWSEQ_CNT_LSB = 20;
      const uint8_t F_RX_SWSEQ_CNT_WDT =  4;
};

} // namespace trorc
