#include <chrono>
#include <thread>

#include "gbt_link.hpp"
#include "trorc.hpp"
#include "bit_manipulation.hpp"

class PccaInterface : public common::BitManipulation
{
  private:
    std::unique_ptr<trorc::GbtLink> link_;

  public:
    PccaInterface(trorc::Bar& bar, uint32_t link_idx) :
        link_(new trorc::GbtLink(bar, link_idx))
      {}

    /** Data pin definitions for the first shift register
     */
    static const uint8_t BIT_SHR_SEN_3 {0};
    static const uint8_t BIT_SHR_SDI_3 {1};
    static const uint8_t BIT_SHR_SDI_1 {2};
    static const uint8_t BIT_SHR_SEN_1 {3};
    static const uint8_t BIT_SHR_TRST  {4};
    static const uint8_t BIT_SHR_TMS   {5};
    static const uint8_t BIT_SHR_TDI   {6};
    static const uint8_t BIT_SHR_SDA   {7};

    /** Data pin definitions for the second shift register
     */
    static const uint8_t BIT_SHR_SDI_0 {0};
    static const uint8_t BIT_SHR_SEN_0 {1};
    static const uint8_t BIT_SHR_SDI_2 {2};
    static const uint8_t BIT_SHR_SCL   {3};
    static const uint8_t BIT_SHR_SEN_2 {4};
    static const uint8_t BIT_SHR_SDI_4 {5};
    static const uint8_t BIT_SHR_SEN_4 {6};
    static const uint8_t BIT_SHR_TME   {7};

    /** Pin definitions for the downlink patterns (direct pins only)
     */
    static const uint8_t BIT_TX_HRSTB      { 0};
    static const uint8_t BIT_TX_CLKBX      { 1};
    static const uint8_t BIT_TX_DINN       { 2};
    static const uint8_t BIT_TX_NBFLOWSTOP { 3};
    static const uint8_t BIT_TX_TRG        { 4};
    static const uint8_t BIT_TX_CLKADC     { 5};
    static const uint8_t BIT_TX_TCLK       { 6};
    static const uint8_t BIT_TX_BXSYNCTRG  { 7};
    static const uint8_t BIT_TX_HBTRG      { 8};
    static const uint8_t BIT_TX_CLKS0      { 9};
    static const uint8_t BIT_TX_SME        {10};
    static const uint8_t BIT_TX_SCLK       {15};

    /** Pin definitions of the uplink capture register
     *    Each pin corresponds to 4 bits in the capture register
     */
    static const uint8_t BIT_RXRED_SERIALOUT_0 { 0/2};  // bits 3..0
    static const uint8_t BIT_RXRED_SERIALOUT_1 { 2/2};  // bits 7..4
    static const uint8_t BIT_RXRED_SERIALOUT_2 { 4/2};  // ...and so on
    static const uint8_t BIT_RXRED_SERIALOUT_3 { 6/2};
    static const uint8_t BIT_RXRED_SERIALOUT_4 { 8/2};
    static const uint8_t BIT_RXRED_NBFLOWSTOP  {10/2};
    static const uint8_t BIT_RXRED_SDO_0       {12/2};
    static const uint8_t BIT_RXRED_SDO_1       {14/2};
    static const uint8_t BIT_RXRED_SDO_2       {16/2};
    static const uint8_t BIT_RXRED_SDO_3       {18/2};
    static const uint8_t BIT_RXRED_SDO_4       {20/2};
    static const uint8_t BIT_RXRED_SMO         {22/2};
    static const uint8_t BIT_RXRED_TDO         {30/2};
    static const uint8_t BIT_RXRED_SDA         {32/2};

    /** Perform SAMPA hard reset by pulsing HRSTB
     */
    inline void sampaHardReset() const
      {
        link_->dftCtrlTxPath(0x1);
        dftControlPinTx(BIT_TX_HRSTB, 1, false);
        dftControlPinTx(BIT_TX_HRSTB, 0, false);
        std::this_thread::sleep_for(std::chrono::nanoseconds(100));
        dftControlPinTx(BIT_TX_HRSTB, 1, false);
        // Cycle CLKS0 (has not effect if clock mux is set to 0, but doesn't harm either)
        for (int i = 0; i < 3; i++)
          dftControlPinTx(BIT_TX_CLKS0, 1, true);
        link_->dftCtrlTxPath(0x0);
        std::this_thread::sleep_for(std::chrono::nanoseconds(1000));  // Give some time to wake up
      }

    /** Initialization for built-in SAMPA memory test via VLDB/PCCA interface board
     */
    inline void memtestInitialize() const
      {
        link_->dftCtrlShrAutoLoad(0x1);
        link_->dftCtrlShrOutput(0x1);
        link_->dftCtrlTxPath(0x1);
        link_->setDftCtrlShr(0x0, 0x0 | (0 << BIT_SHR_TME), 0xff, 0xff);
        link_->setDftCtrlPinStimulus(1 << BIT_TX_HRSTB);
      }

    inline void memtestDeInitialize() const
      {
        dftControlPinTx(BIT_TX_SME, 0, false);
        link_->dftCtrlShrAutoLoad(0x0);
        link_->dftCtrlShrOutput(0x0);
        link_->dftCtrlTxPath(0x0);
      }

    inline void memtestStart() const
      {
        dftControlPinTx(BIT_TX_SME, 1, false);
      }

    inline uint8_t memtestGetResult() const
      { return (link_->getDftRxData() >> BIT_RXRED_SMO) & 0x1; }


    /** Initialize hardware for JTAG (mostlty applies to DFT/JTAG controller)
     */
    inline void jtagInitialize() const
      {
        link_->dftCtrlShrOutput(0x1);
        link_->dftCtrlTxPath(0x1);
        link_->dftCtrlShrAutoLoad(0x1);
        link_->dftCtrlSclkAutoToggle(0x0);
        link_->dftCtrlShrFastLoad(0x0);
      }

    /** DeInitialize hardware
     */
    inline void jtagDeInitialize() const
      {
        link_->dftCtrlShrOutput(0x0);
        link_->dftCtrlTxPath(0x0);
        link_->dftCtrlShrAutoLoad(0x0);
      }

    /** Get TDO bit
     *  @return Current TDO bit value (from DFT controller RX reduction register)
     */
    inline uint8_t jtagGetTdo() const
      { return (link_->getDftRxData() >> BIT_RXRED_TDO) & 0x1; }

    /** Control TCK
     *  @param value Value to drive TCK with
     *  @param cycle If true, automatically set TCK to ~value afterwards
     */
    inline void jtagControlTck(uint8_t value, bool cycle = false) const
      { dftControlPinTx(BIT_TX_TCLK, value, cycle); }

    /** Constants related to JTAG shift
     */
    static constexpr uint8_t TMS  {1 << BIT_SHR_TMS};
    static constexpr uint8_t TDI  {1 << BIT_SHR_TDI};
    static constexpr uint8_t TRST {1 << BIT_SHR_TRST};

    /** JTAG routine to move TAP controller to a new state
     *  @param tms_tdi_trst JTAG signals to set when advancing FSM (TMS, TDI, TRST, all active high)
     *  @return TDO bit value
     */
    inline uint8_t jtagShift(uint8_t tms_tdi_trst) const
      {
        uint8_t mask0((1 << BIT_SHR_TMS) | (1 << BIT_SHR_TDI) | (1 << BIT_SHR_TRST));
        link_->setDftCtrlShr(tms_tdi_trst ^ TRST, 0x0, mask0, 0x0);   // TRST pin is low active

        // DFT/JTAG controller needs time to shift in, plus propagation delay for TDO until seen on T-RORC
        std::this_thread::sleep_until(std::chrono::high_resolution_clock::now() +
                                      std::chrono::nanoseconds(8*25+100));

        uint8_t tdo = jtagGetTdo();
        jtagControlTck(0x1, true);    // Toggle TCK
        return tdo;
      }

    /** Reset TAP controller and bring it to IDLE state
     */
    inline void jtagReset() const
      {
        for (uint8_t i = 0; i < 9; i++)
          jtagShift(TMS);  // Go to reset state from anywhere in state machine
        jtagShift(0x0);    // Idle
      }

    /** JTAG instruction codes
     */
    enum jtag_instruction_t { SAMPLE_PRELOAD = 0x5, EXTEST = 0x6, BYPASS = 0x7 };

    /** Shift JTAG instruction
     *  @param i Instruction to shift
     *  @see jtag_instruction_t
     */
    void jtagShiftIr(jtag_instruction_t i) const
      {
        jtagShift(TMS);   // Select-DR-Scan
        jtagShift(TMS);   // Select-IR-Scan
        jtagShift(0);
        jtagShift(0);
        // Write instructions
        jtagShift(((i & 0x1) == 0x1) ? TDI : 0x0);
        jtagShift(((i & 0x2) == 0x2) ? TDI : 0x0);
        jtagShift((((i & 0x4) == 0x4) ? TDI  : 0x0) | TMS);
        jtagShift(TMS); // Exit1-IR
        jtagShift(0); // Back to IDLE
      }

    /** Shift JTAG data
     *  @param data JTAG data to shift
     *  @param n_bits Number of bits in JTAG data to shift
     *  @return Sequence of received TDO bit values
     */
    uint64_t jtagShiftDr(uint64_t data, uint32_t n_bits) const    // FIXME: Maybe use std::vector here to eliminate parameter and make it more flexible?
      {                                                           //        It's not high-performance anywa
        uint64_t tdo(0x0);
        jtagShift(TMS); // Select-DR-Scan
        jtagShift(0);   // Capture-DR
        jtagShift(0);   // Shift-DR

        // Shift data: LSB first, MSB has TMS set
        for(uint32_t i = 0; i < n_bits; i++) {
          uint8_t sv = (((data >> i) & 0x1) ? TDI : 0x0) | (i == n_bits-1 ? TMS : 0x0);
          tdo |= (static_cast<uint64_t>(jtagShift(sv)) << i);
        }

        jtagShift(TMS); // Exit1-DR -> Update-DR
        jtagShift(0); // Back to IDLE
        return tdo;
      }

    /** Determine Instruction Register length
     *  @return Length of Instruction Register
     */
    uint32_t jtagDetermineIrLength(void)
      {
        jtagShift(TMS); // Idle -> Select-DR-Scan
        jtagShift(TMS); // Select-DR-Scan -> Select-IR-Scan
        jtagShift(0);   // Select-IR-Scan -> Capture-IR
        jtagShift(0);   // Capture-IR -> Shift-IR
        // Send zeroes, the ones until we start seeing the ones
        uint32_t ir_len;
        for(ir_len = 0; ir_len < 10; ir_len++){ jtagShift(static_cast<uint8_t>(0)); }
        for(ir_len = 0; ir_len < 10; ir_len++){ if(jtagShift(TDI) == 1) break; }
        jtagReset(); // Back to Idle
        return ir_len;
      }

    /** Determine number of devices in JTAG chain
     *  @return Number of devices
     */
    uint32_t jtagDetermineChainLength(void)
      {
        jtagShiftIr(BYPASS);
        // Shift zeroes, then shift a one and check where/when it appears
        jtagShiftDr(static_cast<uint64_t>(0x0), 10);
        uint64_t chain = jtagShiftDr(static_cast<uint64_t>(0x1), 10);
        int chain_len;
        for(chain_len=0; chain_len<10; chain_len++){
          if((chain>>chain_len) & 1) break;
        }
        jtagReset(); // Back to Idle
        return chain_len;
      }

    /** Get JTAG chain outputs
     *  @return Outputs in JTAG shift register in correct order
     */
    inline uint32_t jtagGetPrimaryOutputs(void)
      {
        // Bits in JTAG order outputs[0...12] (1...13 in SAMPA datasheet)
        uint32_t dft_pins = dftGetPrimaryOutputs();
        uint32_t jtag_pins = 0;
        jtag_pins = (
                     ( (dft_pins >> BIT_RXRED_SMO)    & 1 )
                    |( ((dft_pins >> BIT_RXRED_SDO_0)  & 1 ) << 1 )
                    |( ((dft_pins >> BIT_RXRED_SDO_1)  & 1 ) << 2 )
                    |( ((dft_pins >> BIT_RXRED_SDO_2)  & 1 ) << 3 )
                    |( ((dft_pins >> BIT_RXRED_SDO_3)  & 1 ) << 4 )
                    |( ((dft_pins >> BIT_RXRED_SDO_4)  & 1 ) << 5 )
                    |( ((dft_pins >> BIT_RXRED_SERIALOUT_0) & 1 ) <<  6 )
                    |( ((dft_pins >> BIT_RXRED_SERIALOUT_1) & 1 ) <<  7 )
                    |( ((dft_pins >> BIT_RXRED_SERIALOUT_2) & 1 ) <<  8 )
                    |( ((dft_pins >> BIT_RXRED_SERIALOUT_3) & 1 ) <<  9 )
                    |( ((dft_pins >> BIT_RXRED_SERIALOUT_4) & 1 ) << 10 )
                    |( ((dft_pins >> BIT_RXRED_NBFLOWSTOP)  & 1 ) << 11 )
                    |( ((dft_pins >> BIT_RXRED_SDA)         & 1 ) << 12 )
                    );
        return jtag_pins;
      }

    /** Initialize hardware for DFT (mostly applies to JTAG/DFT controller)
     */
    inline void dftInitialize() const
      {
        link_->dftCtrlShrOutput(0x1);
        link_->dftCtrlTxPath(0x1);
        link_->dftCtrlShrAutoLoad(0x1);
        link_->dftCtrlSclkAutoToggle(0x0);
        link_->dftCtrlShrFastLoad(0x1);
      }

    inline void dftDeInitialize() const
      {
        link_->dftCtrlShrOutput(0x0);
        link_->dftCtrlTxPath(0x0);
        link_->dftCtrlShrAutoLoad(0x0);
        link_->dftCtrlSclkAutoToggle(0x0);
        link_->dftCtrlShrFastLoad(0x0);
      }

    /** Get (condensed) data from GBT uplink capture register
     *  @return Condensed value of the uplink register. There will be only 1 bit per pin instead of 4, and the
     *          return value has bit is at position LSB_UCR_<PIN>/4 set, if the corresponding 4-bit field in the
     *          capture register is 0xf
     */
    inline uint32_t dftGetPrimaryOutputs() const
      { return link_->getDftRxData(); }

    /** Get SDO data from uplink register only
     */
    inline uint8_t dftGetPrimaryOutputsSdo() const
      { return (dftGetPrimaryOutputs() >> BIT_RXRED_SDO_0) & 0x1f; }

    /** Definition of 'direct' pins in downlink idle/control pattern
     */
    static const std::vector<uint8_t> dftPrimaryInputDirectNormalPins;
    /**
     */
    static const std::vector<uint8_t> dftPrimaryInputDirectClockPins;

    inline void dftSetPrimaryInput(uint32_t v) const { link_->setDftCtrlPinStimulus(v); }

    /** Set GBT downlink frame direct pins (without SCLK)
     */
    inline void dftSetPrimaryInputDirect(uint32_t v, const std::vector<uint8_t>& pins) const
      {
        uint32_t d(link_->getDftCtrlPinStimulus());
        for (auto p: pins)
          d = setBit(d, p, (v >> p) & 0x1);
        link_->setDftCtrlPinStimulus(d);
      }

    inline void dftSetPrimaryInputDirectNormal(uint32_t v) const
      { dftSetPrimaryInputDirect(v, dftPrimaryInputDirectNormalPins); }

    inline void dftSetPrimaryInputDirectClocks(uint32_t v) const
      { dftSetPrimaryInputDirect(v, dftPrimaryInputDirectClockPins); }

    /** Control TX stimulus pin (for JTAG/DFT)
     *  @param value New pin value (0->low, others->high)
     *  @param cycle If set, pin is driven first with <value> first, and afterwards with <not-value>
     */
    inline void dftControlPinTx(uint8_t idx, uint8_t value, bool cycle = false) const
      {
        uint32_t o = link_->getDftCtrlPinStimulus();
        link_->setDftCtrlPinStimulus(setBit(o, idx, value));
        if (cycle)
          link_->setDftCtrlPinStimulus(setBit(o, idx, (~value) & 0x1));
      }

    /** Control SCLK pin (for DFT)
     *  @param value New pin value (0->low, others->high)
     *  @param cycle If set, pin is driven first with <value> first, and afterwards with <not-value>
     */
    inline void dftControlSclk(uint8_t value, bool cycle = false) const
      { dftControlPinTx(BIT_TX_SCLK, value, cycle); }

    /** Special version of shrLoad for DFT that preserves (old) setting of SDA pin
     */
    void dftShrLoad(uint8_t shr0, uint8_t shr1, uint8_t mask0 = 0xff, uint8_t mask1 = 0xff) const
      { link_->setDftCtrlShr(shr0, shr1, mask0, mask1); }

    void dftShrAutoSclk(uint8_t enable = 0x1) const { link_->dftCtrlSclkAutoToggle(enable); }

    /** Selectively apply new values to sen_* inputs
     */
    inline void dftSetPrimaryInputsSen(uint8_t v) const
      {
        uint8_t mask0((1 << BIT_SHR_SEN_3) |
                      (1 << BIT_SHR_SEN_1));
        uint8_t mask1((1 << BIT_SHR_SEN_4) |
                      (1 << BIT_SHR_SEN_2)|
                      (1 << BIT_SHR_SEN_0));
        uint8_t shr0((((v >> 3) & 0x1) << BIT_SHR_SEN_3) |
                     (((v >> 1) & 0x1) << BIT_SHR_SEN_1));
        uint8_t shr1((((v >> 4) & 0x1) << BIT_SHR_SEN_4) |
                     (((v >> 2) & 0x1) << BIT_SHR_SEN_2) |
                     (((v >> 0) & 0x1 << BIT_SHR_SEN_0)));
        dftShrLoad(shr0, shr1, mask0, mask1);
      }

    /** Selectively apply new values to sdi_* inputs
     */
    inline void dftSetPrimaryInputsSdi(uint8_t v) const
      {
        uint8_t mask0((1 << BIT_SHR_SDI_3) |
                      (1 << BIT_SHR_SDI_1));
        uint8_t mask1((1 << BIT_SHR_SDI_4) |
                      (1 << BIT_SHR_SDI_2)|
                      (1 << BIT_SHR_SDI_0));
        uint8_t shr0((((v >> 3) & 0x1) << BIT_SHR_SDI_3) |
                     (((v >> 1) & 0x1) << BIT_SHR_SDI_1));
        uint8_t shr1((((v >> 4) & 0x1) << BIT_SHR_SDI_4) |
                     (((v >> 2) & 0x1) << BIT_SHR_SDI_2) |
                     (((v >> 0) & 0x1 << BIT_SHR_SDI_0)));
        dftShrLoad(shr0, shr1, mask0, mask1);
      }

};


const std::vector<uint8_t> PccaInterface::dftPrimaryInputDirectNormalPins = {
  BIT_TX_HRSTB,
  BIT_TX_BXSYNCTRG,
  BIT_TX_HBTRG,
  BIT_TX_TRG,
  BIT_TX_DINN,
  BIT_TX_NBFLOWSTOP,
  BIT_TX_TCLK,
  BIT_TX_SME
};

const std::vector<uint8_t> PccaInterface::dftPrimaryInputDirectClockPins = {
  BIT_TX_CLKS0,
  BIT_TX_CLKBX,
  BIT_TX_CLKADC
};
