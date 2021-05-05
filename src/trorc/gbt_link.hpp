/**
 *  gbtlink.hpp
 *  Copyright (C) 2016 Heiko Engel <hengel@cern.ch>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 **/
#pragma once

#include <librorc.h>
#include "trorc_registers.h"
#include "trorc.hpp"

#include <iostream>
#include <iomanip>

/** Some very verbose output to debug DFT controller-related register accesses
 */
#ifdef DFT_DEBUG
  #define dftdebug(s) std::cout << __func__ << " " << s << std::endl;
  #define dftdebughex(v) dftdebug(std::hex << std::setfill('0') << " 0x" << std::setw(8) << v << std::dec << std::setfill(' '))
#else
  #define dftdebug(...)
  #define dftdebughex(...)
#endif

namespace trorc {

class GbtLink
{
  private:
    const uint32_t base_addr_;
    const uint32_t base_addr_tx_;
    const uint32_t base_addr_rx_;

    Bar& bar_;

    std::unique_ptr<librorc::link> link_;
    std::unique_ptr<librorc::gtx> gtx_;

  public:
    GbtLink(Bar& bar, uint32_t link_number = 0) :
        base_addr_((link_number + 1) * RORC_CHANNEL_OFFSET),
        base_addr_tx_(base_addr_ + (1 << TRORC_REGFILE_GTXTX_SEL)),
        base_addr_rx_(base_addr_ + (1 << TRORC_REGFILE_GTXRX_SEL)),
        bar_(bar),
        link_(make_unique<librorc::link>(bar_.get(), link_number)),
        gtx_(make_unique<librorc::gtx>(link_.get()))
      {}

    /**
     * read SC register from dmaregfile
     * @param addr register address in dmaregfile
     * @return data at give address
     **/
    uint32_t pciReg(uint32_t addr) const;

    /**
     * write to SC register in dmaregfile
     * @addr register address in dmaregfile
     * @data data to be written
     **/
    void setPciReg(uint32_t addr, uint32_t data) const;

    /**
     * read SC register from gbt_rxfrmclk_regfile
     * @param addr register address in gbt_rxfrmclk_regfile
     * @return data at give address
     **/
    uint32_t gbtRxReg(uint32_t addr) const;

    /**
     * write to SC register in gbt_rxfrmclk_regfile
     * @addr register address in gbt_rxfrmclk_regfile
     * @data data to be written
     **/
    void setGbtRxReg(uint32_t addr, uint32_t data) const;

    /**
     * read SC register from gbt_txfrmclk_regfile
     * @param addr register address in gbt_txfrmclk_regfile
     * @return data at give address
     **/
    uint32_t gbtTxReg(uint32_t addr) const;

    /**
     * write to SC register in gbt_txfrmclk_regfile
     * @addr register address in gbt_txfrmclk_regfile
     * @data data to be written
     **/
    void setGbtTxReg(uint32_t addr, uint32_t data) const;

    /**
     * set GBT reset
     * @param val reset value, 0 or 1
     **/
    void setReset(uint32_t val) const;

    /**
     * get GBT reset state
     * @return 1 if in reset, 0 otherwise
     **/
    uint32_t getReset() const;

    /**
     * check if RX Frame Clock is Ready
     * @return 1 if ready, 0 otherwise
     **/
    uint32_t rxFrameClockReady() const;

    /**
     * check if TX Frame Clock is Ready
     * @return 1 if ready, 0 otherwise
     **/
    uint32_t txFrameClockReady() const;

    /**
     * check if GBT_RX_READY is active
     * @return 1 if RX_READT is active, 0 otherwise
     **/
    uint32_t rxReady() const;

    /**
     * wait for RX_READY to assert
     * @return 0 if RX_READY is active, -1 if RX_READY did not become active
     * within 5 seconds.
     **/
    int waitForRxReady() const;

    /**
     * clear the RX_READY-lost counter
     **/
    void clearRxReadyLostCount() const;

    /**
     * get the current state of the RX_READY-lost counter. Note: the counter does
     * not clear itself with a GBT reset.
     * @return number of RX_READY loss counts since the counter was cleared the
     * last time
     **/
    uint32_t rxReadyLostCount() const;

    /**
     * Set RX polarity of the GTX
     * @param val 0 for regular, 1 for inverted polarity
     **/
    void setRxPolarity(uint32_t val) const;

    /**
     * Set TX polarity of the GTX
     * @param val 0 for regular, 1 for inverted polarity
     **/
    void setTxPolarity(uint32_t val) const;

    /**
     * Get current RX pattern checker mode
     * @return 0: e-links/disabled, 1: static pattern, 2: counter pattern, 3: prbs
     * pattern
     **/
    uint32_t patternCheckerMode() const;

    /**
     * Set RX pattern checker mode
     * @param mode 0: e-links/disabled, 1: static pattern, 2: counter pattern, 3:
     * prbs pattern
     **/
    void setPatternCheckerMode(uint32_t mode) const;

    /**
     * Get current RX pattern checker widebus mode
     * @return 1 if expecting widebus data, 0 if expecting standard data
     **/
    uint32_t patternCheckerWidebus() const;

    /**
     * Set RX pattern checker widebus mode
     * @param mode 1 for widebus, 0 for standard mode
     **/
    void setPatternCheckerWidebus(uint32_t mode) const;

    /**
     * get current pattern checker error count
     * @return number of pattern checker errors
     **/
    uint32_t patternCheckerErrorCount() const;

    /**
     * clear pattern checker error count
     **/
    void clearPatternCheckerErrorCount() const;

    /** Struct to access GBT frame downlink pattern registers
     */
    using downlink_pattern_t = std::array<uint32_t, 3>;

    /**
     * Set TX Idle Pattern
     * @param highdw bit16 is frame-vld flag, bits [15:0] contain txframe[79:64]
     * @param middw contains txframe[63:32]
     * @param lowdw contains txframe[31:0]
     **/
    inline void setTxIdlePattern(uint32_t highdw, uint32_t middw, uint32_t lowdw) const
      {
        setTxIdlePatternLow(lowdw);
        setTxIdlePatternMid(middw);
        setTxIdlePatternHigh(highdw);
      }

    /** Set TX Idle Pattern
     *  @param d Downlink pattern content (of type downlink_patter_t)
     *  @see downlink_pattern_t
     *  @see setTxIdlePattern
     */
    inline void setTxIdlePattern(const downlink_pattern_t& d) const { setTxIdlePattern(d[2], d[1], d[0]); }

    /** Get TX idle pattern
     *  @return Unique pointer to std::array containing all three idle pattern words (0->low, 1->mid, 2->high)
     */
    inline std::unique_ptr<downlink_pattern_t> getTxIdlePattern()
      {
        std::unique_ptr<downlink_pattern_t> p(new downlink_pattern_t);
        (*p)[0] = txIdlePatternLow();
        (*p)[1] = txIdlePatternMid();
        (*p)[2] = txIdlePatternHigh();
        return std::move(p);
      }

    inline void setTxIdlePatternLow(uint32_t v) const { setGbtTxReg(TRORC_REG_GBTTX_IDLEPATTERN_LOW, v); }
    inline void setTxIdlePatternMid(uint32_t v) const { setGbtTxReg(TRORC_REG_GBTTX_IDLEPATTERN_MID, v); }
    inline void setTxIdlePatternHigh(uint32_t v) const { setGbtTxReg(TRORC_REG_GBTTX_IDLEPATTERN_HIGH, v); }

    uint32_t txIdlePatternHigh() const { return gbtTxReg(TRORC_REG_GBTTX_IDLEPATTERN_HIGH); }
    uint32_t txIdlePatternMid() const { return gbtTxReg(TRORC_REG_GBTTX_IDLEPATTERN_MID); }
    uint32_t txIdlePatternLow() const { return gbtTxReg(TRORC_REG_GBTTX_IDLEPATTERN_LOW); }

    /**
     * Set TX Control Pattern
     * @param highdw bit16 is frame-vld flag, bits [15:0] contain txframe[79:64]
     * @param middw contains txframe[63:32]
     * @param lowdw contains txframe[31:0]
     **/
    void setTxControlPattern(uint32_t highdw, uint32_t middw, uint32_t lowdw) const
      {
        setTxControlPatternLow(lowdw);
        setTxControlPatternMid(middw);
        setTxControlPatternHigh(highdw);
      }

    /** Set TX Control Pattern
     *  @param d Downlink pattern content (of type downlink_patter_t)
     *  @see downlink_pattern_t
     *  @see setTxControlPattern
     */
    inline void setTxControlPattern(const downlink_pattern_t& d) const { setTxControlPattern(d[2], d[1], d[0]); }

    /** Get TX control pattern
     *  @return Unique pointer to std::array containing all three control pattern words (0->low, 1->mid, 2->high)
     */
    inline std::unique_ptr<downlink_pattern_t> getTxControlPattern()
      {
        std::unique_ptr<downlink_pattern_t> p(new downlink_pattern_t);
        (*p)[0] = txControlPatternLow();
        (*p)[1] = txControlPatternMid();
        (*p)[2] = txControlPatternHigh();
        return std::move(p);
      }

    inline void setTxControlPatternLow(uint32_t v) const { setGbtTxReg(TRORC_REG_GBTTX_CONTROLPATTERN_LOW, v); }
    inline void setTxControlPatternMid(uint32_t v) const { setGbtTxReg(TRORC_REG_GBTTX_CONTROLPATTERN_MID, v); }
    inline void setTxControlPatternHigh(uint32_t v) const { setGbtTxReg(TRORC_REG_GBTTX_CONTROLPATTERN_HIGH, v); }

    uint32_t txControlPatternHigh() const { return gbtTxReg(TRORC_REG_GBTTX_CONTROLPATTERN_HIGH); }
    uint32_t txControlPatternMid() const { return gbtTxReg(TRORC_REG_GBTTX_CONTROLPATTERN_MID); }
    uint32_t txControlPatternLow() const { return gbtTxReg(TRORC_REG_GBTTX_CONTROLPATTERN_LOW); }


    /**
     * Get the number of cycles the control pattern will be driven
     * @return number of clock cycles
     **/
    uint32_t txControlPatternCycles() const;

    /**
     * Set control pattern cycles
     * @param cycles Number of cycles the control pattern is sent
     * @return Value read back
     */
    uint32_t setTxControlPatternCycles(uint32_t cycles) const;

    /**
     * drive the control pattern for a given amount of clock cycles
     * @param trigger_readout if set true the control pattern also triggers a
     * readout on the RX path
     **/
    void driveControlPattern(bool trigger_readout) const;

    /**
     * trigger the sending of the control pattern without starting a readout
     * process
     **/
    void triggerControlPattern() const;

    /**
     * check if control pattern is currently active
     * @return 1 if active, 0 otherwise
    **/
    uint32_t controlPatternActive() const;

    /**
     * stop sending the control pattern
     **/
    void disableControlPattern() const;

    /**
     * trigger a readout process
     **/
    void triggerReadout() const;

    /**
     * set number of frames that should be read out on a readout trigger
     * @param framecount number of frames to read out
     **/
    void setRxReadoutTargetFrameCount(uint32_t framecount) const;

    /**
     * get the number of frames to be read out on a readout trigger
     * @return number of frames to read out
     **/
    uint32_t rxReadoutTargetFrameCount() const;

    /**
     * get the current number of frames read out
     * @return number of frames read out
     **/
    uint32_t rxReadoutCurrentFrameCount() const;

    /**
     * Clear the current number of frames read out
     **/
    void clearRxReadoutCurrentFrameCount() const;

    /**
     * set the number of frames that should be packed into one event
     * @param framecount number of frames per event
     **/
    void setRxEventSizeFrameCount(uint32_t framecount) const;

    /**
     * get the number of frames that should be packed into one event
     * @return number of frames per event
     **/
    uint32_t rxEventSizeFrameCount() const;

    /**
     * stop the readout process if ongoing
     **/
    void stopReadout() const;

    /**
     * get numer of times the control pattern was activated
     * @return number of times the control pattern was activated
     **/
    uint32_t controlPatternStartCount() const;

    /**
     * clear the control pattern start counter
     **/
    void clearControlPatternStartCount() const;

    /**
     * check if external triggers are enabled
     * @return 1 if enabled, 0 if disbaled
     **/
    uint32_t allowExternalTrigger() const;

    /**
     * enable/disable external triggers
     * @param enable 1 to enable, 0 to disable
     **/
    void setAllowExternalTrigger(uint32_t enable) const;

    /**
    * get current software trigger channel mask. Returns a bit mask with one bit
    * for each GBT link in the firmware. A 1 means that the given link is read
    * out when a software trigger is sent from this link.
    * @return bit mask of links that are triggered from a software trigger on
    * this link.
    **/
    uint16_t softwareTriggerChannelMask() const;

    /**
     * set software trigger channel mask
     * @param mask bit mask of GBT links in the firmware that are read out with
     * a software trigger sent from this link.
     **/
    void setSoftwareTriggerChannelMask(uint16_t mask) const;

    /**
     * get current TX pattern channel mask. Returns a bit mask with one bit for
     * each GBT link in the firmware. A 1 means the TX control pattern is also
     * activated on that link if the control pattern is started on this link.
     * @return bit mask of links that also activate the control pattern on a
     * control pattern enable on this link.
     **/
    uint16_t txPatternChannelMask() const;

    /**
     * Set the TX pattern channel mask.
     * @param mask bit mask of GBT links in the firmware that also send the
     *control pattern if it is enabled for this link.
     **/
    void setTxPatternChannelMask(uint16_t mask) const;

    /**
     * get total number of triggers received on this link
     * @return number of triggers
     **/
    uint32_t triggerCount() const;

    /**
     * clear trigger counter
     **/
    void clearTriggerCount() const;

    /**
     * get error flag for 'control pattern start while active'
     * @return 1 if set, else '0'
     **/
    uint32_t controlPatternStartWhileActiveFlag() const;

    /**
     * clear error flag for 'control pattern start while active'
     **/
    void clearControlPatternStartWhileActiveFlag() const;

    /**
     * get error flag for 'trigger while busy'
     * @return 1 if set, else '0'
     **/
    uint32_t triggerWhileBusyFlag() const;

    /**
     * clear error flag for 'trigger while busy'
     **/
    void clearTriggerWhileBusyFlag() const;

    /**
     * get error flag for 'FIFO write error'
     * @return 1 if set, else '0'
     **/
    uint32_t fifoWriteErrorFlag() const;

    /**
     * clear error flag for 'FIFO write errors'
     **/
    void clearFifoWriteErrorFlag() const;

    /**
     * clear all error flags
     **/
    void clearErrorFlags() const;

    /**
     * ADC clock error counter
     * @param sampa ADC clock of this SAMPA
     * @return number of errors
     **/
    uint32_t decAdcErrorCounter(uint32_t sampa) const;

    /**
     * SyncPattern counter
     * @param halfSampa syncPattern counter if this half SAMPA
     * @return number of detected syncPattern
     **/
    uint32_t decSyncPatternCounter(uint32_t halfSampa) const;

    /**
     * ID error counter
     * @param sampa ID missmatch of this SAMPA
     * @return number of missmatches
     **/
    uint32_t decIdError(uint32_t sampa) const;

    /**
     * ADC clock status
     * @return status of all 3 ADC clocks
     **/
    uint32_t decAdcStatus() const;

    /**
     * ADC SyncPattern status
     * @return status of all 5 sync patterns
     **/
    uint32_t decSyncPatternStatus() const;

    /**
     * Set T-RORC readout mode
     * @param mode 0: readout disabled -> no data, 1: raw GBT frames,
     * 2: decoded data, 3: raw GBT frames + decoded data
     **/
    void setReadoutMode(uint32_t mode) const;

    /**
     * Get T-RORC readout mode
     * @return mode 0: readout disabled -> no data, 1: raw GBT frames,
     * 2: decoded data, 3: raw GBT frames + decoded data
     **/
    uint32_t getReadoutMode() const;

    /**
     * Resets several counter of the decoder
     * @param cnt bit 0: Adc clock monitor, bit 1: Sync pattern, bit 2: ID error
     **/
    void resetDecoderCounter(uint32_t cnt) const;

    /**
     * Resets Decoder and disable/enable
     * @param param bit 0: enable/disable decoder, bit 1: reset decoder,
     * bit 2: reset adc clock monitor, bit 3 reset channel extractor
     **/
    uint32_t setDecoder(uint32_t param) const;


    /** Reset GBT decoder channel extractor
     */
    inline void resetChannelExtractor() const {
        bar_.writeBit(base_addr_rx_ + TRORC_REG_GBTRX_DEC_CTRL, 3, 1);
      }

    /**
     * Decoder status
     * @return status of Decoder
     **/
    bool getDecoderStatus() const;

    /** Reset event counter (which counts the numbers of triggers received on the T-RORC
     */
    void resetEventCounter() const;


    /** Struct to access GBT frame capture registers
     */
    using frame_capture_t = std::array<uint32_t, 4>;

    /** Access to GBT frame capture registers
     *  @return Unique pointer to struct containing values read from registers
     */
    inline std::unique_ptr<frame_capture_t> getFrameCaptureReg() const {
        std::unique_ptr<frame_capture_t> gbt_frame(new frame_capture_t);;

        uint32_t idx(0);
        for (uint32_t& t: *gbt_frame)
          t = getFrameCaptureReg(idx++);

        return std::move(gbt_frame);
      }

    /** Access to specific GBT frame capture register
     *  @param idx GBT frame capture register index [0..3]
     *  @return Register value
     */
    inline uint32_t getFrameCaptureReg(uint32_t idx) const {
        return bar_.read(base_addr_rx_ + TRORC_REG_GBTRX_RX_DATA_0 + idx);
      }

    /** Get (DFT) receive data register (with 4bits -> 1bit reduction)
     */
    inline uint32_t getDftRxData() const
      {
        auto r = bar_.read(base_addr_rx_ + TRORC_REG_GBTRX_RX_DATA_RED);
        dftdebughex(r)
        return r;
      }

    /** Set DFT controller config register
     *  @param v New controller register value
     */
    inline void setDftCtrlConfig(uint32_t v) const
      { bar_.write(base_addr_tx_ + TRORC_REG_GBTTX_DFT_CONFIG, v); }

    /** Get DFT controller config register
     *  @return Current register value
     */
    inline uint32_t getDftCtrlConfig() const
      { return bar_.read(base_addr_tx_ + TRORC_REG_GBTTX_DFT_CONFIG); }

    /** Set individual bits of DFT controller control register
     *  @param idx Index of the bit to set
     *  @param v   Bit value
     *  @return New read-back value
     */
    inline uint8_t setDftCtrlConfig(uint8_t idx, uint8_t v) const
      { return bar_.writeBitRb(base_addr_tx_ + TRORC_REG_GBTTX_DFT_CONFIG, idx, v); }

    const uint8_t BIT_DFTCTRL_SHR_OUT_EN      {0};
    const uint8_t BIT_DFTCTRL_TX_PATH_SELECT  {1};
    const uint8_t BIT_DFTCTRL_SHR_AUTOLOAD    {2};
    const uint8_t BIT_DFTCTRL_SCLK_AUTOTOGGLE {3};
    const uint8_t BIT_DFTCTRL_SHR_REVERSE_CLK {4};
    const uint8_t BIT_DFTCTRL_SHR_FASTLOAD    {5};

    /** Enable or disable SHR outputs (via the DFT controller)
     *  @param enable Disable if zero, enable otherwise
     *  @return Read-back value: 0x1 if enabled, zero otherwise
     */
    inline uint8_t dftCtrlShrOutput(uint8_t enable = 0x1) const
      { return (setDftCtrlConfig(BIT_DFTCTRL_SHR_OUT_EN, enable ? 0x0 : 0x1) ? 0x0 : 0x1); }

    /** Control DFT TX path select
     *  @param enable DFT controller TX path not selected/enabled if zero, selected otherwise
     *  @return Read-back value: 0x1 if selected, zero otherwise
     */
    inline uint8_t dftCtrlTxPath(uint8_t enable = 0x1) const
      { return setDftCtrlConfig(BIT_DFTCTRL_TX_PATH_SELECT, enable); }

    /** Enable or disable DFT controller SHR autoload feature
     *  @param enable Disable if zero, enable otherwise
     *  @return Read-back value: 0x1 if enabled, zero otherwise
     */
    inline uint8_t dftCtrlShrAutoLoad(uint8_t enable = 0x1) const
      { return setDftCtrlConfig(BIT_DFTCTRL_SHR_AUTOLOAD, enable); }

    /** Enable or disable fast loading of SHR (load clk cycle in one instead of two GBT frames)
     *  @param enable Disable if zero, enable otherwise
     *  @return Read-back value: 0x1 if enabled, zero otherwise
     */
    inline uint8_t dftCtrlShrFastLoad(uint8_t enable = 0x1) const
      { return setDftCtrlConfig(BIT_DFTCTRL_SHR_FASTLOAD, enable); }

    /** Enable or disable SHR load clock revert
     *  @param enable Disable if zero, enable otherwise
     *  @return Read-back value: 0x1 if enabled, zero otherwise
     */
    inline uint8_t dftCtrlShrReverseLoadClk(uint8_t enable = 0x1) const
      { return setDftCtrlConfig(BIT_DFTCTRL_SHR_REVERSE_CLK, enable); }

    /** Enable or disable SCLK autotoggle after successful SHR load
     *  @param enable Disable if zero, enable otherwise
     *  @return Read-back value: 0x1 if enabled, zero otherwise
     */
    inline uint8_t dftCtrlSclkAutoToggle(uint8_t enable = 0x1) const
      { return setDftCtrlConfig(BIT_DFTCTRL_SCLK_AUTOTOGGLE, enable); }

    /** Set DFT controller pin stimulus register
     *  @param v New pin stimulus (TX) register
     */
    inline void setDftCtrlPinStimulus(uint32_t v) const
      {
        dftdebughex(v)
        bar_.write(base_addr_tx_ + TRORC_REG_GBTTX_DFT_TX_DATA, v);
      }

    /** Get DFT controller pin stimulus register
     *  @return Read-back pin stimulus (TX) register value
     */
    inline uint32_t getDftCtrlPinStimulus() const
      { return bar_.read(base_addr_tx_ + TRORC_REG_GBTTX_DFT_TX_DATA); }

    /** Set DFT controller SHR data/mask register. The masks control which bits in the
     *    corresponding data are actually set in the corresponding SHRs, e.g. mask0 = 0x5 alters
     *    bits 0 and 2 of SHR0 to the values specified in data0[0] and data0[2]. Data bits with
     *    corresponding mask bit of 0 remain unchanged in the SHR
     *  param data0 Data for first SHR
     *  param data1 Data for second SHR
     *  param mask0 Mask for first SHR
     *  param mask1 Mask for second SHR
     */
    inline void setDftCtrlShr(uint8_t data0, uint8_t data1, uint8_t mask0, uint8_t mask1) const
      {
        uint32_t v = (static_cast<uint32_t>(data0) << 16) | (static_cast<uint32_t>(data1) << 24) |
                     (static_cast<uint32_t>(mask0) << 0)  | (static_cast<uint32_t>(mask1) << 8);
        bar_.write(base_addr_tx_ + TRORC_REG_GBTTX_DFT_VALMASK, v);
        dftdebughex(v)
      }

    /** Get current DFT controller SHR control register value
     *  @return Read-back value
     *  @warn Does not necessarily reflect the actual status of the SHR outputs though)
     */
    inline uint32_t getDftCtrlShr() const
      { return bar_.read(base_addr_tx_ + TRORC_REG_GBTTX_DFT_VALMASK); }
};

}  // namespace trorc
