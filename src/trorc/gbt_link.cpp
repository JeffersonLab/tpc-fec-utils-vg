/**
 *  GbtLink.cpp
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

#include "gbt_link.hpp"

namespace trorc {

/** gbt_rxfrmclk_regfile register access **/
void
GbtLink::setGbtRxReg(uint32_t addr, uint32_t data) const {
  bar_.write(base_addr_ + (1 << TRORC_REGFILE_GTXRX_SEL) + addr, data);
}

uint32_t
GbtLink::gbtRxReg(uint32_t addr) const {
  return bar_.read(base_addr_ + (1 << TRORC_REGFILE_GTXRX_SEL) + addr);
}

/** gbt_txfrmclk_regile register access **/
void
GbtLink::setGbtTxReg(uint32_t addr, uint32_t data) const {
  bar_.write(base_addr_ + (1 << TRORC_REGFILE_GTXTX_SEL) + addr, data);
}

uint32_t
GbtLink::gbtTxReg(uint32_t addr) const {
  return bar_.read(base_addr_ + (1 << TRORC_REGFILE_GTXTX_SEL) + addr);
}

/** dmaregfile register access **/
void
GbtLink::setPciReg(uint32_t addr, uint32_t data) const {
  bar_.write(base_addr_ + addr, data);
}

uint32_t
GbtLink::pciReg(uint32_t addr) const {
  return bar_.read(base_addr_ + addr);
}

/** GBT Reset **/
void
GbtLink::setReset(uint32_t val) const {
  gtx_->setReset(val);
}

uint32_t
GbtLink::getReset() const {
  return gtx_->getReset();
}

uint32_t
GbtLink::rxReady() const {
  return ((pciReg(TRORC_REG_GBT_ASYNC_CTRL) >> 1) & 1);
}

/** Frame Clocks Ready **/
uint32_t
GbtLink::txFrameClockReady() const {
  return ((~pciReg(TRORC_REG_GTX_ASYNC_CFG) >> 7) & 1);
}

uint32_t
GbtLink::rxFrameClockReady() const {
  return ((~pciReg(TRORC_REG_GTX_ASYNC_CFG) >> 8) & 1);
}

int
GbtLink::waitForRxReady() const {
  uint32_t ready = rxReady();
  uint32_t timeout = 5000; // GBT init takes 4sec., so wait 5 sec here...
  while (!ready && timeout != 0) {
    usleep(1000);
#ifndef MODELSIM
    // don't use timeout in Simulation
    timeout--;
#endif
    ready = rxReady();
  }
  return (timeout == 0) ? -1 : 0;
}

/** RX Ready Lost Counter **/
void
GbtLink::clearRxReadyLostCount() const {
  setGbtRxReg(TRORC_REG_GBTRX_READY_LOST_COUNT, 0);
}

uint32_t
GbtLink::rxReadyLostCount() const {
  return gbtRxReg(TRORC_REG_GBTRX_READY_LOST_COUNT);
}

/** GTX RX/TX Polarity **/
void
GbtLink::setTxPolarity(uint32_t val) const {
  uint32_t ctrlreg = pciReg(TRORC_REG_GBT_ASYNC_CTRL);
  ctrlreg &= ~(1<<5);
  ctrlreg |= ((val & 1) << 5);
  setPciReg(TRORC_REG_GBT_ASYNC_CTRL, ctrlreg);
}

void
GbtLink::setRxPolarity(uint32_t val) const {
  uint32_t ctrlreg = pciReg(TRORC_REG_GBT_ASYNC_CTRL);
  ctrlreg &= ~(1<<4);
  ctrlreg |= ((val & 1) << 4);
  setPciReg(TRORC_REG_GBT_ASYNC_CTRL, ctrlreg);
}

/** RX Pattern Checker **/
void
GbtLink::setPatternCheckerMode(uint32_t mode) const {
  uint32_t ctrlreg = gbtRxReg(TRORC_REG_GBTRX_CTRL);
  ctrlreg &= ~(3 << 2);
  ctrlreg |= (mode & 3) << 2;
  setGbtRxReg(TRORC_REG_GBTRX_CTRL, ctrlreg);
}

uint32_t
GbtLink::patternCheckerMode() const {
  return ((gbtRxReg(TRORC_REG_GBTRX_CTRL) >> 2) & 3);
}

void
GbtLink::setPatternCheckerWidebus(uint32_t mode) const {
  uint32_t ctrlreg = gbtRxReg(TRORC_REG_GBTRX_CTRL);
  ctrlreg &= ~(1 << 4);
  ctrlreg |= (mode & 1) << 4;
  setGbtRxReg(TRORC_REG_GBTRX_CTRL, ctrlreg);
}

uint32_t
GbtLink::patternCheckerWidebus() const {
  return ((gbtRxReg(TRORC_REG_GBTRX_CTRL) >> 4) & 1);
}

void
GbtLink::clearPatternCheckerErrorCount() const {
  setGbtRxReg(TRORC_REG_GBTRX_PATTERN_CHECKER_ERROR_COUNT, 0);
}

uint32_t
GbtLink::patternCheckerErrorCount() const {
  return gbtRxReg(TRORC_REG_GBTRX_PATTERN_CHECKER_ERROR_COUNT);
}

/** Current RX Frame Readout Count **/
uint32_t
GbtLink::rxReadoutCurrentFrameCount() const {
  return gbtRxReg(TRORC_REG_GBTRX_FRM_COUNT);
}

void
GbtLink::clearRxReadoutCurrentFrameCount() const {
  setGbtRxReg(TRORC_REG_GBTRX_FRM_COUNT, 0);
}

/** TX pattern selection **/
uint32_t
GbtLink::txControlPatternCycles() const {
  return bar_.read(base_addr_tx_ + TRORC_REG_GBTTX_CONTROLPATTERN_CYCLES);
}

uint32_t
GbtLink::setTxControlPatternCycles(uint32_t cycles) const {
  return bar_.writeRb(base_addr_tx_ + TRORC_REG_GBTTX_CONTROLPATTERN_CYCLES, cycles);
}

void
GbtLink::driveControlPattern(bool trigger_readout) const {
  bar_.writeField(base_addr_tx_ + TRORC_REG_GBTTX_CTRL, 0, 2, (trigger_readout ? 0x3 : 0x1));
}

void GbtLink::stopReadout() const {
  bar_.writeBit(base_addr_rx_ + TRORC_REG_GBTRX_CTRL, 5, 1);
}

void
GbtLink::triggerReadout() const {
  bar_.writeBit(base_addr_tx_ + TRORC_REG_GBTTX_CTRL, 1, 1);
}

void
GbtLink::triggerControlPattern() const {
  bar_.writeBit(base_addr_tx_ + TRORC_REG_GBTTX_CTRL, 0, 1);
}

uint32_t GbtLink::controlPatternActive() const {
  return (gbtTxReg(TRORC_REG_GBTTX_CTRL) >> 0) & 1;
}

void GbtLink::disableControlPattern() const {
  uint32_t txctrl = gbtTxReg(TRORC_REG_GBTTX_CTRL);
  txctrl |= (1 << 3);
  setGbtTxReg(TRORC_REG_GBTTX_CTRL, txctrl);
}

uint32_t GbtLink::controlPatternStartCount() const {
  return gbtTxReg(TRORC_REG_GBTTX_CONTROLPATTERN_START_COUNT);
}

void GbtLink::clearControlPatternStartCount() const {
  setGbtTxReg(TRORC_REG_GBTTX_CONTROLPATTERN_START_COUNT, 0);
}

/** RX Readout Frame Count **/
void GbtLink::setRxReadoutTargetFrameCount(uint32_t framecount) const {
  setGbtRxReg(TRORC_REG_GBTRX_TARGET_FRAME_COUNT, framecount);
}

uint32_t
GbtLink::rxReadoutTargetFrameCount() const {
  return gbtRxReg(TRORC_REG_GBTRX_TARGET_FRAME_COUNT);
}

/** RX Event Size Frame Count **/
void
GbtLink::setRxEventSizeFrameCount(uint32_t framecount) const {
  setGbtRxReg(TRORC_REG_GBTRX_EVENT_SIZE_FRAMES, framecount);
}

uint32_t
GbtLink::rxEventSizeFrameCount() const {
  return gbtRxReg(TRORC_REG_GBTRX_EVENT_SIZE_FRAMES);
}

uint32_t GbtLink::allowExternalTrigger() const {
  return (gbtTxReg(TRORC_REG_GBTTX_CTRL) >> 2) & 1;
}

void GbtLink::setAllowExternalTrigger(uint32_t enable) const {
  uint32_t txctrl = gbtTxReg(TRORC_REG_GBTTX_CTRL);
  txctrl &= ~(1 << 2);
  txctrl |= (enable & 1) << 2;
  setGbtTxReg(TRORC_REG_GBTTX_CTRL, txctrl);
}

uint16_t GbtLink::softwareTriggerChannelMask() const {
  return (gbtTxReg(TRORC_REG_GBTTX_CTRL) >> 20) & 0xfff;
}

void GbtLink::setSoftwareTriggerChannelMask(uint16_t mask) const {
  uint32_t txctrl = gbtTxReg(TRORC_REG_GBTTX_CTRL);
  txctrl &= ~(0xfff << 20);
  txctrl |= (mask & 0xfff) << 20;
  setGbtTxReg(TRORC_REG_GBTTX_CTRL, txctrl);
}

uint16_t GbtLink::txPatternChannelMask() const {
  return (gbtTxReg(TRORC_REG_GBTTX_CTRL) >> 8) & 0xfff;
}

void GbtLink::setTxPatternChannelMask(uint16_t mask) const {
  uint32_t txctrl = gbtTxReg(TRORC_REG_GBTTX_CTRL);
  txctrl &= ~(0xfff << 8);
  txctrl |= (mask & 0xfff) << 8;
  setGbtTxReg(TRORC_REG_GBTTX_CTRL, txctrl);
}

uint32_t GbtLink::triggerCount() const {
  return gbtRxReg(TRORC_REG_GBTRX_TRIGGER_COUNT);
}

void GbtLink::clearTriggerCount() const {
  setGbtRxReg(TRORC_REG_GBTRX_TRIGGER_COUNT, 0);
}

uint32_t GbtLink::controlPatternStartWhileActiveFlag() const {
  return (gbtTxReg(TRORC_REG_GBTTX_CTRL) >> 4) & 1;
}

void GbtLink::clearControlPatternStartWhileActiveFlag() const {
  uint32_t txctrl = gbtTxReg(TRORC_REG_GBTTX_CTRL);
  txctrl &= ~(1 << 4);
  setGbtTxReg(TRORC_REG_GBTTX_CTRL, txctrl);
}

uint32_t GbtLink::triggerWhileBusyFlag() const {
  return (gbtRxReg(TRORC_REG_GBTRX_CTRL) >> 6) & 1;
}

void GbtLink::clearTriggerWhileBusyFlag() const {
  uint32_t rxctrl = gbtRxReg(TRORC_REG_GBTRX_CTRL);
  rxctrl &= ~(1 << 6);
  setGbtRxReg(TRORC_REG_GBTRX_CTRL, rxctrl);
}

uint32_t GbtLink::fifoWriteErrorFlag() const {
  return (gbtRxReg(TRORC_REG_GBTRX_CTRL) >> 1) & 1;
}

void GbtLink::clearFifoWriteErrorFlag() const {
  uint32_t rxctrl = gbtRxReg(TRORC_REG_GBTRX_CTRL);
  rxctrl &= ~(1 << 1);
  setGbtRxReg(TRORC_REG_GBTRX_CTRL, rxctrl);
}

void GbtLink::clearErrorFlags() const {
  // TX Flags
  clearControlPatternStartWhileActiveFlag();

  // RX Flags
  clearTriggerWhileBusyFlag();
  clearFifoWriteErrorFlag();
}

uint32_t GbtLink::decAdcErrorCounter(uint32_t sampa) const {
  sampa %= 3;
  switch (sampa) {
    case 0: return gbtRxReg(TRORC_REG_GBTRX_DEC_ADCCLOCK_0_ERROR_COUNT);
    case 1: return gbtRxReg(TRORC_REG_GBTRX_DEC_ADCCLOCK_1_ERROR_COUNT);
    case 2: return gbtRxReg(TRORC_REG_GBTRX_DEC_ADCCLOCK_2_ERROR_COUNT);
  }
  return 0;
}

uint32_t GbtLink::decSyncPatternCounter(uint32_t halfSampa) const {
  halfSampa %= 5;
  switch (halfSampa) {
    case 0: return gbtRxReg(TRORC_REG_GBTRX_DEC_SYNCPATTERN_0_ERROR_COUNT);
    case 1: return gbtRxReg(TRORC_REG_GBTRX_DEC_SYNCPATTERN_1_ERROR_COUNT);
    case 2: return gbtRxReg(TRORC_REG_GBTRX_DEC_SYNCPATTERN_2_ERROR_COUNT);
    case 3: return gbtRxReg(TRORC_REG_GBTRX_DEC_SYNCPATTERN_3_ERROR_COUNT);
    case 4: return gbtRxReg(TRORC_REG_GBTRX_DEC_SYNCPATTERN_4_ERROR_COUNT);
  }
  return 0;
}

uint32_t GbtLink::decIdError(uint32_t sampa) const {
  sampa %= 2;
  switch (sampa) {
    case 0: return gbtRxReg(TRORC_REG_GBTRX_DEC_ID_0_ERROR_COUNT);
    case 1: return gbtRxReg(TRORC_REG_GBTRX_DEC_ID_1_ERROR_COUNT);
  }
  return 0;
}

uint32_t GbtLink::decAdcStatus() const {
  return gbtRxReg(TRORC_REG_GBTRX_DEC_ADCCLOCK_FOUND) & 0x7;
}

uint32_t GbtLink::decSyncPatternStatus() const {
  return gbtRxReg(TRORC_REG_GBTRX_DEC_SYNCPATTERN_STS) & 0x7FFF;
}

void GbtLink::setReadoutMode(uint32_t mode) const {
  uint32_t ctrlreg = gbtRxReg(TRORC_REG_GBTRX_CTRL);
  ctrlreg &= ~(0x3 << 7);
  ctrlreg |= (mode & 0x3) << 7;
  setGbtRxReg(TRORC_REG_GBTRX_CTRL, ctrlreg);
}

uint32_t GbtLink::getReadoutMode() const {
  return (gbtRxReg(TRORC_REG_GBTRX_CTRL) >> 7) & 0x3;
}

void GbtLink::resetDecoderCounter(uint32_t cnt) const {
  uint32_t ctrlreg = gbtRxReg(TRORC_REG_GBTRX_DEC_CTRL);
  ctrlreg &= ~(0x7 << 4);
  ctrlreg |= (cnt & 0x7) << 4;
  setGbtRxReg(TRORC_REG_GBTRX_DEC_CTRL, ctrlreg);
}

uint32_t GbtLink::setDecoder(uint32_t param) const {
  return bar_.writeFieldRb(base_addr_rx_ + TRORC_REG_GBTRX_DEC_CTRL, 0, 4, param & 0xf);
}

bool GbtLink::getDecoderStatus() const {
  return gbtRxReg(TRORC_REG_GBTRX_DEC_CTRL) & 0x1;
}

void GbtLink::resetEventCounter() const {
  bar_.writeBit(base_addr_rx_ + TRORC_REG_GBTRX_CTRL, 9, 1);
}


}  // namespace trorc
