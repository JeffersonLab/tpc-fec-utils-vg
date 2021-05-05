/**
 *  Trigger.cpp
 *  Copyright (C) 2017 Heiko Engel <hengel@cern.ch>
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

#include "trigger.hpp"

namespace trorc {

uint32_t setBit(uint32_t input, uint32_t bit, uint32_t value) {
  assert(bit < 32);
  uint32_t reg = (input & ~(1u << bit));
  return reg | ((value & 1) << bit);
}

/** sysman_gbt_regile register access **/
void Trigger::setGbtTxReg(uint32_t addr, uint32_t data) const {
  bar_.set32((1 << TRORC_REGFILE_GTXRX_SEL) + addr, data);
}

uint32_t Trigger::gbtTxReg(uint32_t addr) const {
  return bar_.get32((1 << TRORC_REGFILE_GTXRX_SEL) + addr);
}

uint32_t Trigger::lvdsBufferEnable() const {
  return (gbtTxReg(TRORC_REG_SMGBT_TRG_CTRL) >> 31) & 1;
}

void Trigger::setLvdsBufferEnable(uint32_t enable) const {
  uint32_t ctrl = setBit(gbtTxReg(TRORC_REG_SMGBT_TRG_CTRL), 31, enable);
  setGbtTxReg(TRORC_REG_SMGBT_TRG_CTRL, ctrl);
}

uint32_t Trigger::edgeSensitivity() const {
  return (gbtTxReg(TRORC_REG_SMGBT_TRG_CTRL) >> 1) & 1;
}

void Trigger::setEdgeSensitivity(uint32_t type) const {
  uint32_t ctrl = setBit(gbtTxReg(TRORC_REG_SMGBT_TRG_CTRL), 1, type);
  setGbtTxReg(TRORC_REG_SMGBT_TRG_CTRL, ctrl);
}

uint32_t Trigger::triggerEnable() const {
  return (gbtTxReg(TRORC_REG_SMGBT_TRG_CTRL) >> 0) & 1;
}

void Trigger::setTriggerEnable(uint32_t enable) const {
  uint32_t ctrl = setBit(gbtTxReg(TRORC_REG_SMGBT_TRG_CTRL), 0, enable);
  setGbtTxReg(TRORC_REG_SMGBT_TRG_CTRL, ctrl);
}

uint32_t Trigger::triggerCount() const {
  return gbtTxReg(TRORC_REG_SMGBT_TRG_COUNT);
}

void Trigger::clearTriggerCount() const {
  setGbtTxReg(TRORC_REG_SMGBT_TRG_COUNT, 0);
}

uint32_t Trigger::triggerDeadtime() const {
  return gbtTxReg(TRORC_REG_SMGBT_TRG_DEADTIME);
}

void Trigger::setTriggerDeadtime(uint32_t deadtime) const {
  setGbtTxReg(TRORC_REG_SMGBT_TRG_DEADTIME, deadtime);
}

uint32_t Trigger::triggerGeneratorEnable() const {
  return ((gbtTxReg(TRORC_REG_SMGBT_TRG_CTRL) >> 16) & 1);
}

void Trigger::setTriggerGeneratorEnable(uint32_t enable) const {
  uint32_t ctrl = setBit(gbtTxReg(TRORC_REG_SMGBT_TRG_CTRL), 16, enable);
  setGbtTxReg(TRORC_REG_SMGBT_TRG_CTRL, ctrl);
}

uint32_t Trigger::triggerGeneratorWaittime() const {
  return gbtTxReg(TRORC_REG_SMGBT_TRGGEN_WAITTIME);
}

void Trigger::setTriggerGeneratorWaittime(uint32_t waittime) const {
  setGbtTxReg(TRORC_REG_SMGBT_TRGGEN_WAITTIME, waittime);
}

uint32_t Trigger::triggerGeneratorTargetcount() const {
  return gbtTxReg(TRORC_REG_SMGBT_TRGGEN_TARGETCOUNT);
}

void Trigger::setTriggerGeneratorTargetcount(uint32_t count) const {
  setGbtTxReg(TRORC_REG_SMGBT_TRGGEN_TARGETCOUNT, count);
}

void Trigger::shiftAlignment() const {
  uint32_t ctrl = setBit(gbtTxReg(TRORC_REG_SMGBT_TRG_CTRL), 2, 1);
  setGbtTxReg(TRORC_REG_SMGBT_TRG_CTRL, ctrl);
}

uint32_t Trigger::alignmentShiftCount() const {
  return (gbtTxReg(TRORC_REG_SMGBT_TRG_CTRL) >> 3) & 0x7;
}
}
