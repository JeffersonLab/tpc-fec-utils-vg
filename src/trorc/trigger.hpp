/**
 *  trigger.cpp
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

#ifndef _TRORC_UTILS_TRIGGER
#define _TRORC_UTILS_TRIGGER

#include "trorc.hpp"
#include "trorc_registers.h"
#include <librorc.h>

namespace trorc {

class Trigger {
public:
  Trigger(librorc::bar &bar) : bar_(bar){};
  Trigger(trorc::Bar &bar) : bar_(*(bar.get())){};

  /**
   * Get LVDS trigger input buffer status. The LVDS buffer must be
   * enabled in order to receive triggers.
   * @return 1 if enabled, 0 if
   * disabled
   **/
  uint32_t lvdsBufferEnable() const;

  /**
   * Enable/disable the LVDS trigger input buffer
   * @param enable 1 to enable, 0 to disable
   **/
  void setLvdsBufferEnable(uint32_t enable) const;

  /**
   * The trigger input logic waits for an edge on the trigger input signal. The
   * edge sensitivity type (rising or falling) is configurable.
   * @return 0 for rising edge, 1 for falling edge
   **/
  uint32_t edgeSensitivity() const;

  /**
   * Set the trigger edge sensitivity type
   * @param type 0 for rising edge, 1 for falling edge
   **/
  void setEdgeSensitivity(uint32_t type) const;

  /**
   * Get the LVDS trigger input enable status.
   * @return 1 if enabled, 0 if disabled
   **/
  uint32_t triggerEnable() const;

  /**
   * Enable or disable the LVDS trigger input. This enables or disables LVDS
   * trigger inputs for all GBT links at the same time.
   * @param enable 1 to enable, 0 to disable.
   **/
  void setTriggerEnable(uint32_t enable) const;

  /**
   * Get the total number of trigger pulses received.
   * @return number of triggers
   **/
  uint32_t triggerCount() const;

  /**
   * Clear the trigger counter
   **/
  void clearTriggerCount() const;

  /**
   * Get current trigger deadtime. The trigger input is disabled for a
   * configurable number of clock cycles after a trigger edge was received to
   * avoid repeated triggers due to input signal noise or ringing.
   * @return number of clock cycles the trigger input is disabled after a
   * trigger
   **/
  uint32_t triggerDeadtime() const;

  /**
   * Set the trigger deadtime. The trigger input is disabled for a
   * configurable number of clock cycles after a trigger edge was received to
   * avoid repeated triggers due to input signal noise or ringing.
   * @param deadtime number of clock cycles the trigger input should be disabled
   * after a trigger
   **/
  void setTriggerDeadtime(uint32_t deadtime) const;

  /**
   * Get the trigger generator status
   * @return 1 if enabled, 0 if disabled
   **/
  uint32_t triggerGeneratorEnable() const;

  /**
   * Enable or disable the trigger generator. This component allows to emulate
   * an external trigger input with configurable trigger count and trigger
   * spacing.
   * @param enable 1 to enable, 0 to disable.
   **/
  void setTriggerGeneratorEnable(uint32_t enable) const;

  /**
   * Get the number of clock cycles the trigger generator waits between trigger
   * pulses
   * @return number of clock cycles
   **/
  uint32_t triggerGeneratorWaittime() const;

  /**
   * Set the number of clock cycles the trigger generator should wait between
   * trigger pulses
   * @param waittime number of clock cycles
   **/
  void setTriggerGeneratorWaittime(uint32_t waittime) const;

  /**
   * Get the number of trigger pulses the trigger generator will generate. A
   * value of 0 means no limit.
   * @return number of trigger pulses to be generated
   **/
  uint32_t triggerGeneratorTargetcount() const;

  /**
   * Set the number of trigger pulses the trigger generator should generate. A
   * value of 0 means no limit.
   * @param count number of trigger pulses
   **/
  void setTriggerGeneratorTargetcount(uint32_t count) const;

  /**
   * Shift the TX pattern start and software trigger alignment relative to the
   * SAMPA ADC clock. Each call shifts the alignment by one frame. Eight shifts
   * result in the original alignment.
   **/
  void shiftAlignment() const;

  /**
   * Get the current alignment shift count. This is the relative shift of the
   * alignemnt w.r.t. to the original alignment after firmware initialization.
   * @return shift count, range 0 to 7
   **/
  uint32_t alignmentShiftCount() const;

private:
  void setGbtTxReg(uint32_t addr, uint32_t data) const;
  uint32_t gbtTxReg(uint32_t addr) const;
  librorc::bar &bar_;
};
}

#endif
