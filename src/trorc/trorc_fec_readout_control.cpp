/**
 *  trorc_fec_readout_control.cpp
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

#include "gbt_link.hpp"
#include <fcntl.h>
#include <getopt.h>
#include <iostream>
#include <librorc.h>

using namespace std;

int main(int argc, char *argv[]) {
  int deviceId = 0;
  int channelId = 0;
  bool set_sw_trigger_mask = false;
  uint16_t sw_trigger_mask = 0;
  bool set_tx_pattern_mask = false;
  uint16_t tx_pattern_mask = 0;
  bool set_external_trigger = false;
  uint32_t external_trigger = 0;
  bool set_control_pattern_cycles = false;
  uint32_t control_pattern_cycles = 0;
  bool print_status = false;
  bool send_tx_pattern= false;
  bool trigger_readout = false;
  bool do_sampa_reset = false;

  static struct option long_options[] = {
    { "channel", required_argument, 0, 'c' },
    { "controlframecount", required_argument, 0, 'C' },
    { "device", required_argument, 0, 'n' },
    { "exttrigger", no_argument, 0, 'e' },
    { "help", no_argument, 0, 'h' },
    { "sampareset", no_argument, 0, 'S' },
    { "status", no_argument, 0, 's' },
    { "swtrigger", no_argument, 0, 't' },
    { "swtriggermask", required_argument, 0, 'T' },
    { "txpattern", no_argument, 0, 'p' },
    { "txpatternmask", required_argument, 0, 'P' },
    { 0, 0, 0, 0 }
  };

  int opt;
  while ((opt = getopt_long(argc, argv, "c:C:n:ehSstT:pP:", long_options, NULL)) !=
         -1) {
    switch (opt) {
    case 'n':
      deviceId = strtoul(optarg, NULL, 0);
      break;
    case 'c':
      channelId = strtoul(optarg, NULL, 0);
      break;
    case 'P':
      tx_pattern_mask = strtoul(optarg, NULL, 0);
      set_tx_pattern_mask = true;
      break;
    case 'T':
      sw_trigger_mask = strtoul(optarg, NULL, 0);
      set_sw_trigger_mask = true;
      break;
    case 'p':
      send_tx_pattern= true;
      break;
    case 't':
      trigger_readout = true;
      break;
    case 'e':
      set_external_trigger = true;
      external_trigger = strtoul(optarg, NULL, 0) & 1;
      break;
    case 'C':
      set_control_pattern_cycles = true;
      control_pattern_cycles = strtoul(optarg, NULL, 0);
      break;
    case 's':
      print_status = true;
      break;
    case 'S':
      do_sampa_reset = true;
      break;
    case 'h': {
      cout << argv[0] << " command line parameters: " << endl;
      int long_opt_count = (sizeof(long_options) / sizeof(struct option)) - 1;
      for (int i = 0; i < long_opt_count; i++) {
        struct option cur = long_options[i];
        cout << "  ";
        if (cur.val != 0) {
          cout << "-" << char(cur.val) << "/";
        }
        cout << "--" << cur.name;
        if (cur.has_arg == required_argument) {
          cout << " [arg]";
        } else if (cur.has_arg == optional_argument) {
          cout << " ([arg])";
        }
        cout << endl;
      }
    }
      return 0;
      break;
    default:
      return -1;
      break;
    }
  }

  std::unique_ptr<trorc::Device> trorc;
  std::unique_ptr<trorc::Bar> bar;
  std::unique_ptr<trorc::GbtLink> gbt;

  try {
    trorc.reset(new trorc::Device(deviceId));
    bar.reset(new trorc::Bar(*trorc, 1));
    gbt.reset(new trorc::GbtLink(*bar, channelId));
  } catch (int e) {
    std::cerr << "ERROR: Failed to initialize T-RORC: " << librorc::errMsg(e)
              << std::endl;
    exit(1);
  }

  if (set_tx_pattern_mask) {
    gbt->setTxPatternChannelMask(tx_pattern_mask);
  }

  if (set_sw_trigger_mask) {
    gbt->setSoftwareTriggerChannelMask(sw_trigger_mask);
  }

  if (set_external_trigger) {
    gbt->setAllowExternalTrigger(external_trigger);
  }

  if (set_control_pattern_cycles) {
    gbt->setTxControlPatternCycles(control_pattern_cycles);
  }

  if (send_tx_pattern) {
    // send control pattern. if trigger_readout is set as well, start the
    // readout in the same turn.
    gbt->driveControlPattern(trigger_readout);
  } else if (trigger_readout && !do_sampa_reset) {
    // readout without driving the control pattern before
    gbt->triggerReadout();
  }

  if (do_sampa_reset) {
    uint32_t prevCtrlPatternLow[12];
    uint32_t prevCtrlPatternMid[12];
    uint32_t prevCtrlPatternHigh[12];
    uint32_t prevCtrlPatternCycles[12];
    uint16_t mask = gbt->txPatternChannelMask();

    // Store previous IDLE/CTRL patterns for all link in the mask
    // and set the temporary SAMPA reset patterns
    for (int i = 0; i < 12; i++) {
      if (mask & (1 << i)) {
        std::unique_ptr<trorc::GbtLink> lnk;
        try {
          lnk.reset(new trorc::GbtLink(*bar, i));
        } catch (int e) {
          continue;
        }
        prevCtrlPatternLow[i] = lnk->txControlPatternLow();
        prevCtrlPatternMid[i] = lnk->txControlPatternMid();
        prevCtrlPatternHigh[i] = lnk->txControlPatternHigh();
        prevCtrlPatternCycles[i] = lnk->txControlPatternCycles();
        lnk->setTxControlPattern(0x10000, 0x0000ff00, 0x00000000);
        lnk->setTxControlPatternCycles(16);
      }
    }

    // Drive the SAMPA reset patterns on all selected links. Also start a
    // readout if trigger_readout is set.
    gbt->driveControlPattern(trigger_readout);
    while(gbt->controlPatternActive()) {
      usleep(100);
    }

    // restore the previous patterns
    for (int i = 0; i < 16; i++) {
      if (mask & (1 << i)) {
        std::unique_ptr<trorc::GbtLink> lnk;
        try {
          lnk.reset(new trorc::GbtLink(*bar, i));
        } catch (int e) {
          continue;
        }
        lnk->setTxControlPattern(prevCtrlPatternHigh[i],
                                 prevCtrlPatternMid[i],
                                 prevCtrlPatternLow[i]);
        lnk->setTxControlPatternCycles(prevCtrlPatternCycles[i]);
      }
    }
  }

  if (print_status) {
    cout << "TX Pattern Channel Mask: 0x" << hex << gbt->txPatternChannelMask() << endl
	 << "SW Trigger Channel Mask: 0x" << gbt->softwareTriggerChannelMask() << endl
	 << "Allow Ext. Trigger     : 0x" << gbt->allowExternalTrigger() << endl
	 << "Trigger count          : 0x" << gbt->triggerCount() << endl
	 << "Control Pattern active : 0x" << gbt->controlPatternActive() << endl
	 << "Control Pattern started: 0x" << gbt->controlPatternStartCount() << endl;
  }

  return 0;
}
