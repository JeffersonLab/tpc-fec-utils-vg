/**
 *  trorc_trigger_control.cpp
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
#include <fcntl.h>
#include <getopt.h>
#include <iostream>
#include <librorc.h>

using namespace std;

#define LOG_DEC_HEX(x) dec << (x) << "(0x" << hex << (x) << ")" << dec

string cycles2ratestr(uint32_t cycles) {
  double rate = 1.0e09f / (25 * cycles);
  stringstream ss;
  if (rate > 1.0e6f) {
    ss << rate / 1.0e6f << " MHz";
  } else if (rate > 1.0e3f) {
    ss << (rate / 1.0e3f) << " kHz";
  } else {
    ss << rate << " Hz";
  }
  return ss.str();
}


string cycles2timestr(uint32_t cycles) {
  double time = cycles * 25.0e-9f;
  stringstream ss;
  if (time > 1.0e0f) {
    ss << time << " s";
  } else if (time > 1.0e-3f) {
    ss << (time * 1.0e3f) << " ms";
  } else if (time > 1.0e-6f) {
    ss << (time * 1.0e6f) << " us";
  } else if (time > 1.0e-9f) {
    ss << (time * 1.0e9f) << " ns";
  }
  return ss.str();
}

int main(int argc, char *argv[]) {
  int deviceId = 0;

  bool clearTriggerCount = false;
  bool setEdgeSensitivity = false;
  uint32_t edgeSensitivity = 0;
  bool setGeneratorEnable = false;
  uint32_t generatorEnable = 0;
  bool setGeneratorTargetcount = false;
  uint32_t generatorTargetcount = 0;
  bool setGeneratorWaittime = false;
  uint32_t generatorWaittime = 0;
  bool setLvdsBufferEnable = false;
  uint32_t lvdsBufferEnable = 0;
  bool setLvdsTriggerEnable = false;
  uint32_t lvdsTriggerEnable = 0;
  bool setTriggerDeadtime = false;
  uint32_t triggerDeadtime = 0;
  bool getStatus = false;
  bool shiftAlignment = false;

  static struct option long_options[] = {
    { "clear-trigger-count", no_argument, 0, 'r' },
    { "device", required_argument, 0, 'n' },
    { "edge-sensitivity", required_argument, 0, 'S' },
    { "generator-enable", required_argument, 0, 'g' },
    { "generator-targetcount", required_argument, 0, 't' },
    { "generator-waittime", required_argument, 0, 'w' },
    { "help", no_argument, 0, 'h' },
    { "lvds-buffer-enable", required_argument, 0, 'b' },
    { "lvds-trigger-enable", required_argument, 0, 'e' },
    { "shift-alignment", no_argument, 0, 'a' },
    { "status", no_argument, 0, 's' },
    { "trigger-deadtime", required_argument, 0, 'd' },
    { 0, 0, 0, 0 }
  };

  int opt;
  while ((opt = getopt_long(argc, argv, "rn:S:g:t:w:hb:e:asd:", long_options,
                            NULL)) != -1) {
    switch (opt) {
    case 'n':
      deviceId = strtoul(optarg, NULL, 0);
      break;
    case 'r':
      clearTriggerCount = true;
      break;
    case 'S':
      setEdgeSensitivity = true;
      edgeSensitivity = strtoul(optarg, NULL, 0);
      break;
    case 'g':
      setGeneratorEnable = true;
      generatorEnable = strtoul(optarg, NULL, 0);
      break;
    case 't':
      setGeneratorTargetcount = true;
      generatorTargetcount = strtoul(optarg, NULL, 0);
      break;
    case 'w':
      setGeneratorWaittime = true;
      generatorWaittime = strtoul(optarg, NULL, 0);
      break;
    case 'b':
      setLvdsBufferEnable = true;
      lvdsBufferEnable = strtoul(optarg, NULL, 0);
      break;
    case 'e':
      setLvdsTriggerEnable = true;
      lvdsTriggerEnable = strtoul(optarg, NULL, 0);
      break;
    case 'a':
      shiftAlignment = true;
      break;
    case 's':
      getStatus = true;
      break;
    case 'd':
      setTriggerDeadtime = true;
      triggerDeadtime = strtoul(optarg, NULL, 0);
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
      return 0;
      break;
    }
    default:
      return -1;
      break;
    } // switch
  }   // while

  std::unique_ptr<trorc::Device> trorc;
  std::unique_ptr<trorc::Bar> bar;
  std::unique_ptr<trorc::Trigger> trigger;

  try {
    trorc.reset(new trorc::Device(deviceId));
    bar.reset(new trorc::Bar(*trorc, 1));
    trigger.reset(new trorc::Trigger(*bar));
  }
  catch (int e) {
    std::cerr << "ERROR: Failed to initialize T-RORC: " << librorc::errMsg(e)
              << std::endl;
    exit(1);
  }

  if (setLvdsBufferEnable) {
    trigger->setLvdsBufferEnable(lvdsBufferEnable);
  }

  if (setEdgeSensitivity) {
    trigger->setEdgeSensitivity(edgeSensitivity);
  }

  if (clearTriggerCount) {
    trigger->clearTriggerCount();
  }

  if (setTriggerDeadtime) {
    trigger->setTriggerDeadtime(triggerDeadtime);
  }

  if (setGeneratorWaittime) {
    trigger->setTriggerGeneratorWaittime(generatorWaittime);
  }

  if (setGeneratorTargetcount) {
    trigger->setTriggerGeneratorTargetcount(generatorTargetcount);
  }

  if (shiftAlignment) {
    trigger->shiftAlignment();
  }

  if (setGeneratorEnable) {
    trigger->setTriggerGeneratorEnable(generatorEnable);
  }

  if (setLvdsTriggerEnable) {
    trigger->setTriggerEnable(lvdsTriggerEnable);
  }

  if (getStatus) {
    uint32_t triggercount = trigger->triggerCount();
    uint32_t deadtime = trigger->triggerDeadtime();
    uint32_t waittime = trigger->triggerGeneratorWaittime();
    uint32_t targetcount = trigger->triggerGeneratorTargetcount();
    cout << "Trigger Count    : " << LOG_DEC_HEX(triggercount) << endl
         << "Alignment Shift  : " << trigger->alignmentShiftCount() << endl
         << "============ LVDS Trigger Input ============" << endl
         << "Buffer Enabled   : " << trigger->lvdsBufferEnable() << endl
         << "Trigger Enabled  : " << trigger->triggerEnable() << endl
         << "Deadtime         : " << LOG_DEC_HEX(deadtime) << endl
         << "Edge Sensitivity : "
         << (trigger->edgeSensitivity() ? "falling (1)" : "rising (0)") << endl
         << "============ Trigger Generator  ============" << endl
         << "Enabled          : " << trigger->triggerGeneratorEnable() << endl
         << "Waittime [cycles]: " << LOG_DEC_HEX(waittime) << endl
         << "Trigger spacing  : " << cycles2timestr(waittime + 1) << endl
         << "Trigger rate     : " << cycles2ratestr(waittime + 1) << endl
         << "Target Count     : " << LOG_DEC_HEX(targetcount) << endl;
  }

  return 0;
}
