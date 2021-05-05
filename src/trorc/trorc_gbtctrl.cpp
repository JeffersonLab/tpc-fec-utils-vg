/**
 *  trorc_gbtctrl.cpp
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

#include <librorc.h>
#include <getopt.h>
#include <iostream>
#include "gbt_link.hpp"

using namespace std;

typedef struct {
  bool set;
  bool get;
  uint32_t value;
} tControlSet;

tControlSet evalParam(char *param) {
  tControlSet cs = {false, false, 0};
  if (param) {
    cs.value = strtol(param, NULL, 0);
    cs.set = true;
  } else {
    cs.get = true;
  }
  return cs;
}

void list_options(const struct option *long_options, int nargs) {
  cout << "Available arguments:" << endl;
  for (int i = 0; i < nargs; i++) {
    if (long_options[i].name) {
      if(long_options[i].flag==0) {
        cout << "-" << (char)long_options[i].val << "|";
      }
      cout << "--" << long_options[i].name;
    }
    switch (long_options[i].has_arg) {
    case optional_argument:
      cout << "(=[value])";
      break;
    default:
      break;
    }
    cout << endl;
  }
  cout << endl << "Parameters with optional value parameter can be used to"
      " get the current value without specifying a value or set a new value"
    " by adding the value parameter." << endl;
}

int main(int argc, char *argv[]) {
  int deviceId = 0;
  int channelId = 0;
  tControlSet pcErrorCount = {false, false, 0};
  tControlSet pcPatternMode = {false, false, 0};
  tControlSet pcWidebusMode = {false, false, 0};
  tControlSet rxReadyLostCount = {false, false, 0};
  tControlSet decStatus = {false, false, 0};
  tControlSet decConfig = {false, false, 0};
  tControlSet readoutMode = {false, false, 0};

  static struct option long_options[] = {
    { "device", required_argument, 0, 'n' },
    { "channel", required_argument, 0, 'c' },
    { "help", no_argument, 0, 'h' },
    { "pcerrorcount", optional_argument, 0, 'p' },
    { "pcwidebusmode", optional_argument, 0, 'w' },
    { "pcpatternmode", optional_argument, 0, 'm' },
    { "rxreadylostcount", optional_argument, 0, 'r' },
    { "status", no_argument, 0, 's' },
    { "decoderstatus", optional_argument, 0, 'd'},
    { "decoderconfig", optional_argument, 0, 'D'},
    { "readoutmode", optional_argument, 0, 'M'},
    { 0, 0, 0, 0 }
  };
  int nargs = sizeof(long_options) / sizeof(option);

  // generate optstring for getopt_long()
  string optstring = "";
  for (int i = 0; i < nargs; i++) {
    if (long_options[i].flag == 0) {
      optstring += long_options[i].val;
    }
    switch (long_options[i].has_arg) {
    case required_argument:
      optstring += ":";
      break;
    case optional_argument:
      optstring += "::";
      break;
    default:
      break;
    }
  }

  /** Parse command line arguments **/
  if (argc <= 1) {
    list_options(long_options, nargs);
    return 0;
  }

  while (1) {
    int opt = getopt_long(argc, argv, optstring.c_str(), long_options, NULL);
    if (opt == -1) {
      break;
    }
    switch (opt) {
    case 'n':
      deviceId = strtoul(optarg, NULL, 0);
      break;
    case 'c':
      channelId = strtoul(optarg, NULL, 0);
      break;
    case 'h':
      list_options(long_options, nargs);
      return 0;
    case 'p': //pcerrorcount
      pcErrorCount = evalParam(optarg);
      break;
    case 'm': //pcpatternmode
      pcPatternMode = evalParam(optarg);
      break;
    case 'w': //pcwidebusmode
      pcWidebusMode = evalParam(optarg);
      break;
    case 'r': //rxreadylostcount
      rxReadyLostCount = evalParam(optarg);
      break;
    case 's':
      pcErrorCount.get = true;
      pcPatternMode.get = true;
      pcWidebusMode.get = true;
      rxReadyLostCount.get = true;
      break;
    case 'd':
      decStatus = evalParam(optarg);
      break;
    case 'D':
      decConfig = evalParam(optarg);
      break;
    case 'M':
      readoutMode = evalParam(optarg);
      break;
    case '?':
      return -1;
    default:
      continue;
    }
  }

  std::unique_ptr<trorc::Device> dev;
  std::unique_ptr<trorc::Bar> bar;
  std::unique_ptr<trorc::GbtLink> link;

  try {
    dev.reset(new trorc::Device(deviceId));
    bar.reset(new trorc::Bar(*dev, 1));
    link.reset(new trorc::GbtLink(*bar, channelId));
  }
  catch (int e) {
    cerr << "ERROR: Failed to initialize T-RORC: " << librorc::errMsg(e)
         << endl;
    exit(-1);
  }

  if (pcErrorCount.set) {
    link->clearPatternCheckerErrorCount();
  } else if (pcErrorCount.get) {
    uint32_t errorcount = link->patternCheckerErrorCount();
    printf("Ch%d PatternChecker Error Count: %d (0x%08x)\n", channelId,
           errorcount, errorcount);
  }

  if (pcPatternMode.set) {
    link->setPatternCheckerMode(pcPatternMode.value);
  } else if (pcPatternMode.get) {
    printf("Ch%d PatterChecker Pattern Mode: %d\n", channelId,
           link->patternCheckerMode());
  }

  if (pcWidebusMode.set) {
    link->setPatternCheckerWidebus(pcWidebusMode.value);
  } else if (pcWidebusMode.get) {
    printf("Ch%d PatterChecker Widebus Mode: %d\n", channelId,
           link->patternCheckerWidebus());
  }

  if (rxReadyLostCount.set) {
    link->clearRxReadyLostCount();
  } else if (rxReadyLostCount.get) {
    uint32_t count = link->rxReadyLostCount();
    printf("Ch%d RX-Ready Lost Count: %d (0x%08x)\n", channelId, count, count);
  }

  if (decStatus.set) {
    link->resetDecoderCounter(decStatus.value);
  } else if (decStatus.get) {
    for (int i=0; i<3; ++i) {
      uint32_t count = link->decAdcErrorCounter(i);
      printf("Ch%d Sampa %d ADC clock error counter: \t%u (0x%08x)\n", channelId, i, count, count);
    }
    for (int i=0; i<5; ++i) {
      uint32_t count = link->decSyncPatternCounter(i);
      printf("Ch%d Sampa %d ADC Sync Pattern counter: \t%u (0x%08x)\n", channelId, i, count, count);
    }
    for (int i=0; i<2; ++i) {
      uint32_t count = link->decIdError(i);
      printf("Ch%d Sampa %d ID mismatch counter: \t%u (0x%08x)\n", channelId, i, count, count);
    }

    uint32_t adcStatus = link->decAdcStatus();
    printf("Ch%d: Adc clock found for Sampa 0, 1, 2: %d %d %d\n", channelId, adcStatus & 0x1, (adcStatus >>1) & 0x1, (adcStatus >> 2) & 0x1);

    uint32_t syncPatStatus = link->decSyncPatternStatus();
    printf("Ch%d: Sync Pattern found for Sampa 0(l,h), 1(l,h), 2: \t%d %d %d %d %d\n", channelId,
        (syncPatStatus >> 2) & 0x1, 
        (syncPatStatus >> 5) & 0x1,
        (syncPatStatus >> 8) & 0x1, 
        (syncPatStatus >> 11) & 0x1,
        (syncPatStatus >> 14) & 0x1);
    printf("                                       at positions: \t%d %d %d %d %d\n",
        (syncPatStatus >> 0) & 0x3, 
        (syncPatStatus >> 3) & 0x3,
        (syncPatStatus >> 6) & 0x3, 
        (syncPatStatus >> 9) & 0x3,
        (syncPatStatus >> 12) & 0x3);
  }

  if (decConfig.set) {
    link->setDecoder(decConfig.value);
  } else if (decConfig.get) {
    bool st = link->getDecoderStatus();
    if (st) printf("Ch%d: Decoder is enabled.\n", channelId);
    else    printf("Ch%d: Decoder is disabled.\n", channelId);
  }

  if (readoutMode.set) {
    link->setReadoutMode(readoutMode.value);
  } else if (readoutMode.get) {
    uint32_t rMode = link->getReadoutMode();
    printf("Ch%d: readout mode is set to %d\n", channelId, rMode);
  }
  return 0;
}
