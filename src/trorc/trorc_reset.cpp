/**
 *  trorc_reset.cpp
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
#include "gbt_link.hpp"
#include <iostream>
#include <memory>
#include "trorc.hpp"

using namespace std;

#define HELP_TEXT                                                              \
  "crorc_reset usage:\n"                                                       \
  "Parameters:\n"                                                              \
  " - h             Print this help\n"                                         \
  " - n [deviceId]  Target device, default: 0\n"                               \
  " - c [channelId] (optional) channel ID\n, default: all"                     \
  " - f             (optional) do full reset incl. GTXs, default: no\n"

#define NONE 0xffffffff

int main(int argc, char *argv[]) {
  int deviceId = 0;
  uint32_t channelId = NONE;
  int arg;
  int do_full_reset = 0;

  while ((arg = getopt(argc, argv, "hn:c:f")) != -1) {
    switch (arg) {
    case 'h':
      cout << HELP_TEXT;
      return 0;
      break;
    case 'n':
      deviceId = strtol(optarg, NULL, 0);
      break;
    case 'c':
      channelId = strtoul(optarg, NULL, 0);
      break;
    case 'f':
      do_full_reset = 1;
      break;
    default:
      cout << "Unknown parameter (" << arg << ")!" << endl;
      cout << HELP_TEXT;
      return -1;
      break;
    } // switch
  }   // while

  librorc::sysmon *sm = NULL;

  std::unique_ptr<trorc::Device> dev;
  std::unique_ptr<trorc::Bar> bar;

  try {
    dev.reset(new trorc::Device(deviceId));
    bar.reset(new trorc::Bar(*dev, 1));

    sm = new librorc::sysmon(bar->get());
  } catch (int e) {
    cerr << "ERROR: Failed to initialize T-RORC " << deviceId << ": "
         << librorc::errMsg(e) << endl;
    if (sm) {
      delete sm;
    }
    exit(-1);
  }

  if (sm->firmwareType() != TRORC_CFG_PROJECT_trorc) {
    cerr << "ERROR: Device is not running a T-RORC firmware." << endl;
    delete sm;
    exit(-1);
  }

  /** clear PCIe/SlowControl error counters **/
  sm->clearAllErrorCounters();

  uint32_t start_channel = (channelId != NONE) ? channelId : 0;
  uint32_t end_channel =
      (channelId != NONE) ? channelId : sm->numberOfChannels() - 1;

  for (uint32_t chid = start_channel; chid <= end_channel; chid++) {
    /** reset DMA engine **/
    librorc::link *link = new librorc::link(bar->get(), chid);
    librorc::dma_channel *ch = new librorc::dma_channel(link);
    ch->disable();
    ch->setRateLimit(0);
    ch->clearStallCount();
    ch->clearEventCount();
    ch->readAndClearPtrStallFlags();
    ch->clearFifoUnderrunFlag();
    delete ch;
    delete link;

    /** reset GBT readout path **/
    std::unique_ptr<trorc::GbtLink> glink(new trorc::GbtLink(*bar, chid));
    if (do_full_reset) {
      glink->setReset(1);
      glink->setReset(0);
    }

    if (glink->rxFrameClockReady()) {
      glink->setPatternCheckerMode(0);
      glink->setPatternCheckerWidebus(0);
      glink->clearPatternCheckerErrorCount();
      glink->clearRxReadyLostCount();
      glink->clearRxReadoutCurrentFrameCount();
    }

    if (glink->txFrameClockReady()) {
      glink->setSoftwareTriggerChannelMask(0);
      glink->setTxPatternChannelMask(0);
      glink->setAllowExternalTrigger(0);
      glink->stopReadout();
      glink->disableControlPattern();
      glink->setTxIdlePattern(0x10000, 0x0000ffff, 0xffffffff);
      glink->setTxControlPattern(0, 0, 0);
      glink->clearControlPatternStartCount();
      glink->clearErrorFlags();
    }
  }

  delete sm;
  return 0;
}
