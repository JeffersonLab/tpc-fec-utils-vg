/**
 *  trorc_init.cpp
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
#include "git_info.hpp"
#include "gbt_link.hpp"

using namespace std;

int setAllGtxReset(librorc::bar *bar, uint32_t nChannels, uint32_t value) {
  for (uint32_t i = 0; i < nChannels; i++) {
    librorc::link *link = new librorc::link(bar, i);
    uint32_t linkType = link->linkType();
    if (linkType != 5) {
      cerr << "ERROR: invalid link type [" << linkType << "] for channel " << i << endl;
      delete link;
      return -1;
    }
    librorc::gtx *gtx = new librorc::gtx(link);
    gtx->setReset(value);
    delete gtx;
    delete link;
  }
  return 0;
}

void setAllGbtTxPolarity(trorc::Bar& bar, uint32_t nChannels, uint32_t value) {
  for (uint32_t i = 0; i < nChannels; i++) {
    trorc::GbtLink gbt(bar, i);
    gbt.setRxPolarity(0);
    gbt.setTxPolarity(value);
  }
}

void waitForAllRxReady(trorc::Bar& bar, const uint32_t nChannels,
                       uint32_t timeout_ms = 5000)
{
  std::vector<bool> rxReady(nChannels);
  bool allReady = true;

  std::vector<std::unique_ptr<trorc::GbtLink>> link(nChannels);
  for (uint32_t i = 0; i < nChannels; i++) {
    link[i].reset(new trorc::GbtLink(bar, i));
    rxReady[i] = link[i]->rxReady();
    allReady = allReady && rxReady[i];
  }
  cout << "Waiting " << (static_cast<float>(timeout_ms)/1000) << "s"
       << " for GBT RX READY..." << endl;
  if (allReady) {
    cout << "All links are RX READY." << endl;
  } else {
    while (!allReady && timeout_ms != 0) {
      usleep(1000);
#ifndef MODELSIM
      // don't use timeout in Simulation
      timeout_ms--;
#endif
      allReady = true;
      for (uint32_t i = 0; i < nChannels; i++) {
        bool ready = link[i]->rxReady();
        if (ready && !rxReady[i]) {
          cout << "RX READY received on GBT" << i << endl;
          rxReady[i] = ready;
          link[i]->clearPatternCheckerErrorCount();
          link[i]->clearRxReadyLostCount();
        }
        allReady = allReady && ready;
      } // for
    }   // while
  }     // !allReady

  for (uint32_t i = 0; i < nChannels; i++) {
    if (!rxReady[i]) {
      cout << "NO RX READY on GBT" << i << "!" << endl;
    }
  }
}

int main(int argc, char *argv[])
{
  librorc::sysmon *sm = NULL;

  std::unique_ptr<trorc::Device> dev;
  std::unique_ptr<trorc::Bar> bar;

  int vldb = 0;
  int rx_rdy_timeout(10);
  int print_version(0);
  static struct option long_options[] = {
      { "help", no_argument, NULL, 'h' },
      { "timeout", required_argument, NULL, 't' },
      { "vldb", no_argument, &vldb, 1 },
      { "version", no_argument, &print_version, 1 },
      { 0, 0, 0, 0 }
    };
  int opt;
  while ((opt = getopt_long(argc, argv, "h", long_options, NULL)) != -1) {
    switch (opt) {
    case 0:
      // long_opt case
      break;
    case 'h':
      cout << "Paramters:\n" 
           << " --timeout   optional: Timeout when waiting for RX_READY: default: "
           << rx_rdy_timeout << "s\n"
           << " --vldb      optional: Initialize VLDB. default: FEC\n"
           << " --version   optional: Print version info and exit\n";
      return 0;
      break;
    case 't':
      rx_rdy_timeout = strtoul(optarg, NULL, 0);
      break;
    case '?':
      return -1;
      break;
    default:
      return -1;
      break;
    }
  }

  if (print_version) {
    std::cout << GitInfo();
    return 0;
  }

  try {
    dev.reset(new trorc::Device(0));
    bar.reset(new trorc::Bar(*dev, 1));
    sm = new librorc::sysmon(bar->get());
  }
  catch (int e) {
    cerr << "ERROR: Failed to initialize T-RORC: " << librorc::errMsg(e)
         << endl;
    return -1;
  }

  if (sm->firmwareType() != TRORC_CFG_PROJECT_trorc) {
    cerr << "ERROR: Firmware variant is not a T-RORC FW: "
         << sm->firmwareDescription() << endl;
    delete sm;
    return -1;
  }

  uint32_t nChannels = sm->numberOfChannels();

  if (vldb) {
    cout << "*** Initializing GBTs on VLDB ***" << endl;
  } else {
    cout << "*** Initializing GBTs on FEC ***" << endl;
  }

  cout << "Resetting QSFPs..." << endl;
  sm->qsfpSetReset(0, 1);
  sm->qsfpSetReset(1, 1);
  sm->qsfpSetReset(2, 1);

  cout << "Resetting Transceivers..." << endl;
  setAllGtxReset(bar->get(), nChannels, 1);
  setAllGbtTxPolarity(*bar, nChannels, vldb);

  cout << "Reconfiguring Reference Clock to 240 MHz..." << endl;
  try {
    librorc::refclk *refclk = new librorc::refclk(sm);
    refclk->reset();
    librorc::refclkopts refclkopts =
        refclk->getCurrentOpts(LIBRORC_REFCLK_DEFAULT_FOUT);
    librorc::refclkopts new_refclkopts =
        refclk->calcNewOpts(240.0, refclkopts.fxtal);
    refclk->writeOptsToDevice(new_refclkopts);
    delete refclk;
  }
  catch (int e) {
    cout << "ERROR: Failed to reconfigure RefClk: " << librorc::errMsg(e)
         << endl;
    cout << "Keeping QSFPs and GTXs in reset! Don't release reset "
            "unless you know what you're doing! It's probably best "
            "to power cycle..." << endl;
    delete sm;
    return -1;
  }

  cout << "Releasing Transceiver Resets..." << endl;
  setAllGtxReset(bar->get(), nChannels, 0);

  cout << "Releasing QSFP Resets..." << endl;
  sm->qsfpSetReset(0, 0);
  sm->qsfpSetReset(1, 0);
  sm->qsfpSetReset(2, 0);

  waitForAllRxReady(*bar, nChannels, rx_rdy_timeout * 1000);

  delete sm;
  return 0;
}
