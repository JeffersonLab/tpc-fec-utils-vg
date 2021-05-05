/**
 *  trorc_fec_readout.cpp
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

#include "file_writer.hh"
#include "gbt_link.hpp"
#include <fcntl.h>
#include <getopt.h>
#include <iostream>
#include <librorc.h>
#include <signal.h>

using namespace std;

/** Default event buffer size (in Bytes) **/
#ifndef MODELSIM
#define EBUFSIZE (((uint64_t)1) << 31) // 2 GB
#else
#define EBUFSIZE (((uint64_t)1) << 19) // 512 kB
#endif

bool done = false;

// Prototypes
int64_t timediff_us(struct timeval from, struct timeval to);
void printStatusLine(librorc::ChannelStatus *cs_cur,
                     librorc::ChannelStatus *cs_last, int64_t tdiff_us,
                     int error_mask);

// Signal handler
void abort_handler(int s) {
  cerr << "Caught signal " << s << endl;
  if (done == true) {
    exit(-1);
  } else {
    done = true;
  }
}

int main(int argc, char *argv[]) {
  int deviceId = 0;
  int channelId = 0;
  char *dumpDir = NULL;
  uint32_t ctrlFrmCount = 16;
  uint32_t rdoFrmCount = 0;
  bool run_continuously = false;
  bool quiet = false;
  uint32_t dumpMode = 0;
  bool sendControlPattern = false;

  static struct option long_options[] = {
    { "device", required_argument, 0, 'n' },
    { "channel", required_argument, 0, 'c' },
    { "controlframecount", required_argument, 0, 'C' },
    { "readoutframecount", required_argument, 0, 'R' },
    { "continuous", no_argument, 0, 'r' },
    { "swtrigger", no_argument, 0, 't' },
    { "dump", required_argument, 0, 'd' },
    { "quiet", no_argument, 0, 'q' },
    { "mode", required_argument, 0, 'm' },
    { "help", no_argument, 0, 'h' },
    { 0, 0, 0, 0 }
  };

  int opt;
  while ((opt = getopt_long(argc, argv, "n:c:hd:C:R:rqm:t", long_options,
                            NULL)) != -1) {
    switch (opt) {
    case 'n':
      deviceId = strtoul(optarg, NULL, 0);
      break;
    case 'c':
      channelId = strtoul(optarg, NULL, 0);
      break;
    case 'd':
      dumpDir = optarg;
      break;
    case 'R':
      rdoFrmCount = strtoul(optarg, NULL, 0);
      break;
    case 'C':
      ctrlFrmCount = strtoul(optarg, NULL, 0);
      break;
    case 't':
      sendControlPattern = true;
      break;
    case 'r':
      run_continuously = true;
      break;
    case 'q':
      quiet = true;
    case 'm':
      dumpMode = strtoul(optarg, NULL, 0);
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

  // require rdoFrmCount to be set
  if (!rdoFrmCount) {
    cerr << "ERROR: --readoutframecount|-R is not set!" << endl;
    return -1;
  }

  file_writer *dumper = NULL;
  if (dumpDir) {
    try {
      dumper = new file_writer(dumpDir, deviceId, channelId, 0);
    }
    catch (int e) {
      cerr << "ERROR initializing file writer: " << e << endl;
      return -1;
    }
  }

  // Setup device
  std::unique_ptr<trorc::Device> trorc;
  std::unique_ptr<trorc::Bar> bar;

  try {
    trorc.reset(new trorc::Device(deviceId));
    bar.reset(new trorc::Bar(*trorc, 1));
  }
  catch (int& e) {
    std::cerr << "ERROR: Failed to initialize T-RORC: " << std::to_string(e) << ": " << librorc::errMsg(e)
              << std::endl;
    exit(1);
  }

  librorc::event_stream *es = NULL;
  std::unique_ptr<trorc::GbtLink> gbt;
  try {
    es = new librorc::event_stream(trorc->get(), bar->get(), channelId,
                                   librorc::kEventStreamToHost);
    gbt.reset(new trorc::GbtLink(*(bar.get()), channelId));
  }
  catch (int e) {
    cerr << "ERROR: dev " << deviceId << " ch " << channelId
         << ": Exception while setting up event stream: " << librorc::errMsg(e)
         << endl;
    if (dumper) {
      delete dumper;
    }
    if (es) {
      delete es;
    }
    return -1;
  }

#ifdef MODELSIM
  gbt->setReset(1);
  gbt->setReset(0);
#endif

  if (!gbt->rxReady()) {
    cerr << "WARNING: dev " << deviceId << " ch " << channelId
         << ": GBT RX is not ready! Data is likely garbage..." << endl;
  }

  gbt->stopReadout();
  gbt->disableControlPattern();

  // set default IDLE and CONTROL pattern values
  gbt->setTxIdlePattern(0x10000, 0x0000ffff, 0xffffffff);
  gbt->setTxControlPattern(0x10000, 0x0000ff00, 0x000fffff);

  // allow external triggers fom LVDS or trigger generator
  gbt->setAllowExternalTrigger(1);

  // set channel masks to only the own link
  gbt->setSoftwareTriggerChannelMask(1 << channelId);
  gbt->setTxPatternChannelMask(1 << channelId);

  // set the target frame count for each readout, use the same frame count as
  // event size
  gbt->setRxReadoutTargetFrameCount(rdoFrmCount);
  gbt->setRxEventSizeFrameCount(rdoFrmCount);

  if(dumpMode) gbt->setReadoutMode(dumpMode);

  // set the number of cycles the control pattern should be sent when activated
  gbt->setTxControlPatternCycles(ctrlFrmCount);

  es->m_channel->clearEventCount();
  es->m_channel->clearStallCount();
  es->m_channel->readAndClearPtrStallFlags();

  int result = es->initializeDma(2 * channelId, EBUFSIZE);
  if (result != 0) {
    cerr << "ERROR: failed to initialize DMA: " << librorc::errMsg(result)
         << endl;
    delete es;
    if (dumper) {
      delete dumper;
    }
    return -1;
  }

  // register signal handler for event loop
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = abort_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  struct timeval tcur, tlast;
  gettimeofday(&tcur, NULL);
  tlast = tcur;
  librorc::ChannelStatus cs_last;
  memset(&cs_last, 0, sizeof(librorc::ChannelStatus));

  // send [ctrlFrmCount] frames with the control pattern and start the readout
  // if sendControlPattern is set
  if (sendControlPattern) {
    gbt->driveControlPattern(true);
  } else {
    cout << "INFO: dev " << deviceId << " ch " << channelId
         << ": Not sending the CONTROL "
            "pattern, but waiting for externally triggered data..." << endl;
  }

  // Main event loop
  while (!done) {
    gettimeofday(&tcur, NULL);
    librorc::EventDescriptor *report;
    const uint32_t *event;
    uint64_t reference;

    if (es->getNextEvent(&report, &event, &reference)) {
      es->updateChannelStatus(report);
      if (dumper) {
        dumper->dump(report, event);
      }
      es->releaseEvent(reference);
      if (!run_continuously) {
        done = true;
      }
    }

    int64_t tdiff_us = timediff_us(tlast, tcur);
    if (!quiet && (done || tdiff_us > 1000000)) {
      printStatusLine(es->m_channel_status, &cs_last, tdiff_us, 0);
      memcpy(&cs_last, es->m_channel_status, sizeof(librorc::ChannelStatus));
      tlast = tcur;
    }
  }

  gbt->stopReadout();
  gbt->disableControlPattern();
  if (dumper) {
    delete dumper;
  }
  delete es;
  return 0;
}

int64_t timediff_us(struct timeval from, struct timeval to) {
  return ((int64_t)(to.tv_sec - from.tv_sec) * 1000000LL +
          (int64_t)(to.tv_usec - from.tv_usec));
}

void printStatusLine(librorc::ChannelStatus *cs_cur,
                     librorc::ChannelStatus *cs_last, int64_t tdiff_us,
                     int error_mask) {
  uint64_t events_diff = cs_cur->n_events - cs_last->n_events;
  float event_rate_khz = (events_diff * 1000000.0) / tdiff_us / 1000.0;
  uint64_t bytes_diff = cs_cur->bytes_received - cs_last->bytes_received;
  float mbytes_rate = (bytes_diff * 1000000.0) / tdiff_us / (float)(1 << 20);
  float total_receiced_GB = (cs_cur->bytes_received / (float)(1 << 30));
  cout.precision(2);
  cout.setf(ios::fixed, ios::floatfield);
  cout << "Ch" << cs_cur->channel << " -  #Events: " << cs_cur->n_events
       << ", Size: " << total_receiced_GB << " GB, ";
  if (events_diff) {
    cout << "Data Rate: " << mbytes_rate
         << " MB/s, Event Rate: " << event_rate_khz << " kHz";
  } else {
    cout << "Data Rate: - , Event Rate: - ";
  }
  cout << ", Errors: " << cs_cur->error_count;
  if (error_mask) {
    cout << " mask: 0x" << hex << error_mask << dec;
  }
  cout << endl;
}
