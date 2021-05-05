/**
 *  trorc_dma_in.cpp
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

#include <iostream>
#include <librorc.h>
#include <signal.h>
#include <getopt.h>
#include <fcntl.h>
#include <vector>
#include "file_writer.hh"
#include "gbt_link.hpp"

using namespace std;

/** Default event buffer size (in Bytes) **/
#ifndef MODELSIM
#define EBUFSIZE (((uint64_t)1) << 30) // 1 GB
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

  static struct option long_options[] = {
    { "device", required_argument, 0, 'n' },
    { "channel", required_argument, 0, 'c' },
    { "dump", required_argument, 0, 'd' },
    { "help", no_argument, 0, 'h' },
    { 0, 0, 0, 0 }
  };

  int opt;
  while ((opt = getopt_long(argc, argv, "n:c:hd:", long_options,
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

  file_writer *dumper = NULL;
  if (dumpDir) {
    try {
      dumper = new file_writer(dumpDir, deviceId, channelId, 100);
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

  // Setup event stream and GBT link
  librorc::event_stream *es = NULL;
  std::unique_ptr<trorc::GbtLink> gbt;
  try {
    es = new librorc::event_stream(trorc->get(), bar->get(), channelId,
                                   librorc::kEventStreamToHost);
    gbt.reset(new trorc::GbtLink(*(bar.get()), channelId));
  }
  catch (int e) {
    cerr << "ERROR: Exception while setting up event stream: "
         << librorc::errMsg(e) << endl;
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

  if(gbt->waitForRxReady() != 0) {
    cerr << "ERROR: GBT RX is not ready!" << endl;
    delete es;
    if (dumper) {
      delete dumper;
    }
  }

  gbt->stopReadout();
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

  gbt->setRxReadoutTargetFrameCount(0); // continuous readout
  gbt->setRxEventSizeFrameCount(0x100);
  gbt->triggerReadout();

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
    }

    int64_t tdiff_us = timediff_us(tlast, tcur);
    if (tdiff_us > 1000000) {
      printStatusLine(es->m_channel_status, &cs_last, tdiff_us, 0);
      memcpy(&cs_last, es->m_channel_status, sizeof(librorc::ChannelStatus));
      tlast = tcur;
    }
  }

  gbt->stopReadout();
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
