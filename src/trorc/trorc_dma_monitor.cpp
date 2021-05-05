/**
 *  trorc_dma_monitor.cpp
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

#include "trorc.hpp"
#include <fcntl.h>
#include <getopt.h>
#include <iomanip> // std::setw
#include <iostream>
#include <librorc.h>
#include <signal.h>
#include <sys/shm.h>
#include <sys/types.h>
#include "trigger.hpp"

using namespace std;

struct t_extLinkStatus {
  librorc::ChannelStatus *ch;
  uint64_t last_events_received;
  uint64_t last_bytes_received;
  int bufferUsed;
  bool active;
};

bool done = false;

void abort_handler(int s) { (void)s;  done = 1; }

// prototypes
string to_numstr(double value, const char *unit);
double timediff(struct timeval from, struct timeval to);
void statsFillBufferUsed(struct t_extLinkStatus &chstat, librorc::bar *bar);
void print_channel_stats(struct t_extLinkStatus chstats, double tdiff);
void print_summary(uint64_t sum_bytes, uint64_t last_sum_bytes,
                   uint32_t trigger_diff, double tdiff);

int main(int argc, char *argv[]) {
  int deviceId = 0;

  static struct option long_options[] = {
    { "device", required_argument, 0, 'n' }, { "help", no_argument, 0, 'h' },
    { 0, 0, 0, 0 }
  };

  int opt;
  while ((opt = getopt_long(argc, argv, "n:h", long_options, NULL)) != -1) {
    switch (opt) {
    case 'n':
      deviceId = strtoul(optarg, NULL, 0);
      break;
    case 'h':
      cout << "trorc_dma_monitor parameters:" << endl
           << "\t-n [ID]  Select specific T-RORC (default: 0)" << endl
           << "\t-h       Show this help" << endl;
      return 0;
    default:
      return -1;
    }
  }

  librorc::device *dev = NULL;
  librorc::bar *bar = NULL;

  try {
    dev = new librorc::device(deviceId);
    bar = new librorc::bar(dev, 1);
  }
  catch (int e) {
    cerr << "ERROR: Failed to initialize T-RORC: " << librorc::errMsg(e)
         << endl;
    exit(1);
  }

  librorc::sysmon *sm = new librorc::sysmon(bar);
  uint32_t nChannels = sm->numberOfChannels();
  delete sm;

  std::unique_ptr<trorc::Trigger> trigger;
  trigger.reset(new trorc::Trigger(*bar));

  std::vector<struct t_extLinkStatus> chstats(nChannels);

  // initialize channel struct and shared memory
  for (uint32_t i = 0; i < nChannels; i++) {
    chstats[i].last_bytes_received = 0;
    chstats[i].last_events_received = 0;
    chstats[i].ch = NULL;
    int shmId = shmget(SHM_KEY_OFFSET + deviceId * SHM_DEV_OFFSET + i,
                       sizeof(librorc::ChannelStatus), IPC_CREAT | 0666);
    if (shmId == -1) {
      cerr << "ERROR: failed to get shared memory: " << strerror(errno) << endl;
      delete bar;
      delete dev;
      return -1;
    }
    void *shm = (void *)shmat(shmId, 0, 0);
    if (shm == (void *)-1) {
      cerr << "ERROR: failed to attach to shared memory: " << strerror(errno)
           << endl;
      delete bar;
      delete dev;
      return -1;
    }
    chstats[i].ch = (librorc::ChannelStatus *)shm;
  }

  // caputure starting time
  timeval tcur;
  gettimeofday(&tcur, NULL);
  timeval tlast = tcur;
  uint64_t last_sum_bytes = 0;
  uint32_t last_trigger_count = 0;

  while (!done) {

    // fetch DMA buffer fill states from FW
    for (uint32_t i = 0; i < nChannels; i++) {
      statsFillBufferUsed(chstats[i], bar);
    }

    gettimeofday(&tcur, NULL);
    double tdiff = timediff(tlast, tcur);
    uint32_t trigger_count = trigger->triggerCount();
    uint32_t trigger_diff = trigger_count - last_trigger_count;

    uint64_t sum_bytes = 0;
    for (uint32_t i = 0; i < nChannels; i++) {
      print_channel_stats(chstats[i], tdiff);
      sum_bytes += chstats[i].ch->bytes_received;
      chstats[i].last_bytes_received = chstats[i].ch->bytes_received;
      chstats[i].last_events_received = chstats[i].ch->n_events;
    }
    print_summary(sum_bytes, last_sum_bytes, trigger_diff,
                  tdiff);

    tlast = tcur;
    last_sum_bytes = sum_bytes;
    last_trigger_count = trigger_count;
    sleep(1);
  }

  for (uint32_t i = 0; i < nChannels; i++) {
    if (chstats[i].ch != NULL) {
      shmdt((void *)chstats[i].ch);
      chstats[i].ch = NULL;
    }
  }

  delete bar;
  delete dev;
  return 0;
}

string to_numstr(double value, const char *unit) {
  stringstream ss;
  double absval = (value < 0) ? -value : value;
  const char *sign = (value < 0) ? "-" : "";
  ss << sign << fixed << setprecision(1) << setw(5);
  if (absval > 1e12) {
    ss << absval / 1e12 << " T" << unit;
  } else if (absval > 1e9) {
    ss << absval / 1e9 << " G" << unit;
  } else if (absval > 1e6) {
    ss << absval / 1e6 << " M" << unit;
  } else if (absval > 1e3) {
    ss << (absval / 1e3) << " k" << unit;
  } else if (absval >= 1e0 || absval == 0) {
    ss << absval << "  " << unit;
  } else if (absval > 1e-3) {
    ss << absval * 1e3 << " m" << unit;
  } else if (absval > 1e-6) {
    ss << absval * 1e6 << " u" << unit;
  } else {
    ss << absval * 1e9 << " n" << unit;
  }
  return ss.str();
}

double timediff(struct timeval from, struct timeval to) {
  return ((int64_t)(to.tv_sec - from.tv_sec) * 1000000LL +
          (int64_t)(to.tv_usec - from.tv_usec)) /
         1.0e6f;
}

void statsFillBufferUsed(struct t_extLinkStatus &chstat, librorc::bar *bar) {
  librorc::link *link = new librorc::link(bar, chstat.ch->channel);
  librorc::dma_channel *dma = new librorc::dma_channel(link);
  if (dma->getEnable()) {
    uint64_t bufferSize = dma->getEBSize();
    uint64_t hwWriteOffset = dma->getEBDMAOffset();
    uint64_t swReadOffset = dma->getEBOffset();
    uint64_t bytesUsed = (hwWriteOffset > swReadOffset)
                             ? (hwWriteOffset - swReadOffset)
                             : (hwWriteOffset + bufferSize - swReadOffset);
    chstat.bufferUsed = bytesUsed * 100 / bufferSize;
    chstat.active = true;
  } else {
    chstat.bufferUsed = 0;
    chstat.active = false;
  }
  delete dma;
  delete link;
}

void print_channel_stats(struct t_extLinkStatus chstats, double tdiff) {
  double data_rate =
      (chstats.last_bytes_received)
          ? (chstats.ch->bytes_received - chstats.last_bytes_received) / tdiff
          : 0;
  double event_rate =
      (chstats.last_events_received)
          ? (chstats.ch->n_events - chstats.last_events_received) / tdiff
          : 0;
  cout << "Ch" << setw(2) << chstats.ch->channel << " - "
       << "Events: " << to_numstr(chstats.ch->n_events, "")
       << ", Size: " << to_numstr(chstats.ch->bytes_received, "B");
  if (!chstats.active) {
    cout << "   [inactive]" << endl;
  } else {
    cout << ", Rate: " << to_numstr(data_rate, "B/s") << " / "
         << to_numstr(event_rate, "Hz") << ", BufferUsage: " << setw(3)
         << chstats.bufferUsed << "%"
         << ", Err: " << chstats.ch->error_count << endl;
  }
}

void print_summary(uint64_t sum_bytes, uint64_t last_sum_bytes,
                   uint32_t trigger_diff, double tdiff) {
  double total_rate =
      (last_sum_bytes) ? (sum_bytes - last_sum_bytes) / tdiff : 0;
  double trigger_rate = trigger_diff / tdiff;
  cout << "===========================================================" << endl;
  cout << "TotalDataSize: " << to_numstr(sum_bytes, "B")
       << ", TotalDataRate: " << to_numstr(total_rate, "B/s")
       << ", TriggerRate: " << to_numstr(trigger_rate, "Hz") << endl;
  cout << "===========================================================" << endl;
}
