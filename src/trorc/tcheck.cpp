#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <vector>
#include <boost/program_options.hpp>

#include "trorc.hpp"
#include "fec.hpp"
#include "gbt_link.hpp"
#include "git_info.hpp"

namespace bpo = boost::program_options;

//------------------------------------------------------------------------------
class ChannelDecoderStatus
{
  public:
    const uint32_t link_;
    static const uint8_t  cpl_{(Fec::n_sampa_+1)/2};  // Clocks per link
    static const uint8_t  spl_{Fec::n_sampa_};        // SYNC patterns per link 

    ChannelDecoderStatus(uint32_t link) : link_(link) {}

    uint32_t adc_clk_error_count_[cpl_];
    bool     adc_clk_status_[cpl_];      // true -> adc_clk found
    uint32_t sync_pattern_count_[spl_];
    bool     sync_pattern_status_[spl_]; // true -> sync found
    uint8_t  sync_pattern_pos_[spl_];
};


// Color printing for bool
std::string is_good(bool v)
{
  return (v ? "\033[1;32m1\033[;39m" : "\033[1;31m0\033[;39m");
}

// Color printing for error counters
std::string is_zero(int64_t v)
{
  return std::string("\033[1;") + ((v == 0x0) ? "32" : "31") + "m" +
           (((v < 10) && (v >= 0)) ? ("  " + std::to_string(v)) : ">10") +
           "\033[;39m";
}


//------------------------------------------------------------------------------
ChannelDecoderStatus link_decoder_status(trorc::Bar& bar, uint32_t link_idx)
{
  std::unique_ptr<trorc::GbtLink> link(new trorc::GbtLink(bar, link_idx));

  ChannelDecoderStatus cds(link_idx);

  uint32_t adcStatus = link->decAdcStatus();
  for (auto i = 0; i < cds.cpl_; i++) {
    // Register mappings
    //   link even -> sampa 0, 1, 2.0
    //   link odd  -> sampa 3, 4, 2.5
    int l = (link_idx % 2) ? ((i+1) % cds.cpl_) : i;
    cds.adc_clk_error_count_[l] = link->decAdcErrorCounter(i);
    cds.adc_clk_status_[l] = (((adcStatus >> i) & 0x1) == 1);
  }

  uint32_t syncStatus = link->decSyncPatternStatus();
  for (auto i = 0; i < cds.spl_; i++) {
    // Register mappings
    //   link even -> sampa 0[0,1] 1[2,3]  2[5]
    //   link odd  -> sampa 3[7,8] 4[9,10] 2[6]
    int l = (link_idx % 2) ? ((i+1) % cds.spl_) : i;
    cds.sync_pattern_count_[l] = link->decSyncPatternCounter(i);
    cds.sync_pattern_status_[l] = (((syncStatus >> (3*i + 2)) & 0x1) == 1);
    cds.sync_pattern_pos_[l] = ((syncStatus >> (3*i)) & 0x3);
  }

  return cds;
}

void print_link_decoder_status_summary(std::vector<ChannelDecoderStatus> vs, bool detail = false)
{
  std::ostream& s(std::cout);

  s << "        ADC clk[0..4]         Sync[0..9]\n";
  for (uint32_t i = 1; i < vs.size(); i += 2) {
    s << "FEC " << (vs[i-1].link_/Fec::n_gbtx_) << ":   ";

    // ADC clock status
    for (int32_t c = 0; c < vs[i-1].cpl_; c++)
      s << is_good(vs[i-1].adc_clk_status_[c])
        << ((c == (vs[i-1].cpl_ - 1)) ? "|" : " ");
    for (int32_t c = 0; c < vs[i].cpl_; c++)
      s << is_good(vs[i].adc_clk_status_[c]) << " ";
    s << "    ";

    // SYNC pattern status
    for (int32_t c = 0; c < vs[i-1].spl_; c++)
      s << is_good(vs[i-1].sync_pattern_status_[c])
        << ((c == (vs[i-1].spl_ - 1)) ? "|" : " ");
    for (int32_t c = 0; c < vs[i].spl_; c++)
      s << is_good(vs[i].sync_pattern_status_[c]) << " ";
    s << "\n";
  }

  if (!detail)
    return;

  s << "\n";
  s << "           ADC clk errors[0..4]\n";
  for (uint32_t i = 1; i < vs.size(); i += 2) {
    s << "FEC " << (vs[i-1].link_/Fec::n_gbtx_) << ":  ";
    // ADC error count status
    for (int32_t c = 0; c < vs[i-1].cpl_; c++)
      s << is_zero(vs[i-1].adc_clk_error_count_[c])
        << ((c == (vs[i-1].cpl_ - 1)) ? "|" : "  ");
    for (int32_t c = 0; c < vs[i].cpl_; c++)
      s << is_zero(vs[i].adc_clk_error_count_[c]) << "  ";
    s << "\n";
  }

}

void clear_link_status(trorc::Bar& bar, uint32_t link_idx)
{
  std::unique_ptr<trorc::GbtLink> link(new trorc::GbtLink(bar, link_idx));
  link->resetDecoderCounter(0x7);
}


//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  bpo::variables_map vm;
  bpo::options_description opt_general(
    "Usage:\n  " + std::string(argv[0]) + " <cmds/options>\n"
    "  Tool will apply the operations on all SCAs defined by the SCA mask\n"
    "Commands / Options");
  bpo::options_description opt_hidden("");
  bpo::options_description opt_all;
  bpo::positional_options_description opt_pos;

  try
  {
    opt_general.add_options()
      ("help,h", "Print this help message")
      ("version", "Print version information")
      //
      ("mask,m", bpo::value<cl_uint_t>()->default_value(0x3),
         "Mask defining on which FECs/SCAs to execute all following operations")
      
      ("channel-decoder", "Print channel decoder status summary")
      ("detail", "Enable more detailed output")
      ("clear", "Clear counters")
      ;

    opt_all.add(opt_general).add(opt_hidden);

    bpo::store(bpo::command_line_parser(argc, argv).options(opt_all)
                     .positional(opt_pos).run(), vm);

    if (vm.count("help") || argc == 1) {
      std::cout << opt_general << std::endl;
      exit(0);
    }

    bpo::notify(vm);
  }
  catch(bpo::error& e)
  {
    std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
    std::cerr << opt_general << std::endl;
    exit(1);
  }
  catch(std::exception& e)
  {
    std::cerr << e.what() << ", application will now exit" << std::endl;
    exit(2);
  }

  if (vm.count("version"))
    std::cout << GitInfo();

  // Do stuff
  int device_number(0);
  int bar_number(1);
  std::unique_ptr<trorc::Device> trorc;
  std::unique_ptr<trorc::Bar> bar;

  try {
    trorc.reset(new trorc::Device(device_number));
    bar.reset(new trorc::Bar(*trorc, bar_number));
  } catch (int e) {
    std::cerr << "ERROR: Failed to initialize T-RORC: " << librorc::errMsg(e)
              << std::endl;
    exit(1);
  }

  // Clearing statistics
  if (vm.count("clear")) {
    try {
      for (uint32_t fec_idx = 0; fec_idx < 32; fec_idx++) {
        if ((vm["mask"].as<cl_uint_t>()() >> fec_idx) & 0x1) {
          clear_link_status(*bar, 2*fec_idx + 0);
          clear_link_status(*bar, 2*fec_idx + 1);
        }
      }
      std::cout << "Link statistics counters cleared\n";
    }
    catch (std::exception& e) {
      std::cerr << e.what() << ", exiting" << std::endl;
      exit(100);
    }
  }

  // Data collection & printing
  std::vector<ChannelDecoderStatus> vs;

  try {
    // FEC, or SCA loop
    for (uint32_t fec_idx = 0; fec_idx < 32; fec_idx++) {
      if ((vm["mask"].as<cl_uint_t>()() >> fec_idx) & 0x1) {

        if (vm.count("channel-decoder")) {
          vs.push_back(link_decoder_status(*bar, 2*fec_idx + 0));   // FEC n, GBTx0
          vs.push_back(link_decoder_status(*bar, 2*fec_idx + 1));   // FEC n, GBTx1
        }
        
      }
    }   // SCA loop

    if (vm.count("channel-decoder")) {
      print_link_decoder_status_summary(vs, vm.count("detail"));
    }
  }
  catch (std::exception& e) {
    std::cerr << e.what() << ", exiting" << std::endl;
    exit(101);
  }

  return 0;
}
