#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <vector>

#include "utils.hpp"
#include "cl_options.hpp"
#include "fec.hpp"

using namespace gbt;
namespace bpo = boost::program_options;

void
dumpSampaRegisters(const Fec& fec, uint32_t sampa_mask)
{
  for (uint32_t s = 0; s < Fec::n_sampa_; s++) {
    if ((sampa_mask >> s) & 0x1) {
      std::cout << "--- SAMPA " << s << " ----------------------------\n";
      Sampa& sampa = fec.getSampa(s);
      sampa.dumpRegisters();
    }
  }
}


void
readSampaRegister(const Fec& fec, uint32_t sampa_mask, uint32_t addr, bool verbose = false)
{
  std::cout << std::hex << std::setfill('0');
  for (uint32_t s = 0; s < Fec::n_sampa_; s++) {
    if ((sampa_mask >> s) & 0x1) {
      Sampa& sampa = fec.getSampa(s);
      std::cout <<  "0x" << std::setw(2) << static_cast<uint32_t>(sampa.readRegister(addr));
      if (verbose)
        std::cout << " [SAMPA " << s << ", Address 0x" << std::setw(2) << addr << "]";
      std::cout << "\n";

    }
  }
  std::cout << std::dec << std::setfill(' ');
}

void
printSampaClkConfig(const Fec& fec, uint32_t sampa_mask)
{
  uint32_t sampa_powered = fec.sampaPwrStatus();

  std::cout << "SAMPA clk_config\n";

  for (uint32_t s = 0; s < Fec::n_sampa_; s++) {
    if ((sampa_mask >> s) & 0x1) {
      Sampa& sampa = fec.getSampa(s);
      std::cout << "SAMPA " << s << " : ";
      if(((sampa_powered >> s) & 0x1) == 0x0)
        std::cout << "Not powered";
      else
        std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<uint32_t>(sampa.getClkConfig())
                  << std::setfill(' ') << std::dec;
      std::cout << "\n";
    }
  }
}


int
main(int argc, char** argv)
{
  const std::string tool_help = \
    "Usage:\n  " + std::string(argv[0]) + " <cmds/options>\n"\
    "  Tool will apply the operations on all FECs/SCAs defined by the corresponding mask\n"\
    "  Which SAMPAs on a given FEC are affected is controlled by the SAMPA mask\n"\
    "Commands / Options";

  common::CommandLineOptions clo(tool_help);

  clo.addCmdLine("sampa-mask", bpo::value<cl_uint_t>()->default_value(0x1f, "0x1f"),
                 "Mask of SAMPAs to apply commands below to")
     .addCmdLine("scan", "Scan I2C for active (replying) SAMPAs")
     .addCmdLine("pwr-on", "Turn on selected SAMPAs")
     .addCmdLine("pwr-off", "Turn off selected SAMPAs")
     .addCmdLine("pwr-wait", bpo::value<cl_uint_t>()->default_value(0x0)
                 ->notifier([](cl_uint_t v) { if(v() > 1e3) {
                     throw bpo::validation_error(bpo::validation_error::invalid_option_value, "pwr-wait");  } } ),
                 "Time (in ms) to wait between performing power changes on the SAMPAs in the SAMPA mask. "
                 "Valid values are [0..1e3] ms")
     .addCmdLine("elinks", bpo::value<uint32_t>(), "Enable n SAMPA elinks")
     .addCmdLine("clk-cfg", bpo::value<std::vector<cl_uint_t>>()->multitoken()->zero_tokens(), "Set (if arg provided) or print SAMPA clock configuration")
     .addCmdLine("write,w", bpo::value<std::vector<cl_uint_t>>()->multitoken()
                 ->notifier([](std::vector<cl_uint_t> v) { if (v.size() % 2) {
                     throw bpo::invalid_syntax(bpo::invalid_syntax::missing_parameter, "write"); } } ),
                 "Write <value> to SAMPA global register <addr>. Input is expected as 'addr value' pairs, i.e. even number of (integer) parameters."\
                 " Value is written to all SAMPAs in SAMPA mask")
     .addCmdLine("read,r", bpo::value<cl_uint_t>(), "Read SAMPA global register <addr> from all SAMPAs selected by 'sampa-mask'")
     .addCmdLine("dump-registers", "Dump global registers of the SAMPAs selected by 'sampa-mask'")
     ;

  // Process options
  auto vm = clo.Process(argc, argv);

  // Do stuff
  std::unique_ptr<common::Bar> bar;
  try {
    bar.reset(common::BarFactory::makeBar(clo.getId(), clo.getBar()));
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
    exit(1);
  }

  try {
    // FEC, or SCA loop
    for (uint32_t fec_idx = 0; fec_idx < 32; fec_idx++) {
      // Specified FECs/SCAs
      if ((clo.getFecMask() >> fec_idx) & 0x1) {

        std::unique_ptr<common::HdlcCore>
          hdlc_core(common::HdlcFactory::makeHdlcCore(*(bar.get()), fec_idx, true));
        Fec fec(*(hdlc_core.get()));

        if (clo.getVerboseFlag())
          std::cout << "Communcating with SCA" << fec_idx << ". Sampa mask is 0x"
                    << std::hex << vm["sampa-mask"].as<cl_uint_t>()() << std::dec << std::endl;

        if (vm.count("scan")) {
          fec.printIdString(std::cout);
          std::cout << "Scanning SAMPA I2C ports for active devices" << std::endl;
          fec.sampaI2cScan(&std::cout);
        }

        if (vm.count("pwr-off")) {
          std::cout << "Powering off SAMPAs... ";
          fec.sampaPwrOff(vm["sampa-mask"].as<cl_uint_t>()() & ((1 << Fec::n_sampa_) - 1),
                          vm["pwr-wait"].as<cl_uint_t>()());
          std::cout << "[0x" << std::hex << fec.sampaPwrStatus() << std::dec << " now powered]\n";
        }
        else if (vm.count("pwr-on")) {
          std::cout << "Powering on SAMPAs... ";
          fec.sampaPwrOn(vm["sampa-mask"].as<cl_uint_t>()() & ((1 << Fec::n_sampa_) - 1),
                         vm["pwr-wait"].as<cl_uint_t>()());
          std::cout << "[0x" << std::hex << fec.sampaPwrStatus() << std::dec << " now powered]\n";
        }

        if (vm.count("elinks")) {
          std::cout << "Enabling SAMPA elinks" << std::endl;
          fec.sampaEnableElinks(vm["elinks"].as<uint32_t>() & 0xff, vm["sampa-mask"].as<cl_uint_t>()() & 0xff);
        }

        if (vm.count("clk-cfg")) {
          const std::vector<cl_uint_t>& w = vm["clk-cfg"].as<std::vector<cl_uint_t>>();

          if (w.size() == 0)
            printSampaClkConfig(fec, vm["sampa-mask"].as<cl_uint_t>()());
          else
            std::cout << "Set SAMPA clk_cfg to 0x" << std::hex << std::setfill('0') << std::setw(2)
                      << fec.sampaSetClkCfg(w.at(0)()) << "\n";
        }


        if (vm.count("write")) {
          // See above: We (can) assume here, that vector always contains even number n of values, which
          //   are interpreted as 'addr_0 value_0 ... addr_n/2 value_n/2' pairs
          const std::vector<cl_uint_t>& w = vm["write"].as<std::vector<cl_uint_t>>();

          for (uint32_t i = 0; i < w.size()/2; i++) {
            uint32_t addr = w[i]();
            uint32_t val  = w[i+1]();
            std::cout << i << " -> " << addr << " | " << val << "\n";
          }
        }

        if (vm.count("read")) {
          readSampaRegister(fec, vm["sampa-mask"].as<cl_uint_t>()(), vm["read"].as<cl_uint_t>()(), vm.count("verbose") != 0);
        }

        if (vm.count("dump-registers"))
          dumpSampaRegisters(fec, vm["sampa-mask"].as<cl_uint_t>()());

      }   // Specified SCAs
    }   // SCA loop
  }
  catch (std::runtime_error& e) {
    std::cerr << e.what() << std::endl;
    exit(100);
  }

  return 0;
}
