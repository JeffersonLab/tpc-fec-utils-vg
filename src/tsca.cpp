#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <vector>
#include <boost/program_options.hpp>

#include "utils.hpp"
#include "cl_options.hpp"
#include "fec.hpp"

using namespace gbt;
namespace bpo = boost::program_options;


void modify_sca_basic(const bpo::variables_map& vm, common::HdlcCore& hdlc)
{
  gbt::ScaBasic sca_basic(hdlc);

  using rwargs_t = std::tuple<std::string, gbt::Sca::sca_rgf_t, gbt::Sca::sca_rgf_t>;
  const rwargs_t rwargs[] = {
    std::make_tuple("sca-cfg-b", sca_basic.CMD_W_CRB, sca_basic.CMD_R_CRB),
    std::make_tuple("sca-cfg-c", sca_basic.CMD_W_CRC, sca_basic.CMD_R_CRC),
    std::make_tuple("sca-cfg-d", sca_basic.CMD_W_CRD, sca_basic.CMD_R_CRD)
  };

  for (auto s: rwargs) {
    const std::string& cmd = std::get<0>(s);
    if (vm.count(cmd)) {
      const std::vector<cl_uint_t>& values = vm[cmd].as<std::vector<cl_uint_t>>();

      if (values.size()) {
        // Value given -> write to register (only take argument 0 from vector, nothing else makes sense)
        sca_basic.transceive(std::get<1>(s), values[0]());
      }

      std::cout << "[" << cmd << "] 0x" << std::hex << sca_basic.transceive(std::get<2>(s)).data
                << std::dec << "\n";
    }
  }
}


void modify_sca_gpio(const bpo::variables_map& vm, common::HdlcCore& hdlc)
{
  gbt::ScaGpio sca_gpio(hdlc);
  gbt::ScaBasic sca_basic(hdlc);

  if (!sca_basic.channelEnabled(sca_gpio.getChannel())) {
    std::cout << "GPIO port is disabled in SCA configuration register. Unable to proceed.\n";
    return;
  }

  using rwargs_t = std::tuple<std::string, std::function<uint32_t(uint32_t)>, std::function<uint32_t(void)>>;
  const rwargs_t rwargs[] = {
    std::make_tuple("gpio-dir", [&](uint32_t v){ return sca_gpio.setDirection(v); }, [&](){ return sca_gpio.getDirection(); }),
    std::make_tuple("gpio-dout", [&](uint32_t v){ return sca_gpio.setDataOut(v); }, [&](){ return sca_gpio.getDataOut(); })
  };

  // Handle gpio-dir & gpio-dout
  for (auto s: rwargs) {
    const std::string& cmd = std::get<0>(s);
    if (vm.count(cmd)) {
      const std::vector<cl_uint_t>& values = vm[cmd].as<std::vector<cl_uint_t>>();

      if (values.size() == 0) {
        // No value args, only get value
        std::cout << "[" << cmd << "] 0x" << std::hex << std::get<2>(s)() << std::dec << std::endl;
      }
      else {
        // Value args given, apply in order
        for (auto v: values) {
          std::cout << "[" << cmd << "] 0x" << std::hex << std::get<1>(s)(v()) << std::dec << std::endl;
        }
      }
    }
  }

  // Handle gpio-din
  if (vm.count("gpio-din")) {
    std::cout << "[gpio-din] 0x" << std::hex << sca_gpio.getDataIn() << std::dec << std::endl;
  }
}


void modify_sca_i2c(const bpo::variables_map& vm, common::HdlcCore& hdlc)
{
  std::unique_ptr<gbt::ScaI2c> i2c;

  // Select SAMPA or GBTx mode of operation for I2C port
  if (vm["i2c-mode"].as<std::string>() == "10b")
    i2c.reset(new gbt::ScaI2cSampa(hdlc, vm["i2c-port"].as<cl_uint_t>()(), vm["i2c-addr"].as<cl_uint_t>()()));
  else
    i2c.reset(new gbt::ScaI2cGbtx(hdlc, vm["i2c-port"].as<cl_uint_t>()(), vm["i2c-addr"].as<cl_uint_t>()()));

  gbt::ScaBasic sca_basic(hdlc);

  if (!sca_basic.channelEnabled(i2c->getChannel())) {
    std::cout << "I2C port " << vm["i2c-port"].as<cl_uint_t>()() << " is disabled in SCA configuration register. Unable to proceed.\n";
    return;
  }

  i2c->setFreq();

  std::stringstream defo;
  defo << "[i2c" << vm["i2c-port"].as<cl_uint_t>()()
       << "-0x" << std::hex << vm["i2c-addr"].as<cl_uint_t>()() << std::dec << "]";

  if (vm.count("i2c-open-drain")) {
    uint32_t v = vm["i2c-open-drain"].as<cl_uint_t>()();
    auto r = i2c->setOutputMode(v == 0 ? gbt::ScaI2c::OUTPUT_DRIVEN : gbt::ScaI2c::OUTPUT_OPEN_DRAIN);

    std::cout << defo.str() << " output mode: "
              << ((r == gbt::ScaI2c::OUTPUT_OPEN_DRAIN) ? "open drain" : "driven")
              << "\n";
  }

  if (vm.count("i2c-rb")) {
    const std::vector<cl_uint_t>& rd_addresses = vm["i2c-rb"].as<std::vector<cl_uint_t>>();

    for (auto ra: rd_addresses)
      std::cout << defo.str() << std::hex << " [rd 0x" << static_cast<uint32_t>(ra()) << "] 0x"
                << static_cast<uint32_t>(i2c->readByte(ra())) << std::dec << "\n";
  }

  if (vm.count("i2c-wb")) {
    // We always get even number of numbers in the vector (verified by input checking)
    //   these are interpreted as addr/value pairs
    const std::vector<cl_uint_t>& wr_info = vm["i2c-wb"].as<std::vector<cl_uint_t>>();

    for (unsigned int i = 0; i < wr_info.size(); i += 2) {
      std::cout << defo.str() << std::hex << " [wr 0x" << wr_info[i]() << "] 0x" << std::hex
                << static_cast<uint32_t>(i2c->writeByte(wr_info[i](), wr_info[i+1]())) << std::dec << "\n";
    }
  }

  if (vm.count("i2c-status")) {
    std::cout << *i2c;
  }
}


int main(int argc, char** argv)
{
  // Short tool description
  const std::string tool_help = \
    "Usage:\n  " + std::string(argv[0]) + " <cmds/options>\n"\
    "  Tool will apply the operations on all SCAs defined by the SCA mask\n"\
    "Commands / Options";

  // clo will already have the default options added
  common::CommandLineOptions clo(tool_help);

  // Add custom options for this tool
  clo.addCmdLine("status,s", "Print HDLC core status")
     .addCmdLine("svl-connect", "Send supervisory-level command CONNECT to SCA")
     .addCmdLine("svl-reset", "Send supervisory-level command RESET to SCA")
     .addCmdLine("svl-test", "Send supervisory-level command TEST to SCA")
     //
     .addCmdLine("core-rst", "Reset HDLC core")
     .addCmdLine("core-init", "Initialize HDLC core")
     .addCmdLine("core-clr", "Clear HDLC core statistics (if core supports this)")
     //
     .addCmdLine("gpio-dir",  bpo::value<std::vector<cl_uint_t>>()->multitoken()->zero_tokens(),
        "Set (if one or more arguments provided) or get (in case of no argument) SCA GPIO DIR value")
     .addCmdLine("gpio-dout", bpo::value<std::vector<cl_uint_t>>()->multitoken()->zero_tokens(),
        "Set (if one or more arguments provided) or get (in case of no argument) SCA GPIO DOUT value")
     .addCmdLine("gpio-din", "Get SCA GPIO DIN value")
     //
     .addCmdLine(
          "i2c-port",
          bpo::value<cl_uint_t>()->default_value(0)->notifier([](cl_uint_t v) { if(v() > 15) {
            throw bpo::validation_error(bpo::validation_error::invalid_option_value, "i2c port"); } } ),
          "SCA I2C port all other I2C options are applied to (0..15)"
        )
     .addCmdLine("i2c-mode",
          bpo::value<std::string>()->default_value("10b")->notifier([](const std::string& s) {
            if ((s.compare("7b") * s.compare("10b")) != 0) {
              throw bpo::validation_error(bpo::validation_error::invalid_option_value, "i2c mode", s); } } ),
          "Select I2C port operation mode: 10b (for SAMPA) or 7b (for GBTx)"
        )
     .addCmdLine("i2c-addr", bpo::value<cl_uint_t>()->default_value(0),
        "I2C slave address")
     .addCmdLine("i2c-wb",
        bpo::value<std::vector<cl_uint_t>>()->multitoken()->notifier([](const std::vector<cl_uint_t>& v) {
          if (v.size() % 2) { throw bpo::validation_error(bpo::validation_error::invalid_option_value, "i2c write"); } } ),
        "I2C write bytes: Expects one or more <addr> <value> pairs as argument (10b addr)")
     .addCmdLine("i2c-rb",
        bpo::value<std::vector<cl_uint_t>>()->multitoken(),
        "I2C read bytes: Expects one or more <addr> to read (10b addr)")
     .addCmdLine("i2c-open-drain", bpo::value<cl_uint_t>(),
        "Set I2C output of selected port to 'open drain' if unequal 0x0, to 'driven' otherwise")
     .addCmdLine("i2c-status", "Print I2C port status")
     //
     .addCmdLine("sca-cfg-b", bpo::value<std::vector<cl_uint_t>>()->multitoken()->zero_tokens(),
        "Write (if arg given) or read SCA config register B")
     .addCmdLine("sca-cfg-c", bpo::value<std::vector<cl_uint_t>>()->multitoken()->zero_tokens(),
        "Write (if arg given) or read SCA config register C")
     .addCmdLine("sca-cfg-d", bpo::value<std::vector<cl_uint_t>>()->multitoken()->zero_tokens(),
        "Write (if arg given) or read SCA config register D")
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

      // Specified SCAs
      if ((clo.getFecMask() >> fec_idx) & 0x1) {

        if (clo.getVerboseFlag())
          std::cout << "Communicating with SCA" << fec_idx << std::endl;

        std::unique_ptr<common::HdlcCore>
          hdlc_core(common::HdlcFactory::makeHdlcCore(*(bar.get()), fec_idx, true));

        //
        if (vm.count("core-clr")) {
          hdlc_core->clr();
          std::cout << "HDLC core " << fec_idx << " statistics cleared\n";
        }
        if (vm.count("core-rst")) {
          hdlc_core->rst();
          std::cout << "HDLC core " << fec_idx << " reset\n";
        }
        if (vm.count("core-init")) {
          hdlc_core->init();
          std::cout << "HDLC core " << fec_idx << " initialized\n";
        }

        //
        if (vm.count("svl-reset")) {
          hdlc_core->sendSvlReset();
          std::cout << "SVL-RESET sent by HDLC core" << fec_idx << "\n";
        }

        if (vm.count("svl-connect")) {
          hdlc_core->sendSvlConnect();
          std::cout << "SVL-CONNECT sent by HDLC core" << fec_idx << "\n";
        }

        if (vm.count("svl-test")) {
          hdlc_core->sendSvlTest();
          std::cout << "SVL-TEST sent by HDLC core" << fec_idx << "\n";
        }

        // Access basic SCA configuration registers
        if (vm.count("sca-cfg-b") ||vm.count("sca-cfg-c") || vm.count("sca-cfg-d"))
          modify_sca_basic(vm, *(hdlc_core.get()));

        // Access SCA GPIO port
        if (vm.count("gpio-dir") || vm.count("gpio-dout") || vm.count("gpio-din"))
          modify_sca_gpio(vm, *(hdlc_core.get()));

        // Access SCA I2C ports
        if (vm.count("i2c-rb") || vm.count("i2c-wb") || vm.count("i2c-open-drain") || vm.count("i2c-status"))
          modify_sca_i2c(vm, *(hdlc_core.get()));

        // Print HDLC status
        if (vm.count("status"))
          std::cout << *(hdlc_core.get()) << "\n";

      }   // Specified SCAs
    }   // SCA loop
  }
  catch (std::exception& e) {
    std::cerr << e.what() << ", exiting" << std::endl;
    exit(100);
  }

  return 0;
}
