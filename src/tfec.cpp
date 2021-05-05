#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <vector>
#include <chrono>
#include <boost/program_options.hpp>

#include "utils.hpp"
#include "cl_options.hpp"
#include "fec.hpp"

using namespace gbt;
namespace bpo = boost::program_options;

void fecInit(Fec& fec)
{
  fec.init();

  ScaGpio& gpio(fec.getScaGpio());
  std::cout << gpio << std::endl;

  ScaAdc& adc(fec.getScaAdc());
  std::cout << adc << std::endl;
}

void gbtxI2cScan(Fec& fec, uint32_t verbosity = 0)
{
  fec.printIdString(std::cout);
  std::cout << "Scanning addresses on GBTx I2C ports for active devices\n";
  fec.gbtxI2cScan(0, &std::cout, verbosity != 0);
  fec.gbtxI2cScan(1, &std::cout, verbosity != 0);
}

/** Control GBTx test mode register
 *  @param hdlc_core Reference to common::HdlcCore instance
 *  @param gbtx_idx GBT index [0, 1]
 *  @param value Value of the txTestMode register, as defined in GBT manual (v0.15, section 7.1.1)
 *                     - "00" normal (data forward)
 *                     - "01" static pattern
 *                     - "10" counter pattern
 *                     - "11" PRBS pattern
 *  @note Since the GBTx will scramble the EC bits of the GBTx frames in upstream direction (once test_mode /= 0),
 *        this uses a simple push mechanism of underlying HDLC frames to SCA. Reply frames are not
 *        evaluated or checked.
 */
void gbtxTxTestPattern(common::HdlcCore& hdlc_core, uint8_t gbtx_idx, uint8_t value)
{
  std::string modes[] = {"data forwarding", "static pattern", "counter pattern", "pseudo-random"};

  std::unique_ptr<gbt::ScaI2c> i2c_gbtx(new gbt::ScaI2cGbtx(hdlc_core, gbtx_idx, Fec::cfg_i2c_addr_gbtx_[gbtx_idx]));

  // GBTx register 28 controls txTestMode multiplexer and FEC TMR
  //   We dont care about the FEC TMR voter on bit6 (is bypassed in widebus mode anyway)
  i2c_gbtx->pushByte(28, value & 0x3);

  std::cout << "Set GBTx" << static_cast<uint32_t>(gbtx_idx) << " Tx pattern to 0x"
            << std::hex << static_cast<uint32_t>(value) << std::dec << " (" << modes[value] << ")"
            << "\n";
}

/** Alter GBTx slave I2C addresses (temporarily)
 *  @param fec Reference to Fec instance
 *  @param gbtx_idx GBTx index [0,1]
 *  @param val Temporary GBTx I2C slave address
 */
void gbtxSetI2cAddr(Fec& fec, uint8_t gbtx_idx, uint32_t val)
{
  ScaI2c& i2c(fec.getScaI2cGbtx(gbtx_idx));
  fec.printIdString(std::cout);
  std::cout << "  Set GBTx" << static_cast<uint32_t>(gbtx_idx)
            << " I2C slave address temporarily to "
            << i2c.setSlaveAddress(val) << "\n";
}


/**
 */
void benchmarkGpio(Fec& fec, uint32_t iterations)
{
  gbt::ScaGpio& gpio = fec.getScaGpio();

  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

  for (uint32_t i = 0; i < iterations; i++)
    gpio.getDataIn();

  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

  std::cout << "GPIO DataIn Benchmark\n"
            << "  " << std::setw(5) << iterations << " read iterations : " << time_span.count() << "s"
            << "  -> " << (time_span.count() / iterations) << "s per iteration\n";
}


int main(int argc, char** argv)
{
  const std::string tool_help = \
    "Usage:\n  " + std::string(argv[0]) + " <cmds/options>\n"\
    "  Tool will apply the operations on all FECs/SCAs defined by the corresponding mask\n"\
    "  Options marked with '***' are prioritized and will potentially ignore other options\n"\
    "Commands / Options";

  common::CommandLineOptions clo(tool_help);

  clo.addCmdLine("init", "Initialize FEC(s) with basic configuration")
      // gbtx*-addr needs to come before other gbtx* options
      .addCmdLine("gbtx0-addr", bpo::value<cl_uint_t>(),
                  "Use alternative GBTx0 I2C address for following GBTx0 operations [1,..,15]")
      .addCmdLine("gbtx1-addr", bpo::value<cl_uint_t>(),
                  "Use alternative GBTx1 I2C address for following GBTx1 operations [1,..,15]")
      .addCmdLine("gbtx-scan", "Scan for GBTx (addresses) on I2C")
      .addCmdLine("gbtx0-cfg", bpo::value<std::string>(), "Configure GBTx0 with file")
      .addCmdLine("gbtx1-cfg", bpo::value<std::string>(), "Configure GBTx1 with file")
      .addCmdLine("gbtx0-cfg-dump", "Dump GBTx0 configuration")
      .addCmdLine("gbtx1-cfg-dump", "Dump GBTx1 configuration")
      //
      .addCmdLine("status,s", "Print FEC status")
      .addCmdLine("sca-status", "Print relevant SCA registers")
      .addCmdLine("adc-status", "Print relevant SCA ADC registers")
      //
      .addCmdLine("adc-all", "Measure and print all (defined) ADC input ports")
      .addCmdLine("adc-port", bpo::value<cl_uint_t>(), "Measure and print given ADC input port, if defined")
      //
      .addCmdLine("print-sca-id-map", "Dump known mappings of SCA ID to FEC revision (and exit)")
      //
      .addCmdLine("gbtx0-pgen", bpo::value<cl_uint_t>()
                  ->notifier([](cl_uint_t v) { if(v() > 3) {
                     throw bpo::validation_error(bpo::validation_error::invalid_option_value, "gbtx0 pgen"); } } ),
                   "*** Control GBTx0 upstream test pattern (0: data forward, 1-3: static/counter/pbrs)")
      .addCmdLine("gbtx1-pgen", bpo::value<cl_uint_t>()
                  ->notifier([](cl_uint_t v) { if(v() > 3) {
                     throw bpo::validation_error(bpo::validation_error::invalid_option_value, "gbtx1 pgen"); } } ),
                   "*** Control GBTx1 upstream test pattern (0: data forward, 1-3: static/counter/pbrs)")
      //
      .addCmdLine("benchmark-gpio", bpo::value<cl_uint_t>()->implicit_value(1000)
                  ->notifier([](cl_uint_t v) { if(v() == 0) {
                    throw bpo::validation_error(bpo::validation_error::invalid_option_value, "benchmark gpio"); } } ),
                  "Benchmark SCA GPIO register access (Specify iterations as argument)")
      ;

  // Process parameters
  auto vm = clo.Process(argc, argv);

  if (vm.count("print-sca-id-map")) {
    FecRevision::printMap(std::cout);
    exit(0);
  }

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

        if (vm.count("verbose"))
          std::cout << "Communicating with SCA" << fec_idx << std::endl;

        std::unique_ptr<common::HdlcCore>
          hdlc_core(common::HdlcFactory::makeHdlcCore(*(bar.get()), fec_idx, true));

        if (vm.count("gbtx0-pgen") || vm.count("gbtx1-pgen")) {
          // These options are special, as they potentially alter the EC bits on
          //   the upstream GBTx frames, i.e. no HDLC response arrives
          // Do not mix these with the other options
          if (vm.count("gbtx0-pgen"))
            gbtxTxTestPattern(*(hdlc_core.get()), 0, vm["gbtx0-pgen"].as<cl_uint_t>()());
          if (vm.count("gbtx1-pgen"))
            gbtxTxTestPattern(*(hdlc_core.get()), 1, vm["gbtx1-pgen"].as<cl_uint_t>()());
        }
        else {
          Fec fec(*(hdlc_core.get()));

          if (clo.getVerboseFlag())
            fec.setVerbosity(10);

          if (vm.count("init"))
            fecInit(fec);

          // GBTx-related
          if (vm.count("gbtx-scan"))
            gbtxI2cScan(fec, vm.count("verbose"));

          if (vm.count("gbtx0-addr"))
            gbtxSetI2cAddr(fec, 0, static_cast<uint32_t>(vm["gbtx0-addr"].as<cl_uint_t>()()));
          if (vm.count("gbtx1-addr"))
            gbtxSetI2cAddr(fec, 1, static_cast<uint32_t>(vm["gbtx1-addr"].as<cl_uint_t>()()));

          if (vm.count("gbtx0-cfg"))
            fec.gbtxConfigure(0, vm["gbtx0-cfg"].as<std::string>());
          if (vm.count("gbtx1-cfg"))
            fec.gbtxConfigure(1, vm["gbtx1-cfg"].as<std::string>());

          if (vm.count("gbtx0-cfg-dump"))
            fec.gbtxDumpConfiguration(0, std::cout);
          if (vm.count("gbtx1-cfg-dump"))
            fec.gbtxDumpConfiguration(1);

          // GPIO-related
          if(vm.count("benchmark-gpio"))
            benchmarkGpio(fec, vm["benchmark-gpio"].as<cl_uint_t>()());

          // Status & stuff
          if (vm.count("status"))
            std::cout << fec;
          if(vm.count("adc-status"))
            std::cout << fec.getScaAdc();

          if(vm.count("sca-status")) {
            std::cout << fec.getScaBasic() << fec.getScaGpio();
            for (auto i = 0; i < Fec::n_gbtx_; i++)
              std::cout << fec.getScaI2cGbtx(i);
            for (auto i = 0; i < Fec::n_sampa_; i++)
              std::cout << fec.getScaI2cSampa(i);
          }

          // ADC-related
          if(vm.count("adc-all"))
            fec.adcMeasureAll();
          if(vm.count("adc-port"))
            fec.adcMeasureInput(vm["adc-port"].as<cl_uint_t>()());
        }
      } // Specified FECs/SCAs
    } // FEC/SCA loop
  }
  catch (std::exception& e) {
    std::cerr << e.what() << ", exiting" << std::endl;
    exit(100);
  }

  return 0;
}
