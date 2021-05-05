#include <stdexcept>
#include <vector>
#include <chrono>
#include <thread>
#include <boost/program_options.hpp>

#include "sampa_tester.hpp"
#include "utils.hpp"
#include "cl_options.hpp"
#include "pcca_interface.hpp"

using namespace gbt;
namespace bpo = boost::program_options;

//------------------------------------------------------------------------------
// Tests
//------------------------------------------------------------------------------

/** SAMPA I2C read test
 *  @return Number of read-errors encountered
 */
uint32_t test_sampa_i2c(const SampaTester& st, uint32_t iterations = 1000)
{
  Sampa sampa(st.getScaI2cSampa());
  uint32_t i2c_read_errors(0);
  uint32_t i2c_read_transactions(0);
  uint32_t error_msg_limit(5);

  for (uint32_t i = 0; i < iterations; i++) {
    for (auto regs: Sampa::registers_) { // loop over (some - see sampa.hpp) SAMPA global regs
      try {
        i2c_read_transactions++;
        sampa.readRegister(regs.first);
      }
      catch (gbt::ScaException& e) {
        if (i2c_read_errors < error_msg_limit)
          std::cout << "i2c_read_test : Iteration " << i << " failed : " << e.what() << "\n";
        if (i2c_read_errors == (error_msg_limit - 1))
          std::cout << "i2c_read_test : Error message limit reached, future error messages suppressed\n";
        i2c_read_errors++;
      }
    }
  }

  // FIXME: make report a bit nicer
  std::cout << "i2c_read_transactions: " << i2c_read_transactions << "\n"
            << "i2c_read_errors: " << i2c_read_errors << "\n";

  return i2c_read_errors;
}


//------------------------------------------------------------------------------
/** SAMPA ADCTRIM/voltage scan
 */
uint32_t test_adctrim_scan(const SampaTester& st, bool json = false, std::ostream& os = std::cout)
{
  Sampa sampa(st.getScaI2cSampa());
  st.updateSampaI2cAddress();

  const uint8_t adctrim_values(8);
  std::array<uint8_t, 3> inputs = {8, 9, 10};    // V450 .. V750
  std::array<float, inputs.size()*adctrim_values> meas;

  // Vary ADCTRIM and scan voltages
  for (uint8_t v = 0; v < adctrim_values; v++) {  // loop over ADCTRIM values
    sampa.writeRegister(Sampa::REG_ADCTRIM, v);

    for (uint8_t i = 0; i < inputs.size(); i++) {
      const SampaTester::adc_input_t& ai = st.getExtAdcInputDetails(inputs[i]);
      uint16_t adc_raw = st.getScaExtAdc().sample(inputs[i]);
//      std::cout << static_cast<uint32_t>(i) << "->" << static_cast<uint32_t>(adc_raw) << std::endl;
      meas[v*3+i] = adc_raw*std::get<3>(ai);
    }
  }

  // Print output (plain text or JSON version)
  os << (json ? "{\n" : "");

  // Print ADCTRIM line
  os << (json ? "\"" : "") << "ADCTRIM" << (json ? "\": [" : ": ");
  for (uint32_t i = 0; i < adctrim_values; i++)
    os << i << (i != (adctrim_values - 1) ? ", " : "");
  os << (json ? "]," : "") << "\n";

  // Print lines with measured values
  for (uint32_t i = 0; i < meas.size(); i++) {

    if (i == (i % inputs.size())) {
      const SampaTester::adc_input_t& ai = st.getExtAdcInputDetails(inputs[i]);
      os << (json ? "\"" : "") << std::get<0>(ai)
         << (json ? ("\": {\"unit\": \"" + std::get<1>(ai) + "\", [") : ("[" + std::get<1>(ai) + "]: "));
    }

    os << meas[i];

    if (i >= (meas.size()-inputs.size())) {
      os << (json ? "]}" : "") << ((json && (i != (meas.size() - 1))) ? "," : "") << "\n";
      if (i != (meas.size() - 1))
        i = i % inputs.size();
    }
    else {
      os << ", ";
      i += 2;
    }
  }
  os << (json ? "}\n" : "");

  return 0;
}


//------------------------------------------------------------------------------
/** Run ring oscillator test
 *  @param os Output stream to print results to
 */
void test_ring_oscillator(const SampaTester& st, std::ostream& os = std::cout)
{
  Sampa s(st.getScaI2cSampa());
  auto clk_config = st.getClkConfig();  // Remember
  st.setClkConfig(Sampa::CLKCONF_RNGOSC);

  s.writeRegister(Sampa::REG_CMD, Sampa::CMD_RINGCNT);
  uint32_t ring_count = static_cast<uint32_t>(s.readRegister(Sampa::REG_RINGCNT));
  float clk_speed = (255 - (ring_count - 0x7f)) * 16 / 255e-1;
  os << "Ring Oscillator Test\n"
     << "  RNG_CNT register 0x" << std::setfill('0') << std::hex << std::setw(2) << ring_count
     << "  Clock speed      " << std::setprecision(3) << clk_speed << " MHz"
     << " (0x" << std::hex << (ring_count - 0x7f) << std::dec << ")\n"
     << std::setfill(' ');

  st.setClkConfig(clk_config);   // Restore
}


//------------------------------------------------------------------------------
/** Run SAMPA memory test
 */
void test_sampa_memory(const SampaTester& st, const PccaInterface& pcca, std::ostream& os = std::cout)
{
  auto clk_config = st.getClkConfig();  // Remember
  auto mux_sel = st.getMuxSel();        // Remember
  st.setMuxSel(0);
  st.setClkConfig(Sampa::CLKCONF_MEMTEST);

  pcca.memtestInitialize();
  pcca.memtestStart();
  std::this_thread::sleep_for(std::chrono::nanoseconds(100 + 5 * 1024 * 200 /*1/(5 MHz)*/));
  uint8_t r = pcca.memtestGetResult();
  pcca.memtestDeInitialize();

  st.setClkConfig(clk_config);   // Restore
  st.setMuxSel(mux_sel);         // Restore

  os << "SAMPA Memory Test: "
     << ((r == 0x1) ? "successful" : "failed") << " (SMO = " << static_cast<uint32_t>(r) << ")\n";
}


//------------------------------------------------------------------------------
void dump_sampa_registers(const SampaTester& st, uint32_t sca_idx, std::ostream& os = std::cout)
{
  std::cout << "--- SAMPA (setup " << sca_idx << ") ---------------------\n";
  Sampa s(st.getScaI2cSampa());
  s.dumpRegisters(os);
}


//------------------------------------------------------------------------------
// Command-line handling
//------------------------------------------------------------------------------
void control_dac_output(const bpo::variables_map& vm, SampaTester& st)
{
  for (auto op: SampaTester::dac_outputs_) {
    std::string cmd = std::get<0>(op);
    std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

    if (vm.count(cmd)) {
      const std::vector<cl_uint_t>& values = vm[cmd].as<std::vector<cl_uint_t>>();

      if (values.size() == 0) {
        // No value args, only get current DAC value and print
        uint8_t v = st.getScaDac().get(std::get<1>(op));
        std::cout << "[" << std::get<0>(op) << "] " << (v/256.) << "V (0x"
                  << std::hex << static_cast<uint32_t>(v) << std::dec << ")\n";
      }
      else {
        // New value(s) given, apply and read back updated DAC value
        for (auto v: values) {
          uint8_t nv = st.getScaDac().set(std::get<1>(op), v() & 0xff);
          std::cout << "[" << std::get<0>(op) << "] " << (nv/256.) << "V (0x"
                    << std::hex << static_cast<uint32_t>(nv) << std::dec << ")\n";
        }
      }
    }
  }
}


//------------------------------------------------------------------------------
void control_gpio(const bpo::variables_map& vm, SampaTester& st)
{
  using gpio_control_t = std::tuple<std::string /*name*/, std::function<uint32_t(uint32_t)> /*set*/, std::function<uint32_t(void)> /*get*/>;
  const gpio_control_t gpio_cmds[] = {
    std::make_tuple("sampa-pwr",  [&](uint32_t v){ return st.setSampaPower(v); }, [&](){ return st.getSampaPower(); }),
    std::make_tuple("mux-sel",    [&](uint32_t v){ return st.setMuxSel(v); },     [&](){ return st.getMuxSel(); }),
    std::make_tuple("hadd",       [&](uint32_t v){ return st.setHadd(v); },       [&](){ return st.getHadd(); }),
    std::make_tuple("clk-config", [&](uint32_t v){ return st.setClkConfig(v); },  [&](){ return st.getClkConfig(); })
  };

  for (auto gc: gpio_cmds) {
    const std::string& cmd = std::get<0>(gc);
    if (vm.count(cmd)) {
      const std::vector<cl_uint_t>& values = vm[cmd].as<std::vector<cl_uint_t>>();

      if (values.size() == 0) {
        // No value args, only get value
        std::cout << "[" << cmd << "] 0x" << std::hex << std::get<2>(gc)() << std::dec << std::endl;
      }
      else {
        // Value args given, apply in order
        for (auto v: values) {
          std::cout << "[" << cmd << "] 0x" << std::hex << std::get<1>(gc)(v()) << std::dec << std::endl;
        }
      }
    }
  }
}


//------------------------------------------------------------------------------
void control_sampa_registers(const bpo::variables_map& vm, SampaTester& st)
{
  Sampa sampa(st.getScaI2cSampa());
  st.updateSampaI2cAddress();

  size_t i(0);

  if (vm.count("write")) {
    i = 0;
    const std::vector<cl_uint_t>& v = vm["write"].as<std::vector<cl_uint_t>>();
    while (i < v.size()) {
      sampa.writeRegister(v[i]() /*reg_addr*/, v[i+1]() /*value*/);
      std::cout << "[Write GReg 0x" << std::hex << std::setw(2) << std::setfill('0') << v[i]() << "] 0x" << v[i+1]() << " written"
                << std::dec << std::setfill(' ') << "\n";
      i += 2;
    }
  }

  if (vm.count("read")) {
    i = 0;
    const std::vector<cl_uint_t>& v = vm["read"].as<std::vector<cl_uint_t>>();
    while (i < v.size()) {
      uint16_t read_value = sampa.readRegister(v[i]() /*reg_addr*/);
      std::cout << "[Read GReg 0x" << std::hex << std::setw(2) << std::setfill('0') << v[i]() << "] 0x" << static_cast<uint32_t>(read_value)
                << std::dec << std::setfill(' ') << "\n";
      i++;
    }
  }

  if (vm.count("ch-write")) {
    i = 0;
    const std::vector<cl_uint_t>& v = vm["ch-write"].as<std::vector<cl_uint_t>>();
    while (i < v.size()) {
      sampa.writeChannelRegisterMultiple(v[i]() /*channel_mask*/, v[i+1]() /*reg_addr*/, v[i+2]() /*value*/);
      std::cout << "[Write ChReg 0x" << std::hex << std::setw(2) << std::setfill('0') << v[i+1]() << ", ChMask 0x"
                << std::setw(8) << v[i]() << "] 0x" << v[i+2]()
                << std::dec << std::setfill(' ') << "\n";
      i += 3;
    }
  }

  if (vm.count("ch-read")) {
    i = 0;
    const std::vector<cl_uint_t>& v = vm["ch-read"].as<std::vector<cl_uint_t>>();
    while (i < v.size()) {
      std::unique_ptr<std::vector<uint16_t>> read_values(sampa.readChannelRegisterMultiple(v[i]() /*channel_mask*/, v[i+1]() /*reg_addr*/));

      std::cout << "[Read ChReg 0x" << std::hex << std::setw(2) << std::setfill('0') << v[i+1]() << ", ChMask 0x"
                << std::setw(8) << v[i]() << "]";
      for (auto v : *(read_values.get()))
        std::cout << " 0x" << static_cast<uint32_t>(v);
      std::cout << std::dec << std::setfill(' ') << "\n";

      i += 2;
    }
  }
}

//------------------------------------------------------------------------------
// Main
//------------------------------------------------------------------------------
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
  clo.addCmdLine("adc-all", "Sample and print all ext. ADC inputs")
     .addCmdLine("adc-port", bpo::value<cl_uint_t>()->implicit_value(0)->notifier([](cl_uint_t v) {
            if(v() >= SampaTester::adc_inputs_.size()) {
              throw bpo::validation_error(bpo::validation_error::invalid_option_value, "adc port"); } } ),
            "Initiate ext. ADC sampling cycle on port <n>")
     .addCmdLine("sampa-pwr", bpo::value<std::vector<cl_uint_t>>()->multitoken()->zero_tokens(),
                 "Set (if arg given: 1 -> on, 0 -> off) or get SAMPA power status")
     .addCmdLine("sampa-rst", "Pulse HRSTB to initiate SAMPA hard reset")
     .addCmdLine("hadd", bpo::value<std::vector<cl_uint_t>>()->multitoken()->zero_tokens(),
                 "Set (if arg given) or get hardward address (on GPIO)")
     .addCmdLine("clk-config", bpo::value<std::vector<cl_uint_t>>()->multitoken()->zero_tokens(),
                 "Set (if arg given) or get clock config (on GPIO)")
     .addCmdLine("mux-sel", bpo::value<std::vector<cl_uint_t>>()->multitoken()->zero_tokens(),
                 "Set (if arg given) or get mux select (on GPIO)")
     .addCmdLine("cg0", bpo::value<std::vector<cl_uint_t>>()->multitoken()->zero_tokens(),
                 "Set (if arg given) or get value for CG0 [0x0 .. 0xff]")
     .addCmdLine("cg1", bpo::value<std::vector<cl_uint_t>>()->multitoken()->zero_tokens(),
                 "Set (if arg given) or get value for CG1 [0x0 .. 0xff]")
     .addCmdLine("pol", bpo::value<std::vector<cl_uint_t>>()->multitoken()->zero_tokens(),
                 "Set (if arg given) or get value for POL [0x0 .. 0xff]")
     .addCmdLine("cts", bpo::value<std::vector<cl_uint_t>>()->multitoken()->zero_tokens(),
                 "Set (if arg given) or get value for CTS [0x0 .. 0xff]")
     //
     .addCmdLine("read,r", bpo::value<std::vector<cl_uint_t>>()->multitoken(),
                   "Read SAMPA global registers. Accepts multiple addresses, e.g. <reg-addr1> <reg-addr2>")
     .addCmdLine("write,w", bpo::value<std::vector<cl_uint_t>>()->multitoken()
                 ->notifier([](std::vector<cl_uint_t> v) { if (v.size() % 2) {
                     throw bpo::invalid_syntax(bpo::invalid_syntax::missing_parameter, "write"); } } ),
                   "Write <value> to SAMPA global register <addr>. Input is expected as (multiple) pairs of '<reg-addr> <value>',"\
                   " i.e. even number of (integer) parameters")
     .addCmdLine("ch-read", bpo::value<std::vector<cl_uint_t>>()->multitoken()
                 ->notifier([](std::vector<cl_uint_t> v) { if (v.size() % 2) {
                     throw bpo::invalid_syntax(bpo::invalid_syntax::missing_parameter, "ch-read"); } } ),
                   "Read SAMPA channel register <ch-reg-addr> of all channels given in one-hot <ch-mask>."\
                   " Input is expected as (multiple) pairs of '<ch-mask> <reg-ch-addr>'")
     .addCmdLine("ch-write", bpo::value<std::vector<cl_uint_t>>()->multitoken()
                 ->notifier([](std::vector<cl_uint_t> v) { if (v.size() % 3) {
                     throw bpo::invalid_syntax(bpo::invalid_syntax::missing_parameter, "ch-write"); } } ),
                   "Write <value> to SAMPA channel register <ch-reg-addr> of all channels given in one-hot <ch-mask>."\
                   " Input is expected as (multiple) tuples of '<ch-mask> <reg-ch-addr> <value>'")
     .addCmdLine("dump-sampa-registers", "Dump SAMPA registers")
     .addCmdLine("test-mem", "Run SAMPA memory test")
     .addCmdLine("test-ringosc", "Run rin oscillator test")
     .addCmdLine("test-sampa-i2c", "Run SAMPA I2C read test")
     .addCmdLine("test-sampa-i2c-iterations", bpo::value<uint32_t>()->default_value(1000), "Specify number of loops over register set for SAMPA I2C read test")
     .addCmdLine("test-adctrim-scan", "Vary ADCTRIM and scan voltages")
     .addCmdLine("json", "Output results as json (if applicable/supported)")
     ;

  // Process options
  auto vm = clo.Process(argc, argv);

  // Get bar
  std::unique_ptr<common::Bar> bar;
  try {
    bar.reset(common::BarFactory::makeBar(clo.getId(), clo.getBar()));
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
    exit(1);
  }

  // Do stuff
  try {
    // SCA loop
    for (uint32_t sca_idx = 0; sca_idx < 32; sca_idx++) {

      // Specified SCAs
      if ((clo.getFecMask() >> sca_idx) & 0x1) {

        if (clo.getVerboseFlag())
          std::cout << "Communicating with SCA" << sca_idx << std::endl;

        // HDLC core and Sampa tester instances for this SCA
        std::unique_ptr<common::HdlcCore>
          hdlc_core(common::HdlcFactory::makeHdlcCore(*(bar.get()), sca_idx, true));

        SampaTester st(*(hdlc_core.get()), vm.count("verbose") ? 1 : 0);

        // GPIO: SAMPA power, hardware address, clock config, mux sel -----
        if (vm.count("sampa-pwr") || vm.count("hadd") || vm.count("clk-config") || vm.count("mux-sel"))
          control_gpio(vm, st);

        // DAC ------------------------------------------------------------
        if (vm.count("cg1") || vm.count("cg0") || vm.count("cts") || vm.count("pol"))
          control_dac_output(vm, st);

        // ADC ------------------------------------------------------------
        if (vm.count("adc-port"))
          st.sampleExtAdc(1 << vm["adc-port"].as<cl_uint_t>()(), vm.count("json") != 0);

        if (vm.count("adc-all"))
          st.sampleExtAdc(0xfff, vm.count("json") != 0);

        // Tests ----------------------------------------------------------
        if (vm.count("test-sampa-i2c"))
          test_sampa_i2c(st, vm["test-sampa-i2c-iterations"].as<uint32_t>());

        if (vm.count("test-adctrim-scan"))
          test_adctrim_scan(st, vm.count("json") != 0);

        if (vm.count("test-ringosc"))
          test_ring_oscillator(st);

        if (vm.count("test-mem")) {
          std::unique_ptr<PccaInterface> pcca(new PccaInterface(reinterpret_cast<trorc::Bar&>(*(bar.get())), sca_idx*2));
          test_sampa_memory(st, *(pcca.get()));
        }

        // SAMPA hard reset ---------------------------------------------
        if (vm.count("sampa-rst")) {
          std::unique_ptr<PccaInterface> pcca(new PccaInterface(reinterpret_cast<trorc::Bar&>(*(bar.get())), sca_idx*2));
          pcca->sampaHardReset();
          std::cout << "SAMPA reset done\n";
        }

        // SAMPA register r/w ---------------------------------------------
        if (vm.count("write") || vm.count("read") || vm.count("ch-write") || vm.count("ch-read"))
          control_sampa_registers(vm, st);

        if (vm.count("dump-sampa-registers"))
          dump_sampa_registers(st, sca_idx);

      }   // Specified SCAs
    }   // SCA loop
  }
  catch (ScaException& e) {
    std::cerr << "### SCA error ###\n" << e.what() << std::endl;
    exit(101);
  }
  catch (std::exception& e) {
    std::cerr << "### General error ###\n" << e.what() << ", exiting" << std::endl;
    exit(100);
  }

  return 0;
}
