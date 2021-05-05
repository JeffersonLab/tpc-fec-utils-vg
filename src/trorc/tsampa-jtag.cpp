#include <iostream>
#include <string>
#include <iomanip>
#include <boost/program_options.hpp>

#include "simple_log.hpp"
#include "git_info.hpp"
#include "gbt_sca.hpp"
//#include "hdlc_core.hpp"
#include "factory.hpp"
#include "pcca_interface.hpp"
#include "sampa_tester.hpp"

using namespace gbt;
namespace bpo = boost::program_options;

// JTAG shift register names
std::vector<std::string> jtagPinNames = {
                                           "Output control bit",
                                           "smo",
                                           "sdo0",
                                           "sdo1",
                                           "sdo2",
                                           "sdo3",
                                           "sdo4",
                                           "serialOut[0]",
                                           "serialOut[1]",
                                           "serialOut[2]",
                                           "serialOut[3]",
                                           "serialOut[4]",
                                           "NBflowstop_out_SO5",
                                           "sda_o",
                                           "PORin",
                                           "sme",
                                           "NBflowstop_in",
                                           "dinN",
                                           "scl",
                                           "sda_i",
                                           "clk_config[0]",
                                           "clk_config[1]",
                                           "clk_config[2]",
                                           "clk_config[3]",
                                           "clk_config[4]",
                                           "clk_config[5]",
                                           "clk_config[6]",
                                           "hadd[0]",
                                           "hadd[1]",
                                           "hadd[2]",
                                           "hadd[3]",
                                           "hb_trg",
                                           "trg",
                                           "bx_sync_trig",
                                           "Hrstb",
                                           "clkSOin",
                                           "clkBXin",
                                           "clkADCin"
                                          };
// -- set to 1 to enable JTAG clock, 0 to disable
uint32_t control_jtag_clock(SampaTester& st, uint32_t value){ return !st.setMuxSel(!value);  }
// Other SCA GPIO related stuff
uint32_t control_clk_config(SampaTester& st, uint32_t value){ return st.setClkConfig(value); }
uint32_t control_hadd(SampaTester& st, uint32_t value)      { return st.setHadd(value);      }

int main(int argc, char** argv)
{
  uint32_t numberOfErrors(0);
  // Command line setup
  bpo::variables_map vm;
  bpo::options_description opt_general(
      "JTAG test for T-RORC JTAG test stand"
      "Usage:\n " + std::string(argv[0]) + " <cmds/options>\n"
      "Options"
    );
  bpo::options_description opt_hidden("");
  bpo::options_description opt_all;
  bpo::positional_options_description opt_pos;

  // Command line parsing
  try
  {
    opt_general.add_options()
      ("help,h", "Show this help")
      ("version,v", "Show version information")
      ("setup", bpo::value<uint32_t>()->default_value(0)
                ->notifier([](uint32_t s) {if(s>2){
                  throw bpo::validation_error(bpo::validation_error::invalid_option_value, "setup");}}),
                "Specify which SAMPA test setup to use for JTAG test [0..1]")
      ("log-level,l", bpo::value<uint32_t>()->default_value(Log::linfo)
        ->notifier([](uint32_t l) { if (l >= Log::lfatal) {
          throw bpo::validation_error(bpo::validation_error::invalid_option_value, "log-level"); } } ),
          std::string("Control log level. Valid values are " +
                      std::to_string(Log::ldbgl) + " [" + Log::levelName(Log::ldbgl) + "] to [" +
                      std::to_string(Log::lfatal) + " [" + Log::levelName(Log::lfatal) + "]").c_str())
      ("log-no-ts", bpo::bool_switch()->default_value(false), "Do not include timestamps in log output")
      ("all-tests,a", bpo::bool_switch()->default_value(false), "Run the full JTAG test chain")
      ("ir-length", bpo::bool_switch()->default_value(false), "Check length of TAP instruction register")
      ("chain-length", bpo::bool_switch()->default_value(false), "Check number of devices in JTAG chain")
      ("check-outputs", bpo::bool_switch()->default_value(false), "Check output pins")
      ("check-inputs", bpo::bool_switch()->default_value(false), "Check input pins")
    ;
    opt_hidden.add_options()
      ("dbg-jtag-outputs", bpo::bool_switch()->default_value(false), "Debug output check routine")
      ("dbg-jtag-inputs", bpo::bool_switch()->default_value(false), "Debug input check routine")
    ;
    opt_all.add(opt_general).add(opt_hidden);
    bpo::store(bpo::command_line_parser(argc, argv).options(opt_all)
                   .positional(opt_pos).run(), vm);

    if(vm.count("help")) {
      std::cout << opt_general << std::endl;
      exit(0);
    }

    if(vm.count("version")) {
      std::cout << GitInfo();
      exit(0);
    }

    Log::setLevel(static_cast<Log::level_t>(vm["log-level"].as<uint32_t>()));
    Log::setIncludeTimestamp(!vm["log-no-ts"].as<bool>());

    bpo::notify(vm);
  }
  catch(bpo::error& e)
  {
    Log::error(std::string(e.what()) + ".", "Exiting");
    std::cout << opt_general << std::endl;
    exit(-100);
  }
  catch(std::exception& e)
  {
    Log::error(std::string(e.what()) + ".", "Exiting");
    exit(-101);
  }

  std::unique_ptr<trorc::Device> trorc;
  std::unique_ptr<trorc::Bar> bar;
  std::unique_ptr<PccaInterface> pcca_iface;

  try {
    trorc.reset(new trorc::Device(0 /*device_number*/));
    bar.reset(new trorc::Bar(*trorc, 1 /*bar_number*/));
    pcca_iface.reset(new PccaInterface(*bar, vm["setup"].as<uint32_t>() * 2)); // multiply by 2 -> VLDB connected to even link numbers
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
    exit(1);
  }
  // TODO: Fix this, I get compile errors if I do like the other inits
  std::unique_ptr<common::HdlcCore>
    hdlc_core(common::HdlcFactory::makeHdlcCore(*(bar.get()), vm["setup"].as<uint32_t>(), true));
  SampaTester st(*(hdlc_core.get()),0);

  // Store the initial settings
  const std::tuple<std::string, uint32_t, std::function<void(uint32_t)>> initial_settings[] = {
    std::make_tuple("sampa-pwr",  st.getSampaPower(), [&](uint32_t v){ st.setSampaPower(v); }),
    std::make_tuple("mux-sel",    st.getMuxSel(), [&](uint32_t v){ st.setMuxSel(v); }        ),
    std::make_tuple("hadd",       st.getHadd(), [&](uint32_t v){ st.setHadd(v); }            ),
    std::make_tuple("clk-config", st.getClkConfig(), [&](uint32_t v){ st.setClkConfig(v); }  )
  };

  if(st.setSampaPower(1) == 0){
    Log::error("JTAG", "Could not turn on SAMPA power - is SCA GPIO channel enabled?");
    exit(-100);
  }

  // Init and reset
  pcca_iface->jtagInitialize();
  pcca_iface->jtagReset();

  // 1. Check number of devices in chain
  // Set TAP in BYPASS mode, send zeroes, then a one
  if(vm["chain-length"].as<bool>() | vm["all-tests"].as<bool>()) {
    uint32_t chain_len = pcca_iface->jtagDetermineChainLength();
    if (chain_len != 1){
      Log::error("JTAG", "Chain length NOT correct - Expected: 1 device, Got:", chain_len, "device(s)");
      numberOfErrors++;
    } else {
      Log::sys("JTAG", "Chain length OK,", chain_len, "device found");
    }
  }

  // 2. Check length of IR register
  if(vm["ir-length"].as<bool>() | vm["all-tests"].as<bool>()) {
    uint32_t ir_len = pcca_iface->jtagDetermineIrLength();
    if (ir_len != 3){
      Log::error("JTAG", "Instruction register length NOT correct - Expected: ir_len = 3, got: ir_len =", ir_len);
      numberOfErrors++;
    } else {
      Log::sys("JTAG", "Instruction register length OK, got: ir_len =", ir_len);
    }
  }

  // 3. Check outputs
  if(vm["check-outputs"].as<bool>() | vm["all-tests"].as<bool>()) {
    uint64_t jtag_readback;
    // - Disable JTAG clock
    control_jtag_clock(st, 0);
    // - Shift in a running 0 then a running 1
    //   Bit 0 has to be set to 1 to enable outputs,
    uint64_t input_vector;
    uint32_t running_zero = 0;
    uint32_t running_one  = 0;
    for(int runner = 0; runner < 2; runner++){ // Running zero first, then a running one
      for(int i = 0; i<13; i++){
        // Preload values to shift register
        pcca_iface->jtagShiftIr(PccaInterface::SAMPLE_PRELOAD);
        input_vector = (runner ? ((1<<(i+1))|1) : ~(1<<(i+1)));
        pcca_iface->jtagShiftDr(input_vector, 38);
        // Enable extest and read back the outputs from uplink capture register
        pcca_iface->jtagShiftIr(PccaInterface::EXTEST);
        jtag_readback = pcca_iface->jtagGetPrimaryOutputs();
        if( ((jtag_readback>>(i))&1) && !runner ){
          running_zero |= 1<<i;
        }
        if( !((jtag_readback>>(i))&1) && runner ){
          running_one |= 1<<i;
        }

        // Debug stuff
        if(vm["dbg-jtag-outputs"].as<bool>()){
          std::cout << "DEBUG OUTPUT PINS - RUNNING " << runner << std::endl;
          std::cout << "Input vector : ";
          for(int i=12; i>=0; i--)
            std::cout << ((input_vector>>(i+1))&1);
          std::cout << std::endl;
          std::cout << "Output vector: ";
          for(int i=12; i>=0; i--)
            std::cout << ((jtag_readback>>(i))&1);
          std::cout << std::endl;
        }
      }
    }
     // Check the results
    for(int i = 0; i < 13; i++){
      if ( (running_zero>>i)&1 ){
          Log::error("JTAG", "Output pin",jtagPinNames.at(i+1), "(bit position", i+1, ") stuck HIGH");
          numberOfErrors++;
      } else if ( (running_one>>i)&1 ) {
          Log::error("JTAG", "Output pin",jtagPinNames.at(i+1), "(bit position", i+1, ") stuck LOW");
          numberOfErrors++;
      } else {
          Log::sys("JTAG", "Output pin",jtagPinNames.at(i+1), "(bit position", i+1, ") OK");
      }
    }
    // Turn clock back on
    control_jtag_clock(st, 1);
  }
  // 4. Check inputs
  if(vm["check-inputs"].as<bool>() | vm["all-tests"].as<bool>()) {

    // Turn off JTAG clock
    control_jtag_clock(st, 0);

    // First check reset
    uint64_t jtag_readback;
    pcca_iface->jtagShiftIr(PccaInterface::SAMPLE_PRELOAD);
    jtag_readback = (pcca_iface->jtagShiftDr(static_cast<uint64_t>(0), 38));
    if( !((jtag_readback>>14)&1) ){
      Log::error("JTAG", "SAMPA stuck in RESET");
      numberOfErrors++;
      // Restore settings
      for (auto s: initial_settings){ std::get<2>(s)(std::get<1>(s)); }
      exit(-1);
    }
    uint32_t input_vector;
    uint32_t running_zero = 0;
    uint32_t running_one  = 0;
    // Check the input pins
    for(int runner = 0; runner < 2; runner++){ // Running zero first, then a running one
      for(int i = 0; i<23; i++){
        input_vector = (runner ? (1<<i) : ~(1<<i));
        pcca_iface->dftControlPinTx(PccaInterface::BIT_TX_SME,         input_vector&1    ); // Bit pos. 15
        pcca_iface->dftControlPinTx(PccaInterface::BIT_TX_NBFLOWSTOP, (input_vector>>1)&1); // Bit pos. 16
        pcca_iface->dftControlPinTx(PccaInterface::BIT_TX_DINN,       (input_vector>>2)&1); // Bit pos. 17
        uint8_t shr1 = static_cast<uint8_t>( (((input_vector>>3)&1)<<PccaInterface::BIT_SHR_SCL) );
        pcca_iface->dftShrLoad(static_cast<uint8_t>(0),
                               shr1,
                               static_cast<uint8_t>(0),                           // mask0
                               static_cast<uint8_t>(1<<PccaInterface::BIT_SHR_SCL)// mask1
                               );                                                           // Bit pos. 18
        #if 0 // This no longer works, so either be happy about output pin tests or drive it through JTAG
        pcca_iface->dftShrLoad(((((input_vector>>4)&1)<<PccaInterface::BIT_SHR_SDA)),
                               static_cast<uint8_t>(0),
                               static_cast<uint8_t>(1<<PccaInterface::BIT_SHR_SDA),
                               static_cast<uint8_t>(0)
                               );                                                           // Bit pos. 19
        #endif
        control_clk_config(st, (input_vector>>5) & 0x7f);                                   // Bit pos. 20-26
        control_hadd(st, (input_vector>>12) & 0xf);                                         // Bit pos. 27-30
        pcca_iface->dftControlPinTx(PccaInterface::BIT_TX_HBTRG,     (input_vector>>16)&1); // Bit pos. 31
        pcca_iface->dftControlPinTx(PccaInterface::BIT_TX_TRG,       (input_vector>>17)&1); // Bit pos. 32
        pcca_iface->dftControlPinTx(PccaInterface::BIT_TX_BXSYNCTRG, (input_vector>>18)&1); // Bit pos. 33
        pcca_iface->dftControlPinTx(PccaInterface::BIT_TX_HRSTB,     (input_vector>>19)&1); // Bit pos. 34
        pcca_iface->dftControlPinTx(PccaInterface::BIT_TX_CLKS0,     (input_vector>>20)&1); // Bit pos. 35
        pcca_iface->dftControlPinTx(PccaInterface::BIT_TX_CLKBX,     (input_vector>>21)&1); // Bit pos. 36
        pcca_iface->dftControlPinTx(PccaInterface::BIT_TX_CLKADC,    (input_vector>>22)&1); // Bit pos. 37
        jtag_readback = (pcca_iface->jtagShiftDr(static_cast<uint64_t>(0), 38))>>15; // Only look at inputs after POR (15->37)
        if( ((jtag_readback>>i)&1) && !runner ){
          running_zero |= 1<<i;
        }
        if( !((jtag_readback>>i)&1) && runner ){
          running_one |= 1<<i;
        }
        // Debug stuff
        if(vm["dbg-jtag-inputs"].as<bool>()){
          std::cout << "DEBUG INPUT PINS - RUNNING " << runner << std::endl;
          std::cout << "Input vector : ";
          for(int i=22; i>=0; i--)
            std::cout << ((input_vector>>(i))&1);
          std::cout << std::endl;
          //std::cout << static_cast<uint32_t>(((input_vector>>3)&1)<<PccaInterface::BIT_SHR_SCL) <<std::endl;
          std::cout << "Output vector: ";
          for(int i=22; i>=0; i--)
            std::cout << ((jtag_readback>>(i))&1);
          std::cout << std::endl;
        }
      }
    } // runner
    // Check the results
    for(int i = 0; i < 22; i++){
      if ( (running_zero>>i)&1 ){
          if(i!=4){ // Skip SDA
            Log::error("JTAG", "Input pin",jtagPinNames.at(15+i), "(bit position", 15+i, ") stuck HIGH");
            numberOfErrors++;
          }
      } else if ( (running_one>>i)&1 ) {
          if(i!=4){
            Log::error("JTAG", "Input pin",jtagPinNames.at(15+i), "(bit position", 15+i, ") stuck LOW");
            numberOfErrors++;
          }
      } else {
          if(i!=4)
            Log::sys("JTAG", "Input pin",jtagPinNames.at(15+i), "(bit position", 15+i, ") OK");
      }
    }
    // Turn JTAG clock back on
    control_jtag_clock(st, 1);
  }
  // Restore settings
  for (auto s: initial_settings){ std::get<2>(s)(std::get<1>(s)); }

  pcca_iface->jtagDeInitialize();
  return numberOfErrors;
}
