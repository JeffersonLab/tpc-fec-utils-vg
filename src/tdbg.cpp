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


void
test0(common::HdlcCore& hdlc)
{
  std::cout << "Running single HDLC frame test: GPIO direction READ" << std::endl;
  std::cout << "--------------------------------------------------------------------------------\n";
  HdlcEcFramePayload request = {0xab, 0x2, 0x4, 0x21, 0xaddeabab};    // GPIO Direction READ
  HdlcEcFramePayload reply;
  hdlc.executeCommand(request, reply);

  if (reply.error)
    std::cout << hdlc.getPayloadErrorMessage(reply.error) << std::endl;
}

void
test1(common::HdlcCore& hdlc)
{
  std::cout << hdlc << std::endl;
  std::cout << "-------------------------\n";
  Fec fec(hdlc, 3);

//  std::cout << "-------------------------\n";
//  auto sca_basic_ = fec.getScaBasic();
//  sca_basic_.transceive(sca_basic_.CMD_W_CRB, 0x9c);
//  std::cout << sca_basic_.transceive(sca_basic_.CMD_R_CRB) << std::endl;

  fec.init();
  std::cout << "-------------------------\n";
  std::cout << fec << std::endl;
}

void
test2(common::HdlcCore& hdlc)
{
  std::cout << "Running single HDLC frame test: ADC" << std::endl;
  std::cout << "--------------------------------------------------------------------------------\n";
  HdlcEcFramePayload request = {0xf, 0x14, 0x4, 0x2, 0x01010101};    // ADC Tx
  HdlcEcFramePayload reply;
  hdlc.executeCommand(request, reply);

  if (reply.error)
    std::cout << hdlc.getPayloadErrorMessage(reply.error) << std::endl;
}

void
test3(common::HdlcCore& hdlc)
{
  std::cout << "Running multi HDLC frame test" << std::endl;
  std::cout << "--------------------------------------------------------------------------------\n";
  HdlcEcFramePayload request[] = {
    {0x1, 0x2, 0x2, 0x1, 0x0},
    {0x2, 0x2, 0x2, 0x1, 0x0},
    {0x3, 0x2, 0x2, 0x1, 0x0},
    {0x4, 0x2, 0x2, 0x1, 0x0},
    {0x5, 0x2, 0x2, 0x1, 0x0},
    {0x6, 0x2, 0x2, 0x1, 0x0},
    {0x7, 0x2, 0x2, 0x1, 0x0},
    {0x8, 0x2, 0x2, 0x1, 0x0}
  };
  HdlcEcFramePayload reply;
  for (size_t i = 0; i < sizeof(request)/sizeof(HdlcEcFramePayload); i++) {
    hdlc.executeCommand(request[i], reply);
    if (reply.error)
      std::cout << hdlc.getPayloadErrorMessage(reply.error) << std::endl;
  }
}

void
test4(common::HdlcCore& hdlc)
{
  std::cout << "Read GPIO data in" << std::endl;
  HdlcEcFramePayload request;
  HdlcEcFramePayload reply;

  std::cout << "\nHDLC : Register CRB -> 0xffffffff" << std::endl;
  request = {0x0, 0x0, 0x1, 0x2, 0xffffffff};
  hdlc.executeCommand(request, reply);

  std::cout << "\nHDLC : Read GPIO register DIRECTION" << std::endl;
  request = {0x0, 0x2, 0x1, 0x21, 0x0};
  hdlc.executeCommand(request, reply);

  std::cout << "\nHDLC : Set GPIO register DIRECTION -> 0x0a0b0c0d" << std::endl;
  request = {0x0, 0x2, 0x4, 0x20, 0x0a0b0c0d};
  hdlc.executeCommand(request, reply);

  std::cout << "\nHDLC : Read GPIO register DIRECTION" << std::endl;
  request = {0x0, 0x2, 0x1, 0x21, 0x0};
  hdlc.executeCommand(request, reply);

  std::cout << "\nHDLC : Set GPIO register DIRECTION -> 0x00001234" << std::endl;
  request = {0x0, 0x2, 0x4, 0x20, 0x00001234};
  hdlc.executeCommand(request, reply);

  std::cout << "\nHDLC : Read GPIO register DIRECTION" << std::endl;
  request = {0x0, 0x2, 0x1, 0x21, 0x0};
  hdlc.executeCommand(request, reply);

  std::cout << "\nHDLC : Set GPIO register DATAOUT -> 0x00000000" << std::endl;
  request = {0x0, 0x2, 0x4, 0x10, 0x00000000};
  hdlc.executeCommand(request, reply);

  std::cout << "\nHDLC : Set GPIO register DIRECTION -> 0x0000001f" << std::endl;
  request = {0x0, 0x2, 0x4, 0x20, 0x1f000000};
  hdlc.executeCommand(request, reply);

  std::cout << "\nHDLC : Read GPIO register DATAIN" << std::endl;
  request = {0x0, 0x2, 0x1, 0x01, 0x0};
  hdlc.executeCommand(request, reply);
}

void test5(common::HdlcCore& hdlc)
{
  Fec fec(hdlc);
  std::string filename("/tmp/_delme_gbtx1-cfg.txt");
  std::cout << "Dumping GBTx1 configuration to " << filename << std::endl;

  std::ofstream os(filename, std::ios::out);
  if (!os.is_open())
    throw std::runtime_error("Unable to open file " + filename);

  fec.gbtxDumpConfiguration(1, os);
}

void fec_init(common::HdlcCore& hdlc)
{
  Fec fec(hdlc);
  fec.init();

  ScaGpio& gpio(fec.getScaGpio());
  std::cout << gpio << std::endl;

  ScaAdc& adc(fec.getScaAdc());
  std::cout << adc << std::endl;
}


void fec_test(common::HdlcCore& hdlc) {
  Fec fec(hdlc);
  std::cout << fec << std::endl;

  for (uint8_t i = 0; i < 5; i++) {
    try {
      ScaI2c& sampa(fec.getScaI2cSampa(i));
      std::cout << "Sampa" << static_cast<uint32_t>(i) << " read I2C "
                << std::hex << static_cast<uint32_t>(sampa.readByte(0x7)) << std::dec << "\n";
    }
    catch(ScaException& e) {
      std::cout << e.what() << std::endl;
    }
  }

  ScaI2c& i2c1(fec.getScaI2cGbtx(1));
  std::cout << "GBTx1 read I2C " << std::hex << static_cast<uint32_t>(i2c1.readByte(14, 24)) << std::endl;
}


//------------------------------------------------------------------------------

/** Test for SCA communication while switching SAMPAs on/off
 */
class ScaCommStability
{
  private:
    common::HdlcCore& hdlc_;
    Fec fec_;

    uint32_t iterations_;
    uint32_t switch_dly_;
    uint32_t iteration_dly_;

    uint32_t cnt_attempt_recover_ = {0};
    uint32_t cnt_fail_recover_ = {0};
    uint32_t cnt_sca_exception_ = {0};
    uint32_t cnt_hdlc_exception_ = {0};
    uint32_t cnt_wrong_gpio_readback_ = {0};
    uint32_t good_iterations_ = {0};

    std::chrono::high_resolution_clock::time_point tstart_;
    std::chrono::high_resolution_clock::time_point tend_;

    /** Get total number of errors encountered
     *  @return Total error count
     */
    inline uint32_t getErrorsTotal() const
      {
        return cnt_fail_recover_ + cnt_sca_exception_ +
               cnt_hdlc_exception_ + cnt_wrong_gpio_readback_;
      }

    /** Reset HDLC core, send reset command to SCA and
     *    initialize the FEC
     *
     *  Should result in reproducible startup condition
     *    with all SAMPAs off
     */
    void recover()
      {
        bool retry(true);

        while (retry) {
          try {
            cnt_attempt_recover_++;
            hdlc_.rst();
            hdlc_.sendSvlReset();
            fec_.init();
            retry = false;
          }
          catch (std::exception& e) {
            cnt_fail_recover_++;
            std::this_thread::sleep_for(std::chrono::milliseconds(iteration_dly_));
          }
        }
      }

    /** Run one test cycle
     *  @return 0 in case of success.<br/
     *          In case of an error, bit 30 and 31 are set for an SCA/HDLC exception,
     *          respectively, and/or any of the lowest 10 bit are set. In case of simply
     *          an incorrect GPIO readback, only any of the lowest 10 bit are set.
     */
    uint32_t cycle()
      {
        uint32_t r(0x3ff);

        try {
          r ^= (fec_.sampaPwrOn(0x1f, switch_dly_) << 5);
          std::this_thread::sleep_for(std::chrono::milliseconds(switch_dly_));
          r &=  fec_.sampaPwrOff(0x1f, switch_dly_);
          std::this_thread::sleep_for(std::chrono::milliseconds(switch_dly_));

          if (r != 0x0)
            cnt_wrong_gpio_readback_++;
        }
        catch (gbt::ScaException& e) {
          std::cout << e.what() << std::endl;
          r |= (1 << 30);
          cnt_sca_exception_++;
        }
        catch (HdlcException& e) {
          std::cout << e.what() << std::endl;
          r |= (1 << 31);
          cnt_hdlc_exception_++;
        }

        return r;
      }

    /** Determine if we require an recover of the HDLC core/SCA after one cycle<br/>
     *    (should only be necessary when we seen an HDLC exception)
     *
     *  @return True, if recovery is required
     */
    inline bool require_recover(uint32_t cycle_result) const { return (cycle_result & (0x1 << 31)) != 0; }

  public:
    ScaCommStability(common::HdlcCore& hdlc, uint32_t switch_dly = 100, uint32_t iteration_dly = 1000) :
        hdlc_(hdlc),
        fec_(hdlc),
        iterations_(100),
        switch_dly_(switch_dly),
        iteration_dly_(iteration_dly)
      {
        fec_.getScaGpio().setThrowOnScaReplyError(true);
      }

    auto setSwitchDly(uint32_t s) { switch_dly_ = s; }
    auto setIterationDly(uint32_t i) { iteration_dly_ = i; }
    auto setIterations(uint32_t i) { iterations_ = i; }

    void runTest()
      {
        tstart_ = std::chrono::high_resolution_clock::now();

        /* Test init & setup */
        recover();
        cnt_attempt_recover_ = 0;
        cnt_fail_recover_ = 0;

        /* Test loop */
        uint32_t it(0);
        while (it < iterations_) {
          it++;
          std::cout << "*** Iteration " << it << " (of " << iterations_ << ")\n";

          uint32_t r = cycle();
          if (r == 0x0)
            good_iterations_++;

          std::cout << "***** Result -> " << (r ? "FAIL" : "OK")
                    << " [0x" << std::hex << r << std::dec << "]\n";

          if (require_recover(r)) {
            recover();
            std::cout << "* Recovered HDLC core / SCA\n";
          }

          if (it != (iterations_-1))
            std::this_thread::sleep_for(std::chrono::milliseconds(iteration_dly_));
        }

        tend_ = std::chrono::high_resolution_clock::now();
      }


    void report()
      {
        auto time_start = std::chrono::system_clock::to_time_t(tstart_);

        std::cout << "----------------------------------------------\n"
                  << "SCA communication / SAMPA power switching test\n"
                  << "  Test started at        " << std::put_time(std::localtime(&time_start), "%Y-%m-%d %X") << "\n"
                  << "  Test duration          " << std::chrono::duration<double>(tend_ - tstart_).count() << "s\n"
                  << "  SAMPA switch delay     " << switch_dly_ << "ms\n"
                  << "  Iteration delay        " << iteration_dly_ << "ms\n"
                  << "  Iterations             " << iterations_ << "\n"
                  << "    Good iterations        " << good_iterations_ << " ("
                    << static_cast<double>(good_iterations_)/iterations_*100 << "%)\n"
                  << "    Recover attempts       " << cnt_attempt_recover_ << "\n"
                  << "  Errors                 " << getErrorsTotal() << "\n"
                  << "    Init/recover fails     " << cnt_fail_recover_ << "\n"
                  << "    Reply frame error      " << cnt_sca_exception_ << " [SCA exceptions]\n"
                  << "    No SCA reply/timeout   " << cnt_hdlc_exception_ << " [HDLC exceptions]\n"
                  << "    Wrong GPIO readback    " << cnt_wrong_gpio_readback_ << "\n"
                  ;
      }
};


/** Test stability of the communication with the SCA
 */
void testScaCommStability(common::HdlcCore& hdlc, uint32_t iterations = 100,
                          uint32_t switch_dly = 100, uint32_t iteration_dly = 1000)
{
  hdlc.rst();
  ScaCommStability scs(hdlc, switch_dly, iteration_dly);
  scs.setIterations(iterations);
  scs.runTest();
  scs.report();
}

//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  const std::string tool_help = \
    "Usage:  \n" + std::string(argv[0]) + " <cmds/options>\n"\
    "  Developer tool for low-level and feature testing - You know what you do\n"\
    "Commands / Options";

  common::CommandLineOptions clo(tool_help);

  clo.addCmdLine("test-sca-sampa-pwr",
                 "Run test to evaluate stability of the communcation with the SCA when switching SAMPAs on/off")
     .addCmdLine("test-sca-sampa-pwr-switch-dly", bpo::value<cl_uint_t>()->default_value(100),
                 "Specify the time to wait between switching individual SAMPAs (in ms) for SCA/SAMPA power test.")
     .addCmdLine("test-sca-sampa-pwr-it", bpo::value<cl_uint_t>()->default_value(100),
                 "Specify number of on/off iterations for SCA/SAMPA power test.")
     .addCmdLine("test-sca-sampa-pwr-it-dly", bpo::value<cl_uint_t>()->default_value(500),
                 "Specify the time to wait iterations (in ms) for SCA/SAMPA power test.")
     //
     .addCmdLine("dbg0", "Run DBG routine")
     .addCmdLine("dbg1", "Run DBG routine")
     .addCmdLine("test,t", bpo::value<std::vector<int>>()->multitoken(), "Run tests")
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

        // Hidden
//        if (vm.count("dbg0")) { hdlc.dbg0(); std::cout << "DBG0\n"; }
//        if (vm.count("dbg1")) { hdlc.dbg1(); std::cout << "DBG1\n"; }


        if (vm.count("test-sca-sampa-pwr"))
          testScaCommStability(static_cast<common::HdlcCore&>(*(hdlc_core.get())),
                               vm["test-sca-sampa-pwr-it"].as<cl_uint_t>()(),
                               vm["test-sca-sampa-pwr-switch-dly"].as<cl_uint_t>()(),
                               vm["test-sca-sampa-pwr-it-dly"].as<cl_uint_t>()());

        // Run tests
        if (vm.count("test")) {
          auto t(vm["test"].as<std::vector<int>>());
          for (size_t i = 0; i < t.size(); i++) {
             switch(t[i]) {
               case 0: test0(static_cast<common::HdlcCore&>(*(hdlc_core.get()))); break;
               case 1: test1(static_cast<common::HdlcCore&>(*(hdlc_core.get()))); break;
               case 2: test2(static_cast<common::HdlcCore&>(*(hdlc_core.get()))); break;
               case 3: test3(static_cast<common::HdlcCore&>(*(hdlc_core.get()))); break;
               case 4: test4(static_cast<common::HdlcCore&>(*(hdlc_core.get()))); break;
               case 5: test5(static_cast<common::HdlcCore&>(*(hdlc_core.get()))); break;
               case 1000: fec_init(static_cast<common::HdlcCore&>(*(hdlc_core.get()))); break;
               default: break;
             }
          }
        }
      }   // Specified SCAs
    }   // SCA loop
  }
  catch (std::exception& e) {
    std::cerr << e.what() << ", exiting" << std::endl;
    exit(100);
  }

  return 0;
}
