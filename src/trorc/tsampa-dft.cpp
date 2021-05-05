#include <iostream>
#include <fstream>
#include <string>
#include <regex>
#include <chrono>
#include <thread>
#include <boost/program_options.hpp>

#include "simple_log.hpp"
#include "git_info.hpp"
#include "pcca_interface.hpp"

// Affects order of read/compare/drive in testCycle
//   If defined, values are read first, then driven (as the brazilians do it)
//   If not defined, the order is as in the original verilog code
#define BRAZILSTYLE

#ifdef DFT_DEBUG
  #define DFTLOG(s) s
#else
  #define DFTLOG(...)
#endif

namespace bpo = boost::program_options;

//------------------------------------------------------------------------------
/** Pre-allocated buffer with string-style append/assign
 */
class DftBuffer
{
  private:
    static constexpr size_t size_ {122976+1};
    size_t pos_;
    std::array<char, size_> buf_;

  public:
    DftBuffer() : pos_(0) { buf_[0] = '\0'; }

    size_t append(const std::string& s)
      {
        std::strcpy(buf_.data() + pos_, s.c_str());
        pos_ += s.size();
        buf_.data()[pos_] = '\0';
        return s.size();
      }

    size_t append(size_t n, const char c)
      {
        for (size_t i = 0; i < n; i++)
          buf_.data()[pos_++] = c;
        buf_.data()[pos_] = '\0';
        return n;
      }

    size_t assign(const std::string& s)
      {
        pos_ = 0;
        return append(s);
      }

    size_t assign(size_t n, const char c)
      {
        pos_ = 0;
        return append(n, c);
      }

    void fill(const char c)
      {
        for (size_t s = 0; s < size_; s++)
          buf_[s] = c;
        buf_[size_-1] = '\0';
      }

    const std::string substr(size_t p) const { return std::string(buf_.data() + p); }
    const std::string get() const { return std::string(buf_.data()); }
    size_t clear() { buf_[0] = '\0'; pos_ = 0; return pos_; }
    size_t size() const { return pos_; }
    char& at(size_t n) { return buf_.at(n); }
};


/** Wrapper class for various DFT-related buffers
 */
class DftMemory
{
  private:

    size_t vec_sc_start_pos_; /** Remember scan chain start position */

  public:
    DftMemory() : vec_sc_start_pos_(0)
      {
        sci_.fill(cZero);
        scc_.fill(cIgnore);
        rpo_.assign(100, cIgnore);
      }

    // Use strings as we want the buffers to grow/adjust automatically
    //   We don't care for efficiency here
    std::string pi_;  /** Buffer for primary input stimuli */
    std::string ci_;  /** Buffer for clocked input stimuli  */
    std::string po_;  /** Buffer for (expected) outputs */
    std::string rpo_; /** Buffer for read-back outputs */

    static const char cZero{'0'};
    static const char cOne{'1'};
    static const char cIgnore{'X'};

    /** Vector to remember the start position and length of the different scan chains in the buffers (sci & sci)
     *   Initialized with {0}, other positions get added while parsing the file
     *   pair.first -> start pos
     *   pair.second -> scan chain length
     */
    std::vector<std::pair<size_t, size_t>> sc_info_;

    /** Make entry about start of scan chain (wrt to sci_/scc_ buffer start) and its length
     *    in scan chain info vector
     */
    void scInfoMakeEntry()
      {
        sc_info_.push_back(std::make_pair(vec_sc_start_pos_, sci_.size() - vec_sc_start_pos_));
        vec_sc_start_pos_ = sci_.size();
      }

    /** Clear info about detected scan chain and lengths
     */
    void scInfoClear()
      {
        sc_info_.clear();
        vec_sc_start_pos_ = 0;
      }

    DftBuffer sci_; /** Buffer for scan chain input data */
    DftBuffer scc_; /** Buffer for scan chain compare data */

    // std::string sci_; /** Buffer for scan chain input data */
    // std::string scc_; /** Buffer for scan chain compare data */

    uint32_t getNumberPinInputs()          const { return pi_.size(); }
    uint32_t getNumberClockedInputs()      const { return ci_.size(); }
    uint32_t getNumberPinOutputs()         const { return po_.size(); }
    uint32_t getNumberReadbackPinOutputs() const { return rpo_.size(); }
    uint32_t getSizeScanChainInputData()   const { return sci_.size(); }
    uint32_t getSizeScanChainCompareData() const { return scc_.size(); }

    /** Print DftMemory (status) information
     */
    friend std::ostream& operator<<(std::ostream& os, const DftMemory& m)
      {
        os << "DftMemory\n"
           << "    PI size  = " << m.getNumberPinInputs() << "\n"
           << "    CI size  = " << m.getNumberClockedInputs() << "\n"
           << "    PO size  = " << m.getNumberPinOutputs() << "\n"
           << "    RPO size = " << m.getNumberReadbackPinOutputs() << "\n"
           << "    SCI size = " << m.getSizeScanChainInputData() << "\n"
           << "    SCC size = " << m.getSizeScanChainCompareData() << "\n"
           << "";

        os << "  Detected scan chains (id -> start / length)\n";
        uint32_t i(0);
        for (auto v: m.sc_info_) {
          os << "    SC" << i << " -> " << v.first << " / " << v.second << "\n";
          i++;
        }
        return os;
      }
};


//------------------------------------------------------------------------------
class DftInterface
{
  protected:
    PccaInterface& hw_;
    DftMemory mem_;

    static const uint8_t IDX_PI_SCLK {16-1};

    using pin_map_def_t = std::tuple<const uint32_t /*bit pos in buffer string*/,
                                     const uint32_t /*bit pos in hw word*/>;
    static const std::vector<pin_map_def_t> pinMapUplink;
    static const std::vector<pin_map_def_t> pinMapUplinkBypass;
    static const std::vector<uint8_t>       pinsNoAccess;
    static const std::vector<pin_map_def_t> pinMapDownlinkDirect;
    static const std::vector<pin_map_def_t> pinMapDownlinkDirectClocks;
    static const std::vector<pin_map_def_t> pinMapDownlinkShr0;
    static const std::vector<pin_map_def_t> pinMapDownlinkShr1;

    /** DFT statistics */
    uint32_t cmp_mismatch_;
    uint32_t cmp_good_;
    uint32_t cmp_ignored_;

    /** Update sen_* inputs selectively
     */
    inline void setPinInputsSen(uint8_t v) { hw_.dftSetPrimaryInputsSen(v); }

    /** Update sdi_* inputs selectively
     */
    inline void setPinInputsSdi(uint8_t v) { hw_.dftSetPrimaryInputsSdi(v); }

    /** Get read values of sdo_* from frame capture register
     */
    inline uint8_t getPinOutputsSdo() { return hw_.dftGetPrimaryOutputsSdo(); }

    /** Fetch pin values for the GBT frame capture register and
     *    update the corresponding (mapped) 'bits' (chars) in the received primary
     *    outputs buffer
     */
    void updatePinOutputs()
      {
        uint32_t v = hw_.dftGetPrimaryOutputs();

        // Read back SAMPA outputs
        //   tuple elements: 0 -> idx received buffer, 1 -> idx hardware, 2 -> signal polarity
        for (auto m: pinMapUplink)
          mem_.rpo_[std::get<0>(m)] = (((v >> std::get<1>(m)) & 0x1) ? DftMemory::cOne : DftMemory::cZero);

        DFTLOG(Log::dbg(__func__, "Primary outputs updated:", mem_.rpo_, "(" + std::to_string(v) + ")");)

        // Bypass outputs if certain drivers are disabled, updated expected result
        for (auto m: pinMapUplinkBypass) {
          if (mem_.po_[std::get<0>(m)] != DftMemory::cZero)
            mem_.po_[std::get<1>(m)] = DftMemory::cIgnore;
        }

        // Bypass pins without access
        for (auto m: pinsNoAccess)
          if (m < mem_.po_.size())
            mem_.po_[m] = DftMemory::cIgnore;

        DFTLOG(Log::dbg(__func__, "Expected outputs bypassed:", mem_.po_);)
      }

  public:
    DftMemory& getMem() { return mem_; }

    /** Constructor
     */
    DftInterface(PccaInterface& hw) : hw_(hw), cmp_mismatch_(0), cmp_good_(0), cmp_ignored_(0)
      {
        hw_.dftInitialize();
        hw_.dftSetPrimaryInput(1 << PccaInterface::BIT_TX_CLKS0 | 1 << PccaInterface::BIT_TX_HRSTB);

        usleep(10); // Some time to wake up
        hw_.dftShrLoad(0 << PccaInterface::BIT_SHR_SDA, 1 << PccaInterface::BIT_SHR_TME);
        usleep(5);
      }

    ~DftInterface()
      {
        hw_.dftDeInitialize();
      }

    /** Print statistics
     */
    void printStatistics(std::ostream& os)
      {
        os << "DFT Test\n"
           << "  Number of mismatches: " << cmp_mismatch_ << "\n"
           << "  Number of ignores:    " << cmp_ignored_ << "\n"
           << "  Number of matches:    " << cmp_good_ << "\n"
           << "";
      }

    /** Return total number of mismatches encountered
     */
    uint32_t getErrors() { return cmp_mismatch_; }

    /** Test cycle modelled after verilog description
     */
    void testCycle()
      {
        uint32_t mismatch(0);
        uint32_t good(0);
        uint32_t ignored(0);

#ifdef BRAZILSTYLE
        // Get (full) SAMPA output pin pattern
        updatePinOutputs();

        // Compare
        for (size_t i = 0; i < mem_.getNumberPinOutputs(); i++) {
          if (mem_.po_[i] != DftMemory::cIgnore) {
            if (mem_.po_[i] != mem_.rpo_[i])
              mismatch++;
            else
              good++;
          }
          else
            ignored++;

          // Do we need to clear rpo_ here? ...verilog code has it
          //   mem_.rpo_.at(i) = 'X';
        }

        if (mismatch) {
          Log::error(__func__, "Test cycle fail (BRAZIL) - Compares: good =", good, "mismatch =", mismatch, "[ ignored =", ignored, "]");
          Log::error(__func__, "  exp  PO =", mem_.po_);
          Log::error(__func__, "  recv PO =", mem_.rpo_);
        }
        else {
          Log::sys(__func__, "Test cycle success (BRAZIL), recv  PO =", mem_.po_);
          Log::sys(__func__, "  exp  PO =", mem_.po_);
          Log::sys(__func__, "  recv PO =", mem_.rpo_);
        }
#endif


        uint32_t p_direct(0x0), p_clk(0x0);
        uint8_t p_shr0(0x0), p_mask0(0x0);
        uint8_t p_shr1(0x0), p_mask1(0x0);

        // Calculate p_direct pin mask from stimulus string (for the direct pins)
        for (auto p: pinMapDownlinkDirect)
          p_direct |= (((mem_.pi_.at(std::get<0>(p)) == DftMemory::cOne) ? 0x1 : 0x0) << std::get<1>(p));

        // Calculate SHR contents from stimulus string (for the SHR pins)
        for (auto p: pinMapDownlinkShr0) {
          p_shr0 |= (((mem_.pi_.at(std::get<0>(p)) == DftMemory::cOne) ? 0x1 : 0x0) << std::get<1>(p));
          p_mask0 |= (1 << std::get<1>(p));
        }
        for (auto p: pinMapDownlinkShr1) {
          p_shr1 |= (((mem_.pi_.at(std::get<0>(p)) == DftMemory::cOne) ? 0x1 : 0x0) << std::get<1>(p));
          p_mask1 |= (1 << std::get<1>(p));
        }

        hw_.dftShrLoad(p_shr0, p_shr1, p_mask0, p_mask1);            // Load SHR and propagate to output
        //hw_.dftSetPrimaryInputDirectWoSclk(p_direct);    // Apply outputs
        hw_.dftSetPrimaryInputDirectNormal(p_direct);    // Apply outputs

        for (auto p: pinMapDownlinkDirectClocks) {
//          if (std::get<1>(p) == PccaInterface::BIT_TX_CLKS0)
//            p_clk |= (((mem_.pi_.at(std::get<0>(p)) == DftMemory::cZero) ? 0x0 : 0x1) << std::get<1>(p));
//          else
            p_clk |= (((mem_.pi_.at(std::get<0>(p)) == DftMemory::cOne) ? 0x1 : 0x0) << std::get<1>(p));
        }
        hw_.dftSetPrimaryInputDirectClocks(p_clk);    // Apply outputs

        hw_.dftControlSclk((mem_.pi_[IDX_PI_SCLK] == DftMemory::cOne) ? 0x1 : 0x0, false);   // Drive SCLK with value in pi_
        hw_.dftControlSclk((mem_.ci_[IDX_PI_SCLK] == DftMemory::cOne) ? 0x1 : 0x0, false);   // Drive SCLK with value in ci_

#ifndef BRAZILSTYLE
        // Get (full) SAMPA output pin pattern
        updatePinOutputs();

        // Compare
        for (size_t i = 0; i < mem_.getNumberPinOutputs(); i++) {
          if (mem_.po_[i] != DftMemory::cIgnore) {
            if (mem_.po_[i] != mem_.rpo_[i])
              mismatch++;
            else
              good++;
          }
          else
            ignored++;

          // Do we need to clear rpo_ here? ...verilog code has it
          //   mem_.rpo_.at(i) = 'X';
        }

        if (mismatch) {
          Log::error(__func__, "Compares: good =", good, "mismatch =", mismatch, "[ ignored =", ignored, "]");
          Log::error(__func__, "  exp  PO =", mem_.po_);
          Log::error(__func__, "  recv PO =", mem_.rpo_);
        }
        else {
          Log::sys(__func__, "Test cycle success, recv  PO =", mem_.po_);
          Log::sys(__func__, "  exp  PO =", mem_.po_);
          Log::sys(__func__, "  recv PO =", mem_.rpo_);
        }
#endif

        // Global accounting
        cmp_mismatch_ += mismatch;
        cmp_good_ += good;
        cmp_ignored_ += ignored;

        // Clear expected PO buffer
        mem_.po_.assign(mem_.po_.size(), 'X');
     }


    /** Scan routine, modelled after verilog code
     */
    void scanPreconditioningSequence_TM_1_SEQ_1_SOP_1()
      {
        setPinInputsSen(0x1f);
        for (uint8_t i = 21; i <= 26; i++)
    	    mem_.pi_.at(i) = DftMemory::cOne;
      	testCycle();
      }

    /** Scan routine, modelled after verilog code
     */
    void scanSequence_TM_1_SEQ_2_SOP_1(uint32_t max_cycle)
      {
        uint32_t mismatch(0);
        uint32_t good(0);
        uint32_t ignored(0);

        size_t scan_chains = mem_.sc_info_.size();

        auto timestampSdi = std::chrono::high_resolution_clock::now();

        for (uint32_t cycle = 0; cycle < max_cycle; cycle++) {
          uint8_t v(0);

          // Extract new value for sdi from scan chain data
          //   Careful here with the way the individual scan chain buffers are aligned
          //   We need to shift the longest scan chains first (with other SDI bits set to zero),
          //   and in the following cycles shift all chains simultaneously
          for (size_t i = 0; i < scan_chains; i++) {
            const uint32_t& sc_start_pos = mem_.sc_info_[i].first;
            const uint32_t& sc_length = mem_.sc_info_[i].second;

            uint32_t diff = mem_.sc_info_[0].second - sc_length;    // Length of longest chain minus length of current chain

            if (cycle >= diff) {
              v |= (((mem_.sci_.at(sc_start_pos + cycle - diff) == DftMemory::cOne) ? 0x1 : 0x0) << i);
            }
          }

          // Apply new values for sdi and get new readings for sdo
          setPinInputsSdi(v);
          timestampSdi = std::chrono::high_resolution_clock::now();

          // Get pin outputs
          uint8_t sdo = getPinOutputsSdo();

          // Compare
          //   Comparing works different from shifting: Compare all chains right from the start, and in the last cycle
          //   only compare the longest chain
          for (size_t i = 0; i < scan_chains; i++) {
            const uint32_t& sc_start_pos = mem_.sc_info_[i].first;
            const uint32_t& sc_length = mem_.sc_info_[i].second;

            if (cycle < sc_length) {
              if (mem_.scc_.at(sc_start_pos + cycle) != 'X') {    // Compare gets ignored if 'X' in compare vector
                if (((sdo >> i) & 0x1) != ((mem_.scc_.at(sc_start_pos + cycle) == DftMemory::cOne) ? 0x1 : 0x0))
                  mismatch++;
                else
                  good++;
              }
              else {
                ignored++;
              }
            }
          }

          // Make sure we toggle SCLK only after SHR output has been updated
          //   Bus access on T-RORC is in the order of 100ns,
          //   DFT controller needs 16 cycles @ 40MHz to shift values into SHR in normal mode,
          //   half of that in fast mode
          std::this_thread::sleep_until(timestampSdi + std::chrono::nanoseconds((16/2-4)*25));

          // Pulse SCLK
          hw_.dftControlSclk(0x1, true);
        }

        if (mismatch)
          Log::error(__func__, "Compares: good =", good, "mismatch =", mismatch, "[ ignored =", ignored, "]");
        else
          Log::sys(__func__, "Scan chain success");

        // Global accounting
        cmp_mismatch_ += mismatch;
        cmp_good_ += good;
        cmp_ignored_ += ignored;
      }


};


/** Describe how pin output buffer (rpo_/po_) indices map to bits in the word received
 *    when getting the Dft uplink data
 */
const std::vector<DftInterface::pin_map_def_t>
DftInterface::pinMapUplink = {
  std::make_tuple( 3-1, PccaInterface::BIT_RXRED_NBFLOWSTOP),
  std::make_tuple( 5-1, PccaInterface::BIT_RXRED_TDO),
  std::make_tuple(71-1, PccaInterface::BIT_RXRED_SDA),
  std::make_tuple(72-1, PccaInterface::BIT_RXRED_SDO_0),
  std::make_tuple(73-1, PccaInterface::BIT_RXRED_SDO_1),
  std::make_tuple(74-1, PccaInterface::BIT_RXRED_SDO_2),
  std::make_tuple(75-1, PccaInterface::BIT_RXRED_SDO_3),
  std::make_tuple(76-1, PccaInterface::BIT_RXRED_SDO_4),
  std::make_tuple(86-1, PccaInterface::BIT_RXRED_SERIALOUT_0),
  std::make_tuple(87-1, PccaInterface::BIT_RXRED_SERIALOUT_1),
  std::make_tuple(88-1, PccaInterface::BIT_RXRED_SERIALOUT_2),
  std::make_tuple(89-1, PccaInterface::BIT_RXRED_SERIALOUT_3),
  std::make_tuple(90-1, PccaInterface::BIT_RXRED_SERIALOUT_4),
  std::make_tuple(96-1, PccaInterface::BIT_RXRED_SMO)
};

const std::vector<DftInterface::pin_map_def_t>
DftInterface::pinMapUplinkBypass = {
   //           pinChk, pinBypass, name
  std::make_tuple( 4-1,  3-1), // NBFLOWSTOP_off
//  std::make_tuple(71-1, 71-1, "SDA"),
  std::make_tuple(91-1, 86-1), // SerialOut0_off
  std::make_tuple(92-1, 87-1), // SerialOut1_off
  std::make_tuple(93-1, 88-1), // SerialOut2_off
  std::make_tuple(94-1, 89-1), // SerialOut3_off
  std::make_tuple(95-1, 90-1), // SerialOut4_off
};

const std::vector<uint8_t>
DftInterface::pinsNoAccess = {
	1-1,
	2-1,
	4-1,
	6-1,
	7-1,
	8-1,
	9-1,
	10-1,
	11-1,
	12-1,
	13-1,
	14-1,
	15-1,
	16-1,
	17-1,
	18-1,
	19-1,
	20-1,
	21-1,
	22-1,
	23-1,
	24-1,
	25-1,
	26-1,
	27-1,
	28-1,
	29-1,
	30-1,
	31-1,
	32-1,
	33-1,
	34-1,
	35-1,
	36-1,
	37-1,
	38-1,
	39-1,
	40-1,
	41-1,
	42-1,
	43-1,
	44-1,
	45-1,
	46-1,
	47-1,
	48-1,
	49-1,
	50-1,
	51-1,
	52-1,
	53-1,
	54-1,
	55-1,
	56-1,
	57-1,
	58-1,
	59-1,
	60-1,
	61-1,
	62-1,
	63-1,
	64-1,
	65-1,
	66-1,
	67-1,
	68-1,
	69-1,
	70-1,
	77-1,
	78-1,
	79-1,
	80-1,
	81-1,
	82-1,
	83-1,
	84-1,
	85-1,
	91-1,
	92-1,
	93-1,
	94-1,
	95-1,
  97-1
};

const std::vector<DftInterface::pin_map_def_t>
DftInterface::pinMapDownlinkDirect = {
  std::make_tuple( 0, PccaInterface::BIT_TX_HRSTB),
  std::make_tuple( 1, PccaInterface::BIT_TX_NBFLOWSTOP),
  // 2 -> POR
  std::make_tuple( 3, PccaInterface::BIT_TX_TCLK),
  std::make_tuple( 8, PccaInterface::BIT_TX_BXSYNCTRG),
  std::make_tuple(12, PccaInterface::BIT_TX_DINN),
  std::make_tuple(13, PccaInterface::BIT_TX_HBTRG),
  std::make_tuple(26, PccaInterface::BIT_TX_SME),
  std::make_tuple(27, PccaInterface::BIT_TX_TRG)
};

const std::vector<DftInterface::pin_map_def_t>
DftInterface::pinMapDownlinkDirectClocks = {
  std::make_tuple( 9, PccaInterface::BIT_TX_CLKADC),
  std::make_tuple(10, PccaInterface::BIT_TX_CLKBX),
  std::make_tuple(11, PccaInterface::BIT_TX_CLKS0)
};

const std::vector<DftInterface::pin_map_def_t>
DftInterface::pinMapDownlinkShr0 = {
  std::make_tuple( 4, PccaInterface::BIT_SHR_TDI),
  std::make_tuple( 6, PccaInterface::BIT_SHR_TMS),
  std::make_tuple( 7, PccaInterface::BIT_SHR_TRST),
  std::make_tuple(17, PccaInterface::BIT_SHR_SDI_1),
  std::make_tuple(19, PccaInterface::BIT_SHR_SDI_3),
  std::make_tuple(22, PccaInterface::BIT_SHR_SEN_1),
  std::make_tuple(24, PccaInterface::BIT_SHR_SEN_3)
  // SDA is ignored in DFT test
};

const std::vector<DftInterface::pin_map_def_t>
DftInterface::pinMapDownlinkShr1 = {
  std::make_tuple( 5, PccaInterface::BIT_SHR_TME),
  std::make_tuple(14, PccaInterface::BIT_SHR_SCL),
  std::make_tuple(16, PccaInterface::BIT_SHR_SDI_0),
  std::make_tuple(18, PccaInterface::BIT_SHR_SDI_2),
  std::make_tuple(20, PccaInterface::BIT_SHR_SDI_4),
  std::make_tuple(21, PccaInterface::BIT_SHR_SEN_0),
  std::make_tuple(23, PccaInterface::BIT_SHR_SEN_2),
  std::make_tuple(25, PccaInterface::BIT_SHR_SEN_4),
};

    //  .PORin                ( part_PIs[000003] ),      // pinName = PORin;  tf = +SC  ; testOffset = 0.000000;  scanOffset = 0.000000;


//------------------------------------------------------------------------------
class DftFile
{
  private:
    /** Vector file commands (and what we make out as their meaning) */
    enum vec_cmd_t : uint32_t {
      C_STOP = 0,
      C_COMMENT = 100,
      C_UPDATE_PROCESS_NAME = 104,
      C_STIM_PRIMARY_INPUT_DATA = 200,
      C_STIM_CLOCKED_INPUT_DATA = 201,
      C_STIM_PRIMARY_OUTPUT_DATA = 202,
      C_STIM_GLOBAL_TERMINATION = 203,
      C_SCAN_CHAIN_INPUT_DATA = 300,      /* Vector data */
      C_SCAN_CHAIN_COMPARE_DATA = 301,    /* Vector data */
      C_RUN_TEST_CYCLE = 400,
      C_SCAN_CHAIN = 600,
      C_MSG_DEBUG = 900,
      C_MSG_MISCOMPARE = 901
    };

    DftInterface& dfti_;      /** */
    DftMemory& mem_;          /** Storage class for stimuli and read-back data */

    std::ifstream& ifs_;      /** Vector file input stream */
    uint32_t line_;           /** Current line in vector file */
    std::string linebuf_;     /** Vector file line buffer */
    uint32_t cmd_;            /** Last extracted command */


    /** Extract command (number) from vector file line */
    inline uint32_t
    extractCommand(const std::string& line)
      {
        std::stringstream s(line);
        s >> cmd_;
        return cmd_;
      }

    /** Update linebuffer with line from vector file */
    inline size_t
    getLineFromStream()
      {
        size_t r(0);
        if (ifs_.good()) {
          line_++;
          std::getline(ifs_, linebuf_);
          r = linebuf_.size();
        }
        return r;
      }

    /** Process commands
     *  @param sm Regex match expression for line starting with 3 (command-)digits (plus rest)
     */
    void processCmd(const std::smatch& sm, uint32_t line)
      {
        uint32_t mode(0);
        uint32_t seq(0);
        uint32_t len(0);
        std::smatch smx;

        switch(cmd_) {
          case C_STOP:
            Log::sys(__func__, cmd_, "line", line, "Stop requested");
            break;

          case C_COMMENT:
            Log::info(__func__, cmd_, "line", line, sm[2]);
            break;

          case C_UPDATE_PROCESS_NAME:
            Log::info(__func__, cmd_, "line", line, "Updated process name:", sm[2]);
            break;

          case C_STIM_PRIMARY_INPUT_DATA:   // 200
            mem_.pi_.assign(sm[2]);
            DFTLOG(Log::sys(__func__, cmd_, "line", line, "Updated PI =", mem_.pi_);)
            break;

          case C_STIM_CLOCKED_INPUT_DATA:   // 201
            mem_.ci_.assign(sm[2]);
            DFTLOG(Log::sys(__func__, cmd_, "line", line, "Updated CI =", mem_.ci_);)
            break;

          case C_STIM_PRIMARY_OUTPUT_DATA:  // 202
            mem_.po_.assign(sm[2]);                   // Now we know how many outputs there are
            mem_.rpo_.assign(mem_.po_.size(), 'X');   // Initialize read-back output buffer, now that number of outputs is known
            DFTLOG(Log::sys(__func__, cmd_, "line", line, "Updated PO =", mem_.po_, "size = ", mem_.po_.size());)
            break;

          case C_STIM_GLOBAL_TERMINATION:   // 203
            DFTLOG(Log::sys(__func__, cmd_, "line", line, "Ignoring global termination");)  // Brazil has global_term in their code, but they don't use it. What's the point?
            break;

          case C_SCAN_CHAIN_INPUT_DATA:   // 300
            mode = std::stoi(linebuf_.substr(4, 1));
            mem_.sci_.assign(linebuf_.substr(6));       // Stimulus buffer gets filled with data
            mem_.scInfoClear();                         // Clear scan chain information
            DFTLOG(Log::sys(__func__, cmd_, "line", line, "Scan chain input, MODE =", mode, "SCI length =", mem_.sci_.size());)
            DFTLOG(Log::dbg(__func__, cmd_, " SCI =", mem_.sci_.get());)
            break;

          case C_SCAN_CHAIN_COMPARE_DATA: // 301
            mode = std::stoi(linebuf_.substr(4, 1));
            mem_.scc_.assign(linebuf_.substr(6));
            DFTLOG(Log::sys(__func__, cmd_, "line", line, "Scan chain compare, MODE =", mode, "SCC length =", mem_.scc_.size());)
            DFTLOG(Log::dbg(__func__, cmd_, " SCC =", mem_.scc_.get());)
            break;

          case C_RUN_TEST_CYCLE:  // 400
            Log::sys(__func__, cmd_, "line", line, "Running test cycle");
            dfti_.testCycle();
            break;

          case C_SCAN_CHAIN:  // 600
            DFTLOG(Log::sys("Scan chain info: SCI length =", mem_.sci_.size(), "SCC length =", mem_.scc_.size());)

            std::regex_match(linebuf_, smx, std::regex("(^\\d\\d\\d) (\\d) (\\d) (\\d+)(.*)"));
            mode = std::stoi(smx[2]);

            switch (mode) {
              case 1:
                seq = std::stoi(smx[3]);
                len = std::stoi(smx[4]);
                Log::sys(__func__, cmd_, "line", line, "Scan chain, MODE =", mode, "SEQ =", seq, "LEN =", len);

                switch (seq) {
                  case 1:
                    Log::info("Running scan precondition sequence");
                    dfti_.scanPreconditioningSequence_TM_1_SEQ_1_SOP_1();
                    break;

                  case 2:
                    Log::info("Running scan sequence");
                    dfti_.scanSequence_TM_1_SEQ_2_SOP_1(len);
                    break;

                  default:
                    Log::fatal(__func__, "SEQ =", seq, "not supported");
                } // seq
                break;

              default:
                Log::fatal(__func__, "MODE =", mode, "not supported");
            } // mode
            break;

          case C_MSG_DEBUG:
            DFTLOG(Log::sys(__func__, cmd_, "line", line, "Simulating pattern:", sm[2]);)
            break;

          case C_MSG_MISCOMPARE:
            DFTLOG(Log::sys(__func__, cmd_, "line", line, "Simulating pattern:", sm[2]);)
            break;

          default:
            Log::fatal(__func__, cmd_, "line", line, "Unknown command / Ignored ->", sm[0]);
            // TODO: Maybe throw some exception here?
        }
      }

    /** Process vector commands
     *    for scan chain input data and scan chain compare data
     *
     *  When processing input data, try to detect the positions and lengths of the different scan chains in the buffer
     */
    void processVectorCmd(uint32_t line)
      {
        uint32_t inserted(0);
        switch(cmd_) {
          case C_SCAN_CHAIN_INPUT_DATA:
            inserted = addVectorData(mem_.sci_, "-> SCI");
//            mem_.scc_.append(inserted, 'X');  // Keep pre-filling compare buffer with 'X'

            if (inserted != 1000) {   // Compare to 1000: Maybe not universally correct, but works for SAMPA MPW2 & 3
              mem_.scInfoMakeEntry();
              DFTLOG(Log::sys(__func__, cmd_, "line", line, "Detected new scan chain start position: SC" +
                              std::to_string(mem_.sc_info_.size() - 1) +
                              " -> pos " + std::to_string(mem_.sc_info_.back().first) +
                              ", len " + std::to_string(mem_.sc_info_.back().second));)
            }

            break;
          case C_SCAN_CHAIN_COMPARE_DATA:
            addVectorData(mem_.scc_, "-> SCC");
            break;
          default:
            Log::fatal(__func__, cmd_, "line", line, "Illegally associating vector data to command");
            // TODO: Maybe throw some exception here?
        }
      }

    /** Add vector data to buffer
     *    @param dest Destination buffer, either SCI (scan chain input data) or SCC (scan chain compare data SCC)
     *    @param name Buffer identifier, for logging only
     *    @return Number of symbols added to buffer. Typically this is 1000 -- if a vector file line contains less, that
     *              signals the beginning of a new scan chain
     */
    size_t addVectorData(DftBuffer& dest, const std::string& name)
      {
        (void)name; // Avoid 'unused variable' compiler warning in case log output is not enabled
        size_t s = dest.size();
        size_t first_non_ws = linebuf_.find_first_not_of(" \t\n");
        dest.append(linebuf_.substr(first_non_ws));   // Clear whitespace and append to buffer
        size_t added = dest.size() - s;
        DFTLOG(Log::dbg(__func__, cmd_, name + " added =", added, " length =", dest.size());)
        DFTLOG(Log::dbgl(__func__, cmd_, name + "[" + std::to_string(s) + ".." + std::to_string(dest.size() - 1) + "] =",
                  dest.substr(s));)
        return added;
      }


  public:
    DftFile(DftInterface& dfti, std::ifstream& ifs) :
        dfti_(dfti), mem_(dfti.getMem()), ifs_(ifs), line_(0), cmd_(0)
      {}

    void process();

    /** Pin definitions from verilog file (for input pin stimuli)
     */
    static const uint8_t IDX_PI_SEN_0 {22-1};
    static const uint8_t IDX_PI_SEN_1 {23-1};
    static const uint8_t IDX_PI_SEN_2 {24-1};
    static const uint8_t IDX_PI_SEN_3 {25-1};
    static const uint8_t IDX_PI_SEN_4 {26-1};

    static const uint8_t IDX_PI_SDI_0 {17-1};
    static const uint8_t IDX_PI_SDI_1 {18-1};
    static const uint8_t IDX_PI_SDI_2 {19-1};
    static const uint8_t IDX_PI_SDI_3 {20-1};
    static const uint8_t IDX_PI_SDI_4 {21-1};
};


void
DftFile::process()
{
  std::regex cmd_match("(^\\d\\d\\d\\s?)(.*)");

  while(ifs_.good()) {
    if(getLineFromStream() == 0)
      break;

//    std::cout << "*** L" << line_ << " ***\n"
//              << linebuf_ << "\n";

    if ((linebuf_.size()) && (linebuf_[0] == '#')) {
      Log::sys("Ignoring commented line: ", line_);
      continue;
    }

    // Check if the command is a 'regular' line (3-digit command + space) or part of a vector command
    std::smatch sm;
    if (std::regex_match(linebuf_, sm, cmd_match))
      extractCommand(linebuf_);

    // Log::dbg(cmd_);

    if (!sm.size()) {
      // Couldn't find command code at beginning of line, assume this is a vector command
      processVectorCmd(line_);
    }
    else {
      // Regular line with command code at the beginning
      processCmd(sm, line_);
    }

  }
}


//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  bpo::variables_map vm;
  bpo::options_description opt_general(
      "DFT test for T-RORC SAMPA test stand"
      "Usage:\n  " + std::string(argv[0]) + " <cmds/options>\n"
      "Options"
    );
  bpo::options_description opt_hidden("");
  bpo::options_description opt_all;
  bpo::positional_options_description opt_pos;

  /* Command line parsing
   */
  try
  {
    opt_general.add_options()
      ("help,h", "Show this help")
      ("version,v", "Show version information")
      ("setup", bpo::value<uint32_t>()->default_value(0)
        ->notifier([](uint32_t s) { if (s > 2) {
          throw bpo::validation_error(bpo::validation_error::invalid_option_value, "setup"); } } ),
        "Specify which SAMPA test setup to use for DFT test [0..1]")
      ("input-file,i", bpo::value<std::string>()->default_value("vectors.txt"), "Specify DFT test input file")
      ("log-level,l", bpo::value<uint32_t>()->default_value(Log::linfo)
         ->notifier([](uint32_t l) { if (l >= Log::lfatal) {
           throw bpo::validation_error(bpo::validation_error::invalid_option_value, "log-level"); } } ),
          std::string("Control log level. Valid values are " +
                      std::to_string(Log::ldbgl) + " [" + Log::levelName(Log::ldbgl) + "] to [" +
                      std::to_string(Log::lfatal) + " [" + Log::levelName(Log::lfatal) + "]").c_str())
      ("log-no-ts", bpo::bool_switch()->default_value(false),
          "Do not include timestamps in log output")
      // Test routines below
      ("test-dft-shr", bpo::bool_switch()->default_value(false), "Test DFT controller SHR module")
      ("shr0", bpo::value<cl_uint_t>(), "Set SHR0 pattern")
      ("shr-mask0", bpo::value<cl_uint_t>(), "Set SHR0 mask")
      ("shr1", bpo::value<cl_uint_t>(), "Set SHR1 pattern")
      ("shr-mask1", bpo::value<cl_uint_t>(), "Set SHR1 mask")
      ;

    opt_all.add(opt_general).add(opt_hidden);
    bpo::store(bpo::command_line_parser(argc, argv).options(opt_all)
                     .positional(opt_pos).run(), vm);

    if (vm.count("help")) {
      std::cout << opt_general << std::endl;
      exit(0);
    }

    if (vm.count("version")) {
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


  /* Access to specific T-RORC GBT link (for uplink capture register and to drive downlink)
   *   and corresponding HDLC core (needed for SCA access)
   */
  std::unique_ptr<trorc::Device> trorc;
  std::unique_ptr<trorc::Bar> bar;
  std::unique_ptr<PccaInterface> pcca_iface;

  try {
    trorc.reset(new trorc::Device(0 /*device_number*/));
    bar.reset(new trorc::Bar(*trorc, 1 /*bar_number*/));
    pcca_iface.reset(new PccaInterface(*bar, vm["setup"].as<uint32_t>() * 2));   // multiply by 2 -> VLDB connected to even link numbers
  } catch (int e) {
    std::cerr << "ERROR: Failed to initialize T-RORC: " << librorc::errMsg(e)
              << std::endl;
    exit(-1);
  }

  /** SHR testing options
   */
  if (vm["test-dft-shr"].as<bool>()) {
    pcca_iface->dftInitialize();

    uint8_t shr0(0x0), shr0_mask(0x0);
    uint8_t shr1(0x0), shr1_mask(0x0);

    if (vm.count("shr0")) {
      shr0 = vm["shr0"].as<cl_uint_t>()() & 0xff;
      shr0_mask = 0xff;
    }
    if (vm.count("shr1")) {
      shr1 = vm["shr1"].as<cl_uint_t>()() & 0xff;
      shr1_mask = 0xff;
    }

    if (vm.count("shr0-mask"))
      shr0_mask = vm["shr0-mask"].as<cl_uint_t>()() & 0xff;
    if (vm.count("shr1-mask"))
      shr1_mask = vm["shr1-mask"].as<cl_uint_t>()() & 0xff;

    if (vm.count("shr0") && (vm.count("shr1") == 0))
      shr1_mask = 0x0;
    if (vm.count("shr1") and (vm.count("shr0") == 0))
      shr0_mask = 0x0;

    std::cout << std::hex << std::setfill('0') << std::setw(2)
              << "SHR0 = 0x" << static_cast<uint32_t>(shr0) << " "
              << "MASK0 = 0x" << static_cast<uint32_t>(shr0_mask) << "\n"
              << "SHR1 = 0x" << static_cast<uint32_t>(shr1) << " "
              << "MASK1 = 0x" << static_cast<uint32_t>(shr1_mask) << "\n"
              << std::dec << std::setfill(' ');
    pcca_iface->dftShrLoad(shr0, shr1, shr0_mask, shr1_mask);
  }

  /** Testing only - do not run DFT test
   */
  if (vm["test-dft-shr"].as<bool>())
    return 0;

  /** DFT test with vector file
   */
  int dft_errors(0);
  try {
    // Try to open DFT file
    const std::string& infile = vm["input-file"].as<std::string>();
    std::ifstream instream(infile, std::ifstream::in);
    if (!instream.good())
      throw std::runtime_error("Unable to open DFT vector file '" + infile + "'.\n"\
                               "Maybe use option '-i/--input-file' to specify vector file?\n");

    // Process file and start DFT test
    DftInterface dfti(*(pcca_iface).get());

    DftFile dft(dfti, instream);

    dft.process();
    dfti.printStatistics(std::cout);
    dft_errors = dfti.getErrors();

    instream.close();
  }
  catch (std::exception &e) {
    std::cout << e.what() << std::endl;
    exit(-10);
  }

  return dft_errors;
}
