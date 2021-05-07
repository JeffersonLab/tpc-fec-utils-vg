#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <thread>
#include <atomic>
#include <bitset>
#include <fstream>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>           // Use boost::thread, C++11's threads doesn't support thread groups
#include <boost/filesystem.hpp>
#include <csignal>
#include <ctime>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "simple_log.hpp"
#include "trorc.hpp"
#include "git_info.hpp"
#include "gbt_link.hpp"
#include "trigger.hpp"

namespace bpo = boost::program_options;
namespace bfs = boost::filesystem;

// #define SIMULATION_NORORC
static const uint32_t MAX_FECS(6);
static const uint32_t MAX_CHANNELS(2*MAX_FECS);
using fec_mask_t = std::bitset<MAX_FECS>;
using channel_mask_t = std::bitset<MAX_CHANNELS>;

static int ss_port; // vg 05.2021
std::string ss_host_ip; // vg 05.2021

//------------------------------------------------------------------------------
/** File writer
 *
 *  Encapsulates the filename generation and handling/placing of the DMA
 *  channel output files
 */
class FileWriter
{
private:
    std::ofstream ofs_;

public:
    /** File writer options struct
     */
    struct Options {
        uint32_t device_id;
        channel_mask_t active_channel_mask;
        std::string base_dir;
        std::string number_prefix;
        std::string fn_postfix;
        uint32_t number;
        bool omit_output_dir;
        bool overwrite;
    };

    /** Construct file writer object
     *  @param o       File writer options struct
     *  @param channel T-RORC DMA channel ID
     */
    FileWriter(const Options& o, uint32_t channel) :
            ofs_((outputDirPath(o) /= outputFilename(o, channel)).native(),
                 std::ofstream::out | std::ofstream::binary | std::ofstream::trunc)
    {
        if (!ofs_.is_open())
            throw std::runtime_error("Unable to open output stream " +
                                     (outputDirPath(o) /= outputFilename(o, channel)).native() +
                                     " for writing");
    }

    /** Destruct
     */
    ~FileWriter()
    { ofs_.close(); }

    /** Write data to output stream
     *  @param data Pointer to the data to write
     *  @param size Data size in bytes
     */
    void write(const char* data, ssize_t size) { ofs_.write(data, size); }

    /** Flush output stream
     */
    void flush() { ofs_.flush(); }

    /** Construct output directory path from
     *    base_dir and, if requested, number
     *    @param  o File writer option struct
     *    @return   Path of form 'base_dir/run<06d>' or 'base_dir'
     */
    static bfs::path outputDirPath(const Options& o)
    {
        bfs::path p(o.base_dir);
        if (!o.omit_output_dir) {
            std::stringstream r;
            r << std::dec << std::setfill('0') << std::setw(6) << o.number;
            p /= bfs::path(o.number_prefix + r.str());
        }
        return p;
    }

    /** Construct output file name
     *    @param o       File writer option struct
     *    @param link_id Link id, in this case equivalent to T-RORC DMA channel ID
     *    @return        File path of form '<nr_prefix><06d>_trorc<02d>_link<02d>[_<fn_postfix>]?.bin'
     */
    static bfs::path outputFilename(const Options& o, uint32_t link_id)
    {
        std::stringstream r, t, c;
        r << std::dec << std::setfill('0') << std::setw(6) << o.number;
        t << std::dec << std::setfill('0') << std::setw(2) << o.device_id;
        c << std::dec << std::setfill('0') << std::setw(2) << link_id;
        return bfs::path(o.number_prefix + r.str() + "_trorc" + t.str() +
                         "_link" + c.str() +
                         (o.fn_postfix.size() ? ("_" + o.fn_postfix) : "") +
                         ".bin");
    }

    /** Create output directories, if it doesn't exist yet
     */
    static bool createOutputDirectory(const Options& o)
    {
        bfs::path p(outputDirPath(o));
        return bfs::exists(p) ? false : bfs::create_directories(p);
    }

    /** Filessytem checks to run at startup
     *    - Check base output directory or closest existing parent (existing? writeable?)
     *    - Check output file path (already existing? overwrite?)
     *    @params o FileWriter options struct
     *    @return True, if checks succeed, false otherwise
     */
    static bool startupCheck(const Options& o)
    {
        try {
            // Check base output directory
            bfs::path pord(o.base_dir);

            // Find base directory or closest existing parent of it
            do {
                if (bfs::exists(pord)) {
                    bfs::canonical(pord);
                    bfs::is_directory(pord);
                }
                else
                    pord = pord.parent_path();
            }
            while (!bfs::exists(pord));
            // pord -> should be writeable base directory of writeable closest parent here

            // Base output dir or closest existing parent writeable?
            bfs::path test(pord);
            test  /= bfs::unique_path();
            std::ofstream test_file(test.native(), std::ios::out);
            test_file << "test\n";
            bool test_write_ok = test_file.good();
            test_file.close();
            if (!test_write_ok)
                throw(bfs::filesystem_error("Writing to directory " + pord.native(),
                                            boost::system::errc::make_error_code(boost::system::errc::permission_denied)));
            else
                bfs::remove(test);

            // Test if output files exist for active channels
            //   and throw if file exists and overwrite is not specified
            for (uint32_t c = 0; c < o.active_channel_mask.size(); c++) {
                if (o.active_channel_mask[c]) {
                    bfs::path p = (outputDirPath(o) /= outputFilename(o, c));

                    if (bfs::exists(p)) {
                        if (o.overwrite)
                            Log::warn("File", p.native(), "will be overwritten");
                        else
                            throw(bfs::filesystem_error(p.native(),
                                                        boost::system::errc::make_error_code(boost::system::errc::file_exists)));
                    }
                }
            }
        }
        catch (bfs::filesystem_error& e) {
            Log::error(e.what());
            return false;
        }
        catch (std::runtime_error& e) {
            Log::error(e.what());
            return false;
        }
        catch (...) {
            Log::error("Unknown error");
            return false;
        }

        return true;
    }
};  // FileWriter



//------------------------------------------------------------------------------
class ChannelReadout
{
public:
    static std::mutex active_mtx_;
    static channel_mask_t readout_active_mask_;

    /** Test if any channel is still active
     *    (e.g. has not yet acquired the requested number
     *    of events)
     */
    static bool anyChannelActive()
    {
        std::lock_guard<std::mutex> g(active_mtx_);
        return ChannelReadout::readout_active_mask_.any();
    }

    /** Request stop and trigger end of event loop
     */
    static void requestStop()
    {
        ChannelReadout::stop_requested_ = true;
    }

    struct Options {
        channel_mask_t active_channel_mask;
        uint32_t event_limit;
        uint32_t readout_frame_count;
        uint32_t control_pattern_cycles;
        uint32_t data_type_mode;
        std::string readout_mode;
        bool     sampa_tester_mode;
    };

private:
    static std::atomic<bool> stop_requested_;
    static const uint64_t    dmaBufferSize_ = (uint64_t)1 << 31;

    /** Flip channel bit in active readout mask
     */
    void deactivate()
    {
        std::lock_guard<std::mutex> g(active_mtx_);
        readout_active_mask_.reset(channel_);
        Log::dbg("Ch" + std::to_string(channel_) + " deactivated", readout_active_mask_);
        std::this_thread::yield();
    }

    /** Initialize global readout thread synchronization
        @param acm Mask of active channels
     */
    void initialize(channel_mask_t acm)
    {
        std::lock_guard<std::mutex> g(active_mtx_, std::adopt_lock);
        readout_active_mask_ = acm;
        Log::dbg("Ch" + std::to_string(channel_) + " thread sync initialized",
                 readout_active_mask_);
    }

    std::atomic<uint64_t>    events_recorded_;
    ssize_t                  expected_event_size_;
    const uint32_t           header_size_;
    Options cro_;

    std::unique_ptr<librorc::event_stream> es_;
    std::unique_ptr<trorc::GbtLink>        gbt_;
    uint32_t channel_;
    std::unique_ptr<FileWriter>            fw_;

public:
    /** Construct channel readout
     *  @param es  Unique pointer to channels RORC event stream, ChannelReadout takes ownership
     *  @param gbt Unique pointer to channels GBT link, ChannelReadout takes ownership
     *  @param fw  Unique pointer to channels file writer, ChannelReadout takes ownership
     @  @param o   Channel readout options struct
     */
    ChannelReadout(
#if not defined SIMULATION_NORORC
    std::unique_ptr<librorc::event_stream> es,
        std::unique_ptr<trorc::GbtLink> gbt,
#else
            uint32_t channel,
#endif
            std::unique_ptr<FileWriter> fw,
            const Options& o) :
            events_recorded_(0),
            expected_event_size_(0),
            header_size_(8*sizeof(uint32_t)),   // FIXME: manage header size in a nicer way
            cro_(o),
#if not defined SIMULATION_NORORC
            es_(std::move(es)), gbt_(std::move(gbt)),
#else
            channel_(channel),
#endif
            fw_(std::move(fw))
    {
#if not defined SIMULATION_NORORC
        // Mirror info about our channel
        channel_ = es_->m_channel_status->channel;
#endif
        Log::sys("Created channel", channel_, "readout thread");

        // Initialize global synchronization variables
        //   (called multiple time by all readout thread at construction, but all
        //   threads provide the same data)
        initialize(o.active_channel_mask);

#if not defined SIMULATION_NORORC
        // GBT setup
        gbt_->stopReadout();
        gbt_->disableControlPattern();

        // Set default idle and control patterns, masks (, and cycles)
        if (cro_.sampa_tester_mode) {
            // Settings required for SAMPA tester readout (SHR output disabled)
            gbt_->setTxIdlePattern(0x10000, 0x000f0f00, 0xf0f0000f);
            gbt_->setTxControlPattern(0x10000, 0x000f0f00, 0xf000000f);
        }
        else {
            // Settings for regular readout with a FEC
            if (cro_.readout_mode == "das") {
                std::cout << "\nReadout configured in DAS mode\n" << std::endl;
                gbt_->setTxIdlePattern(0x10000, 0x0000ffff, 0xffffffff);	// for DAS mode
                gbt_->setTxControlPattern(0x10000, 0x0000ff00, 0x000fffff);	// for DAS mode
            }
            else if (cro_.readout_mode == "dsp") {
                std::cout << "\nReadout configured in DSP mode\n" << std::endl;
                gbt_->setTxIdlePattern(0x10000, 0x0000f000, 0x000fffff);	// for DSP mode
                gbt_->setTxControlPattern(0x10000, 0x0000f000, 0x000fffff);	// for DSP mode
            }
            else
                std::cout << "\nERROR: Invalid readout mode (must be das or dsp)\n" << std::endl;
        }

        gbt_->setSoftwareTriggerChannelMask(o.active_channel_mask.to_ulong());
        gbt_->setTxPatternChannelMask(o.active_channel_mask.to_ulong());
        gbt_->setTxControlPatternCycles(o.control_pattern_cycles);

        gbt_->resetEventCounter();
        gbt_->resetChannelExtractor();

        // Set frame counts (equal in our case)
        gbt_->setRxReadoutTargetFrameCount(cro_.readout_frame_count);  // -> Gate opening after trigger
        gbt_->setRxEventSizeFrameCount(cro_.readout_frame_count);      // -> DMA block size

        gbt_->setReadoutMode(cro_.data_type_mode);

        es_->m_channel->clearEventCount();
        es_->m_channel->clearStallCount();
        es_->m_channel->readAndClearPtrStallFlags();

        // Expected event size is fixed and depending on the readout mode
        switch (cro_.data_type_mode) {
          case 0:
                  expected_event_size_ = static_cast<ssize_t>(cro_.readout_frame_count) << 4;
                  break;
          case 1:
          case 2: expected_event_size_ = static_cast<ssize_t>(cro_.readout_frame_count + 2) << 4;
                  break;
          case 3: expected_event_size_ = static_cast<ssize_t>(2*cro_.readout_frame_count + 2) << 4;
                  break;
        }

        // Allow external triggers fom LVDS or trigger generator
        gbt_->setAllowExternalTrigger(1);

        int ec = es_->initializeDma(2*channel_, dmaBufferSize_);
        if (ec != 0)
          throw trorc::Exception("Failed to initialize DMA" + std::string(librorc::errMsg(ec)));
#endif
    }


    /** Destructor
     */
    ~ChannelReadout()
    {
        Log::sys("Finalizing channel", channel_, "readout");

#if not defined SIMULATION_NORORC
        gbt_->setAllowExternalTrigger(0);
        gbt_->stopReadout();
        es_->m_channel->clearEventCount();
        es_->m_channel->clearStallCount();
        es_->m_channel->readAndClearPtrStallFlags();
#endif
    }


    /** Request SYNC event from SAMPAs, and record it
     */
    void requestSyncEvent()
    {
        std::stringstream tm, rm;
        tm << std::hex << gbt_->txPatternChannelMask();
        rm << std::hex << gbt_->softwareTriggerChannelMask();
#if not defined SIMULATION_NORORC
        gbt_->driveControlPattern(true);
#endif
        Log::info("Initiated SYNC pattern and recording of event [txPatMask 0x"
                  + tm.str() + ", roMask 0x" + rm.str() + "]");
    }


    /** Request recording of event
     */
    void requestEvent()
    {
#if not defined SIMULATION_NORORC
        gbt_->triggerReadout();
#endif
        Log::info("Initiated software trigger");
    }

    /**
     * Create a TCP client to send SAMPA data to the server.
     * vg 05.2021
     */
int createTCPClient(const std::string &host, int port) {

        // Create a socket
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock == -1) {
	  std::cerr << "Can't create a socket port = " << port << std::endl;
            return -1;
        }

        // Create a hint structure for the server: host IP, port,etc.
        sockaddr_in hint;
        hint.sin_family = AF_INET;
        hint.sin_port = htons(port);
        inet_pton(AF_INET, host.c_str(), &hint.sin_addr);

        // Connect to the server on the socket
        int connectRes = connect(sock, (sockaddr * ) & hint, sizeof(hint));
        if (connectRes == -1) {
	  std::cerr << "Can't connect to the socket port = " << port << std::endl;
            return -1;
        }
        return sock;
    }


    /** Thread worker with event loop
     */
    void run()
    {
        sigset_t signal_set;
        sigaddset(&signal_set, SIGINT);
        sigaddset(&signal_set, SIGTERM);
        pthread_sigmask(SIG_BLOCK, &signal_set, NULL);

        Log::info("Channel " + std::to_string(channel_) + " readout thread active");

	std::cout << "DDD " + std::to_string(channel_) + " readout thread active" << std::endl; //vg 05.2021

#if not defined SIMULATION_NORORC
        librorc::EventDescriptor *report;
        const uint32_t *event;
        uint64_t reference;
#endif

        // Create a client socket to send data to streaming server. vg 05.2021
        int sock = createTCPClient(ss_host_ip, ss_port + channel_);

        // Event polling loop
        while (!stop_requested_) {
#if not defined SIMULATION_NORORC
            if (es_->getNextEvent(&report, &event, &reference)) {
            es_->updateChannelStatus(report);

            ssize_t event_size = ssize_t((report->calc_event_size & 0x3fffffff) << 2) + 4;

            if ((expected_event_size_ == 0) && (event_size != (expected_event_size_ << 4))) {
              // Event size does not match expected size, which indicates
              //   PCIe output FIFO overflow, drop this event
              es_->m_channel_status->error_count++;
            }
            else {
              // Record event
                    // Instead of file writing write event to a streaming socket
                    // in case streaming client creation is successful. vg 05.2021
                    if (sock > 0) {
                        // Stream data to the listening server.
                        int sendRes = send(sock, reinterpret_cast<const char *>(event), event_size, 0);
                        if (sendRes == -1) {
			  std::cout << "Can't send data to server. " << std::endl;
                            continue;
                        }
                    } else {
                        // Streaming socket creation and connection failed, write to the file.
                        fw_->write(reinterpret_cast<const char *>(event), event_size);
                    }
                    // vg 05.2021

              es_->releaseEvent(reference);
              events_recorded_++;
            }
          }
#else
            Log::info("Recording event " + std::to_string(events_recorded_));
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            // Dummy writes for testing
            fw_->write(reinterpret_cast<char*>(&channel_), sizeof(uint32_t));
            fw_->write(reinterpret_cast<char*>(&events_recorded_), sizeof(uint64_t));
            fw_->write("deadbeaf", 8);
            events_recorded_++;
#endif
            if ((cro_.event_limit != 0) && (events_recorded_ >= cro_.event_limit)) {
                Log::info("Event limit reached");
                break;
            }
        }
        // Close the socket. vg 05.2021
        close(sock);

        fw_->flush();
        deactivate();
        Log::info("Readout done: " + std::to_string(events_recorded_) + " events recorded");
    }
}; // ChannelReadout

std::mutex ChannelReadout::active_mtx_;
std::atomic<bool> ChannelReadout::stop_requested_{false};
channel_mask_t ChannelReadout::readout_active_mask_{0x0};



//------------------------------------------------------------------------------
void handleSignal(int sig)
{
    signal(sig, [](int signum) {
        // Calling requestStop() is ok, as atomic<bool>.is_lock_free = true
        //   Do not log anything here... Log::* is not signal-safe
        (void)signum;
        ChannelReadout::requestStop();
    });
}


int main(int argc, char** argv)
{
#if defined SIMULATION_NORORC
    Log::warn("*** RUNNING SIMULATION MODE WITHOUT T-RORC ACCESS ***");
#endif

    enum exit_codes_t : int {
        ec_success = 0,
        ec_err_cmdline = 1,
        ec_err_filesystem = 2,
        ec_err_device = 3,
        ec_err_channel_readout = 4
    };

    bpo::variables_map vm;
    bpo::options_description opt_general(
            "T-RORC readout for multiple boards\n"
            "  Each FEC n provides two data streams originating at GBTx0 and GBTx1, which\n"
            "  are connected to channels 2n and 2n+1, respectively. Readout is active\n"
            "  until the event limit is reached (if set), or the runtime limit is reached\n"
            "  (if set), or a program abort is requested by the operator.\n"
            "  If VLDBs are connected, it is assumed that their GBTx is connected to\n"
            "  channels 2n, and odd channels are ignored\n"
            "Usage:\n  " + std::string(argv[0]) + " <cmds/options>\n"
                                                  "Options"
    );
    bpo::options_description opt_hidden("");
    bpo::options_description opt_all;
    bpo::positional_options_description opt_pos;

    FileWriter::Options fw_options;
    ChannelReadout::Options cr_options;

    try
    {
        opt_general.add_options()

                // Add streaming server initial port and server host setup parameters.
                // Stream server initial port number vg 05.2021
                ("port,p", bpo::value<int>(&ss_port)->default_value(6000),
                 "Stream listening server initial port number")

                // Stream server host IP.  vg 05.2021
                ("host_ip", bpo::value<std::string>(&(ss_host_ip)),
                 "Specify the stream listening server host IP address")
                // vg 05.2021

                // FEC mask (with range check 0x1..0x3f)
                ("mask,m", bpo::value<cl_uint_t>()->required()
                         ->notifier([](cl_uint_t v) {
                             if (v() < 0x1 || v() > ((uint32_t(1) << (fec_mask_t().size())) - 1))
                             { throw bpo::validation_error(bpo::validation_error::invalid_option_value, "mask"); } } ),
                 "REQUIRED: Mask of FECs to read out. Valid values are: [0x1 .. 0x3f]")

                // define readout mode
                ("mode", bpo::value<std::string>(&(cr_options.readout_mode))->required(),
                        "REQUIRED: Readout mode [das or dsp]")

                // No sync event at start-of-readout
                ("no-sync", bpo::bool_switch(),
                 "Do no request SYNC pattern and record no SYNC event at the start of readout")

                // Number of triggered events
                ("events", bpo::value<uint32_t>(&(cr_options.event_limit))->default_value(50000),
                 "Number of triggered events to record (0 -> unlimited)")

                // Runtime
                ("runtime", bpo::value<uint32_t>()->default_value(0),
                 "Specify the runtime of the readout program in seconds (0 -> unlimited)")

                // Frames per event
                ("frames", bpo::value<uint32_t>(&(cr_options.readout_frame_count))->default_value(160),
                 "Number of GBT frames to read out (per trigger)")

                // Data type
                ("data-type", bpo::value<cl_uint_t>()->default_value(0x3)
                         ->notifier([](cl_uint_t v) { if ((v() < 1) || (v() > 3)) {
                             throw bpo::validation_error(bpo::validation_error::invalid_option_value, "data-type"); } } ),
                 "Set data type (readout mode) on the T-RORC. Valid are 1..3, see header document for definition")

                // File I/O
                ("output-dir", bpo::value<std::string>(&(fw_options.base_dir))
                         ->default_value("/local/data/tpc-beam-test-2017"),
                 "Specify the base output directory for recorded data")

                // Overwrite
                ("overwrite", bpo::bool_switch(&(fw_options.overwrite))->default_value(false),
                 "Overwrite potentially existing files output directory")

                // Specialization for SAMPA tester
                ("sampa-tester", bpo::bool_switch(),
                 "Specialization for SAMPA tester: "
                 "Adapted IDLE/CTRL patterns, single GBTx and SCA connect to even T-RORC channels, odd channels unused")

                // Set number
                ("nr", bpo::value<uint32_t>(&(fw_options.number))->default_value(0x0),
                 "Specify current [run/SAMPA] number ")

                // Set number prefix
                ("nr-prefix", bpo::value<std::string>(&(fw_options.number_prefix))->default_value("run"),
                 "Specific prefix prepended to the number (e.g. run -> run<nr>, sampa -> sampa<nr>)")

                // Allow for arbitrary filename postfix
                ("fn-postfix", bpo::value<std::string>(&(fw_options.fn_postfix))->default_value(""),
                 "Specify arbitrary string appended to output filename")

                // No output directory creation
                ("no-dir", bpo::bool_switch(&(fw_options.omit_output_dir)),
                 "Do not create <output-dir>/<nr-prefix><nr> directory to store files in")

                // Verbosity / log level
                ("log,l", bpo::value<int32_t>()->default_value(Log::lsys)
                         ->notifier([](int32_t v) { if ((v < Log::ldbg) || (v > Log::lfatal)) {
                             throw bpo::validation_error(bpo::validation_error::invalid_option_value, "log"); } } ),
                 std::string("Control log level. Valid values are " +
                             std::to_string(Log::ldbgl) + " [" + Log::levelName(Log::ldbgl) + "] to [" +
                             std::to_string(Log::lfatal) + " [" + Log::levelName(Log::lfatal) + "]").c_str())

                ("version", "Print version information and exit")
                ("help,h", "Print this help message")
                ;

        opt_hidden.add_options()
                ("device", bpo::value<cl_uint_t>()->default_value(0),
                 "Specify the RORC device ID")

                ("cp-cyc", bpo::value<cl_uint_t>()->default_value(0x10)
                         ->notifier([](cl_uint_t v) { if(v() < 0x1 || v() > 0x80) {
                             throw bpo::validation_error(bpo::validation_error::invalid_option_value, "cp-cyc"); } } ),
                 "Number of cycles the control pattern is driven")
                ;

        opt_all.add(opt_general).add(opt_hidden);

        bpo::store(bpo::command_line_parser(argc, argv).options(opt_all)
                           .positional(opt_pos).run(), vm);

        if (vm.count("help")) {
            std::cout << opt_general << std::endl;
            exit(ec_success);
        }

        if (vm.count("version")) {
            std::cout << GitInfo();
            exit(ec_success);
        }

        // Get the streaming listening server initial port number.  vg 05.2021
        if (vm.count("port")) {
            ss_port = vm["port"].as<int>();
            std::cout << "ss_port = " << ss_port << std::endl;
        }
        // vg 05.2021

        bpo::notify(vm);
    }
    catch(bpo::error& e)
    {
        Log::error(std::string(e.what()) + ".", "Exiting");
        std::cout << opt_general << std::endl;
        exit(ec_err_cmdline);
    }
    catch(std::exception& e)
    {
        Log::error(std::string(e.what()) + ".", "Exiting");
        exit(ec_err_cmdline);
    }

    //
    // Signal handling
    //   Register SIGINT for normal console usage, but
    //   also SIGUSR* for use in non-interactive login shells
    handleSignal(SIGINT);
    handleSignal(SIGTERM);

    //
    // Update log level
    Log::setLevel(static_cast<Log::level_t>(vm["log"].as<int32_t>()));

    //
    // Calculate active channel mask and lowest active channel
    //   - normal mode assumes FEC n on channels 2*n and 2*n+1
    //   - vldb/sampa-tester mode assumes VLDB n on channels 2*n, odd channels are ignored
    fec_mask_t active_fec_mask(vm["mask"].as<cl_uint_t>()());
    channel_mask_t active_channel_mask(0);
    int lowest_channel(-1);

    for (uint32_t c = 0; c < 2*active_fec_mask.size(); c++) {
        if (active_fec_mask[c/2] && (lowest_channel == -1))
            lowest_channel = c;
        if (active_fec_mask[c/2])
            active_channel_mask[c] = true;
        if (vm["sampa-tester"].as<bool>() == true)    // VLDBs are using even channels only
            c +=1;                              //   and have only 1 GBTx (transceiver, with SCA)
    }

    Log::info("Starting readout [" + fw_options.number_prefix + " " + std::to_string(fw_options.number) + "]");

    Log::sys("Assuming ", vm["sampa-tester"].as<bool>() ? "SAMPA-TESTER" : "FEC",
             "mode. Using board mask:", active_fec_mask.to_string(),
             ", Channel mask:", active_channel_mask.to_string());

    //
    // Filesystem checking
    fw_options.device_id = vm["device"].as<cl_uint_t>()();
    fw_options.active_channel_mask = active_channel_mask;

    if (!FileWriter::startupCheck(fw_options)) {
        Log::error("Exiting");
        exit(ec_err_filesystem);
    }

    //
    // Setup device
    std::unique_ptr<trorc::Device> trorc;
    std::unique_ptr<trorc::Bar> bar;
    std::unique_ptr<trorc::Trigger> lvds_trigger;

    try {
        trorc.reset(new trorc::Device(vm["device"].as<cl_uint_t>()()));
        bar.reset(new trorc::Bar(*trorc, 1));
        lvds_trigger.reset(new trorc::Trigger(*bar));
    }
    catch (int& e) {
        Log::fatal("Failed to initialize T-RORC:", std::to_string(e), ": ", librorc::errMsg(e));
        exit(ec_err_device);
    }

    //
    // Setup channel readout
    FileWriter::createOutputDirectory(fw_options);
    std::array<std::unique_ptr<ChannelReadout>, active_channel_mask.size()> channels;
    cr_options.control_pattern_cycles = vm["cp-cyc"].as<cl_uint_t>()();
    cr_options.data_type_mode = vm["data-type"].as<cl_uint_t>()();
    cr_options.active_channel_mask = active_channel_mask.to_ulong();
    cr_options.sampa_tester_mode = vm["sampa-tester"].as<bool>();

    try {
        for (uint32_t c = 0; c < active_channel_mask.size(); c++) {
            if (active_channel_mask[c]) {
                channels[c].reset(new ChannelReadout(
#if not defined SIMULATION_NORORC
                make_unique<librorc::event_stream>(trorc->get(), bar->get(), c, librorc::kEventStreamToHost),
            make_unique<trorc::GbtLink>(*(bar.get()), c),
#else
                        c,
#endif
                        make_unique<FileWriter>(fw_options, c),
                        cr_options
                ));
            }
        }
    }
    catch (int& e) {
        Log::fatal("Failed setup T-RORC channel readout:", std::to_string(e), ", ", librorc::errMsg(e));
        exit(ec_err_channel_readout);
    }
    catch (trorc::Exception& e) {
        Log::fatal(e.what());
        exit(ec_err_channel_readout);
    }
    catch (std::runtime_error& e) {
        Log::fatal(e.what());
        exit(ec_err_channel_readout);
    }

    //
    // Clear external trigger count
    lvds_trigger->clearTriggerCount();

    //
    // Create thread group with one readout thread per link
    boost::thread_group readout_threads;

    for (uint32_t c = 0; c < channel_mask_t().size(); c++) {
        if (active_channel_mask[c]) {
            readout_threads.create_thread(boost::bind(&ChannelReadout::run, channels[c].get()));
        }
    }

    //
    // Request SYNC patter from SAMPAs and record associated event
    if (!vm["no-sync"].as<bool>()) {
        channels[lowest_channel]->requestSyncEvent();
    }

    //
    // Implement program runtime
    uint32_t time_limit(vm["runtime"].as<uint32_t>());
    std::chrono::time_point<std::chrono::system_clock> t_start = std::chrono::system_clock::now();
    while (ChannelReadout::anyChannelActive()) {
        std::chrono::time_point<std::chrono::system_clock> t_now = std::chrono::system_clock::now();
        if ((time_limit != 0) &&
            (std::chrono::duration_cast<std::chrono::seconds>(t_now - t_start).count() > time_limit)) {
            ChannelReadout::requestStop();
            Log::info("Time limit reached, deactivating readout threads");
            break;
        }
        std::this_thread::yield();
    }

    //
    // Wait for threads to finalize
    Log::sys("Waiting for threads to join");
    readout_threads.join_all();
    Log::info("Readout done [" + fw_options.number_prefix + " " + std::to_string(fw_options.number) + "]");

    return ec_success;
}
