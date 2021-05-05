#pragma once
#include <tuple>
#include <chrono>
#include <thread>
#include <cassert>
#include <sstream>

#include "hdlc_core.hpp"

#ifdef SCA_DEBUG
  #define scadebug(s) std::cout << __func__ << " : " << s << std::endl;
  #define adcdebug(s) std::cout << s << std::endl;
#else
  #define scadebug(...)
  #define adcdebug(...)
#endif

namespace gbt {

//------------------------------------------------------------------------------
/** SCA Exception class
 */
class ScaException : public std::runtime_error
{
  public:
    ScaException(const std::string& s, uint8_t channel) :
      std::runtime_error("ERROR : SCA [CH" + std::to_string(channel) + "] : " + s) {}
};



//------------------------------------------------------------------------------
/** GBT SCA common features
 */
class Sca
{
  public:
    /** Type to specify the byte order definition for SCA transceives
     *    - BO_NORMAL [31..0]
     *    - BO_REVERSED [7..0, 15..8, 23..16, 31..24]
     *    - BO_REVERSED_HW [15..8, 7..0, 31..24, 23..16]
     */
    enum sca_byteorder_t : uint8_t { BO_NORMAL, BO_REVERSED, BO_REVERSED_HW };

    /** Tuple for the SCA register file information\n
     *    - <0> TX command
     *    - <1> TX length
     *    - <2> RX length
     *    - <3> Byte order: BO_NORMAL, BO_REVERSED, BO_REVERSED_HW
     */
    using sca_rgf_t = std::tuple<uint8_t, uint8_t, uint8_t, sca_byteorder_t>;

  protected:
    common::HdlcCore& hdlc_core_;
    uint8_t channel_;
    std::ostream& rstream_;
    bool report_sca_reply_error_;
    bool throw_on_sca_reply_error_;

    /** Reverse byte order
     *  @param a[31..0] input value
     *  @return a[7..0, ..., 31..24]
     */
    inline uint32_t rbo(uint32_t a) const
      {
        return ((a & 0xff) << 24) | ((a & 0xff00) << 8) |
               ((a & 0xff0000) >> 8) | ((a & 0xff000000) >> 24);
      }

    /** Reverse halfword order
     *  @param a[31..0] input value
     *  @return a[15..8, 7..0, 31..24, 23..16]
     */
    inline uint32_t rhwo(uint32_t a) const
      {
        return ((a & 0xffff) << 16) | ((a & 0xffff0000) >> 16);
      }

    /** Adjust data byte order according to register file definition
     *  @param rgf SCA register file definition
     *  @param data Input data
     *  @return Data with adjusted byte order
     */
    inline uint32_t adjustByteOrder(const sca_rgf_t& rgf, const uint32_t data) const
      {
        uint32_t adjusted_data;
        switch(std::get<3>(rgf)) {
          case BO_REVERSED:    adjusted_data = rbo(data); break;
          case BO_REVERSED_HW: adjusted_data = rhwo(data); break;
          case BO_NORMAL:
          default: adjusted_data = data; break;
        }
        return adjusted_data;
      }

    /** Handle reply frames with error condition set
     *
     *  If throw_on_sca_reply_error_ is true, function throws an exception with
     *    information about the detected error<br/>
     *  If report_sca_reply_errors_ is true (and throw_exception is false), information
     *    about the detected error is printed to rstream_<br/>
     *
     *  @param request The request frame (for reporting in case of error)
     *  @param reply   The reply frame (containing the error field to analyze)
     */
    inline void processErrorField(const HdlcEcFramePayload& request, const HdlcEcFramePayload& reply) const
      {
        if (reply.error != 0x0) {
          std::stringstream ss;
          ss << hdlc_core_.getPayloadErrorMessage(reply.error)
             << " [err_code: 0x" << std::hex << static_cast<uint32_t>(reply.error) << std::dec << "]\n"
             << "       -> Request frame: {" << request << "}\n"
             << "       -> Reply frame  : {" << reply << "}";

          if (throw_on_sca_reply_error_)
            throw ScaException(ss.str(), request.channel);

          if (report_sca_reply_error_)
            rstream_ << "  ERROR : SCA [CH" << std::to_string(request.channel) << "] : " << ss.str() << "\n";
        }
      }

  public:
    /** Sca base communication
     *  @param hdlc_core Reference to the HDLC core for communication with the SCA
     *  @param channel SCA channel to communicate with
     *  @param report_sca_reply_error If true, print information on potential SCA reply error to report_stream
     *  @param throw_on_sca_reply_error If true, throws an ScaException in case an SCA reply error is encountered
     *  @param report_stream Output stream for error reporting
     */
    Sca(common::HdlcCore& hdlc, uint8_t channel,
        bool report_sca_reply_error = true,
        bool throw_on_sca_reply_error = false,
        std::ostream& report_stream = std::cout
       ) :
        hdlc_core_(hdlc),
        channel_(channel),
        rstream_(report_stream),
        report_sca_reply_error_(report_sca_reply_error),
        throw_on_sca_reply_error_(throw_on_sca_reply_error)
      {}

    /** Enable/disable exception throwing on SCA reply error
     */
    void setThrowOnScaReplyError(bool value) { throw_on_sca_reply_error_ = value; }

    /** Get the SCA channel information
     *  @return Currently configured SCA channel
     */
    uint8_t getChannel() const { return channel_; }

    /** Give reference to underlying HDLC core
     *  @return HDLC core reference
     */
    common::HdlcCore& getHdlcCore() const { return hdlc_core_; }

    /** Send and receive packet to/from SCA
     *
     *  @note Sending and receiving is raw, not length masking of received data or
     *        automatic bit(re-)ordering is applied at this stage
     *  @param request Input frame payload
     *  @return Payload of the received frame
     */
    inline HdlcEcFramePayload transceive(HdlcEcFramePayload& request) const
      {
        HdlcEcFramePayload reply;
        hdlc_core_.executeCommand(request, reply);
        return reply;
      }

    /** Send and receive packet to/from SCA
     *
     *  Uses the byte ordering information from the register file information to
     *    potentially apply byte reordering to the Tx and Rx data\n
     *  Uses the Rx length from the register file information to apply
     *    a corresponding mask on the received input data
     *
     *  @note By pinning the TrId to 0x0 here, we rely on the automatic request trid
     *          adjustment of the HdlcCore
     *  @see HdlcCore::request_trid_auto
     *
     *  @param rgf SCA register file definition (command and length part of the request)
     *  @return Payload of the received frame with potentially reordered and masked data field
     */
    inline HdlcEcFramePayload transceive(const sca_rgf_t& rgf, const uint32_t data = 0x0) const
      {
        HdlcEcFramePayload request{0x0, channel_, std::get<1>(rgf), {std::get<0>(rgf)}, adjustByteOrder(rgf, data)};
        HdlcEcFramePayload reply = transceive(request);

        reply.data = adjustByteOrder(rgf, reply.data);
        reply.data &= ((std::get<2>(rgf) == 4) ? 0xffffffff :
                                                 ((uint32_t(1) << std::get<2>(rgf) * 8) - 1));

        processErrorField(request, reply);
        return reply;
      }

    /** Send packet to GBT SCA (do not wait for, or expect, reply)
     *
     *  Primarily used to push configuration packets to GBT SCA if GBTx0 pattern generator mode is (to be)
     *  enabled or disabled
     *
     *  @param request Input frame payload
     */
    inline void transmit(HdlcEcFramePayload& request) const { hdlc_core_.transmitCommand(request); }

    /** Send packet to GBT SCA (do not wait for, or expect, reply)
     *
     *  @param rgf SCA register file definition
     *  @param data Data to be sent
     */
    inline void transmit(const sca_rgf_t& rgf, const uint32_t data = 0x0) const
      {
        HdlcEcFramePayload request{0x0, channel_, std::get<1>(rgf), {std::get<0>(rgf)}, adjustByteOrder(rgf, data)};
        transmit(request);
      }
};



//------------------------------------------------------------------------------
/** GBT SCA ADC base class
 */
class ScaAdc : public Sca
{
  private:
    virtual sca_rgf_t getRgfRdInputSelection() const = 0;
    virtual sca_rgf_t getRgfWrInputSelection() const = 0;
    virtual sca_rgf_t getRgfRdCurrentSource() const = 0;
    virtual sca_rgf_t getRgfWrCurrentSource() const = 0;
    virtual sca_rgf_t getRgfRdData() const = 0;
    virtual sca_rgf_t getRgfRdRawData() const = 0;
    virtual sca_rgf_t getRgfRdGain() const = 0;
    virtual sca_rgf_t getRgfWrGain() const = 0;

  public:
    ScaAdc(common::HdlcCore& hdlc_core) : Sca(hdlc_core, 0x14) {}

    /** Disable all current sources
     */
    void disableCurrentSources() const { transceive(getRgfWrCurrentSource(), 0x0); }

    /** Get input select MUX setting
     *  @return Selected ADC input channel
     */
    uint8_t getInputSelection() const { return transceive(getRgfRdInputSelection()).data; }

    /** Control the input selection MUX
     *  Selects the ADC input, bit position corresponds to channel number
     *  @param idx ADC input channel
     *  @return Selected channel
     */
    uint8_t setInputSelection(uint8_t idx) const
      {
        transceive(getRgfWrInputSelection(), static_cast<uint32_t>(idx) & 0x1f);
        return getInputSelection();
      }

    /** Get the currently enabled current source
     *  @return Read-back current enable register value
     */
    uint32_t getCurrentSource() const { return transceive(getRgfRdCurrentSource()).data; }

    /** Set the current source enable
     *  @param idx ADC input pin index
     *  @return Read-back current enable register value
     */
    uint32_t setCurrentSource(uint8_t idx) const
      {
        transceive(getRgfWrCurrentSource(), (uint32_t(1) << idx));
        return getCurrentSource();
      }

    /** Get value of latest conversion\n
     *  Value includes offset and gain correction
     *  @return ADC data
     */
    uint32_t getData() const { return transceive(getRgfRdData()).data; }

    /** Get raw value of conversion\n
     *  Raw value i.e. no offset and gain correction
     *  @return ADC raw data
     */
    uint32_t getRawData() const { return transceive(getRgfRdRawData()).data; }

    /** Get gain calibration value
     *  @return Read-back gain calibration value
     */
    uint16_t getGain() const { return transceive(getRgfRdGain()).data; }

    /** Set the gain calibration value
     *  @param Value to set
     *  @return Read-back gain calibration register value
     *  @note Gain calibration register widths: SCAv1 13bit, SCAv2 16bit,
     *        we do not distinguish between different widths here
     */
    uint16_t setGainCalib(uint16_t value) const
      {
        transceive(getRgfWrGain(), value & 0xffff);
        return getGain();
      }

    /** Initialize ADC\n
     *    e.g., disable all current sources
     *  @return 0 in case of success, != 0 in case of error
     */
    virtual uint32_t init() const = 0;

    /** Initiate a conversion of the currently selected input
     *  @return Conversion result
     */
    virtual uint16_t runConversion() const = 0;

    /** Sample ADC input
     *  @param idx Index of the input to sample
     *  @param enableCs Enable corresponding current source
     *  @return 0xffff in case of illegal input idx, or, in case of successful conversion:\n
     *          - [11..0] Conversion result
     *          - [12]    Voltage exceeded during conversion flag
     *          - [15..13] Zero
     */
    uint16_t sample(uint8_t idx, bool enableCs = false) const
      {
        uint16_t v(0xffff);

        if (idx <= 0x1f) {
          setInputSelection(idx);
          if (enableCs)
            setCurrentSource(idx);

          v = runConversion();

          adcdebug(__func__ << std::hex
                            << " : INPUT SELECT = 0x" << static_cast<uint32_t>(getInputSelection())
                            << " / CURRENT ENABLE = 0x" << getCurrentSource()
                            << " / SAMPLE = 0x" << static_cast<uint32_t>(v) << std::dec)
          if (enableCs)
            disableCurrentSources();
        }

        return v;
      }


    /** Dump relevant ADC registers
     */
    friend std::ostream& operator<<(std::ostream& os, const ScaAdc& f)
      {
        os << "GBT-SCA ADC status\n"
           << std::hex << std::right << std::setfill('0')
           << "  gain            0x" << std::setw(4) << f.getGain() << "\n"
           << "  current enable  0x" << std::setw(8) << f.getCurrentSource() << "\n"
           << std::dec << std::left << std::setfill(' ')
           << "  input select    0x" << std::dec << static_cast<uint32_t>(f.getInputSelection()) << "\n"
           << "";
        return os;
      }
};


/** GBT SCA ADC v1 interfacing
 */
class ScaAdcVersion1 : public ScaAdc
{
  private:
    virtual sca_rgf_t getRgfRdInputSelection() const final { return CMD_R_INSEL; }
    virtual sca_rgf_t getRgfWrInputSelection() const final { return CMD_W_INSEL; }
    virtual sca_rgf_t getRgfRdCurrentSource()  const final { return CMD_R_CUREN; }
    virtual sca_rgf_t getRgfWrCurrentSource()  const final { return CMD_W_CUREN; }
    virtual sca_rgf_t getRgfRdData()           const final { return CMD_R_DATA; }
    virtual sca_rgf_t getRgfRdRawData()        const final { return CMD_R_RAW; }
    virtual sca_rgf_t getRgfRdGain()           const final { return CMD_R_GAINCALIB; }
    virtual sca_rgf_t getRgfWrGain()           const final { return CMD_W_GAINCALIB; }

    /* ADC register file information, from SCA manual v8.0 p60ff */
    const sca_rgf_t CMD_W_GO        {0xB2, 4, 2, BO_REVERSED};
    const sca_rgf_t CMD_W_INSEL     {0x30, 4, 2, BO_REVERSED};
    const sca_rgf_t CMD_R_INSEL     {0x31, 1, 4, BO_REVERSED};
    const sca_rgf_t CMD_W_CUREN     {0x40, 4, 2, BO_REVERSED};
    const sca_rgf_t CMD_R_CUREN     {0x41, 1, 4, BO_REVERSED};
    const sca_rgf_t CMD_R_DATA      {0x21, 1, 4, BO_REVERSED};
    const sca_rgf_t CMD_R_RAW       {0x51, 1, 4, BO_REVERSED};
    const sca_rgf_t CMD_W_GAINCALIB {0x70, 2, 1, BO_REVERSED};
    const sca_rgf_t CMD_R_GAINCALIB {0x71, 1, 4, BO_REVERSED};

    const uint32_t cfg_adc_gaincalib_{0x0d80};  /**< ADC gain calibration (guessed value) */

  public:
    ScaAdcVersion1(common::HdlcCore& hdlc_core) : ScaAdc(hdlc_core) {}

    /** Initialize
     *  @see ScaAdc::init
     */
    virtual uint32_t init() const final
      {
        disableCurrentSources();
        setGainCalib(cfg_adc_gaincalib_);
        return 0;
      }

    /** Initialize conversion
     *  @return Conversion result
     */
    virtual uint16_t runConversion() const final
      {
        transceive(CMD_W_GO);
        return transceive(CMD_R_RAW).data;
      }
};


/** GBT SCA ADC v2 interfacing
 */
class ScaAdcVersion2 : public ScaAdc
{
  private:
    virtual sca_rgf_t getRgfRdInputSelection() const final { return CMD_R_MUX; }
    virtual sca_rgf_t getRgfWrInputSelection() const final { return CMD_W_MUX; }
    virtual sca_rgf_t getRgfRdCurrentSource()  const final { return CMD_R_CURR; }
    virtual sca_rgf_t getRgfWrCurrentSource()  const final { return CMD_W_CURR; }
    virtual sca_rgf_t getRgfRdData()           const final { return CMD_R_DATA; }
    virtual sca_rgf_t getRgfRdRawData()        const final { return CMD_R_RAW; }
    virtual sca_rgf_t getRgfRdGain()           const final { return CMD_R_GAIN; }
    virtual sca_rgf_t getRgfWrGain()           const final { return CMD_W_GAIN; }

    /* Register definitions for SCA V2, from SCA manual v8.2 p53*/
    const sca_rgf_t CMD_W_GO   {0x02, 4, 2, BO_REVERSED};
    const sca_rgf_t CMD_W_MUX  {0x50, 4, 1, BO_REVERSED};
    const sca_rgf_t CMD_R_MUX  {0x51, 1, 4, BO_REVERSED};
    const sca_rgf_t CMD_W_CURR {0x60, 4, 1, BO_REVERSED};
    const sca_rgf_t CMD_R_CURR {0x61, 1, 4, BO_REVERSED};
    const sca_rgf_t CMD_W_GAIN {0x10, 4, 1, BO_REVERSED};
    const sca_rgf_t CMD_R_GAIN {0x11, 1, 4, BO_REVERSED};
    const sca_rgf_t CMD_R_DATA {0x21, 1, 4, BO_REVERSED};
    const sca_rgf_t CMD_R_RAW  {0x31, 1, 4, BO_REVERSED};
    const sca_rgf_t CMD_R_OFS  {0x41, 1, 4, BO_REVERSED};

  public:
    ScaAdcVersion2(common::HdlcCore& hdlc_core) : ScaAdc(hdlc_core) {}

    /** Initialize
     *  @see ScaAdc::init
     */
    virtual uint32_t init() const final
      {
        disableCurrentSources();
        return 0;
      }

    /** Initialize conversion
     *  @return Conversion result
     */
    virtual uint16_t runConversion() const final { return transceive(CMD_W_GO, 1).data; }

    /** Get offset value determined during last conversion (SCAv2 only)
     *  @return Value of ADC offset register
     */
    uint32_t getOffset() const { return transceive(CMD_R_OFS).data; }
};


//------------------------------------------------------------------------------
/** GBT SCA DAC interfacing
 */
class ScaDac : public Sca
{
  public:
    ScaDac(common::HdlcCore& hdlc_core) : Sca(hdlc_core, 0x15) {}

    /** Get current register value for DAC output port [0..3]
     *
     *  @param port_idx Index of the DAC output port (A=0, ..., D=3)
     *  @return Current register value
     */
    inline uint8_t get(uint8_t port_idx) {
        assert(port_idx <= 3);

        const sca_rgf_t* read_cmds[] = {&CMD_R_A, &CMD_R_B, &CMD_R_C, &CMD_R_D};
        return static_cast<uint8_t>(transceive(*read_cmds[port_idx]).data);
      }

    /** Set register value for DAC output port [0..3]
     *
     *  @param port_idx Index of the DAC output port (A=0, ..., D=3)
     *  @param value New value to set [0x0 .. 0xff]
     *  @return Read-back register value
     */
    inline uint8_t set(uint8_t port_idx, uint8_t value) {
        assert(port_idx <= 3);

        const sca_rgf_t* write_cmds[] = {&CMD_W_A, &CMD_W_B, &CMD_W_C, &CMD_W_D};
        transceive(*write_cmds[port_idx], value);
        return get(port_idx);
      }

  private:
    const sca_rgf_t CMD_W_A {0x10, 4, 1, BO_NORMAL};
    const sca_rgf_t CMD_R_A {0x11, 1, 4, BO_NORMAL};
    const sca_rgf_t CMD_W_B {0x20, 4, 1, BO_NORMAL};
    const sca_rgf_t CMD_R_B {0x21, 1, 4, BO_NORMAL};
    const sca_rgf_t CMD_W_C {0x30, 4, 1, BO_NORMAL};
    const sca_rgf_t CMD_R_C {0x31, 1, 4, BO_NORMAL};
    const sca_rgf_t CMD_W_D {0x40, 4, 1, BO_NORMAL};
    const sca_rgf_t CMD_R_D {0x41, 1, 4, BO_NORMAL};
};

//------------------------------------------------------------------------------
/** GBT SCA GPIO interfacing
 */
class ScaGpio : public Sca
{
  public:
    ScaGpio(common::HdlcCore& hdlc_core) : Sca(hdlc_core, 0x2) {}

    /** Get GPIO direction
     *  @return Current DIRECTION value
     */
    uint32_t getDirection() const { return transceive(CMD_R_DIRECTION).data; }

    /** Set GPIO direction
     *  @param value Desired value of the DIRECTION register
     *  @return Current DIRECTION value
     */
    uint32_t setDirection(uint32_t value) const
      {
        transceive(CMD_W_DIRECTION, value);
        return getDirection();
      }

    /** Read register DATAIN
     *  @return Current DATAIN value
     */
    uint32_t getDataIn() const { return transceive(CMD_R_DATAIN).data; }

    /** Read register DATAOUT
     *  @return Current DATAOUT value
     */
    uint32_t getDataOut() const { return transceive(CMD_R_DATAOUT).data; }

    /** Write register DATAOUT
     *  @param value Desired value of the DATAOUT register
     *  @return Current DATAOUT value
     */
    uint32_t setDataOut(uint32_t value) const
      {
        transceive(CMD_W_DATAOUT, value);
        return getDataOut();
      }

    /** Dump GPIO registers
     */
    friend std::ostream& operator<<(std::ostream& os, const ScaGpio& f)
      {
        os << "GBT-SCA GPIO status\n"
           << std::hex << std::right << std::setfill('0')
           << "  DIR  0x" << std::setw(8) << f.getDirection() << "\n"
           << "  DIN  0x" << std::setw(8) << f.getDataIn() << "\n"
           << "  DOUT 0x" << std::setw(8) << f.getDataOut() << "\n"
           << std::dec << std::left << std::setfill('0');
        return os;
      }

    /* GPIO register file information, from GBT SCA manual v8.0 p51ff */
    const sca_rgf_t CMD_R_DATAIN    {0x01, 1, 4, BO_REVERSED};
    const sca_rgf_t CMD_W_DATAOUT   {0x10, 4, 2, BO_REVERSED};
    const sca_rgf_t CMD_R_DATAOUT   {0x11, 1, 4, BO_REVERSED};
    const sca_rgf_t CMD_W_DIRECTION {0x20, 4, 2, BO_REVERSED};
    const sca_rgf_t CMD_R_DIRECTION {0x21, 1, 4, BO_REVERSED};
};



//------------------------------------------------------------------------------
/** GBT SCA ID interfacing
 *    technically, this uses the channel ID of the SCA ADC, but logically,
 *    it doesn't belong with the ADC
 */
class ScaId : public Sca
{
  protected:
    uint32_t version_;
    uint32_t id_;

  public:
    ScaId(common::HdlcCore& hdlc_core) :
        Sca(hdlc_core, 0x14, false, false), /* <- make sure throw_on_sca_reply_error is disabled here! */
        version_(0x0),
        id_(0x0)
      {
        setVersion(read());
      }

    friend std::ostream& operator<<(std::ostream& os, const ScaId& f)
      {
        os << "GBT-SCA ID information\n"
           << "  id 0x" << std::hex << std::right << std::setfill('0') << std::setw(8) << f.getId()
           << " / version " << std::dec << std::left << std::setfill(' ') << f.getVersion()
           << "\n";
        return os;
      }

    /** Read the SCA chip ID and update the internal ID information
     *
     *  v1 SCAs do not contain a valid ID number, data read is 0x0 or SCA_ID_DUMMY_V1
     *  v2 SCAs do contain a valid ID number that is 1..2^24-1
     *
     *  @return 0x0 in case of an error/something stange, otherwise the received SCA ID for
     *          v2 chips or SCA_ID_DUMMY_V1 for v1 chips
     *
     *  @see SCA_ID_DUMMY_V1
     */
    uint32_t read()
      {
        // v1 and v2 SCAs require different CMD fields the read the chip ID
        //
        // It was tested that reading the v1 CMD on v2 chips results in an
        //   'invalid command request' error in the HDLC reply frame
        // Same for v2 CMD read on v1 chips
        //
        // -> test both to determine which SCA version we have
        //    (we switched of 'throw_on_sca_reply_error' when initializing the SCA base class)
        //
        HdlcEcFramePayload f1 = transceive(CMD_R_ID_V1, 0x1);
        HdlcEcFramePayload f2 = transceive(CMD_R_ID_V2, 0x1);

        if ((f1.error & HdlcEcFramePayload::ERR_INV_COMMAND) == 0x0) {
          id_ = SCA_ID_DUMMY_V1;
        }
        else if ((f2.error & HdlcEcFramePayload::ERR_INV_COMMAND) == 0x0) {
          id_ = f2.data;
        }

        return id_;
      }

    /** Get internally stored chip ID
     *  @see read
     *  @return SCA chip IS
     */
    uint32_t getId() const { return id_; }

    /** Determine the SCA chip version based on the chip ID and update internal information
     *  @param sca_chip_id Chip ID read from SCA
     *  @return SCA version
     */
    inline uint32_t setVersion(uint32_t sca_chip_id)
      {
        if (sca_chip_id == SCA_ID_INVALID)
          version_ = SCA_VINVALID;
        else if (sca_chip_id == SCA_ID_DUMMY_V1)
          version_ = SCA_V1;
        else
          version_ = SCA_V2;

        return getVersion();
      }

    /** Get SCA chip version
     *  @return SCA version (internally stored, determined from SCA ID)
     */
    inline uint32_t getVersion() const { return version_; }

    /** ID register file information, from GBT SCA manual v8.2 p21) */
    const sca_rgf_t CMD_R_ID_V1 {0x91, 1, 3, BO_REVERSED};
    const sca_rgf_t CMD_R_ID_V2 {0xD1, 1, 3, BO_REVERSED};

    static const uint32_t SCA_VINVALID {0x0};    /**< Constant Invalid SCA revision */
    static const uint32_t SCA_V1       {0x1};    /**< Constant SCA revision 1 */
    static const uint32_t SCA_V2       {0x2};    /**< Constant SCA revision 2 */

    static const uint32_t SCA_ID_INVALID  {0x0};      /**< Constant for SCA invalid ID (should never be seen in field) */
    static const uint32_t SCA_ID_DUMMY_V1 {0xffffff}; /**< Constant for SCA v1 dummy ID (as they do not possess a valid ID */
};



//------------------------------------------------------------------------------
/** GBT SCA basic configuration
 */
class ScaBasic : public Sca
{
  public:
    ScaBasic(common::HdlcCore& hdlc_core) : Sca(hdlc_core, 0x0) {}

    friend std::ostream& operator<<(std::ostream& os, const ScaBasic& f)
      {
        os << "GBT-SCA basic configuration\n"
           << std::hex << std::right << std::setfill('0')
           << "  CRB 0x" << std::setw(8) << ((f.transceive(f.CMD_R_CRB)).data) << "\n"
           << "  CRC 0x" << std::setw(8) << ((f.transceive(f.CMD_R_CRC)).data) << "\n"
           << "  CRD 0x" << std::setw(8) << ((f.transceive(f.CMD_R_CRD)).data) << "\n"
           << std::dec << std::left << std::setfill('0');
        return os;
      }

    const sca_rgf_t CMD_W_CRA {0x0, 1, 1, BO_NORMAL};
    const sca_rgf_t CMD_R_CRA {0x1, 1, 1, BO_NORMAL};
    const sca_rgf_t CMD_W_CRB {0x2, 1, 1, BO_NORMAL};
    const sca_rgf_t CMD_R_CRB {0x3, 1, 1, BO_NORMAL};
    const sca_rgf_t CMD_W_CRC {0x4, 1, 1, BO_NORMAL};
    const sca_rgf_t CMD_R_CRC {0x5, 1, 1, BO_NORMAL};
    const sca_rgf_t CMD_W_CRD {0x6, 1, 1, BO_NORMAL};
    const sca_rgf_t CMD_R_CRD {0x7, 1, 1, BO_NORMAL};

    /** Test if SCA channel is enabled
     *  @param channel Channel number [0x01, .., 0x15]. Channel 0 is always enabled (this is this SCA configuration)
     *  @return Unequal zero if channel is enabled, zero otherwise
     */
    inline uint8_t channelEnabled(uint8_t channel) const {
        assert(channel <= 0x15);
        const sca_rgf_t& rgf = (channel < 8) ? CMD_R_CRB : ((channel < 16) ?  CMD_R_CRC : CMD_R_CRD);
        return ((transceive(rgf).data >> (channel % 8)) & 0x1) | (channel == 0x0);
      }
};

} // namespace gbt

#undef scadebug
#undef adcdebug
