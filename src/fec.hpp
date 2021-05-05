#pragma once

#include <fstream>
#include <array>
#include <cassert>
#include <chrono>
#include <thread>
#include <cmath>

#include "bit_manipulation.hpp"
#include "gbt_sca.hpp"
#include "gbt_sca_i2c.hpp"
#include "fec_factory.hpp"
#include "sampa.hpp"

/** Class encapsulating definitions and conversions related to the ADC
 *  on the FEC
 */
class FecAdc
{
  protected:
    /** ADC port definition
     *
     *   0 : pointer to conversion function from raw to processed value\n
     *   1 : current source enable for this port (true/false)\n
     *   2 : unit string\n
     *   3 : description string\n
     */
    using port_def_t = std::tuple<float(*)(uint16_t, float), bool, const std::string, const std::string>;

    /** ADC port map definition
     */
    using port_map_t = std::map<uint8_t, port_def_t>;

    /**< Calibration resistor input on SCA ADC */
    static const uint8_t calib_resistor_input_{18};

    /** ADC result type definition
     *
     *   0: Converted value\n
     *   1: Unit string (of converted value)\n
     *   2: Input description\n
     *   3: Measured ADC raw value\n
     */
    using result_t = std::tuple<float, const std::string, const std::string, uint16_t>;

    /** Calculate input currents (from current monitor ADC inputs)
     *  @param input0 ADC port number (connector)
     *  @param input1 ADC port number (in)
     *  @param r      Resistor value
     *  @param adc    Reference to SCA ADC instance
     *  @param desc   Output description string
     */
    static result_t calcCurrent(const uint8_t input0, const uint8_t input1, const float r,
                                const gbt::ScaAdc& adc, const std::string desc) {
        auto p0 = getInput(input0, adc);
        auto p1 = getInput(input1, adc);
        auto diff_raw = std::get<3>(p0) - std::get<3>(p1);
        auto i = abs(std::get<0>(p0) - std::get<0>(p1))/r;
        return std::make_tuple(i, "A", desc, diff_raw);
      }

  public:

    /** Map that defines all ADC input ports, together with conversion functions, etc
     *  @see FecAdc::port_def_t
     */
    static const port_map_t ports_;

    /** Get converted and raw result of ADC input
     *  @param input ADC input number
     *  @param adc Reference to ScaAdc instance
     *  @param applyCsCorrection Apply correction for inputs with current source (Larsistor)
     *  @return Tuple with ADC results
     *  @see result_t
     *  @throws std::out_of_range if key 'input' is not found in ports definition
     */
    static result_t getInput(const uint8_t input, const gbt::ScaAdc& adc, bool applyCsCorrection = true) {
         uint16_t adc_raw = adc.sample(input, std::get<1>(ports_.at(input)));
         float corr(1.);

         if (applyCsCorrection && (input != calib_resistor_input_) && std::get<1>(ports_.at(input))) {
           corr = convCalibrationResistor(adc.sample(calib_resistor_input_, std::get<1>(ports_.at(input)))) / 100.0;
         }

         return std::make_tuple(std::get<0>(ports_.at(input))(adc_raw, corr),
                                std::get<2>(ports_.at(input)),
                                std::get<3>(ports_.at(input)),
                                adc_raw);
       }

    /** Calculate 2r25 input current
     *  @return Tuple with calulated results
     *  @see result_t
     *  @see calcCurrent
     */
    static result_t calcCurrent2r25(const gbt::ScaAdc& adc)
      { return calcCurrent(19, 20, .1, adc, "Current 2r25"); }

    /** Calculate 3r25 input current
     *  @return Tuple with calulated results
     *  @see result_t
     *  @see calcCurrent
     */
    static result_t calcCurrent3r25(const gbt::ScaAdc& adc)
      { return calcCurrent(21, 22, .1, adc, "Current 3r25"); }

    /** Convert voltage reading of current source calibration resistor
     *
     *   @param adc_raw Raw ADC value
     *   @param corr Correction factor (unused)
     *   @return Converted current (in uA)
     */
    static float convCalibrationResistor(uint16_t adc_raw, float corr = 1.0) {
        return corr*1e3*adc_raw/4096.;
      }

    /** Convert voltage reading of SCA temperature sensor to actual temperature
     *
     *   Constants m, c are extracted from figure 11.2 in SCA manual v8.2, p52
     *   @param adc_raw Raw ADC value
     *   @param corr Correction factor
     *   @return Converted temperature (in ºC)
     */
    static float convScaTemp(uint16_t adc_raw, float corr = 1.0) {
        const float m(-1.86e-3);
        const float c(0.7175);
        return corr*((float(adc_raw)/4096) - c) / m;
      }

    /** Convert voltage reading to VTRx photo current
     *
     *  From Versatile Link Application Note v2.7, p13 follows i_avg = 1/r1*(-1*(r1+r2)/r2*v_rssi+v_cc)\n
     *  From schematic follows v_cc=2.5V, r1=4.99e3, r2=3.4e3
     *
     *  @param adc_raw Raw ADC value
     *  @param corr Correction factor
     *  @return VTRx photo current (in mA)
     */
    static float convVtrx(uint16_t adc_raw, float corr = 1.0) {
        const float v_cc(2.5);
        const float r1(4.99e3);
        const float r2(3.4e3);
        float v_rssi = corr*float(adc_raw)/4096;
        return 1e3/r1 * (-1 * (r1+r2)/r2 * v_rssi + v_cc);
      }

    /** Convert RTD reading to temperature
     *
     *  TODO: Rev1 should have 100 ohm (PTS060301B100RP100) according to schematics\n
     *        But it is most likely 1kohm (PTS1206M1B1K00P100) as in schematics of rev1a
     *
     *  @param adc_raw Raw ADC value
     *  @param corr Correction factor
     *  @return RTD temperature (in ºC)
     */
    static float convRtd(uint16_t adc_raw, float corr = 1.0) {
        const float i_cs = corr*100e-6;
        const float r0(1e3);
        const float c_a(3.9083e-3);
        const float c_b(-5.775e-7);
        float v_in = float(adc_raw)/4096;
        return (-c_a + sqrt((c_a*c_a) - (4.*c_b*(1.-(v_in/(i_cs*r0)))))) / (2.*c_b);
      }

    friend std::ostream& operator<<(std::ostream& os, const result_t& r);
};

/** Overload output operator for FecAdc::result_t
 *  @param os The output stream to print to
 *  @param r Reference to FecAdc::result_t
 *  @return Output stream os
 */
std::ostream& operator<<(std::ostream& os, const FecAdc::result_t& r);



/** FEC class encapsulating
 *    GPIO, I2C, ...
 */
class Fec : public common::BitManipulation
{
  public:
    static const uint8_t n_gbtx_{2};
    static const uint8_t n_sampa_{5};

    static const uint8_t cfg_i2c_addr_gbtx_[n_gbtx_];   /**< I2C addresses of GBTx0 and GBTx1 */
    static const uint8_t cfg_i2c_addr_sampa_[n_sampa_]; /**< I2C addresses of the SAMPAs */

  protected:
    std::unique_ptr<gbt::ScaBasic> sca_basic_;
    std::unique_ptr<gbt::ScaId> sca_id_;
    std::unique_ptr<gbt::ScaGpio> sca_gpio_;
    std::unique_ptr<gbt::ScaAdc> sca_adc_;

    std::array<std::unique_ptr<gbt::ScaI2c>, n_gbtx_>  i2c_gbtx_;
    std::array<std::unique_ptr<gbt::ScaI2c>, n_sampa_> i2c_sampa_;

    std::array<std::unique_ptr<Sampa>, n_sampa_> sampa_;

    const uint8_t REG_SAMPA_SOCFG  {0x12};

    FecRevision::rev_tuple_t rev_info_;   /**< Holds (determined) FEC & SAMPA revision */
    FecDefaults defaults_;
    //const float i_cs_; /**< Holds calibrated SCA current source value */
    uint8_t verbosity_;

  private:
    /** Get ADC reading and convert it to user-ready value
     *  @param adc_pin ADC input pin to sample
     *  @param scaling Scaling parameter
     *  @param raw Show ADC raw value instead of converted value. If true, scaling param is ignored
     *  @return Raw ADC value if raw = true, else (ADC read value) / 4096 * scaling
     */
    inline float adcConversion(uint8_t adc_pin, float scaling, bool current_source = false, bool raw = false) const
      {
        return static_cast<float>(sca_adc_->sample(adc_pin, current_source)) * (raw ? 1. : (1. / 4096. * scaling));
      }

  public:
    Fec(Fec const &) = delete;
    Fec &operator=(Fec const &) = delete;

    Fec(common::HdlcCore& hdlc, uint8_t verbosity = 0) :
        sca_basic_(new gbt::ScaBasic(hdlc)),
        sca_id_(new gbt::ScaId(hdlc)),
        sca_gpio_(new gbt::ScaGpio(hdlc)),
        sca_adc_((sca_id_->getVersion() == gbt::ScaId::SCA_V1) ?
                   static_cast<gbt::ScaAdc*>(new gbt::ScaAdcVersion1(hdlc)) :
                   static_cast<gbt::ScaAdc*>(new gbt::ScaAdcVersion2(hdlc))),
        rev_info_(FecRevision::get(sca_id_->getId())),
        defaults_(FecDefaultsFactory::makeDefaults(FecRevision::getSampaRev(rev_info_))),
        verbosity_(verbosity)
      {
        for (uint8_t i = 0; i < n_gbtx_; i++)
          i2c_gbtx_[i].reset(new gbt::ScaI2cGbtx(hdlc, i, cfg_i2c_addr_gbtx_[i]));
        for (uint8_t i = 0; i < n_sampa_; i++) {
          i2c_sampa_[i].reset(new gbt::ScaI2cSampa(hdlc, i + 4, cfg_i2c_addr_sampa_[i]));
          sampa_[i].reset(new Sampa(static_cast<gbt::ScaI2c&>(*i2c_sampa_[i])));
        }
      }


    /** Initialize FEC\n
     *    - Enable SCA channels via CRB, CRC, CRD
     *    - Initialize GPIO
     *      - Set DATAOUT to 0
     *      - Set DIRECTION to cfg_direction_
     *    - Initialize ADC
     *      - Set gain calibration to cfg_adc_gaincalib_
     *      - Select default input
     *    - Initialize I2C interfaces
     *      - Set port frequency
     *
     *    - TODO: Add some specific configuration options based on the FEC revision?
     */
    void init();

    /** Set class verbosity\n
     *  Will write some information to std::cout if verbosity_ > 0
     */
    uint8_t setVerbosity(uint8_t v)
      {
        verbosity_ = v;
        return verbosity_;
      }

    /** Get pointer to GBT SCA ID instance
     *  @note Object lifetime managed by FEC class
     *  @return ID instance pointer
     */
    gbt::ScaId& getScaId() const { return *(sca_id_.get()); }

    /** Get pointer to GBT SCA GPIO instance
     *  @note Object lifetime managed by FEC class
     *  @return GPIO instance pointer
     */
    gbt::ScaGpio& getScaGpio() const { return *(sca_gpio_.get()); }

    /** Get pointer to GBT SCA basic instance
     *  @note Object lifetime managed by FEC class
     *  @return SCA basic instance pointer
     */
    gbt::ScaBasic& getScaBasic() const { return *(sca_basic_.get()); }

    /** Get pointer to the GBT SCA ADC instance
     *  @note Object lifetime managed by FEC class
     *  @return ADC instance pointer
     */
    gbt::ScaAdc& getScaAdc() const { return *(sca_adc_.get()); }

    /** Get pointer to GBT SCA I2C for GBTx instance
     *  @note Object lifetime managed by FEC class
     *  @param gbtx_idx GBTx index, valid are [0, 1]
     *  @return SCA I2C GBTx instance pointer
     */
    gbt::ScaI2c& getScaI2cGbtx(uint8_t gbtx_idx = 0) const
      {
        assert((gbtx_idx | 0x1) == 0x1);
        return *(i2c_gbtx_[gbtx_idx & 0x1].get());
      }

    /** Get reference to GBT SCA I2C for SAMPA instance
     *  @note Object lifetime managed by FEC class
     *  @param sampa_idx SAMPA index, valid are [n_sampa_-1..0]
     *  @return SCA I2C SAMPA reference
     *  @see n_sampa_
     */
    gbt::ScaI2c& getScaI2cSampa(uint8_t sampa_idx) const
      {
        assert(sampa_idx <= n_sampa_);
        return *(i2c_sampa_[sampa_idx].get());
      }

    /** Get reference to SAMPA instance
     * @note Object lifetime manage by FEC class
     *  @param sampa_idx SAMPA index, valid are [n_sampa_-1..0]
     *  @return SAMPA instance reference
     *  @see n_sampa_
     */
    Sampa& getSampa(uint8_t sampa_idx) const
      {
        assert(sampa_idx <= n_sampa_);
        return *(sampa_[sampa_idx].get());
      }

    /** Switch individual SAMPA on/off
     *  @param sampa_idx SAMPA to switch
     *  @param value     Desired power value: 0|1 for off|on
     */
    inline void sampaPwr(uint32_t sampa_idx, uint8_t value) const
      {
         assert(sampa_idx <= n_sampa_);
         sca_gpio_->setDataOut(setBit(sca_gpio_->getDataIn(), sampa_idx, value));
      }

    /** Control SAMPA power on SCA GPIO[4..0]
     *  @param value Value to set on the selected SAMPA power GPIO pins (on/off)
     *  @param mask Mask [4..0] controlling which GPIO pins [4..0] actually are altered
     *  @param wait Give wait interval to wait inbetween performing power-related
     *          actions on each of the active SAMPAs in the SAMPA mask (in milliseconds)
     *  @return Current power status (0x1f if all SAMPAs powered)
     */
    uint32_t sampaPwrCtrl(uint32_t value, uint32_t mask = 0x1f, uint32_t wait = 100) const;

    /** Switch multiple SAMPAs on
     *  @param mask Mask which SAMPAs to switch on
     *  @param wait Wait period between switching individual SAMPAs (in ms)
     *  @return Current power status
     *  @see sampaPwrCtrl
     */
    inline uint32_t sampaPwrOn(uint32_t mask = 0x1f, uint32_t wait = 100) const
      { return sampaPwrCtrl(0x1, mask, wait); }

    /** Switch multiple SAMPAs off
     *  @param mask Mask which SAMPAs to switch off
     *  @param wait Wait period between switching individual SAMPAs (in ms)
     *  @return Current power status
     *  @see sampaPwrCtrl
     */
    inline uint32_t sampaPwrOff(uint32_t mask = 0x1f, uint32_t wait = 100) const
      { return sampaPwrCtrl(0x0, mask, wait); }

    /** Get SAMPA power status
     *  @return Vreg power enable, i.e. value of GPIO pins [4..0]
     */
    inline uint32_t sampaPwrStatus() const { return getField(sca_gpio_->getDataIn(), 0, 5); }

    /** Enable SAMPA e-links
     *  @param sampa_mask Mask of SAMPAs to apply this setting to
     *  @param n Number of enabled e-links
     *  @return True if setting applied correctly
     */
    bool sampaEnableElinks(uint8_t n = 11, uint32_t sampa_mask = 0x1f) const;

    /** Scans for SAMPAs on I2C lines and allows to print information about found SAMPAs
     *  @param os Pointer to output stream or nullptr if printing is not desired
     *  @return Mask of active SAMPAs found
     */
    uint32_t sampaI2cScan(std::ostream* const os = nullptr) const;

    /** Get SAMPA clock configuration
     * @return Sampa clock configuration
     * @note Available on FEC >= rev1 only
     */
    inline uint32_t sampaGetClkCfg() const { return getField(sca_gpio_->getDataIn(), 23, 7); }

    /** Set SAMPA clock configuration
     * @param value Sampa clock configuration
     * @return Sampa clock configuration
     * @note Available on FEC >= rev1 only
     */
    inline uint32_t sampaSetClkCfg(uint32_t value) const
      {
        sca_gpio_->setDataOut(setField(sca_gpio_->getDataIn(), 23, 7, value));
        return sampaGetClkCfg();
      }

    /** Read FEC board ID
     *  @return Board ID, i.e. value of GPIO pins [21..10]
     */
    inline uint16_t boardId() const { return getField(sca_gpio_->getDataIn(), 10, 12); }

    /** Configure GBTx via SCA I2C\n
     *    Configuration is read from file
     *  @param gbtx_idx GBTx index
     *  @param cfg_file Path to GBTx configuration file (in standard format with one register value per line)
     *  @throws std::runtime_error
     */
    void gbtxConfigure(uint8_t gbt_idx, const std::string& cfg_file) const;

    /** Dump GBTx configuration to stream
     *  @param gbt_idx GBTx index
     *  @param os Output stream
     */
    void gbtxDumpConfiguration(uint8_t gbt_idx, std::ostream& os = std::cout) const;

    /** Reset GBTx via SCA I2C\n
     *    Pulses bits 5..3 of GTBx register 50 (wdogCtr0, i2cResetRx[C, B, A])
     *    to reset RX control state machine and RX data path
     *  @param GBT index [0, 1]
     */
    void gbtxReset(uint8_t gbt_idx) const;

    /** Scans addresses for reply of GBTx via I2C and allows to print information about the
     *    scanning process
     *  @param gbt_idx GBTx I2C interface to probe [0,1]
     *  @param os Pointer to output stream or nullptr if printing is not desired
     *  @return Address of the GBTx found, -1 in case no GBTx is found on given I2C interface
     */
    int32_t gbtxI2cScan(uint8_t gbt_idx = 0, std::ostream* const os = nullptr, bool verbose = false) const;

    /** Print FEC board ID string
     *  @param Stream to output to
     */
    void printIdString(std::ostream& os) const
      {
        uint32_t board_id = static_cast<uint32_t>(boardId());
        os << "FEC #" << board_id << " [0x" << std::hex << board_id << std::dec << "]"
           << " [" << FecRevision::getFecName(FecRevision::getFecRev(rev_info_)) << "]\n";
      }

    /** Measure and print conversion result of all defined ADC inputs
     *  @see FecAdc::port_def_t
     *  @see FecAdc::result_t
     */
    void adcMeasureAll(std::ostream& os = std::cout)
      {
        const gbt::ScaAdc& adc = *(sca_adc_.get());
        for (auto& port : FecAdc::ports_)
          os << FecAdc::getInput(port.first, adc) << "\n";
        os << FecAdc::calcCurrent2r25(adc) << "\n"
           << FecAdc::calcCurrent3r25(adc) << "\n";
      }

    /** Measure and print conversion of a given ADC input
     *  @param input ADC input number
     */
    void adcMeasureInput(uint8_t input)
      {
        try {
          std::cout << FecAdc::getInput(input, *(sca_adc_.get())) << "\n";
        }
        catch (std::out_of_range) {
          std::cerr << "ERROR: ADC input " << static_cast<uint32_t>(input) << " is not defined\n";
        }
      }


    /** Print FEC status summary
     */
    friend std::ostream& operator<<(std::ostream& os, const Fec& f)
      {
        f.printIdString(os);
        os << "  GBTx0           I2C addr " << f.getScaI2cGbtx(0).getSlaveAddress() << " (sw_cfg)\n"
           << "  GBTx1           I2C addr " << f.getScaI2cGbtx(1).getSlaveAddress() << " (sw_cfg)\n"
           << "  SCA             id 0x" << std::hex << f.sca_id_->getId() << " -> v" << std::dec << f.sca_id_->getVersion() << "\n"
           << "  SAMPAs          " << FecRevision::getSampaName(FecRevision::getSampaRev(f.rev_info_)) << "\n"
           << "  SAMPAs powered  0x" << std::hex << static_cast<uint32_t>(f.sampaPwrStatus()) << std::dec << "\n"
           << "";
        return os;
      }
};
