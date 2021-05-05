#include <iostream>
#include <iomanip>
#include <cassert>

#include "gbt_sca_i2c_adc.hpp"
#include "sampa.hpp"
#include "bit_manipulation.hpp"

/** Class wrapping relevant functionality of the SAMPA tester */
class SampaTester : public common::BitManipulation
{
  private:
    uint8_t verbosity_;

    /** Some SampaTester-related constants/bits */
    const uint8_t idx_sca_i2c_sampa_ = {0};
    const uint8_t idx_sca_i2c_ext_adc_ = {5};

  public:
    /** ADC inputs and mappings */
    using adc_input_t = std::tuple<std::string /*name*/, std::string /*unit*/, uint8_t /*adc_input*/, float /*conv_factor*/>;
    static const std::array<adc_input_t, 11> adc_inputs_;

    /** DAC output and mappings */
    using dac_output_t = std::tuple<std::string /*name*/, uint8_t /*dac_output*/>;
    static const std::array<dac_output_t, 4> dac_outputs_;

  protected:
    std::unique_ptr<gbt::ScaBasic> sca_basic_;
    std::unique_ptr<gbt::ScaId> sca_id_;
    std::unique_ptr<gbt::ScaGpio> sca_gpio_;
    std::unique_ptr<gbt::ScaDac> sca_dac_;
    std::unique_ptr<gbt::ScaI2cMax1161x> sca_ext_adc_;
    std::unique_ptr<gbt::ScaI2cSampa> sca_i2c_sampa_;


  public:
    SampaTester(SampaTester const &) = delete;
    SampaTester &operator=(SampaTester const &) = delete;

    /** Construct */
    SampaTester(common::HdlcCore& hdlc, uint8_t verbosity = 0) :
        verbosity_(verbosity),
        sca_basic_(new gbt::ScaBasic(hdlc)),
        sca_id_(new gbt::ScaId(hdlc)),
        sca_gpio_(new gbt::ScaGpio(hdlc)),
        sca_dac_(new gbt::ScaDac(hdlc)),
        sca_ext_adc_(new gbt::ScaI2cMax1161x(hdlc, idx_sca_i2c_ext_adc_)),
        sca_i2c_sampa_(new gbt::ScaI2cSampa(hdlc, idx_sca_i2c_sampa_))
      {
        sca_gpio_->setThrowOnScaReplyError(true);
        sca_dac_->setThrowOnScaReplyError(true);
        sca_ext_adc_->setThrowOnScaReplyError(true);

        // Update I2C frequency setting (will throw errors if I2C port is disabled)
        sca_i2c_sampa_->setFreq();

        // Auto-update I2C slave address to current value of hadd
        updateSampaI2cAddress();
      }

    /** Reference to GPIO port */
    gbt::ScaGpio& getScaGpio() const { return *(sca_gpio_.get()); }

    /** Reference to DAC port */
    gbt::ScaDac& getScaDac() const { return *(sca_dac_.get()); }

    /** Reference to SAMPA I2C instance */
    gbt::ScaI2c& getScaI2cSampa() const { return *(sca_i2c_sampa_.get()); }

    /** Reference to external ADC instance (connected via SCA I2C) */
    gbt::ScaI2cMax1161x& getScaExtAdc() const { return *(sca_ext_adc_.get()); }

    /** Get SAMPA power status
     *  @return 1 if on, 0 if off
     */
    inline uint8_t getSampaPower() const { return getBit(sca_gpio_->getDataOut(), 30); }

    /** Set SAMPA power status
     *  @param state Desired power state
     *  @return Read-back power state
     */
    inline uint8_t setSampaPower(uint8_t state) const {
        sca_gpio_->setDataOut(setBit(sca_gpio_->getDataOut(), 30, state));
        return getSampaPower();
      }

    /** Get GPIO MuxSel bit status  */
    inline uint8_t getMuxSel() const { return getBit(sca_gpio_->getDataOut(), 11); }

    /** Set GPIO MuxSel bit status
     *  @param val New value
     *  @return Read-back MuxSel bit status
     */
    inline uint8_t setMuxSel(uint8_t val) const {
        sca_gpio_->setDataOut(setBit(sca_gpio_->getDataOut(), 11, val));
        return getMuxSel();
      }

    /** Get GPIO clk_config
     *  @return Read-back clk_config
     */
    inline uint8_t getClkConfig() const { return getField(sca_gpio_->getDataOut(), 4, 7); }

    /** Set GPIO clk_config
     *  @param val New clk_config value
     *  @return Read-back clk_config
     */
    inline uint8_t setClkConfig(uint8_t val) const {
        sca_gpio_->setDataOut(setField(sca_gpio_->getDataOut(), 4, 7, val));
        return getClkConfig();
      }

    /** Get GPIO hardware address
     *  @return Read-back hardware address
     */
    inline uint8_t getHadd() const { return getField(sca_gpio_->getDataOut(), 0, 4); }

    /** Set GPIO hardware address
     *  @param New hadd value
     *  @return Read-back hadd
     */
    inline uint8_t setHadd(uint8_t val) const {
        sca_gpio_->setDataOut(setField(sca_gpio_->getDataOut(), 0, 4, val));
        return getHadd();
      }

    /** Update SAMPA I2C slave default address to current value of hadd
     */
    inline void updateSampaI2cAddress() const { sca_i2c_sampa_->setSlaveAddress(getHadd()); }

    /** Sample ADC inputs specified by mask
     *  @param input_mask One-hot mask of ADC inputs to sample
     *  @bool
     */
    void sampleExtAdc(uint32_t input_mask = 0xfff, bool json = false, std::ostream& os = std::cout) const {
      bool first(true);
      os << (json ? "{\n" : "");

      for (auto i: adc_inputs_) {
        if (input_mask & (1 << std::get<2>(i))) {
          auto adc_raw = sca_ext_adc_->sample(std::get<2>(i));

          os << (first ? "" : (json ? ",\n" : "\n"))
             << (json ? "  \"" : "[Ext ADC] ") << std::right << std::setw(json ? 0 : 8) << std::get<0>(i) << (json ? "\": {" : " : ")
             << (json ? "\"value\": " : "") << std::setw(6) << std::fixed << std::setprecision(6)
             << std::left << adc_raw*std::get<3>(i) << (json ? ", " : "")
             << (json ? "\"unit\": \"" : " " ) << std::right << std::get<1>(i) << (json ? "\"}": "");

          first = false;
        }
      }

      os << (json ? "\n}" : "") << "\n";
    }

    /** Get information about input to external ADC
     *  @param channel External ADC input
     *  @return adc_input_t tuple with name, channel number, conversion factor, etc
     */
    const adc_input_t& getExtAdcInputDetails(uint8_t channel) const {
        assert(channel < adc_inputs_.size());

        unsigned int i(0);
        for (i = 0; i < adc_inputs_.size(); i++)
          if (std::get<2>(adc_inputs_[i]) == channel)
            break;

        return adc_inputs_[i];
      }
};
