#pragma once
#include <array>

#include "bit_manipulation.hpp"
#include "gbt_sca.hpp"

namespace gbt {

//------------------------------------------------------------------------------
/** GBT SCA I2C core class<br/>
 *    Provides basic functionality of SCA I2C core implementations
 */
class ScaI2cCore : public Sca, public common::BitManipulation
{
  public:
    /** Valid I2C clock frequencies
     */
    enum i2c_freq_t : uint32_t {
        FREQ_100k = 0x0,
        FREQ_200k = 0x1,
        FREQ_400k = 0x2,
        FREQ_1M   = 0x3
      };

  protected:
    static const std::array<std::string, 4> freqNames_;

    uint32_t cfg_slave_daddr_; /**< Internally stored slave device address, used in the
                                *   read/write variants with slave register addressing only
                                */

    /** Test I2C transaction for success
     *  @param Status byte
     *  @param Status byte mask (if != 0x0, this allows to ignore the corresponding flags,
     *           e.g. for a detected LevelError)
     *  @return True if I2C transaction was successful, false otherwise
     */
    inline bool isSuccess(uint8_t status, uint8_t mask = 0xff) const
      { return (status & mask) == 0x4; }

    /** Status bit names
     */
    static const std::array<std::string, 8> statusBitNames_;

    /** Assemble string containing resolved status bit names
     */
    inline std::string
    getStatusBitNames(uint8_t status_code, bool dump_if_not_success = true) const
    {
      std::stringstream os;
      for(uint8_t i = 0; i < statusBitNames_.size(); i++)
        if ((1 << i) & status_code)
          os << " +" << statusBitNames_[i];

      if (dump_if_not_success && !isSuccess(status_code))
        os << " [" << std::hex << "0x" << static_cast<uint32_t>(status_code) << "]" << std::dec;

      return os.str();
    }

    /** Get CONTROL register
     *  @return CONTROL register value
     */
    inline uint32_t getControl() const { return transceive(I2C_R_CTRL).data; }

    /** Set CONTROL register
     *  @param ctrl Register value
     *  @return Read-back CONTROL register value
     */
    inline uint32_t setControl(uint32_t ctrl) const
      {
        transceive(I2C_W_CTRL, ctrl);
        return getControl();
      }

  public:
    /** Construct ScaI2cCore instance
     *  @param hdlc_core Reference to the HDLC core for communication to the SCA
     *  @param i2c_if_idx I2C interface index (0..15)
     *  @param slave_i2c_addr Slave I2C address used when using the
     *         read/write variants with slave register addressing only
     */
    ScaI2cCore(common::HdlcCore& hdlc_core, uint8_t i2c_if_idx = 0, uint32_t slave_i2c_addr = 0) :
        Sca(hdlc_core, 0x3 + i2c_if_idx),
        cfg_slave_daddr_(slave_i2c_addr)
      {}

    /** Get I2C slave device address
     *  @return Slave device address used in the r/w variants with slave register addressing
     */
    inline uint32_t getSlaveAddress() const { return cfg_slave_daddr_; }

    /** Set I2C slave device address
     *  @param addr New I2C slave device address [1, .., 15]
     *  @return New slave device address value
     */
    inline uint32_t setSlaveAddress(uint32_t addr)
      {
         if ((addr >= 0x1) && (addr <= 0x7f))
           cfg_slave_daddr_ = addr;
         return getSlaveAddress();
      }

    /** I2C port pin output modes */
    enum i2c_output_mode_t { OUTPUT_OPEN_DRAIN = 0, OUTPUT_DRIVEN = 1 };

    /** Set output mode (SCLMODE)
     *  @param m Output mode
     *  @return Read-back output mode
     */
    inline i2c_output_mode_t setOutputMode(i2c_output_mode_t m)
      {
        return static_cast<i2c_output_mode_t>(getBit(setControl(setBit(getControl(), 7, m)), 7));
      }

    /** Get output mode (SCLMODE)
     *  @return Read-back output mode
     */
    inline i2c_output_mode_t getOutputMode()
      {
        return static_cast<i2c_output_mode_t>(getBit(getControl(), 7));
      }

    /** Selectively set the I2C frequency
     *  @param freq Frequency
     *  @return Read-back frequency setting
     */
    inline i2c_freq_t setFreq(const i2c_freq_t freq) const
      { return static_cast<i2c_freq_t>(setControl(setField(getControl(), F_FREQ_LSB, F_FREQ_WDT, freq))); }

    /** Set default I2C core frequency. May be overriden by derived classes for individual settings
     *  @return Read-back frequency setting
     */
    virtual i2c_freq_t setFreq() const
      { return setFreq(FREQ_400k); };

    /** Read back I2C frequency setting
     *  @return Read-back frequency
     */
    inline i2c_freq_t getFreq() const
      { return static_cast<i2c_freq_t>(getField(getControl(), F_FREQ_LSB, F_FREQ_WDT)); }

    /** Selectively set NBYTE field of CONTROL register
     *  @note Only evaluated during multi-byte transmissions (SCA manual v8.0, p18)
     *  @param nbyte Value of the NBYTE field [0..16]
     *  @param no_reply
     *  @return Read-back NBYTE value
     */
    inline uint32_t setNbyte(uint8_t nbyte) const
      { return setControl(setField(getControl(), F_NBYTE_LSB, F_NBYTE_WDT, nbyte)); }

    /** Selectively set SCLMODE
     *  @param v New value for SCLMODE bit
     *  @return Read-back SCLMODE value
     *  @note Functionality for v2 SCA only
     */
    inline uint32_t setSclMode(uint8_t v) const
      { return setControl(setBit(getControl(), B_SCLMODE, v)); };

    /** Get I2C command status register
     *  @see statusBitNames_
     *  @set getStatusBitNames
     *  @note Normally, requesting this explicitly is not needed, as the value of this register is
     *          shipped as part of the SCA reply frame when issuing I2C commands
     *  @return Value of the I2C core status register
     */
    inline uint32_t getStatus() const { return transceive(I2C_R_STAT).data; }

    /** Print I2C port status summary
     */
    friend std::ostream& operator<<(std::ostream& os, const ScaI2cCore& s)
      {
        ScaBasic sca_basic(s.hdlc_core_);

        os << "SCA I2C port " << (s.getChannel() - 0x3) << " [CH" << static_cast<uint32_t>(s.getChannel()) << "]";

        if (sca_basic.channelEnabled(s.channel_)) {
          uint32_t control = s.getControl();
          uint32_t status = s.getStatus();

          os << "\n" << std::hex << std::setfill('0')
             << "  Ctrl   0x" << std::setw(2) << control
             << " : " << s.freqNames_[s.getField(control, s.F_FREQ_LSB, s.F_FREQ_WDT)] << "Hz"
             << ", " << ((s.getBit(control, s.B_SCLMODE) == OUTPUT_OPEN_DRAIN) ? "open drain" : "driven") << "\n"
             << "  Status 0x" << std::setw(2) << status << " : " << s.getStatusBitNames(status, false) << "\n"
             << std::dec << std::setfill(' ')
             << "";
        }
        else {
          os << " is disabled\n";
        }

        return os;
      }

    const sca_rgf_t I2C_R_STAT  {0x11, 2, 2, BO_NORMAL};
    const sca_rgf_t I2C_W_MASK  {0x20, 2, 2, BO_NORMAL};
    const sca_rgf_t I2C_R_MASK  {0x21, 2, 2, BO_NORMAL};
    const sca_rgf_t I2C_W_CTRL  {0x30, 2, 2, BO_NORMAL};
    const sca_rgf_t I2C_R_CTRL  {0x31, 2, 2, BO_NORMAL};

    const sca_rgf_t I2C_W_DATA0 {0x40, 4, 4, BO_NORMAL};
    const sca_rgf_t I2C_R_DATA0 {0x41, 4, 4, BO_NORMAL};
    const sca_rgf_t I2C_W_DATA1 {0x50, 4, 4, BO_NORMAL};
    const sca_rgf_t I2C_R_DATA1 {0x51, 4, 4, BO_NORMAL};
    const sca_rgf_t I2C_W_DATA2 {0x60, 4, 4, BO_NORMAL};
    const sca_rgf_t I2C_R_DATA2 {0x61, 4, 4, BO_NORMAL};
    const sca_rgf_t I2C_W_DATA3 {0x70, 4, 4, BO_NORMAL};
    const sca_rgf_t I2C_R_DATA3 {0x71, 4, 4, BO_NORMAL};

    const sca_rgf_t I2C_S_7B_W  {0x82, 2, 2, BO_NORMAL};
    const sca_rgf_t I2C_S_7B_R  {0x86, 2, 2, BO_NORMAL};
    const sca_rgf_t I2C_M_7B_W  {0xda, 1, 2, BO_NORMAL};
    const sca_rgf_t I2C_M_7B_R  {0xde, 1, 2, BO_NORMAL};
    const sca_rgf_t I2C_S_10B_W {0x8a, 2, 2, BO_NORMAL};
    const sca_rgf_t I2C_S_10B_R {0x8e, 2, 2, BO_NORMAL};
    const sca_rgf_t I2C_M_10B_W {0xe2, 2, 2, BO_NORMAL};
    const sca_rgf_t I2C_M_10B_R {0xe6, 2, 2, BO_NORMAL};

    // Bits in control register
    const uint8_t F_FREQ_LSB  {0};
    const uint8_t F_FREQ_WDT  {2};
    const uint8_t F_NBYTE_LSB {2};
    const uint8_t F_NBYTE_WDT {5};
    const uint8_t B_SCLMODE   {7};
};


//------------------------------------------------------------------------------
/** GBT SCA I2C communication base class<br/>
 *    Adds readByte, writeByte, pushByte to SCA I2C core functions
 */
class ScaI2c : public ScaI2cCore
{
  public:
    /** Construct ScaI2c instance
     *  @param hdlc_core Reference to the HDLC core for communication to the SCA
     *  @param i2c_if_idx I2C interface index (0..15)
     *  @param slave_i2c_addr Slave I2C address used when using the
     *         read/write variants with slave register addressing only
     */
    ScaI2c(common::HdlcCore& hdlc_core, uint8_t i2c_if_idx = 0, uint32_t slave_i2c_addr = 0x0) :
        ScaI2cCore(hdlc_core, i2c_if_idx, slave_i2c_addr)
      {}

    /** Read byte from I2C slave using the stored slave device address
     *  @param raddr Slave register address
     *  @return Byte received from slave at [cfg_slave_daddr_, raddr]
     *  @see cfg_slave_daddr_
     *  @throws ScaException
     */
    virtual uint8_t readByte(uint32_t raddr) const
      {
        return readByte(cfg_slave_daddr_, raddr);
      }

    /** Read byte from I2C slave
     *  @param daddr Slave device address
     *  @param raddr Slave register address
     *  @return Received byte
     *  @throws ScaException
     */
    virtual uint8_t readByte(uint32_t daddr, uint32_t raddr) const = 0;

    /** Write byte to I2C slave using the stored slave device address
     *  @param raddr Slave register address
     *  @return Value read back from slave at [cfg_slave_daddr_, raddr]
     *  @see cfg_slave_daddr_
     *  @throws ScaException
     */
    virtual inline uint8_t writeByte(uint32_t raddr, uint8_t value) const
      {
        return writeByte(cfg_slave_daddr_, raddr, value);
      }

    /** Write byte to I2C slave
     *  @param daddr Slave device address
     *  @param raddr Slave register address
     *  @return Value read back from slave at [daddr,raddr]
     *  @throws ScaException
     */
    virtual uint8_t writeByte(uint32_t daddr, uint32_t raddr, uint8_t value) const = 0;

    /** Push byte to I2C slave using the stored slave device address
     *
     *  This is similar to writeByte, but there is:\n
     *   - no read-back of register value
     *   - no checking of underlying SCA reply frame
     *   - potentially applies defaults to I2C control register, see implementation
     *
     *  @param raddr Slave register address
     *  @see writeByte
     *  @see cfg_slave_daddr_
     */
    inline void pushByte(uint32_t raddr, uint8_t value) const
      {
        return pushByte(cfg_slave_daddr_, raddr, value);
      }

    /** Push byte to I2C slave
     *  @param daddr Slave device address
     *  @param raddr Slave register address
     *  @param value Register value to transmit
     *  @see pushByte
     */
    virtual void pushByte(uint32_t daddr, uint32_t raddr, uint8_t value) const = 0;
};

//------------------------------------------------------------------------------
/** Class for I2C communication with the GBTx via GBT SCA
 *  (7B addressing)
 */
class ScaI2cGbtx : public ScaI2c
{
  private:
    const i2c_freq_t i2c_freq_{FREQ_400k}; /**< I2C clock frequency for GBTx */

  public:
    ScaI2cGbtx(common::HdlcCore& hdlc_core, uint8_t i2c_if_idx = 0, uint32_t slave_i2c_addr = 0) :
        ScaI2c(hdlc_core, i2c_if_idx, slave_i2c_addr)
      {}

    using ScaI2c::readByte;
    using ScaI2c::writeByte;
    using ScaI2c::pushByte;
    uint8_t readByte(uint32_t daddr, uint32_t raddr) const override;
    uint8_t writeByte(uint32_t daddr, uint32_t raddr, uint8_t value) const override;

    /** Push byte to I2C slave
     *
     *  Applies defaults to I2C frequency and SCL_MODE in I2C control register,
     *    as we assume that we can't read back the current value to alter specific
     *    bits only
     *
     *  @param daddr Slave device address
     *  @param raddr Slave register address
     *  @param value Register value to transmit
     *  @see ScaI2c::pushByte
     */
    void pushByte(uint32_t daddr, uint32_t raddr, uint8_t value) const override;

    /** Allow for individual GBTx I2C core frequency setting
     *  @return Read-back frequency setting
     *  @see ScaI2cCore::setFreq
     */
    i2c_freq_t setFreq() const override
      { return ScaI2cCore::setFreq(i2c_freq_); }
};


//------------------------------------------------------------------------------
/** Class for I2C communication with the SAMPAs via GBT SCA
 *  (10B addressing)
 */
class ScaI2cSampa : public ScaI2c
{
  private:
   /** Convert device addresses to D0/D1 data field contents
    *  @param daddr Slave device address
    *  @param raddr Slave register address
    *  @return Value of the 'D1[7..0] & D0[7..0]' field as '[15..8] & [7..0]'
    */
   inline uint16_t addressToD1D0(uint32_t daddr, uint32_t raddr) const
     {
       return static_cast<uint16_t>(
              (((daddr & 0x3) << 14) | ((raddr & 0x3f) << 8)) | // D1
               (0x78 | ((daddr >> 2) & 0x3))                    // D0
              );
     }

  public:
    ScaI2cSampa(common::HdlcCore& hdlc_core, uint8_t i2c_if_idx = 0, uint32_t slave_i2c_addr = 0) :
        ScaI2c(hdlc_core, i2c_if_idx, slave_i2c_addr)
      {}

    using ScaI2c::readByte;
    using ScaI2c::writeByte;
    using ScaI2c::pushByte;
    uint8_t readByte(uint32_t daddr, uint32_t raddr) const override;
    uint8_t writeByte(uint32_t daddr, uint32_t raddr, uint8_t value) const override;
    void pushByte(uint32_t daddr, uint32_t raddr, uint8_t value) const override;

    /** Allow for individual SAMPA I2C core frequency setting
     *  @return Read-back frequency setting
     *  @see ScaI2cCore::setFreq
     */
    i2c_freq_t setFreq() const override
      { return ScaI2cCore::setFreq(FREQ_400k); }
};

} // namespace gbt
