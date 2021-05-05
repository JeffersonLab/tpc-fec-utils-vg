#pragma once

#include <map>
#include <vector>
#include "gbt_sca_i2c.hpp"

class Sampa
{
  private:
    gbt::ScaI2c& i2c_;

    using reg_def_t = std::tuple<const std::string>;

  public:
    Sampa(gbt::ScaI2c& i2c) : i2c_(i2c) {}

    /** Map defining the SAMPA registers
     */
    static const std::map<uint8_t, reg_def_t> registers_;

    /** Sampa register definitions & constants */
    /* Global registers */
    static const uint8_t REG_HWADD;
    static const uint8_t REG_VACFG;
    static const uint8_t REG_CMD;
    static const uint8_t REG_ADCDEL;
    static const uint8_t REG_ADCTRIM;
    static const uint8_t REG_SOCFG;
    static const uint8_t REG_SODRVST;
    static const uint8_t REG_PMADDL;
    static const uint8_t REG_PMADDH;
    static const uint8_t REG_CHRGADD;   // [4:0] | RW | Channel register address
    static const uint8_t REG_CHRGWDATL; // [7:0] | RW | Ch. reg. write data, lower byte
    static const uint8_t REG_CHRGWDATH; // [4:0] | RW | Ch. reg. write data, upper byte
    static const uint8_t REG_CHRGCTL;   // [7:0] | RW | Ch. reg. ctrl.
                                                 //       | [4:0] -- Channel number
                                                 //       | [5]   -- Broadcast
                                                 //       | [6]   -- Write, not read (returns to read after write)
                                                 //       | [7]   -- Increment PMADD (returns auto to zero)
    static const uint8_t REG_CHRGRDATL; // [7:0] | R  | Ch. reg. read data, lower byte
    static const uint8_t REG_CHRGRDATH; // [4:0] | R  | Ch. reg. read data, upper byte
    static const uint8_t REG_BYPASS;
    static const uint8_t REG_RINGCNT;
    static const uint8_t REG_CLKCONF;
    static const uint8_t REG_BOUNDARY;
    static const uint8_t REG_CHEN0;
    static const uint8_t REG_CHEN1;
    static const uint8_t REG_CHEN2;
    static const uint8_t REG_CHEN3;
    /* Channel registers */
    static const uint8_t REGCH_PMDATA;
    /* others */
    static const uint8_t CH_BROADCAST;
    static const uint8_t MAX_CHANNELS;
    static const uint32_t CHANNEL_MASK_ALL;

    /** Commands for command register */
    static const uint8_t CMD_RINGCNT {0x7};

    /** Misc constants */
    static const uint8_t CLKCONF_RNGOSC {0x13};
    static const uint8_t CLKCONF_MEMTEST {0x13};
    static const uint8_t CLKCONF_TPCREADOUT {0x41};



    /** Dump SAMPA global registers
     *  @param os Output stream to dump to
     */
    void dumpRegisters(std::ostream& os = std::cout) const;

    /** Read global register via I2C
     *  @param addr SAMPA register address
     *  @return Read value
     */
    inline uint8_t readRegister(uint8_t addr) const { return i2c_.readByte(static_cast<uint32_t>(addr)); }

    /** Write global register via I2C
     *  @param addr SAMPA register address
     *  @param value Write value
     *  @return Read-back value
     */
    inline uint8_t writeRegister(uint8_t addr, uint8_t value) const { return i2c_.writeByte(static_cast<uint32_t>(addr), value); }

    /** Read from SAMPA channel specific register
     *  @param channel SAMPA channel number (0-31)
     *  @param raddr Register address
     *  @return SAMPA channel register value
     */
    inline uint16_t readChannelRegister(uint8_t channel, uint8_t raddr) const;

    /** Read channel register for multiple channels
     *  @param channel_mask SAMPA channel mask, one-hot
     *  @param raddr Channel register address
     *  @return Vector with values read back from channels with mask bit set to 1 (lowest channel first)
     */
    std::unique_ptr<std::vector<uint16_t>> readChannelRegisterMultiple(uint32_t channel_mask, uint8_t raddr) const;

    /** Write to SAMPA channel specific register
     *  @param channel SAMPA channel number (0-31)
     *  @param raddr Register address
     *  @param val Value to write to register
     */
    void writeChannelRegister(uint8_t channel, uint8_t raddr, uint16_t value) const;

    /** Write to channel register of multiple channels
     *  @param channel_mask SAMPA channel mask, one-hot
     *  @param raddr Channel register address
     *  @param value Value to write to channel registers
     */
    void writeChannelRegisterMultiple(uint32_t channel_mask, uint8_t raddr, uint16_t value) const;

    /** Write to SAMPA channel specific register of all channels
     *  @param raddr Channel register address
     *  @param val Value to write to registers
     */
    inline void writeChannelRegisterBroadcast(uint8_t raddr, uint16_t value) const { writeChannelRegister(CH_BROADCAST, raddr, value); }

    /** Read pedestal memory
     *  @param channel SAMPA channel number
     *  @param paddr Pedestal memory address
     *  @return Value read back from pedestal memory at address paddr
     */
    uint16_t readPedestal(uint8_t channel, uint16_t paddr) const;

    /** Write pedestal memory
     *  @param channel SAMPA channel number
     *  @param paddr Pedestal memory address
     *  @param value Pedestal memory value to wirte
     */
    void writePedestal(uint8_t channel, uint16_t paddr, uint16_t value) const;

    /** Write to SAMPA pedestal memory on all channels
     *  @param paddr Pedestal memory address
     *  @param value Data to write (to all channel pedestal memories at given address)
     */
    void writePedestalBroadcast(uint16_t paddr, uint16_t value) { writePedestal(CH_BROADCAST, paddr, value); };

    /** Get SAMPA clock config
     */
    inline uint8_t getClkConfig() const { return readRegister(REG_CLKCONF); }

};
