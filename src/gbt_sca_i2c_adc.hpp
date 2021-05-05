#pragma once
#include "gbt_sca_i2c.hpp"

namespace gbt {

//------------------------------------------------------------------------------
class ScaI2cMax1161x : public ScaI2cCore
{
  private:
    inline void writeByte(uint8_t v) {
      uint32_t sd = transceive(I2C_S_7B_W, ((v & 0xff) << 8) | (getSlaveAddress() & 0x7f)).data;
      if (!isSuccess(sd & 0xff, 0xf7))   // Ignore LevelError - it is always set, not sure if it matters :/
        throw ScaException("I2C single byte write 7b" + getStatusBitNames(sd & 0xff), channel_);
    }

    inline void writeSetup(uint8_t sel, uint8_t clk, uint8_t bip, uint8_t rst)
      { 
        writeByte((BIT_SETUP << 7) | 
                  ((sel & 0x7) << 4) |
                  ((clk & 0x1) << 3) |
                  ((bip & 0x1) << 2) |
                  ((rst & 0x1) << 1) |
                  (0 << 1));
      }

    inline void writeConfig(uint8_t scan, uint8_t channel)
      {
        writeByte((BIT_CONFIG << 7) |
                  ((scan & 0x3) << 5) |
                  ((channel & 0xf) << 1) |
                  (BIT_SGL & 0x1)); 
      }

    inline void selectChannel(uint8_t channel) { writeConfig(FIELD_SCAN, channel); }

    const uint8_t BIT_SETUP {1};
    const uint8_t BIT_CONFIG {0};
    const uint8_t FIELD_SCAN {0x3};   // Table 5: Channels to convert selected by CS[0..3]
    const uint8_t FIELD_SEL {0x5};    // Table 6: Internal reference, always on
    const uint8_t BIT_CLK {1};        // 1 -> External conversion clock, internal doesn't work with i2c_freq > 100k
    const uint8_t BIT_BIP {0};        // 0 -> Unipolar 
    const uint8_t BIT_RST {0};        // 0 -> Don't reset config register at startup
    const uint8_t BIT_SGL {1};        // 1 -> Single-ended

    const uint8_t n_channels {12};

  public:
    ScaI2cMax1161x(common::HdlcCore& hdlc_core,
                   uint8_t i2c_if_idx = 5,    // SCA I2C port 5 used on SAMPA tester
                   uint32_t slave_i2c_addr = 0x35) :
        ScaI2cCore(hdlc_core, i2c_if_idx, slave_i2c_addr)
      {
        setFreq();
        setSclMode(0); // We want open drain
        writeSetup(FIELD_SEL, BIT_CLK, BIT_BIP, BIT_RST);
      }

    uint16_t sample(uint8_t channel)
      {
        selectChannel(channel);

        setNbyte(2);
        uint32_t sd = transceive(I2C_M_7B_R, getSlaveAddress() & 0x7f).data;
        if (!isSuccess(sd & 0xff, 0xf7))   // Ignore LevelError - it is always set, not sure if it matters :/
          throw ScaException("I2C multi byte read 7b" + getStatusBitNames(sd & 0xff), channel_);

        uint32_t rv = transceive(I2C_R_DATA3).data;
        return ((rv & 0xf) << 8) | ((rv & 0xff00) >> 8);
      }
};

} // namespace gbt
