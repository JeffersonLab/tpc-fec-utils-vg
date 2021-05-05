#ifdef SCA_I2C_DEBUG
  #define debug(s)  std::cout << __func__ << " : " << s << std::endl;
#else
  #define debug(...)
#endif

#include "gbt_sca_i2c.hpp"

namespace gbt {

//------------------------------------------------------------------------------
const std::array<std::string, 8> ScaI2cCore::statusBitNames_ = {
  "Reserved(0)", "Reserved(1)", "Success", "LevelError",
  "Reserved(4)", "InvalidComm", "NoAck", "Reserved(7)"
};

const std::array<std::string, 4> ScaI2cCore::freqNames_ = {
  "100k", "200k", "400k", "1M"
};

//------------------------------------------------------------------------------
uint8_t
ScaI2cGbtx::readByte(uint32_t daddr, uint32_t raddr) const
{
  setNbyte(2);
  transceive(I2C_W_DATA0, (raddr & 0xffff));

  uint32_t sd = transceive(I2C_M_7B_W, (daddr & 0x7f)).data;
  if (!isSuccess(sd & 0xff))
    throw ScaException("I2C multi byte write 7b :" + getStatusBitNames(sd & 0xff), channel_);

  sd = transceive(I2C_S_7B_R, (uint32_t(1) << 8) | (daddr & 0x7f)).data;
  if (!isSuccess(sd & 0xff))
    throw ScaException("I2C single byte read 7b :" + getStatusBitNames(sd & 0xff), channel_);

  return uint8_t((sd >> 8) & 0xff);
}

uint8_t
ScaI2cGbtx::writeByte(uint32_t daddr, uint32_t raddr, uint8_t value) const
{
  setNbyte(3);
  transceive(I2C_W_DATA0, (value << 16) | (raddr & 0xffff));

  uint32_t sd = transceive(I2C_M_7B_W, (daddr & 0x7f)).data;
  if (!isSuccess(sd & 0xff))
    throw ScaException("I2C multi byte write 7b :" + getStatusBitNames(sd & 0xff), channel_);

  return readByte(daddr, raddr);
}

void
ScaI2cGbtx::pushByte(uint32_t daddr, uint32_t raddr, uint8_t value) const
{
  // Set up I2C to transmit 3 bytes
  //   Push default values for i2c_freq_ and scl_mode, as we can't read
  //   back anything
  uint8_t scl_mode(0);
  transmit(I2C_W_CTRL, (scl_mode << 7) | (3 << 2) | (i2c_freq_ & 0x3));
  transmit(I2C_W_DATA0, (value << 16) | (raddr & 0xffff));
  transmit(I2C_M_7B_W, (daddr & 0x7f));
}


//------------------------------------------------------------------------------
uint8_t
ScaI2cSampa::readByte(uint32_t daddr, uint32_t raddr) const
{
  debug("SCA CH " << static_cast<uint32_t>(getChannel())
         << ", I2C dev addr " << daddr << ", reg " << raddr << ", 10b addr 0x"
         << std::setw(4) << std::setfill('0') << std::hex
         << addressToD1D0(daddr, raddr)
         << std::dec << std::setfill(' '))

  uint32_t sd = transceive(I2C_S_10B_R, addressToD1D0(daddr, raddr)).data;
  if (!isSuccess(sd & 0xff))
    throw ScaException("I2C single byte read 10b :" + getStatusBitNames(sd & 0xff), channel_);

  return (sd >> 8) & 0xff;
}

uint8_t
ScaI2cSampa::writeByte(uint32_t daddr, uint32_t raddr, uint8_t value) const
{
  debug("SCA CH " << static_cast<uint32_t>(getChannel())
         << ", I2C dev addr " << daddr << ", reg " << raddr << ", val 0x"
         << std::hex << static_cast<uint32_t>(value) << ", 10b addr 0x"
         << std::setw(4) << std::setfill('0')
         << addressToD1D0(daddr, raddr)
         << std::dec << std::setfill(' '))

  uint32_t sd = transceive(I2C_S_10B_W, (uint32_t(value) << 16) |
                                        addressToD1D0(daddr, raddr)).data;
  if (!isSuccess(sd & 0xff))
    throw ScaException("I2C single byte write 10b :" + getStatusBitNames(sd & 0xff), channel_);

  return readByte(daddr, raddr);
}

void
ScaI2cSampa::pushByte(uint32_t daddr, uint32_t raddr, uint8_t value) const
{
  debug("SCA CH " << static_cast<uint32_t>(getChannel())
         << ", I2C dev addr " << daddr << ", reg " << raddr << ", val 0x"
         << std::hex << static_cast<uint32_t>(value) << ", 10b addr 0x"
         << std::setw(4) << std::setfill('0')
         << addressToD1D0(daddr, raddr)
         << std::dec << std::setfill(' '))

  transmit(I2C_S_10B_W, (uint32_t(value) << 16) | addressToD1D0(daddr, raddr));
}

} // namespace gbt

#undef debug
