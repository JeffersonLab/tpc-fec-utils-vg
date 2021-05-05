#include "sampa.hpp"
/* Global registers */
const uint8_t Sampa::REG_HWADD     = 0x00;
const uint8_t Sampa::REG_VACFG     = 0x0d;
const uint8_t Sampa::REG_CMD       = 0x0e;
const uint8_t Sampa::REG_ADCDEL    = 0x10;
const uint8_t Sampa::REG_ADCTRIM   = 0x11;
const uint8_t Sampa::REG_SOCFG     = 0x12;
const uint8_t Sampa::REG_SODRVST   = 0x13;
const uint8_t Sampa::REG_PMADDL    = 0x15;
const uint8_t Sampa::REG_PMADDH    = 0x16;
const uint8_t Sampa::REG_CHRGADD   = 0x17; // [4:0] | RW | Channel register address
const uint8_t Sampa::REG_CHRGWDATL = 0x18; // [7:0] | RW | Ch. reg. write data, lower byte
const uint8_t Sampa::REG_CHRGWDATH = 0x19; // [4:0] | RW | Ch. reg. write data, upper byte
const uint8_t Sampa::REG_CHRGCTL   = 0x1a; // [7:0] | RW | Ch. reg. ctrl.
                                         //       | [4:0] -- Channel number
                                         //       | [5]   -- Broadcast
                                         //       | [6]   -- Write, not read (returns to read after write)
                                         //       | [7]   -- Increment PMADD (returns auto to zero)
const uint8_t Sampa::REG_CHRGRDATL = 0x1b; // [7:0] | R  | Ch. reg. read data, lower byte
const uint8_t Sampa::REG_CHRGRDATH = 0x1c; // [4:0] | R  | Ch. reg. read data, upper byte
const uint8_t Sampa::REG_BYPASS    = 0x1f;
const uint8_t Sampa::REG_RINGCNT   = 0x21;
const uint8_t Sampa::REG_CLKCONF   = 0x22;
const uint8_t Sampa::REG_BOUNDARY  = 0x23;
const uint8_t Sampa::REG_CHEN0     = 0x24;
const uint8_t Sampa::REG_CHEN1     = 0x25;
const uint8_t Sampa::REG_CHEN2     = 0x26;
const uint8_t Sampa::REG_CHEN3     = 0x27;
/* Channel registers */
const uint8_t Sampa::REGCH_PMDATA =  0x10;
/* others */
const uint8_t Sampa::CH_BROADCAST = 0xbc;

const uint8_t Sampa::MAX_CHANNELS = 30;
const uint32_t Sampa::CHANNEL_MASK_ALL = (1 << MAX_CHANNELS) - 1;

const std::map<uint8_t, Sampa::reg_def_t>
Sampa::registers_ = {
  {REG_HWADD,     std::make_tuple("HWADD"    )},
  {REG_VACFG,     std::make_tuple("VACFG"    )},
  {REG_CMD,       std::make_tuple("CMD"      )},
  {REG_ADCDEL,    std::make_tuple("ADCDEL"   )},
  {REG_ADCTRIM,   std::make_tuple("ADCTRIM"  )},
  {REG_SOCFG,     std::make_tuple("SOCFG"    )},
  {REG_SODRVST,   std::make_tuple("SODRVST"  )},
  {REG_PMADDL,    std::make_tuple("PMADDL"   )},
  {REG_PMADDH,    std::make_tuple("PMADDH"   )},
  {REG_CHRGADD,   std::make_tuple("CHRGADD"  )},
  {REG_CHRGWDATL, std::make_tuple("CHRGWDATL")},
  {REG_CHRGWDATH, std::make_tuple("CHRGWDATH")},
  {REG_CHRGCTL,   std::make_tuple("CHRGCTL"  )},
  {REG_CHRGRDATL, std::make_tuple("CHRGRDATL")},
  {REG_CHRGRDATH, std::make_tuple("CHRGRDATH")},
  {REG_BYPASS,    std::make_tuple("BYPASS"   )},
  {REG_RINGCNT,   std::make_tuple("RINGCNT"  )},
  {REG_CLKCONF,   std::make_tuple("CLKCONF"  )},
  {REG_BOUNDARY,  std::make_tuple("BOUNDARY" )},
  {REG_CHEN0,     std::make_tuple("CHEN0"    )},
  {REG_CHEN1,     std::make_tuple("CHEN1"    )},
  {REG_CHEN2,     std::make_tuple("CHEN2"    )},
  {REG_CHEN3,     std::make_tuple("CHEN3"    )}
};


void
Sampa::dumpRegisters(std::ostream& os) const
{
  os << std::setfill('0') << std::hex;
  for (auto& reg : registers_)
    os << "  [0x" << std::setw(2) << static_cast<uint32_t>(reg.first) << "]"
       << "  " << std::left << std::setw(10) << std::setfill('.') << std::get<0>(reg.second)
       << std::right << std::setfill('0')
       << " : 0x" << std::setw(2) << static_cast<uint32_t>(i2c_.readByte(reg.first))
       << "\n";
  os << std::setfill(' ') << std::dec;
}


void
Sampa::writeChannelRegister(uint8_t channel, uint8_t raddr, uint16_t value) const
{
  writeRegister(REG_CHRGADD, raddr);
  writeRegister(REG_CHRGWDATH, (value >> 8) & 0x3);
  writeRegister(REG_CHRGWDATL, value & 0xff);
  writeRegister(REG_CHRGCTL, ((0 << 7) | (1 << 6) | ((channel == CH_BROADCAST ? 1 : 0) << 5) | (channel & 0x20)));
}

void
Sampa::writeChannelRegisterMultiple(uint32_t channel_mask, uint8_t raddr, uint16_t value) const
{
  if (channel_mask == CHANNEL_MASK_ALL)
    writeChannelRegisterBroadcast(raddr, value);
  else {
    uint8_t ch(0);
    while (ch < MAX_CHANNELS) {
      if ((channel_mask >> ch) & 0x1)
        writeChannelRegister(ch, raddr, value);
      ch++;
    }
  }
}

uint16_t
Sampa::readChannelRegister(uint8_t channel, uint8_t raddr) const
{
  writeRegister(REG_CHRGADD, raddr);
  writeRegister(REG_CHRGCTL, ((0 << 7) | (0 << 6) | (channel & 0x1f)));
  return (static_cast<uint16_t>(readRegister(REG_CHRGRDATH)) << 8) | readRegister(REG_CHRGRDATL);
}

std::unique_ptr<std::vector<uint16_t>>
Sampa::readChannelRegisterMultiple(uint32_t channel_mask, uint8_t raddr) const
{
  uint8_t ch(0);
  std::unique_ptr<std::vector<uint16_t>> values(new std::vector<uint16_t>);

  while (ch < MAX_CHANNELS) {
    if ((channel_mask >> ch) & 0x1) {
      values->push_back(readChannelRegister(ch, raddr));
      std::cout << std::hex << "-> 0x" << *(values->rbegin()) << std::dec << std::endl;
    }
    ch++;
  }

  return values;
}


uint16_t
Sampa::readPedestal(uint8_t channel, uint16_t paddr) const
{
  writeRegister(REG_PMADDL, paddr & 0xff);
  writeRegister(REG_PMADDH, (paddr >> 8) & 0x3);
  return readChannelRegister(channel, REGCH_PMDATA);
}

void
Sampa::writePedestal(uint8_t channel, uint16_t paddr, uint16_t value) const
{
  writeRegister(REG_PMADDL, paddr & 0xff);
  writeRegister(REG_PMADDH, (paddr >> 8) & 0x3);
  writeChannelRegister(channel, REGCH_PMDATA, value);
}
