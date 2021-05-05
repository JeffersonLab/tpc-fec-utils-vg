#include "fec.hpp"

//------------------------------------------------------------------------------
const FecAdc::port_map_t
FecAdc::ports_ = {
  {18, std::make_tuple(convCalibrationResistor, true, "uA", "ADC Current source")}, // 18
  //
  {19, std::make_tuple([](uint16_t adc_raw, float)->float{return float(adc_raw)/4096*3;}, false, "V", "Voltage 2r25 (conn)")},
  {20, std::make_tuple([](uint16_t adc_raw, float)->float{return float(adc_raw)/4096*3;}, false, "V", "Voltage 2r25 (in)")},
  //
  {21, std::make_tuple([](uint16_t adc_raw, float)->float{return float(adc_raw)/4096*6;}, false, "V", "Voltage 3r25 (conn)")},
  {22, std::make_tuple([](uint16_t adc_raw, float)->float{return float(adc_raw)/4096*6;}, false, "V", "Voltage 3r25 (in)")},
  //
  {23, std::make_tuple([](uint16_t adc_raw, float)->float{return float(adc_raw)/4096*3;}, false, "V", "Voltage 2v5")},
  {24, std::make_tuple([](uint16_t adc_raw, float)->float{return float(adc_raw)/4096*2;}, false, "V", "Voltage 1v5")},
  //
  {25, std::make_tuple(convRtd, true, "ºC", "RTD1 temperature")},
  {26, std::make_tuple(convRtd, true, "ºC", "RTD0 temperature")},
  {27, std::make_tuple(convRtd, true, "ºC", "RTD3 temperature")},
  {28, std::make_tuple(convRtd, true, "ºC", "RTD2 temperature")},
  {29, std::make_tuple(convRtd, true, "ºC", "RTD4 temperature")},
  //
  {30, std::make_tuple(convVtrx, false, "mA",  "VTRx photo current")},
  {31, std::make_tuple(convScaTemp, false, "ºC", "SCA temperature")}
};

std::ostream&
operator<<(std::ostream& os, const FecAdc::result_t& r)
{
  os << std::left << std::setw(20) << std::get<2>(r) << ": "
     << std::setw(7) << std::setprecision(4) << std::get<0>(r) << " "
     << std::setw(2) << std::get<1>(r)
     << "    [raw 0x" << std::right << std::hex << std::setw(4) << std::setfill('0')
     << std::get<3>(r) << "]" << std::left << std::dec << std::setfill(' ')
     << (((std::get<3>(r) >> 12) & 0x1) ? "  *** OVERFLOW ***" : "");
  return os;
}


//------------------------------------------------------------------------------
const uint8_t Fec::cfg_i2c_addr_gbtx_[Fec::n_gbtx_] = {0xf, 0xe};                  /**< I2C addresses of GBTx0 and GBTx1 */
const uint8_t Fec::cfg_i2c_addr_sampa_[Fec::n_sampa_] = {0x0, 0x1, 0x2, 0x3, 0x4}; /**< I2C addresses of the SAMPAs */

void
Fec::init()
{
   // Set basic GBT SCA registers (CRB, CRC, CRD)
   if (verbosity_ >= 1)
     std::cout << "Configuring GBT SCA basic registers\n";
   sca_basic_->transceive(sca_basic_->CMD_W_CRB, defaults_.scacfg_basic_crb_);
   sca_basic_->transceive(sca_basic_->CMD_W_CRC, defaults_.scacfg_basic_crc_);
   sca_basic_->transceive(sca_basic_->CMD_W_CRD, defaults_.scacfg_basic_crd_);

   // Check if SCA version info (now that ADC channel should be enabled) was correct,
   //   update ADC instance if we guessed incorrectly before
   auto sca_version = sca_id_->getVersion();
   if (verbosity_ >= 1)
     std::cout << "Updating SCA ID and version information\n";
   sca_id_->setVersion(sca_id_->read());
   if (sca_version != sca_id_->getVersion()) {
     common::HdlcCore& hdlc(sca_adc_->getHdlcCore());
     sca_adc_.reset((sca_id_->getVersion() == gbt::ScaId::SCA_V1) ?
       static_cast<gbt::ScaAdc*>(new gbt::ScaAdcVersion1(hdlc)) :
       static_cast<gbt::ScaAdc*>(new gbt::ScaAdcVersion2(hdlc)));
   }

   // Basic configuration for GPIO
   if (verbosity_ >= 1)
     std::cout << "Configuring GBT SCA GPIO\n";
   sca_gpio_->setDirection(defaults_.scacfg_gpio_direction_);
   sca_gpio_->setDataOut(defaults_.scacfg_gpio_dataout_);

   // Configuration for ADC
   if (verbosity_ >= 1)
     std::cout << "Configuring GBT SCA ADC\n";
   sca_adc_->init();

   if (verbosity_ >= 1)
     std::cout << "Configuring GBT SCA I2C interfaces\n";
   for (uint32_t i = 0; i < 2; i++)
     i2c_gbtx_[i]->setFreq();
   for (uint32_t i = 0; i < 5; i++)
     i2c_sampa_[i]->setFreq();
}


//------------------------------------------------------------------------------
void
Fec::gbtxConfigure(uint8_t gbt_idx, const std::string& cfg_file) const
{
  std::ifstream gcfg(cfg_file, std::ios::in);
  if (!gcfg.is_open())
    throw std::runtime_error("Unable to open file " + cfg_file);

  if (verbosity_ >= 1)
    std::cout << "Configuring GBTx" << static_cast<uint32_t>(gbt_idx)
              << " with file " << cfg_file << "\n";

  const gbt::ScaI2c& i2c(*i2c_gbtx_[gbt_idx]);

  std::string line;
  uint32_t addr(0);
  uint32_t val;
  while (gcfg.good()) {
    if (gcfg.peek() == '#') {
      getline(gcfg, line);
      continue;
    }
    gcfg >> std::hex >> val;
    if (!gcfg.good())
      break;
    i2c.writeByte(addr, val & 0xff);
    addr++;
  }

  gcfg.close();

  // Force 'config ready'
  uint32_t cfg_done = i2c.writeByte(365, 0xaa);
  if (verbosity_ >= 2)
    std::cout << "Forcing GBTx config done to 0x" << std::hex
              << cfg_done << std::dec << "\n";

  if (verbosity_ >= 2)
    std::cout << "Forcing GBTx reset\n";
  gbtxReset(gbt_idx);
}


void
Fec::gbtxDumpConfiguration(uint8_t gbt_idx, std::ostream& os) const
{
  const gbt::ScaI2c& i2c(*i2c_gbtx_[gbt_idx]);
  os << "#GBTx " << static_cast<uint32_t>(gbt_idx) << " configuration\n";
  for (uint32_t addr = 0; addr < 366; addr++)
    os << std::right << std::dec << std::setfill(' ') << std::setw(3)
       << addr << " : 0x"
       << std::hex << std::setfill('0') << std::setw(2)
       << static_cast<uint32_t>(i2c.readByte(addr)) << "\n";
}


void
Fec::gbtxReset(uint8_t gbt_idx) const
{
  const gbt::ScaI2c& i2c(*i2c_gbtx_[gbt_idx]);
  uint8_t r = i2c.readByte(50) & 0xc7;
  i2c.writeByte(50, r | 0x38);
  i2c.writeByte(50, r);
}


int32_t
Fec::gbtxI2cScan(uint8_t gbt_idx, std::ostream* const os, bool verbose) const
{
  int32_t addr(-1);
  for (uint8_t i = 1; i < 16; i++) {    // 0 is broadcast address
    try {
      if (os && verbose)
        *os << ".a" << static_cast<uint32_t>(i);
      const gbt::ScaI2c& gbt(*i2c_gbtx_[gbt_idx]);
      gbt.readByte(i, 62); // Phase aligner track mode register, but anything should do here
      if (os) {
        *os << (verbose ? " ->" : "")
            << "  Found GBTx" << static_cast<uint32_t>(gbt_idx)
            << " [I2C addr " << static_cast<uint32_t>(i)
            << ", SCA CH" << static_cast<uint32_t>(gbt.getChannel()) << "]\n";
      }
      addr = i;
      break;
    }
    catch (gbt::ScaException& e) { /* Ignore these here */ }
  }

  if ((addr == -1) && os)
    *os << (verbose ? " ->" : "")
        << "  Unable to detect GBTx" << static_cast<uint32_t>(gbt_idx) << " I2C"
        << " [SCA CH" << static_cast<uint32_t>(i2c_gbtx_[gbt_idx]->getChannel()) << "]\n";

  return addr;
}

//------------------------------------------------------------------------------
uint32_t
Fec::sampaPwrCtrl(uint32_t value, uint32_t mask, uint32_t wait) const
{
  // Apply power SAMPA-by-SAMPA, wait inbetween
  for (uint8_t i = 0; i < n_sampa_; i++) {
    if ((mask >> i) & 0x1) {
      sampaPwr(i, value);
      std::this_thread::sleep_for(std::chrono::milliseconds(wait));
    }
  }

  return sampaPwrStatus();
}


bool
Fec::sampaEnableElinks(uint8_t n, uint32_t sampa_mask) const
{
  uint8_t nchk(n);
  for (uint8_t i = 0; i < n_sampa_; i++) {
    if ((sampa_mask >> i) & 0x1)
      nchk &= i2c_sampa_[i]->writeByte(REG_SAMPA_SOCFG, n);
  }
  return (nchk == n);
}


uint32_t
Fec::sampaI2cScan(std::ostream* const os) const
{
  uint32_t active_mask(0x0);
  for (uint8_t i = 0; i < n_sampa_; i++) {
    try {
      const gbt::ScaI2c& sampa(*i2c_sampa_[i]);
      sampa.readByte(0x7);
      if (os) {
        *os << "  Found SAMPA" << static_cast<uint32_t>(i)
            << " [I2C addr " << sampa.getSlaveAddress()
            << ", SCA CH" << static_cast<uint32_t>(sampa.getChannel()) << "]\n";
      }
      active_mask |= (1 << i);
    }
    catch(gbt::ScaException& e) { /* Ignore these here */ }
  }

  if (active_mask == 0x0)
    *os << "  Found no active SAMPAs\n";

  return active_mask;
}
