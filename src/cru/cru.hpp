#pragma once

#include "ReadoutCard/ChannelFactory.h"
#include "bar_common.hpp"

/** CRU classes */
namespace cru {

/** Wrapper class for AliceO2 ReadoutCard library bar objects
 */
class Bar : public common::Bar
{
  private:
    AliceO2::roc::Parameters::CardIdType card_id_;
    AliceO2::roc::Parameters parameters_;
    std::shared_ptr<AliceO2::roc::BarInterface> bar_;

  public:
    /** Create CRU bar
     *  @param bdf_or_serial Device identifier (PCI bus-device-function
     *                       or serial number of the card)
     *  @param bar_number    Bar number
     */
    Bar(const std::string bdf_or_serial, const int bar_number = 2) :
        card_id_(AliceO2::roc::Parameters::cardIdFromString(bdf_or_serial)),
        parameters_(AliceO2::roc::Parameters::makeParameters(card_id_,
          AliceO2::roc::Parameters::ChannelNumberType(bar_number))),
        bar_(AliceO2::roc::ChannelFactory().getBar(parameters_))
      {}

    /** Read 32 bit
     *  @param addr Register address
     *  @return 32-bit value at address 'addr'
     */
    inline uint32_t
    read(uint32_t addr) const override
      { return bar_->readRegister(addr); }

    /** Write 32 bit
     *  @param addr Register address
     *  @param data Data to write
     */
    inline void
    write(uint32_t addr, uint32_t data) const override
      { bar_->writeRegister(addr, data); }
};

} // namespace cru
