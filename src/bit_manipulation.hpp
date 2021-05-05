#pragma once

#include <cstdint>

namespace common {

class BitManipulation
{
  protected:
    /** Use inheritance if you want to use functions in this class */
    BitManipulation() {};
    BitManipulation(const BitManipulation&) {};

    /** Calculate bit mask (32-bit)
      * @return 32-bit mask with bits [width+lsb-1 .. lsb] set to one
      */
    inline uint32_t
    mask(uint8_t width, uint8_t lsb = 0) const
      { return ((width == 32) ? 0xffffffff : ((1 << width) - 1)) << lsb; }


  public:
    /** Extract field from value
     *  @return Content of [lsb+width-1 .. lsb] of vin
     */
    inline uint32_t
    getField(uint32_t vin, uint8_t lsb, uint8_t width) const
      { return (vin >> lsb) & mask(width); }

    /** Extract / test bit value
     *  @return Value of bit 'idx' of 'vin'
     */
    inline uint8_t
    getBit(uint32_t vin, uint8_t idx) const
      { return getField(vin, idx, 1); }

    /** Set field of width 'width' at position 'lsb' to 'value'
     *  @param vin Input value
     *  @param lsb Field least significant bit
     *  @param width Field width
     *  @param value New field value
     *  @return Value of vin with field change to 'value'
     */
    inline uint32_t
    setField(uint32_t vin, uint8_t lsb, uint8_t width, uint8_t value) const
      {
        uint32_t r = vin & ~mask(width, lsb);
        return r | ((value & mask(width)) << lsb);
      }

    /** Set 'bit' at position 'idx' to 'value'
     *  @param vin Input value
     *  @param idx Bit index
     *  @param value Desired bit value
     *  @return Value of vin with 'bit' at position 'idx' set to 'value'
     */
    inline uint32_t
    setBit(uint32_t vin, uint8_t idx, uint8_t value) const
      {
        return setField(vin, idx, 1, value);
      }
};

}   // namespace common
