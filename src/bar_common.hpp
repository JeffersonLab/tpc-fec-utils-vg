#pragma once

#include <cstdint>
#include "bit_manipulation.hpp"

namespace common {

class Bar : public BitManipulation
{
  public:
    /** Read 32 bit
     *  @return 32-bit value at address 'addr'
     */
    virtual uint32_t read(uint32_t addr) const = 0;

    /** Write 32 bit
     *  @return 32-bit value at address 'addr' (read-back)
     */
    virtual void write(uint32_t addr, uint32_t data) const = 0;

    /** Read field from hardware
     *  @return Content of [lsb+width-1 .. lsb] at address 'addr'
     */
    inline uint32_t
    readField(uint32_t addr, uint8_t lsb, uint8_t width) const
      { return getField(read(addr), lsb, width); }

    /** Write field, leaving all other bits unchanged
     *  @param addr  Address
     *  @param lsb   Field LSB
     *  @param width Field width
     *  @param value Field value to write
    */
    inline void
    writeField(uint32_t addr, uint8_t lsb, uint8_t width, uint32_t value) const
      { write(addr, setField(read(addr), lsb, width, value)); }

    /** Write field, leaving all other bits unchanged, and read back new value
     *  @param addr  Address
     *  @param lsb   Field LSB
     *  @param width Field width
     *  @param value Field value to write
    *   @return Content of [lsb+width-1 .. lsb] at address 'addr' (read-back)
    */
    inline uint32_t
    writeFieldRb(uint32_t addr, uint8_t lsb, uint8_t width, uint32_t value) const
      {
        writeField(addr, lsb, width, value);
        return readField(addr, lsb, width);
      }

    /** Read bit from hardware
     *  @return Value of bit 'idx' at address 'addr'
     */
    inline uint8_t
    readBit(uint32_t addr, uint8_t idx) const
      { return readField(addr, idx, 1); }

    /** Write bit, leaving all other bits unchanged
     *  @param addr Address
     *  @param idx  Bit index
     *  @param value Bit value to set
     */
    inline void
    writeBit(uint32_t addr, uint8_t idx, uint32_t value) const
      { writeField(addr, idx, 1, value); }

    /** Write bit, leaving all other bits unchanged, and read back new bit value
     *  @return Value of bit 'idx' at address 'addr' (read-back)
     */
    inline uint32_t
    writeBitRb(uint32_t addr, uint8_t idx, uint32_t value) const
      {
        writeField(addr, idx, 1, value);
        return readBit(addr, idx);
      }

    /** Write 32 bit & read back value
     *  @param addr Address
     *  @param data Write data
     *  @return (New) 32-bit value at address 'addr' (read-back)
     */
    inline uint32_t
    writeRb(uint32_t addr, uint32_t data) const
      {
        write(addr, data);
        return read(addr);
      }
};

} // namespace common
