#pragma once

#include <stdexcept>
#include <librorc.h>

#include "utils.hpp"
#include "bar_common.hpp"

#ifdef TRORC_DEBUG
  #define tdebug(s) std::cout << s << std::endl;
#else
  #define tdebug(...)
#endif


/** T-RORC classes */
namespace trorc {

/** T-RORC wrapper class for librorc::device
 */
class Device
{
  private:
    std::unique_ptr<librorc::device> dev_;

  public:
    /** Create T-RORC device
     *  @param device_number Device number
     */
    Device(int device_number = 0) :
        dev_(new librorc::device(device_number))
      {}

    /** Get underlying librorc device pointer
     */
    inline librorc::device* get() const { return dev_.get(); }
};



/** T-RORC wrapper class for librorc::bar with
 *  more flexible bar read/write
 */
class Bar : public common::Bar
{
  private:
    std::unique_ptr<librorc::device> device_;  /**< Device is only used when constructing with Bar(device_number, bar_number) */
    std::unique_ptr<librorc::bar> bar_;

  public:
    /** Create T-RORC bar and attach it to already existing device
     *  @param dev        T-RORC device the bar gets is attached to
     *  @param bar_number Bar number
     */
    Bar(const Device& dev, const int bar_number = 1) :
        device_(nullptr),
        bar_(new librorc::bar(dev.get(), bar_number))
      {}

    /** Create T-RORC device and bar, which is attached to the newly
     *    created device device
     *  @param device_number Device number
     *  @param bar_number    Bar number
     */
    Bar(const int device_number = 0, const int bar_number = 1) :
        device_(new librorc::device(device_number)),
        bar_(new librorc::bar(device_.get(), bar_number))
      {}

    /** Get underlying librorc bar pointer
     */
    inline librorc::bar* get() const { return bar_.get(); }

    /** Read 32 bit
     *  @param addr Register address
     *  @return 32-bit value at address 'addr'
     */
    inline uint32_t
    read(uint32_t addr) const override
      { return bar_->get32(addr); }

    /** Write 32 bit
     *  @param addr Register address
     *  @param data Data to write
     */
    inline void
    write(uint32_t addr, uint32_t data) const override
      { bar_->set32(addr, data); }
};



/** T-RORC exception class
 */
class Exception : public std::exception
{
  private:
    std::string what_arg;

  public:
    Exception(const std::string& s) : what_arg("ERROR : T-RORC : " + s) {}
    virtual const char* what() const noexcept { return what_arg.c_str(); }
};

}  // namespace trorc
