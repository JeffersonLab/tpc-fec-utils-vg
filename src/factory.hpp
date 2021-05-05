#pragma once

#include <string>

#include "hdlc_core.hpp"

#if defined BUILD_FOR_TRORC
  #include "trorc.hpp"
  #include "hdlc_sw_trorc.hpp"
#elif defined BUILD_FOR_CRU
  #include "cru.hpp"
  #include "hdlc_cern_me.hpp"
#endif

/** Common classes and factories */
namespace common {

/** Handle command-line defaults depending on whether we're compiling for CRU or T-RORC
 */
class OptionDefaultsFactory
{
  public:
#if defined BUILD_FOR_TRORC
    static constexpr const char* id = "0";
    static constexpr const char* id_help = "(device number, e.g. 0 [TRORC])";
    static constexpr uint32_t bar = 1;
#elif defined BUILD_FOR_CRU
    static constexpr const char* id = "06:00.0";
    static constexpr const char* id_help = "(PCI BDF notation, e.g. 06:00.0, or card serial number [CRU])";
    static constexpr uint32_t bar = 2;
#endif
};


/** Handle bar creation depending on whether we're compiling for CRU or T-RORC
 */
class BarFactory
{
  public:
    /** Crate common::Bar object capable of handling both CRU and T-RORC
     *
     *  Selection whether this is a T-RORC bar or CRU bar is done at compile time
     *  @param id Card id string<br/>
     *            In case of the CRU, this can be a bus-device-function string or
     *            serial number of the card<br/>
     *            In case of the TRORC, it has to be a plain device number in the range [0..3]
     *  @param bar Bar number
     */
    static Bar* makeBar(const std::string& id, const uint32_t bar)
      {
        Bar* b(nullptr);
#if defined BUILD_FOR_TRORC
        try {
          int32_t max_trorcs_supported(4);
          int32_t device_number = stoi(id);
          if ((device_number < 0) || (device_number >= max_trorcs_supported))
            throw trorc::Exception(std::string("Device number not supported: ") + id);
          b = new trorc::Bar(device_number, bar);
        }
        catch (int e) {
          // Uhh... need to get rid of librorc stuff here... catch and rethrow std::runtime_error
          throw trorc::Exception(std::string(" ERROR: Failed to initialize T-RORC: ") + librorc::errMsg(e));
        }
#elif defined BUILD_FOR_CRU
        try {
          b = new cru::Bar(id, bar);
        }
        catch (...) {
          throw std::runtime_error(std::string(" ERROR: Failed to initialize CRU: ") + id);
        }
#endif
        return b;
      }
};


/** Handle SCA interface creation depending on whether we're compiling for CRU or T-RORC
 */
class HdlcFactory
{
  public:
    static HdlcCore* makeHdlcCore(const common::Bar& bar,
                             uint8_t sca_idx,
                             bool request_trid_auto = false)
      {
        HdlcCore* h(nullptr);
#if defined BUILD_FOR_TRORC
        h = new trorc::HdlcSw(bar, sca_idx, request_trid_auto, false);
#elif defined BUILD_FOR_CRU
        h = new cru::HdlcCernMe(bar, sca_idx, request_trid_auto);
#endif
        return h;
      }
};

} // namespace common
