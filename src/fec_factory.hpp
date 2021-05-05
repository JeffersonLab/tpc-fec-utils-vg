#pragma once

#include "fec_revision.hpp"

struct FecDefaults
{
  uint32_t scacfg_basic_crb_{0x9c};            /**< Configuration for GBT-SCA register CRB */
  uint32_t scacfg_basic_crc_{0x0f};            /**< Configuration for GBT-SCA register CRC */
  uint32_t scacfg_basic_crd_{0x10};            /**< Configuration for GBT-SCA register CRD */
  uint32_t scacfg_gpio_direction_{0xffc003ff}; /**< GPIO pin direction: All output, except for board Id*/
  uint32_t scacfg_gpio_dataout_{0x29800140};   /**< GPIO data out values */
};


/** Factory class for varying FEC settings
 *
 *  Allows to adjust FEC default settings that are applied on initialization depending
 *    on which SCA id (and consequently, after a lookup, SAMPA version) is detected on the FEC
 */
class FecDefaultsFactory
{
  public:
    static FecDefaults makeDefaults(FecRevision::sampa_rev_t sampa_rev)
      {
        FecDefaults d;

        switch (sampa_rev) {
          case FecRevision::SAMPA_REV_UNKNOWN:
          case FecRevision::SAMPA_REV_1:
          case FecRevision::SAMPA_REV_2:
            /* No changes with respect to default */
            break;
          case FecRevision::SAMPA_REV_3:
          case FecRevision::SAMPA_REV_4:
            d.scacfg_gpio_dataout_ = 0x20800140;    /* Adapted SAMPA clk_config to 0x41 for SAMPA v3 & v4 */
            break;
        }

        return d;
      }
};
