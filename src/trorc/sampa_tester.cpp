#include "sampa_tester.hpp"

// SampaTester ADC mappings/conversions
const std::array<SampaTester::adc_input_t, 11> SampaTester::adc_inputs_ = {
    std::make_tuple("VDDA",     "V",   0, 1./4096*2.048),
    std::make_tuple("VREF",     "V",   5, 1./4096*2.048),
    std::make_tuple("VDD ",     "V",   6, 1./4096*2.048),
    std::make_tuple("V450",     "V",   9, 1./4096*2.048),
    std::make_tuple("V600",     "V",   8, 1./4096*2.048),
    std::make_tuple("V750",     "V",  10, 1./4096*2.048),
    std::make_tuple("FE1_CURR", "V",  1, 1./4096*2.048),
    std::make_tuple("FE2_CURR", "V",  3, 1./4096*2.048),   // Wrong input in schematic: 7
    std::make_tuple("AD_CURR",  "V",  2, 1./4096*2.048),
    std::make_tuple("DR_CURR",  "V",  7, 1./4096*2.048),   // Wrong input in schematic: 3
    std::make_tuple("DG_CURR",  "V",  4, 1./4096*2.048)
    /* Measurements from Anders:
       Current in mA is 1000*(y-m)/50/0.1 = 200*(y-m), where y is in V
       m is different for different stations
        => we do the conversion to mA "offline", from the logs
    */
  };


// SampaTester DAC mappings
const std::array<SampaTester::dac_output_t, 4> SampaTester::dac_outputs_ = {
    std::make_tuple("CG0", 0),
    std::make_tuple("POL", 1),
    std::make_tuple("CTS", 2),
    std::make_tuple("CG1", 3)
  };
