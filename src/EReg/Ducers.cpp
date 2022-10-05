#include "Ducers.h"

namespace Ducers {

    void init() {

    }

    float readHPPT() {
        int16_t adc0 = HAL::adc.readADC_SingleEnded(HAL::hpPT);
        double voltage = (adc0 * 0.1875)/1000;
        return Util::voltageToLowPressure(voltage);
        // return 4;
    }

    float readLPPT() {
        int16_t adc0 = HAL::adc.readADC_SingleEnded(HAL::lpPT);
        double voltage = (adc0 * 0.1875)/1000;
        return Util::voltageToLowPressure(voltage);
        // return 3;
    }

    float readInjectorPT() {
        return 0;
    }
}