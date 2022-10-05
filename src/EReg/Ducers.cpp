#include "Ducers.h"

namespace Ducers {

    void init() {

    }

    float interpolate1000(uint16_t rawValue) {
        float tmp = (float) (rawValue - 6406);
        return tmp / 51.7;
    }

    float interpolate5000(uint16_t rawValue) {
        float tmp = (float) rawValue;
        return tmp / 12.97;
    }

    float readHPPT() {
        int16_t adc0 = HAL::adc.readADC_SingleEnded(HAL::hpPT);
        double voltage = (adc0 * 0.1875)/1000;
        return interpolate1000(voltage);
        // return 4;
    }

    float readLPPT() {
        int16_t adc0 = HAL::adc.readADC_SingleEnded(HAL::lpPT);
        double voltage = (adc0 * 0.1875)/1000;
        return interpolate1000(voltage);
        // return 3;
    }

    float readInjectorPT() {
        return 0;
    }
}