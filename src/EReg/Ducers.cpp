#include "Ducers.h"

namespace Ducers {

    void init() {

    }

    float readHPPT() {
        HAL::adc.startADCReading(HAL::hpPT, true);
        float reading = HAL::adc.computeVolts(HAL::adc.getLastConversionResults());
        return Util::voltageToHighPressure(reading);
    }

    float readLPPT() {
        HAL::adc.startADCReading(HAL::lpPT, true);
        float reading = HAL::adc.computeVolts(HAL::adc.getLastConversionResults());
        return Util::voltageToLowPressure(reading);
    }

    float readInjectorPT() {
        return 0;
    }
}