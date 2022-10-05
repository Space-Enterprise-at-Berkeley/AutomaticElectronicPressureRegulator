#include "Ducers.h"

namespace Ducers {
    Adafruit_ADS1115 ads;
   
    void init() {

    }

    float readHPPT() {
        int16_t adc0;
        int16_t Voltage0;
        adc0 = ads.readADC_SingleEnded(HAL::hpPT);
        Voltage0 = (adc0 * 0.1875)/1000;
        Serial.print(Voltage0);
        return Util::voltageToHighPressure(Voltage0);
    }

    float readLPPT() {
        int16_t adc1;
        int16_t Voltage1;
        adc1 = ads.readADC_SingleEnded(HAL::lpPT);
        Voltage1 = (adc1 * 0.1875)/1000;
        Serial.print(Voltage1);
        return Util::voltageToLowPressure(Voltage1);
    }

    float readInjectorPT() {
        return 0;
    }
}