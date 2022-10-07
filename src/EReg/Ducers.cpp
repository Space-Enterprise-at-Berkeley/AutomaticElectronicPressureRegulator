#include "Ducers.h"

namespace Ducers {

    int16_t hpPT;
    int16_t lpPT;

    TaskHandle_t adcTask;

    void readADC(void * parameter) {
    for(;;) {
        hpPT = HAL::adc.readADC(0);
        lpPT = HAL::adc.readADC(1);
    }
    }

    void init() {
        xTaskCreatePinnedToCore(
            readADC,
            "adcTask",
            10000,
            NULL,
            0,
            &adcTask,
            0
        );
    }

    float interpolate1000(uint16_t rawValue) {
        float tmp = (float) (rawValue - 6406);
        return tmp / 51.7;
    }

    //TODO verify
    float interpolate5000(uint16_t rawValue) {
        float tmp = (float) rawValue;
        return tmp / 12.97;
    }

    float readHPPT() {
        double voltage = (hpPT * 0.1875)/1000;
        return interpolate1000(voltage);
        // return 4;
    }

    float readLPPT() {
        double voltage = (lpPT * 0.1875)/1000;
        return interpolate1000(voltage);
        // return 3;
    }

    float readInjectorPT() {
        return 0;
    }
}