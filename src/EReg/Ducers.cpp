#include "Ducers.h"

namespace Ducers {

    int16_t upstreamPT;
    int16_t downstreamPT;

    TaskHandle_t adcTask;

    void readADC(void * parameter) {
    for(;;) {
        upstreamPT = HAL::adc.readADC(HAL::upstreamPT);
        downstreamPT = HAL::adc.readADC(HAL::downstreamPT);
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

    float interpolate5000(uint16_t rawValue) {
        float tmp = (float) (rawValue - 0.5);
        return tmp * (5000/4.5);
    }

    float readPressurantPT() {
        double voltage = (upstreamPT * 0.1875)/1000;
        return interpolate5000(voltage);
    }

    float readTankPT() {
        double voltage = (downstreamPT * 0.1875)/1000;
        return interpolate1000(voltage);
    }

    float readInjectorPT() {
        double voltage = (downstreamPT * 0.1875)/1000;
        return interpolate1000(voltage);
    }
}