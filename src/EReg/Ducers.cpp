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

    float interpolate1000(double rawValue) {
        return rawValue * 269.38 - 123.87;
    }

    float interpolate5000(double rawValue) {
        return rawValue * 1368.15 + 26.65;
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