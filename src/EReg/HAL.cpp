#include "HAL.h"

namespace HAL {

    ADS1115 adc(0x48);
    ESP32Encoder encoder;

    void init() {
        pinMode(mainValve1, OUTPUT);
        pinMode(mainValve2, OUTPUT);

        pinMode(motor1, OUTPUT);
        pinMode(motor2, OUTPUT);

        pinMode(enc1, INPUT);
        pinMode(enc2, INPUT);

        Wire.begin(5, 32);
        adc.begin();
        encoder.attachHalfQuad(enc1, enc2);
    }
}