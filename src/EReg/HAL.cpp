#include "HAL.h"

namespace HAL {

    Adafruit_ADS1115 adc;
    ESP32Encoder encoder;

    void init() {
        pinMode(mainValve1, OUTPUT);
        pinMode(mainValve2, OUTPUT);

        pinMode(motor1, OUTPUT);
        pinMode(motor2, OUTPUT);

        ledcAttachPin(motor1, motor1Channel);
        ledcAttachPin(motor2, motor2Channel);

        ledcSetup(motor1Channel, 5000, 8);
        ledcSetup(motor2Channel, 5000, 8);

        pinMode(enc1, INPUT);
        pinMode(enc2, INPUT);

        Wire.begin(5, 32);
        Wire.setClock(400000);
        adc.begin(0x48, &Wire);

        adc.setDataRate(RATE_ADS1115_860SPS);
        encoder.attachHalfQuad(enc1, enc2);
    }
}