#include "HAL.h"

namespace HAL {

    ADS1115 adc;
    ESP32Encoder encoder;

    void init() {
        pinMode(mainValve1, OUTPUT);
        pinMode(mainValve2, OUTPUT);

        pinMode(motor1, OUTPUT);
        pinMode(motor2, OUTPUT);

        ledcSetup(motor1Channel, 5000, 8);
        ledcSetup(motor2Channel, 5000, 8);

        ledcAttachPin(motor1, motor1Channel);
        ledcAttachPin(motor2, motor2Channel);


        #if defined(IS_INJECTOR)
        pinMode(motor3, OUTPUT);
        pinMode(motor4, OUTPUT);

        ledcSetup(motor3Channel, 5000, 8);
        ledcSetup(motor4Channel, 5000, 8);
        
        ledcAttachPin(motor3, motor3Channel);
        ledcAttachPin(motor4, motor4Channel);
        #endif
        
        pinMode(enc1, INPUT);
        pinMode(enc2, INPUT);

        Wire.begin(5, 32);
        Wire.setClock(400000);
        adc.begin();
        adc.setMode(1); // single shot
        adc.setDataRate(7);
        encoder.attachFullQuad(enc1, enc2);
    }

    
    float readUpstreamPT() {
        #ifndef IS_INJECTOR
        return Ducers::readPressurantPT();
        #else
        return Ducers::readTankPT();
        #endif
    }

    float readDownstreamPT() {
        #ifndef IS_INJECTOR
        return Ducers::readTankPT();
        #else
        return Ducers::readInjectorPT();
        #endif
    }
}