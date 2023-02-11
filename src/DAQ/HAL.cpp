#include "HAL.h"

namespace HAL {

    HX711 lcAmp0;
    HX711 lcAmp1;

    MAX31855 tcAmp0;
    MAX31855 tcAmp1; 
    MAX31855 tcAmp2;

    void initHAL() {
        // Initialize I2C buses
        Wire.begin();
        Wire.setClock(1000000);
        Wire1.begin();
        Wire1.setClock(1000000);
        DEBUGLN("began wire");

        SPI.begin();

        DEBUGLN("began SPI");

        // MAX31855 TC amps (cs)
        pinMode(2, OUTPUT);
        pinMode(5, OUTPUT);
        pinMode(13, OUTPUT);
        tcAmp0.init(&SPI, 2);
        DEBUGLN("did init 1");
        tcAmp1.init(&SPI, 5);
        DEBUGLN("did init 2");
        tcAmp2.init(&SPI, 13);
        DEBUGLN("did init 3");
        
        
        DEBUGLN("init SPI");

        // HX711 load cell amps (data out, clk)
        pinMode(26, INPUT);
        pinMode(27, OUTPUT);
        pinMode(32, INPUT);
        pinMode(33, OUTPUT);
        lcAmp0.begin(32, 33);
        lcAmp1.begin(26, 27);
        DEBUGLN("lcAmp begin");

        pinMode(hBrg1Pin1, OUTPUT);
        pinMode(hBrg1Pin2, OUTPUT);
        digitalWrite(hBrg1Pin1, LOW);
        digitalWrite(hBrg1Pin2, LOW);
        DEBUGLN("actuators begin");
    }
};
