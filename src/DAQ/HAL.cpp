#include "HAL.h"

namespace HAL {

    // INA219 chan0;
    // INA219 chan1;
    // INA219 chan2;
    // INA219 chan3;
    // INA219 chan4;
    // INA219 chan5;
    // INA219 chan6;
    // INA219 chan7;
    // INA219 chan8;
    // INA219 chan9;
    // INA219 chan10;

    HX711 lcAmp0;
    HX711 lcAmp1;

    MAX31855 tcAmp0;
    MAX31855 tcAmp1; 
    MAX31855 tcAmp2;

    // ADS1115 adc(0x48);


    void initHAL() {
        // Initialize I2C buses
        // Wire.begin();
        // Wire.setClock(1000000);
        // Wire1.begin();
        // Wire1.setClock(1000000);
        DEBUGLN("began wire");

        SPI.begin();

        DEBUGLN("began SPI");

        // MAX31855 TC amps (cs)
        tcAmp0.init(&SPI, 2);
        DEBUGLN("did init 1");
        tcAmp1.init(&SPI, 5);
        DEBUGLN("did init 2");
        tcAmp2.init(&SPI, 13);
        DEBUGLN("did init 3");
        
        
        DEBUGLN("init SPI");

        // HX711 load cell amps (data out, clk)
        lcAmp0.begin(32, 33);
        lcAmp1.begin(26, 27);

        DEBUGLN("lcAmp begin");

        // initChannel(&chan4, &Wire, 0x44);
        // initChannel(&chan5, &Wire, 0x45);
        // initChannel(&chan6, &Wire, 0x46);
        // initChannel(&chan7, &Wire, 0x47);
        // initChannel(&chan8, &Wire, 0x48);
        // initChannel(&chan9, &Wire, 0x49);
        // initChannel(&chan10, &Wire, 0x4A);

        // Actuator control
        // pinMode(hBrg1Pin1, OUTPUT);
        // pinMode(hBrg1Pin2, OUTPUT);
        // pinMode(hBrg2Pin1, OUTPUT);
        // pinMode(hBrg2Pin2, OUTPUT);
        // pinMode(hBrg3Pin1, OUTPUT);
        // pinMode(hBrg3Pin2, OUTPUT);
        // pinMode(hBrg4Pin1, OUTPUT);
        // pinMode(hBrg4Pin2, OUTPUT);
        // pinMode(hBrg5Pin1, OUTPUT);
        // pinMode(hBrg5Pin2, OUTPUT);
        // pinMode(hBrg6Pin1, OUTPUT);
        // pinMode(hBrg6Pin2, OUTPUT);
        // pinMode(hBrg7Pin1, OUTPUT);
        // pinMode(hBrg7Pin2, OUTPUT);

        // Heater control
        // pinMode(ctl12vChan1, OUTPUT);
        // pinMode(ctl12vChan2, OUTPUT);
        // pinMode(ctl24vChan1, OUTPUT);
        // pinMode(ctl24vChan2, OUTPUT);
    }
};
