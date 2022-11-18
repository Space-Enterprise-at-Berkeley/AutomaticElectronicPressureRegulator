#pragma once

#include <Common.h>

#include <ADS8167.h>
#include <ADS1X15.h>
#include <MAX31855.h>
#include <HX711.h>

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

namespace HAL {

    // extern ADS1115 adc;
    
    extern HX711 lcAmp0;
    extern HX711 lcAmp1;

    extern MAX31855 tcAmp0;
    extern MAX31855 tcAmp1; 
    extern MAX31855 tcAmp2;

    const float battShuntR = 0.002;
    const float battCurrMax = 8.0;
    const float supplyShuntR = 0.01;
    const float supplyCurrMax = 4.0;

    const float chanShuntR = 0.033;
    const float chanCurrMax = 4.0;


    const uint8_t hBrg1Pin1 = 2; // TODO change these
    const uint8_t hBrg1Pin2 = 3; 

    const uint8_t ctl12vChan1 = 14;
    const uint8_t ctl12vChan2 = 15;
    
    const uint8_t ctl24vChan1 = 22;
    const uint8_t ctl24vChan2 = 23;

    void initHAL();
    
};