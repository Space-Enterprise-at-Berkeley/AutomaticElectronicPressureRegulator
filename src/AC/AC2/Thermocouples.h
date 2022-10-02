
#include "HAL.h"
#include "../Comms.h"

#include <Arduino.h>
#include "Config.h"

namespace Thermocouples {
    extern uint32_t tcUpdatePeriod;

    extern float tc0Value;
    extern float tc1Value;
    extern float tc2Value;
    extern float tc3Value;
    extern float tc4Value; 

    extern float tc0ROC;
    extern float tc1ROC;
    extern float tc2ROC;
    extern float tc3ROC;
    extern float tc4ROC;

    extern Task *abortTC;

    const float thermocoupleAbsoluteThreshold = 200;
    const float thermocoupleThreshold = 150;
    const float thermocoupleRateThreshold = 20;
    const uint8_t hysteresisThreshold = 10;

    void initThermocouples();
    uint32_t tcSample(MAX31855 *amp, uint8_t packetID, float *value, float *thermocoupleValues, float *ROCValue);

    uint32_t tc0Sample();
    uint32_t tc1Sample();
    uint32_t tc2Sample();
    uint32_t tc3Sample();
    uint32_t tc4Sample();
    uint32_t checkForAbort();
    void sendTCAbortPackets(); 
};
