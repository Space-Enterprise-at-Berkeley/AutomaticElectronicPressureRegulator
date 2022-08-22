
#include "HAL.h"
#include "../Comms.h"

#include <Arduino.h>

namespace Thermocouples {
    extern uint32_t tcUpdatePeriod;

    extern float tc0Value;
    extern float tc1Value;
    extern float tc2Value;
    extern float tc3Value;
    extern float tc4Value; 

    void initThermocouples();
    uint32_t tcSample();

    uint32_t tc0Sample();
    uint32_t tc1Sample();
    uint32_t tc2Sample();
    uint32_t tc3Sample();
    uint32_t tc4Sample();
};
