#include "FlowProfiles.h"
#include "Config.h"
#include "Arduino.h"

namespace FlowProfiles {
    float linearRampup(unsigned long flowTime) {
        return min((Config::pressureSetpoint/Config::rampDuration) * flowTime, Config::pressureSetpoint);
    }
    float pressurizationRamp(unsigned long flowTime) {
        return min((Config::pressureSetpoint/Config::pressurizationRampDuration) * flowTime, Config::pressurizationCutoff);
    }
    float constantPressure(unsigned long flowTime) {
        return Config::pressureSetpoint;
    }
}