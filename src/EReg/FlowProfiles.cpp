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
    float angleCvCharacterization(unsigned long flowTime) {
        float steps[] = {500, 400, 300, 200, 150};
        int numSteps = sizeof(steps)/sizeof(steps[0]);
        unsigned long stepTime = Config::flowDuration/numSteps;
        unsigned int index = flowTime/stepTime;
        index = (index >= numSteps)?(numSteps-1):index;
        return steps[index];
    }
}