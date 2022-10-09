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
        const float steps[] = {500, 400, 300, 200, 150};
        const int numSteps = sizeof(steps)/sizeof(steps[0]);
        unsigned long stepTime = Config::flowDuration/numSteps;
        unsigned int index = flowTime/stepTime;
        index = (index >= numSteps)?(numSteps-1):index;
        return steps[index];
    }
    float throttledFlowLox(unsigned long flowTime) {
        const int numKeypoints = 6;
        const unsigned long keyPointTimes[numKeypoints] = { // these should be arranged in ascending order
            0UL,
            3*1000*1000UL,
            6*1000*1000UL,
            10*1000*1000UL,
            12*1000*1000UL,
            Config::flowDuration
        };
        const float keyPointPressures[numKeypoints] = { // these correspond to keypoints
            250.0,
            250.0,
            480.0,
            480.0,
            250.0,
            250.0
        };

        for (int i = 1; i<numKeypoints; i++) {
            if (flowTime <= keyPointTimes[i]) {
                float p = float(flowTime - keyPointTimes[i-1])/float(keyPointTimes[i] - keyPointTimes[i-1]);
                return p * keyPointPressures[i] + (1-p) * keyPointPressures[i-1];
            }
        }
        // flow ended
        return 0;
    }
    float throttledFlowFuel(unsigned long flowTime) {
        return 0.9034 * throttledFlowLox(flowTime) + 12.32;
    }

    /*
    DONT USE FOR ACTUAL BURNS AND HOTFIRE!
    */
    float nominalTestFlowLox(unsigned long flowTime) {
        const int numKeypoints = 6;
        const unsigned long keyPointTimes[numKeypoints] = { // these should be arranged in ascending order
            0UL,
            3*1000*1000UL,
            6*1000*1000UL,
            10*1000*1000UL,
            12*1000*1000UL,
            Config::flowDuration
        };
        const float keyPointPressures[numKeypoints] = { // these correspond to keypoints
            120.0,
            120.0,
            240.0,
            240.0,
            120.0,
            120.0
        };

        for (int i = 1; i<numKeypoints; i++) {
            if (flowTime <= keyPointTimes[i]) {
                float p = float(flowTime - keyPointTimes[i-1])/float(keyPointTimes[i] - keyPointTimes[i-1]);
                return p * keyPointPressures[i] + (1-p) * keyPointPressures[i-1];
            }
        }
        // flow ended
        return 0;
    }
    float nominalTestFlowFuel(unsigned long flowTime) {
        return 0.9034 * throttledFlowLox(flowTime) + 12.32;
    }

    float flowProfile(unsigned long flowTime) {
        #if defined(FUEL)
            #if defined(IS_INJECTOR)
                return nominalTestFlowFuel(flowTime);
                // return throttledFlowFuel(flowTime);
            #else
                return linearRampup(flowTime);
            #endif
        #elif defined(LOX)
            #if defined(IS_INJECTOR)
                return nominalTestFlowLox(flowTime);
                // return throttledFlowLox(flowTime);
            #else
                return linearRampup(flowTime);
            #endif
        #endif
    }
}