#include "FlowProfiles.h"
#include "Config.h"
#include "Arduino.h"

namespace FlowProfiles {
    float linearRampup(unsigned long flowTime) {
        float p = float(flowTime)/float(Config::rampDuration);
        return min((p*Config::pressureSetpoint) + (1-p)*Config::rampStart, Config::pressureSetpoint);
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
        const int numKeypoints = 7;
        const unsigned long keyPointTimes[numKeypoints] = { // these should be arranged in ascending order
            0UL,
            800*1000UL,
            4*1000*1000UL,
            6*1000*1000UL,
            10*1000*1000UL,
            12*1000*1000UL,
            Config::flowDuration
        };
        const float keyPointPressures[numKeypoints] = { // these correspond to keypoints
            0.0,
            350.0,
            350.0,
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

    float throttledFlowRateLox(unsigned long flowTime) {
        const int numKeypoints = 7;
        const unsigned long keyPointTimes[numKeypoints] = { // these should be arranged in ascending order
            0UL,
            800*1000UL,
            4*1000*1000UL,
            6*1000*1000UL,
            10*1000*1000UL,
            12*1000*1000UL,
            Config::flowDuration
        };
        const float keyPointPressures[numKeypoints] = { // these correspond to keypoints
            0.0,
            11.0,
            11.0,
            12.0,
            12.0,
            10.0,
            10.0
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
    float throttledFlowRateFuel(unsigned long flowTime) {
        return throttledFlowRateLox(flowTime);
    }

    float fullDurationFlowLox(unsigned long flowTime) {
        const int numKeypoints = 3;
        const unsigned long keyPointTimes[numKeypoints] = { // these should be arranged in ascending order
            0UL,
            800*1000UL,
            Config::flowDuration
        };
        const float keyPointPressures[numKeypoints] = { // these correspond to keypoints
            0.0,
            440.0,
            440.0
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
    float fullDurationFlowFuel(unsigned long flowTime) {
        return 0.9034 * fullDurationFlowLox(flowTime) + 12.32;
    }

    float fullDurationFlowRateLox(unsigned long flowTime) {
        const int numKeypoints = 3;
        const unsigned long keyPointTimes[numKeypoints] = { // these should be arranged in ascending order
            0UL,
            800*1000UL,
            Config::flowDuration
        };
        const float keyPointPressures[numKeypoints] = { // these correspond to keypoints
            0.0,
            12.0,
            12.0,
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
    float fullDurationFlowRateFuel(unsigned long flowTime) {
        return fullDurationFlowRateLox(flowTime);
    }


    /*
    DONT USE FOR ACTUAL BURNS AND HOTFIRE!
    This gives pressure setpoint profile for a nominal rate test, for Lox side
    */
    float nominalTestFlowLox(unsigned long flowTime) {
        // const int numKeypoints = 7;
        const int numKeypoints = 5;
        const unsigned long keyPointTimes[numKeypoints] = { // these should be arranged in ascending order
            0UL,
            400*1000UL,
            3*1000*1000UL,
            6*1000*1000UL,
            // 10*1000*1000UL,
            // 12*1000*1000UL,
            Config::flowDuration
        };
        const float keyPointPressures[numKeypoints] = { // these correspond to keypoints
            0.0,
            120.0,
            120.0,
            240.0,
            240.0,
            // 120.0,
            // 120.0
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

    /*
    DONT USE FOR ACTUAL BURNS AND HOTFIRE!
    This gives pressure setpoint profile for a nominal rate test, for fuel side
    */
    float nominalTestFlowFuel(unsigned long flowTime) {
        return 0.9034 * nominalTestFlowLox(flowTime) + 12.32;
    }

    /*
    DONT USE FOR ACTUAL BURNS AND HOTFIRE!
    This gives pressure setpoint profile for a MEOP test, for Lox side
    */
    float meopTestFlowLox(unsigned long flowTime) {
        const int numKeypoints = 7;
        const unsigned long keyPointTimes[numKeypoints] = { // these should be arranged in ascending order
            0UL,
            800*1000UL,
            4*1000*1000UL,
            6*1000*1000UL,
            10*1000*1000UL,
            12*1000*1000UL,
            Config::flowDuration
        };
        const float keyPointPressures[numKeypoints] = { // these correspond to keypoints
            0.0,
            350.0,
            350.0,
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

    /*
    DONT USE FOR ACTUAL BURNS AND HOTFIRE!
    This gives pressure setpoint profile for a MEOP test, for fuel side
    */
    float meopTestFlowFuel(unsigned long flowTime) {
        return 0.9034 * meopTestFlowLox(flowTime) + 12.32;
    }

    /*
    DONT USE FOR ACTUAL BURNS AND HOTFIRE!
    This gives flowrate profile (gallons/min) for a nominal rate test, for Lox side
    */
    float nominalTestFlowRateLox(unsigned long flowTime) {
        const int numKeypoints = 7;
        const unsigned long keyPointTimes[numKeypoints] = { // these should be arranged in ascending order
            0UL,
            800*1000UL,
            4*1000*1000UL,
            6*1000*1000UL,
            10*1000*1000UL,
            12*1000*1000UL,
            Config::flowDuration
        };
        const float keyPointPressures[numKeypoints] = { // these correspond to keypoints
            0.0,
            11.0,
            11.0,
            12.0,
            12.0,
            10.0,
            10.0
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

    /*
    DONT USE FOR ACTUAL BURNS AND HOTFIRE!
    This gives flowrate profile (gallons/min) for a nominal rate test, for fuel side
    */
    float nominalTestFlowRateFuel(unsigned long flowTime) {
        return nominalTestFlowRateLox(flowTime);
    }

    float flowPressureProfile(unsigned long flowTime) {
        #if defined(FUEL)
            #if defined(IS_INJECTOR)
                // return meopTestFlowFuel(flowTime);
                // return throttledFlowFuel(flowTime);
                return fullDurationFlowFuel(flowTime);
            #else
                return linearRampup(flowTime);
            #endif
        #elif defined(LOX)
            #if defined(IS_INJECTOR)
                // return meopTestFlowLox(flowTime);
                // return throttledFlowLox(flowTime);
                return fullDurationFlowLox(flowTime);
            #else
                return linearRampup(flowTime);
            #endif
        #endif
    }

    float flowRateProfile(unsigned long flowTime) {
        #if defined(IS_INJECTOR)
            #if defined(FUEL)
                // return nominalTestFlowRateFuel(flowTime);
                // return throttledFlowRateFuel(flowTime);
                return fullDurationFlowRateFuel(flowTime);
            #elif defined(LOX)
                // return nominalTestFlowRateLox(flowTime);
                // return throttledFlowRateLox(flowTime);
                return fullDurationFlowRateLox(flowTime);
            #endif
        #endif
        return 0;
    }
}