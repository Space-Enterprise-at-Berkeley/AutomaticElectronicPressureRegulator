#pragma once

namespace FlowProfiles {
    float linearRampup(unsigned long flowTime);
    float pressurizationRamp(unsigned long flowTime);
    float constantPressure(unsigned long flowTime);
    float throttledFlowLox(unsigned long flowTime);
    float throttledFlowFuel(unsigned long flowTime);
}