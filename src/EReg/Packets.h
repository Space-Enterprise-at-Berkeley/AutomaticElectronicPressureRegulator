#pragma once

#include <Arduino.h>

namespace Packets {

    const uint8_t TELEMETRY_ID = 0;
    const uint8_t CONFIG_ID = 2;
    const uint8_t DIAGNOSTIC_PASS_ID = 11;
    const uint8_t DIAGNOSTIC_FAIL_ID = 12;
    const uint8_t STATE_TRANSITION_FAIL_ID = 13;

    void sendTelemetry(
        float highPressure,
        float lowPressure,
        float injectorPressure,
        float encoderAngle,
        float angleSetpoint,
        float pressureSetpoint,
        float motorPower,
        float pressureControlP,
        float pressureControlI,
        float pressureControlD
    );
    void sendConfig();
    void sendDiagnostic(boolean pass, char* message);
    void sendStateTransitionError(char* message);
}