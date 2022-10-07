#pragma once

#include <Arduino.h>

namespace Packets {

    const uint8_t ABORT_ID = 201;
    const uint8_t TELEMETRY_ID = 1;
    const uint8_t CONFIG_ID = 2;
    const uint8_t DIAGNOSTIC_ID = 3;
    const uint8_t STATE_TRANSITION_FAIL_ID = 4;
    const uint8_t FLOW_STATE = 5;

    void sendTelemetry(
        float upstreamPressure,
        float downstreamPressure,
        float encoderAngle,
        float angleSetpoint,
        float pressureSetpoint,
        float motorPower,
        float pressureControlP,
        float pressureControlI,
        float pressureControlD
    );
    void sendConfig();
    void sendDiagnostic(uint8_t motorDirPass, uint8_t servoPass);
    void sendStateTransitionError(uint8_t errorCode);
    void sendFlowState(uint8_t flowState);
    void broadcastAbort();
}