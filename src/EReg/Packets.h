#pragma once

#include <Arduino.h>

namespace Packets {

    const int TELEMETRY_ID = 0;
    const int CONFIG_ID = 2;
    const int DIAGNOSTIC_ID = 3;

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
    void sendDiagnostic();

}