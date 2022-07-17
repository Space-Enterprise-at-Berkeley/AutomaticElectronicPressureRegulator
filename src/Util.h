#pragma once

namespace Util {
    double encoderToAngle(double encoderValue);
    double voltageToPressure(double voltage);
    double voltageToHighPressure(double voltage);
    double readPot();
    double compute_feedforward(double pressure_setpoint, double hp);
}