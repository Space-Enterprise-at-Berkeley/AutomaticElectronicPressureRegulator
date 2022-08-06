#pragma once

#include <Arduino.h>
#include <data_buff.h>
#include "math.h"

class PIDController {

    private:
    double k_p, k_i, k_d;
    double latestP_, latestI_, latestD_;
    double minOutput_, maxOutput_;
    double lastUpdate_;
    double previousError_;
    double intError_;
    Buffer* errorBuffer_;
    double (PIDController::*antiwindup_)(double, double, double, long);
    double antiwindupStd(double integral, double rawOutput, double error, long dt);
    double antiwindupTransientCtrl(double integral, double rawOutput, double error, long dt);

    public:
    enum AntiwindupMode { standard, transientControl };
    PIDController(double kp, double ki, double kd, double minOutput, double maxOutput, AntiwindupMode antiwindup);
    double update(double error);
};