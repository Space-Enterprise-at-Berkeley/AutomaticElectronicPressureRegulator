#pragma once

#include <Arduino.h>
#include <data_buff.h>
#include "math.h"

class PIDController {
    private:
    double k_p, k_i, k_d;
    double minOutput_, maxOutput_;
    double lastUpdate_;
    double previousError_;
    long intError_;
    Buffer* errorBuffer_;


    public:
    PIDController(double kp, double ki, double kd, double minOutput, double maxOutput);
    double update(double error);
};