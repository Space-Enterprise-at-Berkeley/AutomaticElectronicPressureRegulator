#pragma once

#include <Arduino.h>

class PIDController {
    private:
    double p_, i_, d_;
    double minOutput_, maxOutput_;
    double lastUpdate_;
    double previousError_;
    long intError_;

    public:
    PIDController(double p, double i, double d, double minOutput, double maxOutput);
    double update(double error);
};