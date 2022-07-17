#pragma once

#include <Arduino.h>

class PIDController {
    private:
    double p_, i_, d_;
    double lastUpdate_;
    double previousError_;

    public:
    PIDController(double p, double i, double d);
    double update(double error);
};