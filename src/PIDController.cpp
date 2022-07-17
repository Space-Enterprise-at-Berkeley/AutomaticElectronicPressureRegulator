#include "PIDController.h"

PIDController::PIDController(double p, double i, double d, double minSpeed, double maxSpeed) {
    p_, i_, d_ = p, i, d;
    minOutput_, maxOutput_ = minSpeed, maxSpeed;
    lastUpdate_ = micros();
    previousError_ = 0;
    intError_ = 0;
}

double PIDController::update(double error) {
    float dt = micros() - lastUpdate_;
    double output = p_*error+d_*(error-previousError_)/dt;
    previousError_ = error;

    if(output < maxOutput_ && output > minOutput_){ //anti-windup
        intError_ += error * dt;
        output += i_ * intError_;
    }else {
        intError_ = 0;
    }

    return output;
}