#include "PIDController.h"

PIDController::PIDController(double p, double i, double d, double minSpeed, double maxSpeed) {
    p_, i_, d_ = p, i, d;
    minOutput_, maxOutput_ = minSpeed, maxSpeed;
    lastUpdate_ = micros();
    previousError_ = 0;
    intError_ = 0;
}

double PIDController::update(double error) {
    float curr_time = micros();
    float dt = curr_time - lastUpdate_;
    double output = p_*error + d_*(error-previousError_)/dt;

    intError_ += error * dt;
    output += i_ * intError_;

    output = output < minOutput_ ? minOutput_ : output;
    output = output > maxOutput_ ? maxOutput_ : output;

    previousError_ = error;
    lastUpdate_ = curr_time;

    return output;
}