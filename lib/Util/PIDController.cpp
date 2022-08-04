#include "PIDController.h"

PIDController::PIDController(double kp, double ki, double kd, double minSpeed, double maxSpeed) {
    k_p, k_i, k_d = kp, ki, kd;
    minOutput_, maxOutput_ = minSpeed, maxSpeed;
    lastUpdate_ = micros();
    previousError_ = 0;
    intError_ = 0;
    errorBuffer_ = new Buffer(5);
}

double PIDController::update(double error) {
    long curr_time = micros();
    long dt = curr_time - lastUpdate_;
    intError_ += error * dt;
    errorBuffer_->insert(dt/1.0e6, error);

    double p = k_p * error;
    double i = k_i * intError_;
    double d = k_d * errorBuffer_->get_slope();

    double output = p + i + d;
    min(max(output, minOutput_), maxOutput_);

    previousError_ = error;
    lastUpdate_ = curr_time;

    return output;
}