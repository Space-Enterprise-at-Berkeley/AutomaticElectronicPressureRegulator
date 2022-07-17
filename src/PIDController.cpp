#include "PIDController.h"

PIDController::PIDController(double p, double i, double d) {
    p_, i_, d_ = p, i, d;
    lastUpdate_ = micros();
    previousError_ = 0;
}

double PIDController::update(double error) {
    float dt = micros() - lastUpdate_;
    return p_*error+d_*(error-previousError_)/dt;
}