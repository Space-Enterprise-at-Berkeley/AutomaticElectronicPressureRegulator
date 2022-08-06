#include "PIDController.h"

PIDController::PIDController(double kp, double ki, double kd, double minOutput, double maxOutput, PIDController::AntiwindupMode antiwindup) {
    k_p, k_i, k_d = kp, ki, kd;
    minOutput_, maxOutput_ = minOutput, maxOutput;
    latestP_, latestI_, latestD_ = 0, 0, 0;
    lastUpdate_ = micros();
    previousError_ = 0;
    intError_ = 0;
    errorBuffer_ = new Buffer(5);
    switch (antiwindup) {
        case standard:
        antiwindup_ = &(PIDController::antiwindupStd);
        break;
        case transientControl:
        antiwindup_ = &(PIDController::antiwindupTransientCtrl);
        break;
    }
}

double PIDController::update(double error) {
    long curr_time = micros();
    long dt = curr_time - lastUpdate_;
    errorBuffer_->insert(dt/1.0e6, error);

    latestP_ = k_p * error;
    latestD_ = k_d * errorBuffer_->get_slope();
    double output = latestP_ + latestD_;
    intError_ = (this->*antiwindup_)(intError_, output, error, dt);
    latestI_ = k_i * intError_;
    output += latestI_;

    min(max(output, minOutput_), maxOutput_);

    previousError_ = error;
    lastUpdate_ = curr_time;

    return output;
}

double PIDController::antiwindupStd(double integral, double rawOutput, double error, long dt) {
    if (rawOutput < maxOutput_ && rawOutput > minOutput_) {
        return integral + error * dt;
    } else {
        return 0;
    }
}

double PIDController::antiwindupTransientCtrl(double integral, double rawOutput, double error, long dt) {
    // TODO: Implement proper antiwindup that accounts for inner loop
    if (rawOutput < maxOutput_ && rawOutput > minOutput_) {
        return integral + error * dt;
    } else {
        return 0;
    }
}