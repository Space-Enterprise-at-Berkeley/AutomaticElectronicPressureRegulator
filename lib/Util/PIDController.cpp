#include "PIDController.h"
#include "TimeUtil.h"

PIDController::PIDController(double kp, double ki, double kd, double minOutput, double maxOutput, PIDController::AntiwindupMode antiwindup) {
    k_p = kp, k_i = ki, k_d = kd;
    k_p_nominal = kp, k_i_nominal = ki, k_d_nominal = kd;
    minOutput_ = minOutput, maxOutput_ = maxOutput;
    errorBuffer_ = new Buffer(5);
    switch (antiwindup) {
        case standard:
        antiwindup_ = &PIDController::antiwindupStd;
        break;
        case transientControl:
        antiwindup_ = &PIDController::antiwindupTransientCtrl;
        break;
    }
    this->reset();
}

double PIDController::update(double error) {
    unsigned long curr_time = micros();
    unsigned long dt = TimeUtil::timeInterval(lastUpdate_, curr_time);
    errorBuffer_->insert(TimeUtil::timeInterval(timeStarted_, curr_time)/1.0e6, error);

    latestP_ = -k_p * error;
    latestD_ = -k_d * errorBuffer_->get_slope();
    double output = latestP_ + latestD_;
    intError_ = (this->*antiwindup_)(intError_, output, error, dt);
    latestI_ = -k_i * intError_;
    output += latestI_;

    output = min(max(output, minOutput_), maxOutput_);

    previousError_ = error;
    lastUpdate_ = curr_time;

    return output;
}

double PIDController::antiwindupStd(double integral, double rawOutput, double error, unsigned long dt) {
    if (rawOutput < maxOutput_ && rawOutput > minOutput_) {
        return integral + error * dt;
    } else {
        return 0;
    }
}

double PIDController::antiwindupTransientCtrl(double integral, double rawOutput, double error, unsigned long dt) {
    // TODO: Implement proper antiwindup that accounts for inner loop
    if (rawOutput < maxOutput_ && rawOutput > minOutput_) {
        return integral + error * dt;
    } else {
        return 0;
    }
}

double PIDController::getPTerm() {
    return latestP_;
}

double PIDController::getITerm() {
    return latestI_;
}

double PIDController::getDTerm() {
    return latestD_;
}

void PIDController::reset() {
    k_p = k_p_nominal, k_i = k_i_nominal, k_d = k_d_nominal;
    latestP_ = 0, latestI_ = 0, latestD_ = 0;
    lastUpdate_ = micros();
    timeStarted_ = micros();
    previousError_ = 0;
    intError_ = 0;
    errorBuffer_->clear();
}

void PIDController::updateConstants(double kp, double ki, double kd) {
    k_p = kp;
    k_i = ki;
    k_d = kd;
}