#include "Util.h"
#include <Arduino.h>
#include "HAL.h"
#include "Config.h"
#include <Encoder.h>

namespace Util {

    // valve angle based on pressure setpoint
    PIDController outerController(Config::p_outer, Config::i_outer, Config::d_outer, -PID_RANGE, PID_RANGE, PIDController::transientControl);
    // motor angle based on valve setpoint

    // motor angle based on encoder/angle setpoint
    PIDController innerController(Config::p_inner, Config::i_inner, Config::d_inner, MIN_SPD, MAX_SPD, PIDController::standard);

    Encoder encoder(HAL::enc1, HAL::enc2);

    PIDController* getInnerController() {
        return &innerController;
    }
    PIDController* getOuterController() {
        return &outerController;
    }
    Encoder* getEncoder() {
        return &encoder;
    }

    /**
     * Converts encoder angle to valve angle in degrees based on motor gear ratio.
     * Currently uncalibrated. DO NOT USE
     */
    double encoderToAngle(double encoderValue) {
        return (encoderValue/560.0)*360*1/3.0;
    }

    /**
     * Converts analogRead values from low pressure PT to PSI value
     * PT voltage frange .4-4.5
     * PT reads from 0-1000
     * @param voltage analog reading from [0, 1023]
     * @return PSI pressure 
     */
    double voltageToLowPressure(double voltage) {
        return max(1, (((voltage/1024.0*5-0.5)*1000/4.0) - 25)); //TODO remove the -30
    }

    /**
     * Converts analogRead values from high pressure PT to PSI value
     * @param voltage analog reading from [0, 1023]
     * @return PSI pressure 
     */
    double voltageToHighPressure(double voltage) {
        return max(1, ((5000.0 * (voltage/1024.0)))); //5V corresponds to 5k psi (after voltage divider)
    }

    /**
     * Computes feedforward value for valve angle during regulated flow
     * @param pressureSetpoint pressure setpoint of propellant tank
     * @param hp high pressure reading. this is necessary because it determines flow rate
     * @return feedforward valve angle in encoder ticks 
     */
    double compute_feedforward(double pressureSetpoint, double hp) {
        return 300 + (pressureSetpoint/hp) * 79; //CHANGED
    }

    /**
     * Non-blocking function to run motors at specified speed. Note that motors will keep running at specified speed until this function is called again.
     * TODO: Implement better control (see DRV8871 documentation) that uses braking function (instead of coasting motors)
     * @param speed Desired speed
     */
    void runMotors(float speed) {
        analogWrite(HAL::motor1,-min(0,speed));
        analogWrite(HAL::motor2,max(0,speed));
    }

    /**
     * Clips specified value to [minOutput, maxOutput]
     * @param value Value to clip
     * @param minOutput lower bound of range to clip to
     * @param maxOutput upper bound of range to clip to
     * @return Clipped value
     */
    double clip(double value, double minOutput, double maxOutput) {
        return min(max(value, minOutput), maxOutput);
    }
}