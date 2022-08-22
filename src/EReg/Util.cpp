#include "Util.h"
#include <Arduino.h>
#include "HAL.h"
#include "Config.h"
#include <Encoder.h>

namespace Util {

    // valve angle based on pressure setpoint
    PIDController outerController(Config::p_outer_nominal, Config::i_outer_nominal, Config::d_outer_nominal, -PID_RANGE, PID_RANGE, PIDController::transientControl, OUTER_BUFFER_SIZE);
    // motor angle based on valve setpoint

    // motor angle based on encoder/angle setpoint
    PIDController innerController(Config::p_inner, Config::i_inner, Config::d_inner, MIN_SPD, MAX_SPD, PIDController::standard, INNER_BUFFER_SIZE);

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
        return max(1, ((voltage/1024.0*5-0.5)*1000/4.0));
    }

    /**                                        
     * Converts analogRead values from high pressure PT to PSI value
     * @param voltage analog reading from [0, 1023]
     * @return PSI pressure 
     */
    double voltageToHighPressure(double voltage) {
        return max(1, ((((5000.0 * (voltage/1024.0))) * 0.814)+15)); //5V corresponds to 5k psi (after voltage divider)
    }

    /**
     * Computes feedforward value for valve angle during regulated flow
     * @param pressureSetpoint pressure setpoint of propellant tank
     * @param hp high pressure reading. this is necessary because it determines flow rate
     * @return feedforward valve angle in encoder ticks 
     */
    double compute_feedforward(double pressureSetpoint, double hp) {
        return 250 + min(1, pressureSetpoint/hp) * 79;
    }

    /**
     * Compute dynamic PID constants. Since upstream and downstream pressures can change the system dynamics substantially, our PID constants must adapt to reflect this.
     * Note that this function takes calibrated values from Config.h
     * @param highPressure Current upstream pressure in PSI
     * @param lowPressure Current downstream pressure in PSI
     * @return Pid constants
     */
    PidConstants computeDynamicPidConstants(double highPressure, double lowPressure) {
        double dynamicFactor = 1.0;
        // double dynamicFactor = clip(((14.7 + lowPressure)/max(1.0, highPressure)), 0, 1) * (7.8); // nominal is 4000 -> 500 psi flow
        PidConstants dynamicConstants = {
            .k_p = dynamicFactor * Config::p_outer_nominal,
            .k_i = dynamicFactor * Config::i_outer_nominal,
            .k_d = dynamicFactor * Config::d_outer_nominal
            };
        return dynamicConstants;
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