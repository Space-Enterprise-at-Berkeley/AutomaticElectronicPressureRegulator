#include "Util.h"

namespace Util {

    // valve angle based on pressure setpoint
    PIDController outerController(
        Config::p_outer_nominal, Config::i_outer_nominal, Config::d_outer_nominal, 
        MIN_ANGLE, MAX_ANGLE,
        ANTIWINDUP_RANGE_LOWER, ANTIWINDUP_RANGE_UPPER, 
        PIDController::transientControl, 
        OUTER_BUFFER_SIZE);

    // motor angle based on encoder/angle setpoint
    PIDController innerController(
        Config::p_inner, Config::i_inner, Config::d_inner, 
        MIN_SPD, MAX_SPD, 
        PIDController::standard, 
        INNER_BUFFER_SIZE);

    PIDController* getInnerController() {
        return &innerController;
    }
    PIDController* getOuterController() {
        return &outerController;
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
        return 222.22 * voltage - 61.1;
    }

    /**                                        
     * Converts analogRead values from high pressure PT to PSI value
     * @param voltage analog reading from [0, 1023]
     * @return PSI pressure 
     */
    double voltageToHighPressure(double voltage) {
        return (1111.1 * voltage) + 65; //5V corresponds to 5k psi (after voltage divider)
    }

    /**
     * Computes feedforward value for valve angle during regulated flow
     * @param pressureSetpoint pressure setpoint of propellant tank
     * @param hp high pressure reading. this is necessary because it determines flow rate
     * @return feedforward valve angle in encoder ticks 
     */
    double compute_feedforward(double pressureSetpoint, double hp, unsigned long flowTime) {
        double p = min(1, double(flowTime)/double(Config::rampDuration));
        return p*(250 + min(1, pressureSetpoint/hp) * 79) + (1-p)*150;
    }

    /**
     * Computes feedforward value for injector eReg valve angle during regulated flow
     * Based on characterized Cv data for injector eReg valves
     * @param pressureSetpoint injector pressure setpoint in psi
     * @param tankPressure measured pressure in tank in psi
     * @param flowRate expected flow rate in gallons/min
     * @return feedforward valve angle in encoder ticks 
     */
    double compute_injector_feedforward(double pressureSetpoint, double tankPressure, double flowRate) {
        const float minFeedforwardAngle = Config::minInjectorFeedforwardAngle;
        const float maxFeedforwardAngle = Config::maxInjectorFeedforwardAngle;
        const double cvGradient = 0.004167;
        const double cvInterceptX = 250.0;
        double deltaP = tankPressure - pressureSetpoint;
        if (deltaP < 0.01) { // tank pressure is too low, just open valve all the way
            return maxFeedforwardAngle;
        }
        double feedforwardCv = flowRate * sqrt(PROPELLANT_GRAVITY / abs(deltaP));
        double feedforwardAngle = (feedforwardCv/cvGradient) + cvInterceptX;
        return clip(feedforwardAngle, minFeedforwardAngle, maxFeedforwardAngle);
    }

    /**
     * Computes feedforward value for injector eReg valve angle during characterization
     * Steps open and closed
     * @param flowTime
     * @return feedforward valve angle in encoder ticks 
     */
    double injector_characterization(unsigned long flowTime) {
        // const float steps[] = {600, 500, 550, 450, 500, 400, 450, 350, 400};
        const float steps[] = {350, 450, 400, 500, 450, 550, 500, 600, 550, 700};
        const int numSteps = sizeof(steps)/sizeof(steps[0]);
        unsigned long stepTime = Config::flowDuration/numSteps;
        unsigned int index = flowTime/stepTime;
        index = (index >= numSteps)?(numSteps-1):index;
        return steps[index];
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
     * Compute dynamic PID constants for injector. Ramps up PID constants over 0.5 seconds
     * @param flowTime time in microseconds since flow started
     * @return Pid constants
     */
    PidConstants computeInjectorDynamicPidConstants(unsigned long flowTime) {
        double dynamicFactor = 0;
        if (flowTime > Config::injectorPidStart) {
            dynamicFactor = min(1, double(flowTime - Config::injectorPidStart)/double(Config::injectorPidFull - Config::injectorPidStart));
        } else {
            dynamicFactor = 0;
        }
        PidConstants dynamicConstants = {
            .k_p = dynamicFactor * Config::p_outer_nominal,
            .k_i = dynamicFactor * Config::i_outer_nominal,
            .k_d = dynamicFactor * Config::d_outer_nominal
            };
        return dynamicConstants;
    }

    /**
     * Sine function for debugging
     * @param time timestamp in micros
     */
    double heartBeat(unsigned long time) {
        return sin(1.5*(time/1.0e6));
    }

    /**
     * Non-blocking function to run motors at specified speed. Note that motors will keep running at specified speed until this function is called again.
     * TODO: Implement better control (see DRV8871 documentation) that uses braking function (instead of coasting motors)
     * @param speed Desired speed
     */
    void runMotors(float speed) {
        ledcWrite(HAL::motor1Channel,-min(0,speed));
        ledcWrite(HAL::motor2Channel,max(0,speed));
        #if defined(IS_INJECTOR)
        ledcWrite(HAL::motor3Channel,-min(0,speed));
        ledcWrite(HAL::motor4Channel,max(0,speed));
        #endif
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
