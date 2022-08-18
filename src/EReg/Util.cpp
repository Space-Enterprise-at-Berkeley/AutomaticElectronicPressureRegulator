#include "Util.h"
#include <Arduino.h>
#include "HAL.h"
#include "Config.h"
#include <Encoder.h>

namespace Util {

    // valve angle based on pressure setpoint
    PIDController outerController(Config::p_outer, Config::i_outer, Config::d_outer, MIN_ANGLE, MAX_ANGLE, PIDController::transientControl);
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


    double encoderToAngle(double encoderValue) {
    //convert encoder angle to degrees
    // return 45.0 + (encoderValue/3200.0)*360*26/48.0;
    return (encoderValue/560.0)*360*1/3.0;
    }

    double voltageToLowPressure(double voltage) {
        //1024 bits in analog read
        //PT voltage frange .4-4.5
        //PT reads from 0-1000
        //Arduino measures voltage from 0 to 5 V
        return max(1, (((voltage/1024.0*5-0.5)*1000/4.0) - 25)); //TODO remove the -30
    }

    double voltageToHighPressure(double voltage) {
        // return max(1, 6.5929*voltage - 1257.3);
        // return 6.2697*voltage - 1286.7; // new HP PT, based on empirical characterization 12 Feb 22
        // // double current = (((voltage/220.0)/1024.0)*5.0);
        // // return (current-.004)/.016*5000.0;
        return max(1, ((5000.0 * (voltage/1024.0)))); //5V corresponds to 5k psi (after voltage divider)
    }

    double compute_feedforward(double pressure_setpoint, double hp) {
        //return 700 + (pressure_setpoint/hp) * 140.0; // computed value for ff constant is 140
        //new feedforward value value
        //42.65 = (700/3200.0)*360*26/48.0
        //42.65/360*1680*3 = 597
        //42.65/360*1120*3 = 398

        //8.53=140/3200*360*26/48
        //8.53/360*1120*3 = 79
        return 300 + (pressure_setpoint/hp) * 79; //CHANGED
    }

    void runMotors(float speed) {
        analogWrite(HAL::motor1,-min(0,speed));
        analogWrite(HAL::motor2,max(0,speed));
    }

}