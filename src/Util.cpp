#include <Util.h>
#include <Arduino.h>
#include <HAL.h>

namespace Util {
    double encoderToAngle(double encoderValue) {
    //convert encoder angle to degrees
    // return 45.0 + (encoderValue/3200.0)*360*26/48.0;
    return (encoderValue/560.0)*360*1/3.0;
    }

    double voltageToPressure(double voltage) {
        //1024 bits in analog read
        //PT voltage frange .4-4.5
        //PT reads from 0-1000
        //Arduino measures voltage from 0 to 5 V
        return (voltage/1024.0*5-0.5)*1000/4.0;
    }

    double voltageToHighPressure(double voltage) {
        return max(1, 6.5929*voltage - 1257.3);
        // return 6.2697*voltage - 1286.7; // new HP PT, based on empirical characterization 12 Feb 22
        // double current = (((voltage/220.0)/1024.0)*5.0);
        // return (current-.004)/.016*5000.0;
    }

    double readPot(){
        return (analogRead(HAL::potPin)/1024.0)*90.0;
    }

    double compute_feedforward(double pressure_setpoint, double hp) {
        //return 700 + (pressure_setpoint/hp) * 140.0; // computed value for ff constant is 140
        //new feedforward value value
        //42.65 = (700/3200.0)*360*26/48.0
        //42.65/360*1680*3 = 597
        //42.65/360*1120*3 = 398

        //8.53=140/3200*360*26/48
        //8.53/360*1120*3 = 79
        return 398 + (pressure_setpoint/hp) * 79;
    }

}