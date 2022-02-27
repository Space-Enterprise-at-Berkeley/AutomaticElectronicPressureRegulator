#include <utility.h>

namespace utility {
    void runMotor() {
        analogWrite(MOTOR1,-min(0,speed));
        analogWrite(MOTOR2,max(0,speed));
    }

    double encoderToAngle(double encoderValue) {
        //convert encoder angle to degrees
        // return 45.0 + (encoderValue/3200.0)*360*26/48.0;
        return (encoderValue/3200.0)*360*26/48.0;
    }

    double voltageToPressure(double voltage) {
        //1024 bits in analog read
        //PT voltage frange .4-4.5
        //PT reads from 0-1000
        //Arduino measures voltage from 0 to 5 V
        return (voltage/1024.0*5-0.5)*1000/4.0;
    }

    double voltageToHighPressure(double voltage) {
        return 6.2697*voltage - 1286.7; // new HP PT, based on empirical characterization 12 Feb 22
        // double current = (((voltage/220.0)/1024.0)*5.0);
        // return (current-.004)/.016*5000.0;  
    }

    double readPot() {
        return (analogRead(POTPIN)/1024.0)*90.0;
    }

    int waitConfirmation(){
        #ifndef USE_DASHBOARD
        Serial.println("Waiting for numerical input...");
        #endif
        while (true) {
            while (Serial.available() > 0) {
                //Read incoming commands
                int inChar = Serial.read();
                if (isDigit(inChar)|| inChar=='-') {
                inString += (char)inChar;
                }
                if (inChar == '\n') {
                    return inString.toInt();
                }
            }
        }
    }
}