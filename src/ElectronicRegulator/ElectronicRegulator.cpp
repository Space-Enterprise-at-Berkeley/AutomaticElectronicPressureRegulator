#include <Arduino.h>
#include <Encoder.h>

#define MOTOR1 6
#define MOTOR2 5
#define ENC1 2
#define ENC2 3

#define MAX_SPD 255
#define MIN_SPD -255

#define POTPIN A0
#define HP_PT A1
#define LP_PT A2

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder encoder(ENC1, ENC2);
//   avoid using pins with LEDs attached

// Note: 1 rev on main shaft is 3200 counts
// Encoder itself is 64CPM (including all edges)

int speed=0;

double motorAngle;

double potAngle;

double HPpsi;
double LPpsi;

void runMotor(){
    analogWrite(MOTOR1,-min(0,speed));
    analogWrite(MOTOR2,max(0,speed));
}

double encoderToAngle(double encoderValue) {
    //convert encoder angle to degrees
    return (long) 45+encoderValue/3200*360*26/48.0;
}

double voltageToPressure(double voltage) {
    //1024 bits in analog read
    //PT voltage frange .4-4.5
    //PT reads from 0-1000
    //Arduino measures voltage from 0 to 5 V
    return (voltage/1024.0*5-0.5)*1000/4.0;
}

double setpoint = 150;
double kP = 10;
double kI = 0;
double kD = 0;

void setup() {
    //Start with valve line perpendicular to body (90 degrees)
    Serial.begin(115200);
    delay(2000);
    analogWrite(MOTOR1,0);
    analogWrite(MOTOR2,0);
}

double lastTime = 0;
double currentTime;
double lastError = 0;
double currentError;
double dt = 0;

double p;
double i;
double dI;
double prevI = 0;
double d;

double max_i = 50;

void loop() {
    motorAngle = encoderToAngle(encoder.read());
    potAngle = analogRead(POTPIN)/1024.0*90.0;
    HPpsi = voltageToPressure(analogRead(HP_PT));
    LPpsi = voltageToPressure(analogRead(LP_PT));

    currentTime = millis();
    dt = currentTime - lastTime;
    lastTime = currentTime;

    currentError = setpoint-LPpsi;
    
    p = currentError * kP;

    //Anti Integral    
    dI = (currentError - lastError)*dt*kI;;

    if (prevI > max_i && dI < 0) {
        i = prevI + dI;
    }
    else if (prevI < -max_i && dI > 0) {
        i = prevI + dI;
    }
    else {
        i = prevI + dI;
    }
    
    prevI = i;
    
    d = (currentError-lastError)/dt*kD;

    lastError = currentError;

    speed = p + i + d;

    
    //Potentiometer control

    // if (motorAngle > potAngle) {
    //     speed = -100;
    // }
    // else {
    //     speed = +100;
    // }

    // if (abs(potAngle-motorAngle)<5) {
    //     speed = 0;
    // }

    if (motorAngle < 0) {
        if (speed<0) {
            speed = 0;
        }
    }
    if (motorAngle > 90) {
        if (speed>0) {
            speed = 0;
        }
    }

    speed=min(max(MIN_SPD,speed),MAX_SPD);
    runMotor();
    //Serial.println("High Presure PSI: " + String(HPpsi) + "Low Presure PSI: " + String(LPpsi)  + "\t Motor Angle:"+String(motorAngle) + "\t Speed: " + String(speed) + "\t Pot Angle:" + String(potAngle));
    
    //Check values on serial plot.
    Serial.println(setpoint);
    Serial.println(LPpsi);
}







