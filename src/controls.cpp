#include <controls.h>
#include <config.h>


PID::PID(float kp, float ki, float kd, double setPoint, bool method) {
    kp = kp;
    ki = ki;
    kd = kd;
    setPoint = setPoint;
    method = method; //true is outer, false is inner
}

//Outer loop does pressure and returns angle to inner loop, 
//and inner loop takes angle and returns speed
//control class should only be for the actual control loop

double PID::update(double input) {
    //input is LPpsi for pressure and angle for angle
    Buffer* p_buff = new Buffer(BUFF_SIZE);
    long dt = micros() - time;
    time += dt;
    error = input - setPoint;
    //isAngleUpdate = (angle != oldPosition);
    

    //PID control
    float rawValue = -(kp * error + kd * (p_buff->get_slope()));
    oldError = error;

    //anti-windup
    if (method) {
        if (rawValue < maxAngle && (rawValue > minAngle || errorInt < 0)) {
            errorInt += error * dt;
            rawValue -= ki * errorInt;
        }
        return min(maxAngle, max(minAngle, rawValue));
    } else {
        if (rawValue < maxSpd && rawValue > minSpd) {
            errorInt += error * dt;
            rawValue -= ki * errorInt;
        } else {errorInt = 0;}
        rawValue += ((rawValue < 0)? -staticSpd : staticSpd);
        return min(max(minSpd, rawValue), maxSpd);
        // if (isAngleUpdate) {
        //     oldPosition = input;
        // }
        
    }
    
}

//create transmit function to return value of time and pressure_error
double PID::transmit() {
    return time, error;
}
