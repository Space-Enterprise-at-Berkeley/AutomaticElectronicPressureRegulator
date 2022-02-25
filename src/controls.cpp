# include <controls.h>


PID::PID(float kp, float ki, float kd, long angle, long setPoint) {
    kp = kp;
    ki = ki;
    kd = kd;
    angle = angle;
    setPoint = setPoint;
}

void PID::runMotor() {
    analogWrite(MOTOR1,-min(0,speed));
    analogWrite(MOTOR2,max(0,speed));
}

double PID::encoderToAngle(double encoderValue) {
    //convert encoder angle to degrees
    // return 45.0 + (encoderValue/3200.0)*360*26/48.0;
    return (encoderValue/3200.0)*360*26/48.0;
}

double PID::voltageToPressure(double voltage) {
    //1024 bits in analog read
    //PT voltage frange .4-4.5
    //PT reads from 0-1000
    //Arduino measures voltage from 0 to 5 V
    return (voltage/1024.0*5-0.5)*1000/4.0;
}

double PID::voltageToHighPressure(double voltage) {
    return 6.2697*voltage - 1286.7; // new HP PT, based on empirical characterization 12 Feb 22
    // double current = (((voltage/220.0)/1024.0)*5.0);
    // return (current-.004)/.016*5000.0;
}

double PID::readPot() {
    return (analogRead(POTPIN)/1024.0)*90.0;
}

void PID::updatePT() {
    Buffer p_buff(BUFF_SIZE);
    inString="";
    pressure = voltageToPressure(analogRead(LP_PT));
    time = micros();
    p_buff.insert(double(time)/1.0e6, pressure);
    old_pressure = pressure;
    old_time = time;
}

void PID::updateAngle(long angle) {
    dt = micros() - t2;
    t2 += dt;
    isAngleUpdate = (angle != oldPosition);
    e = angle - setPoint;
    //PI control
    float rawSpd = -(kp * e + kd * (e - oldError) / float(dt));
    if (rawSpd < MAX_SPD && rawSpd > MIN_SPD) { //anti-windup
        errorInt += e * dt;
        rawSpd -= ki * errorInt;
    }
    else {errorInt = 0;}
    rawSpd += ((rawSpd < 0)? -STATIC_SPD : STATIC_SPD);
    speed = min(max(MIN_SPD,rawSpd), MAX_SPD);

    if (isAngleUpdate) {
        oldPosition = angle;
    }
    oldError = e;

}
