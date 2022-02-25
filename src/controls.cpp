#include <controls.h>
#include <Comms.h>


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
    rawSpd = -(kp * e + kd * (e - oldError) / float(dt));
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

int PID::waitConfirmation(){
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

void PID::angleSweep(long startAngle, long endAngle, unsigned long flowDuration, long extraTime) {
    while (true) {
        updateAngle(encoder.read());
        runMotor();
        if (isPrint && (millis()-lastPrint > printFreq)){
            #ifndef USE_DASHBOARD
            Serial.println(String(millis()) + "\t" + String(speed)+"\t"+String(angle)+"\t"+String(setPoint) + "\t" + String(voltageToHighPressure(analogRead(HP_PT))) + "\t" + String(voltageToPressure(analogRead(LP_PT))));
            #else
            Comms::Packet packet = {.id = 1};
            // Comms::packetAddFloat(&packet, sin(t2/1e6));
            Comms::packetAddFloat(&packet, float(setPoint));
            Comms::packetAddFloat(&packet, 0);
            Comms::packetAddFloat(&packet, float(speed));
            Comms::packetAddFloat(&packet, angle);
            Comms::packetAddFloat(&packet, voltageToHighPressure(analogRead(HP_PT)));
            Comms::packetAddFloat(&packet, voltageToPressure(analogRead(LP_PT)));
            Comms::packetAddFloat(&packet, 0);
            Comms::packetAddFloat(&packet, 0);
            Comms::packetAddFloat(&packet, 0);
            Comms::emitPacket(&packet);
            #endif
            
            lastPrint = millis();
        }

        if (millis()-flowStart > flowDuration) { // flow has ended, close valves
            setPoint = 0;
            printFreq = 5000;
        } else {
            float prog = float(millis()-flowStart)/float(flowDuration);
            setPoint = prog * endAngle + (1-prog) * startAngle;
        }

        if (millis()-flowStart > flowDuration + extraTime) { // flow has ended, close valves
            return;
        }
    }
}


void PID::updatePressure(double kp_outer, double ki_outer, double kd_outer, double pressure_setpoint) {
    //Compute Outer Pressure Control Loop
    outer_e = LPpsi - pressure_setpoint;
    p_buff_2->insert(t2/1.0e6, LPpsi);
    double rawAngle = -( kp_outer*outer_e + kd_outer*(p_buff_2->get_slope()) );
    if(rawAngle<MAX_ANGLE && (rawAngle>MIN_ANGLE || outer_errorInt<0)){
        outer_errorInt += outer_e * dt;
        rawAngle -= ki_outer * outer_errorInt;
    }
    outer_e_old = outer_e;

    // Constrain angles and speeds
    angle_setpoint = min(MAX_ANGLE, max(MIN_ANGLE, rawAngle));
    speed = min(max(MIN_SPD,rawSpd),MAX_SPD);

}

//outerloop will be implemented in the method
//method will take in settings for outerloop as args
//main class loop will be inner loop
boolean PID::pressurize_tank() {
    double pressure_e = 0;
    double pressure_e_old = 0;
    double pressure_errorInt = 0;
    Buffer* p_buff_2; //use for pressureize_tank
    p_buff_2 = new Buffer(BUFF_SIZE);
    unsigned long endFlow = 0;

    while (true) {
        motorAngle = encoderToAngle(angle);
        potAngle = readPot();
        HPpsi = voltageToHighPressure(analogRead(HP_PT));
        LPpsi = voltageToPressure(analogRead(LP_PT));
        InjectorPT = voltageToPressure(analogRead(INJECTOR_PT));
        
        // LPpsi = analogRead(POTPIN)/1024.0*360
        //Compute Inner PID Servo loop
        updateAngle(encoder.read());
        

        //Compute Outer Pressure Control Loop
        pressure_e = LPpsi - pressure_setpoint;
        p_buff_2->insert(t2/1.0e6, LPpsi);
        double rawAngle = -( kp_pressurize*pressure_e + kd_pressureize*(p_buff_2->get_slope()) );
        if(rawAngle<MAX_ANGLE && (rawAngle>MIN_ANGLE || pressure_errorInt<0)){
            pressure_errorInt += pressure_e * dt;
            rawAngle -= ki_pressureize * pressure_errorInt;
        }
        pressure_e_old = pressure_e;

        // Constrain angles and speeds
        angle_setpoint = min(MAX_ANGLE, max(MIN_ANGLE, rawAngle));
        //speed = min(max(MIN_SPD,rawSpd),MAX_SPD);

        if (endFlow > 0) {
            angle_setpoint = 0;
        }

        runMotor();

        if (t2 - lastPrint > 50) {
            #ifndef USE_DASHBOARD
            Serial.println( String(t2) + "\t"+ String(angle_setpoint) + "\t"+ String(pressure_setpoint) +"\t" + String(speed) + "\t" + String(motorAngle) + "\t" + String(HPpsi) + "\t" + String(LPpsi) + "\t" + String(InjectorPT) + "\t" + String(p_buff->get_slope()) + "\t" + String(pressure_errorInt) );     
            #else
            Comms::Packet packet = {.id = 1};
            // Comms::packetAddFloat(&packet, sin(t2/1e6));
            Comms::packetAddFloat(&packet, float(angle_setpoint));
            Comms::packetAddFloat(&packet, pressure_setpoint);
            Comms::packetAddFloat(&packet, float(speed));
            Comms::packetAddFloat(&packet, motorAngle);
            Comms::packetAddFloat(&packet, HPpsi);
            Comms::packetAddFloat(&packet, LPpsi);
            Comms::packetAddFloat(&packet, InjectorPT);
            Comms::packetAddFloat(&packet, p_buff_2->get_slope());
            Comms::packetAddFloat(&packet, pressure_errorInt);
            Comms::emitPacket(&packet);
            #endif
            lastPrint = micros();
        }

        if (LPpsi > pressure_setpoint*0.95 && endFlow <=0) {
            endFlow = millis();
        }

        if (endFlow > 0 && (millis() > (endFlow + 3000))) {
           // finish flow
            speed = 0;
            runMotor();
            return true;
        }

    }
}