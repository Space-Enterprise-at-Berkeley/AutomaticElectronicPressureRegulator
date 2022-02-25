#include <Arduino.h>
#include <Encoder.h>
#include <data_buff.h>
<<<<<<< HEAD
#include <Comms.h>
#include <controls.h>
#include <test.h>
=======
#include <ElectronicRegulator.h>
>>>>>>> a4a9d3d7c4127e0e327824ef44f00aef11cd2d29

#define MOTOR1 6
#define MOTOR2 5
#define ENC1 2
#define ENC2 3

#define PRESSURE_MAXIMUM 1000

#define MAX_SPD 255
#define MIN_SPD -255
// #define STATIC_SPD 60
#define STATIC_SPD 0

#define MAX_ANGLE 1296
#define MIN_ANGLE 0

#define POTPIN A0
#define HP_PT A1
#define LP_PT A2
#define INJECTOR_PT A3

#define BUFF_SIZE 5


#define USE_DASHBOARD 1

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   avoid using pins with LEDs attached
<<<<<<< HEAD
Encoder encoder(ENC1, ENC2);
PID control = PID(11.5, 1.5e-6, 0.1665e6, 0, 100)


// Note: 1 rev on main shaft is 3200 counts
// Encoder itself is 64CPM (including all edges)
int speed=0;

double motorAngle;
double potAngle;
double HPpsi;
double LPpsi;
double InjectorPT;


//start of main loop
=======
Ereg ereg;

//TESTING FUNCTIONS

void motorDirTest() {
    // check encoder and motor directions match (essential for PID loop)
    // run motors in positive direction for 2 seconds (encoder count should increase)
    // run motors in negative direction for 2 seconds (encoder count should decrease)
    long startTime = millis(); 
    long theta0 = encoder.read();

    Serial.println("Starting motor/encoder direction test...");

    ereg.speed = 250;
    ereg.runMotor(); //
    while (millis() - startTime < 1000) {}
    long theta1 = encoder.read();
    String msg = ((theta1-theta0) > 0) ? "\tPASS":"\tFAIL";
    Serial.println("Running motors + direction. t0, t1" + String(theta0) + "\t" +  String(theta1) + msg);
    startTime = millis();
    ereg.speed = -ereg.speed;
    ereg.runMotor();
    while (millis() - startTime < 1000) {}
    long theta2 = encoder.read();
    msg = ((theta2-theta1) < 0) ? "\tPASS":"\tFAIL";
    Serial.println("Running motors + direction. t1, sweep.t2: " + String(theta1) + "\t" + String(theta2) + msg);
    ereg.speed = 0;
    ereg.runMotor();
}

void motorPowerTest() {
    // String inString = "";
    // ereg.speed = 0;
    // unsigned long lastPrint = 0;
    // long angle = 0;
    Ereg motor;

    while (true){
        motor.runMotor();

        if (millis()-motor.lastPrint > 200){
            motor.angle = encoder.read();
            Serial.println(String(motor.speed)+"\t"+String(motor.angle));
            motor.lastPrint = millis();
        }

        while (Serial.available() > 0) {
            //Read incoming commands
            int inChar = Serial.read();
            
            if (inChar == '\n') {
                if (motor.inString == "fin"){
                    return;
                }else{
                    motor.speed = motor.inString.toInt();
                }
                motor.inString = "";
            } else {
                motor.inString += (char)inChar;
            }
            
        }

    }
}

void ptTest() {
    // print PT reading 6 times, at 0.5s intervals
    Serial.println("Starting PT test...");
    ereg.lastPrint = 0;
    Buffer p_buff(BUFF_SIZE);
    float old_p = 0;
    float p = 0;
    long t = 0;
    long old_t = 0;
    int count = 0;
    while (true) {
        count++;
        String inString="";
        p = ereg.voltageToPressure(analogRead(LP_PT)); 
        t = micros();
        p_buff.insert(double(t)/1.0e6, p);
        if (millis() - ereg.lastPrint > 250) {
            Serial.println(String(count) + "\t Injector: \t" + String(ereg.voltageToPressure(analogRead(INJECTOR_PT))) + "\t HP: \t" + String(ereg.voltageToPressure(analogRead(HP_PT))) + "\t LP: \t" + String(p) + "\t" + String((p-old_p)/((t-old_t)/1e6)) + "\t" + String(p_buff.get_slope()));
            ereg.lastPrint = millis();
            count = 0;
        }
        while (Serial.available() > 0) {
            //Read incoming commands
            int inChar = Serial.read();
            if (isDigit(inChar)|| inChar=='-') {
            inString += (char)inChar;
            }
            if (inChar == '\n') {
                return;
            }
        }
        old_p = p;
        old_t = t;
    }
}

void potTest() {
    Serial.println("Starting potentiometer test...");
    Serial.print("Set pot to 0.");
    while (ereg.readPot() > 1){}
    Serial.println("Finished \t" + String(ereg.readPot()));
    Serial.print("Set pot to 90.");
    while (ereg.readPot() < 89){
        Serial.print(" " + String(ereg.readPot()));
        delay(250);
    }
    Serial.println("Finished \t" + String(ereg.readPot()));
}

void servoTest() {
    Serial.println("Starting servo test...");
    Servo test;

    while (true) {
        test.dt=micros()-test.t2;
        test.t2+=test.dt;
        test.angle = encoder.read();
        test.isAngleUpdate=(test.angle!=test.oldPosition);
        test.e=test.angle-test.setPoint;
        //PI control
        //float rawSpd = -(kp*sweep.e+kd*(sweep.e-oldError)/float(sweep.sweep.dt));
        if(test.rawSpd<MAX_SPD && test.rawSpd>MIN_SPD){ //anti-windup
            test.errorInt+=test.e*test.dt;
            test.rawSpd-=test.ki*test.errorInt;
        }
        else{test.errorInt=0;}
        test.rawSpd += ((test.rawSpd<0) ? -STATIC_SPD : STATIC_SPD);
        test.speed=min(max(MIN_SPD,test.rawSpd),MAX_SPD);
        test.runMotor();
        if (test.isPrint && (millis()-test.lastPrint > 200)){
            Serial.println(String(test.speed)+"\t"+String(test.angle)+"\t"+String(test.setPoint) + "\t" + String(test.voltageToPressure(analogRead(HP_PT))) + "\t" + String(test.voltageToPressure(analogRead(LP_PT))));
            test.lastPrint = millis();
        }
        
        while (Serial.available() > 0) {
            //Read incoming commands
            int inChar = Serial.read();
            
            if (inChar == '\n') {
                if (test.inString == "fin"){
                    return;
                } else if (test.inString == "quiet"){
                    test.isPrint = false;
                } else if (test.inString == "loud"){
                    test.isPrint = true;
                } else{
                    test.setPoint=test.inString.toInt();
                }
                test.inString = "";
            } else {
                test.inString += (char)inChar;
            }
            
        }
        if (test.isAngleUpdate) {
            test.oldPosition = test.angle;
            
        }
        test.oldError=test.e;
    }

}

int waitConfirmation(){
    String inString="";
    Serial.println("Waiting for numerical input...");
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

void servoCharacterization() {
    Serial.println("Starting servo-based characterization...");
    Servo characterization;
    characterization.flowDuration = 2500;
    characterization.printFreq = 50; // in millis

    while (true) {
        characterization.dt=micros()-characterization.t2;
        characterization.t2+=characterization.dt;
        characterization.angle = encoder.read();
        characterization.isAngleUpdate=(characterization.angle!=characterization.oldPosition);
        characterization.e=characterization.angle-characterization.setPoint;
        //PI control
        float rawSpd = characterization.rawSpd;
        if(rawSpd<MAX_SPD && rawSpd>MIN_SPD){ //anti-windup
            characterization.errorInt+=characterization.e*characterization.dt;
            rawSpd -= characterization.ki * characterization.errorInt;
        }
        else{characterization.errorInt=0;}
        rawSpd += ((rawSpd<0) ? -STATIC_SPD : STATIC_SPD);
        characterization.speed=min(max(MIN_SPD,rawSpd),MAX_SPD);
        characterization.runMotor();
        if (characterization.isPrint && (millis()-characterization.lastPrint > characterization.printFreq)){
            Serial.println(String(millis()) + "\t" + String(characterization.speed)+"\t"+String(characterization.angle)+"\t"+String(characterization.setPoint) + "\t" + String(characterization.voltageToPressure(analogRead(HP_PT))) + "\t" + String(characterization.voltageToPressure(analogRead(LP_PT))));
            characterization.lastPrint = millis();
        }

        if (millis()-characterization.flowStart > characterization.flowDuration) {
            characterization.setPoint = 0;
            characterization.printFreq = 5000;
        }
        
        while (Serial.available() > 0) {
            //Read incoming commands
            int inChar = Serial.read();
            
            if (inChar == '\n') {
                if (characterization.inString == "fin"){
                    return;
                } else if (characterization.inString == "quiet"){
                    characterization.isPrint = false;
                } else if (characterization.inString == "loud"){
                    characterization.isPrint = true;
                } else{
                    // Start a flow
                    characterization.isPrint = true;
                    characterization.setPoint=characterization.inString.toInt();
                    characterization.flowStart = millis();
                    characterization.printFreq = 50;
                }
                characterization.inString = "";
            } else {
                characterization.inString += (char)inChar;
            }
            
        }
        if (characterization.isAngleUpdate) {
            characterization.oldPosition = characterization.angle;
            
        }
        characterization.oldError=characterization.e;
    }
}

void angleSweep(long startAngle, long endAngle, unsigned long flowDuration, long extraTime) {
    Servo sweep;
    sweep.setPoint=startAngle;
    // unsigned long sweep.flowDuration = 10000; 
    // long startAngle=500; // in encoder counts
    // long endAngle=1000;
    sweep.printFreq = 20; // in millis
    
    while (true) {
        sweep.dt=micros()-sweep.t2;
        sweep.t2+=sweep.dt;
        sweep.angle = encoder.read();
        sweep.isAngleUpdate=(sweep.angle!=sweep.oldPosition);
        sweep.e=sweep.angle-sweep.setPoint;
        //PI control
        float rawSpd=sweep.rawSpd;
        if(rawSpd<MAX_SPD && rawSpd>MIN_SPD){ //anti-windup
            sweep.errorInt+=sweep.e*sweep.dt;
            rawSpd-=sweep.ki*sweep.errorInt;
        }
        else{sweep.errorInt=0;}
        rawSpd += ((rawSpd<0) ? -STATIC_SPD : STATIC_SPD);
        sweep.speed=min(max(MIN_SPD,rawSpd),MAX_SPD);
        sweep.runMotor();
        if (sweep.isPrint && (millis()-sweep.lastPrint > sweep.printFreq)){
            Serial.println(String(millis()) + "\t" + String(sweep.speed)+"\t"+String(sweep.angle)+"\t"+String(sweep.setPoint) + "\t" + String(sweep.voltageToPressure(analogRead(HP_PT))) + "\t" + String(sweep.voltageToPressure(analogRead(LP_PT))));
            sweep.lastPrint = millis();
        }

        if (millis()-sweep.flowStart > sweep.flowDuration) { // flow has ended, close valves
            sweep.setPoint = 0;
            sweep.printFreq = 5000;
        } else {
            float prog = float(millis()-sweep.flowStart)/float(sweep.flowDuration);
            sweep.setPoint = prog * endAngle + (1-prog) * startAngle;
        }

        if (millis()-sweep.flowStart > sweep.flowDuration + extraTime) { // flow has ended, close valves
            return;
        }
       
        if (sweep.isAngleUpdate) {
            sweep.oldPosition = sweep.angle;
            
        }
        sweep.oldError=sweep.e;
    }
}
>>>>>>> a4a9d3d7c4127e0e327824ef44f00aef11cd2d29

long angle;
bool isAngleUpdate;
long oldPosition=-999;
long e=0;
long oldError=0;
String inString = "";

// Inner loop constants
float kp=11.5;
float ki=1.5e-6;
float kd=0.1665e6;
long angle_setpoint=0;
long angle_errorInt=0;

// Constants should be tuned
// kp = 35
// sweep.ki = 25e-6
// kd = 2.0e6
// perhaps run filtering on pressure derivative
double kp_outer = 30; // encoder counts per psi
double ki_outer = 30.0e-6; // time in micros
double kd_outer = 2.5; // time in s
double pressure_setpoint = 130; //100
double pressure_e = 0;
double pressure_e_old = 0;
double pressure_errorInt = 0;

Buffer* p_buff;

unsigned long t2;
unsigned long dt;
unsigned long start_time;



void setup() {
    //Start with valve line perpendicular to body (90 degrees)
    Serial.begin(115200);
    while(!Serial);
    p_buff = new Buffer(BUFF_SIZE);

    delay(500);

    #ifndef USE_DASHBOARD
    waitConfirmation();
    // move to close motors
    Serial.println("Zeroing valve");
<<<<<<< HEAD
    #endif

    control.speed = -150;
    control.runMotor();
    delay(2000);
    control.speed = 0;
    control.runMotor();
=======
    ereg.speed = -150;
    ereg.runMotor();
    delay(2000);
    ereg.speed = 0;
    ereg.runMotor();
>>>>>>> a4a9d3d7c4127e0e327824ef44f00aef11cd2d29
    // zero encoder value (so encoder readings range from -x (open) to 0 (closed))
    encoder.write(-20);

    #ifndef USE_DASHBOARD
    waitConfirmation();
    #endif
    // motorDirTest();
    tests::ptTest();
    delay(500);
    // servoTest();
    // motorPowerTest();
    control.pressurize_tank(1.0, 5.0e-6, 0.0, 130);
    
    control.waitConfirmation();

<<<<<<< HEAD
    #ifndef USE_DASHBOARD
    Serial.println("Next input will start servo loop, starting setpoint = "+String(pressure_setpoint));
=======
    // long startAngle = 600*1.08;
    // long endAngle = 1200*1.08;
    // long thirdAngle = 500*1.08;
    // long sweep.flowDuration = 4000; //time in ms for one way
    // Serial.println("Starting sweep.angle sweep from "+String(startAngle)+" to "+String(endAngle)+" then back to " + String(thirdAngle) + " over "+String(2*sweep.flowDuration)+" ms...");
>>>>>>> a4a9d3d7c4127e0e327824ef44f00aef11cd2d29
    waitConfirmation();
    #endif
    // long startAngle = 300*1.08;
    // long endAngle = 1200*1.08;
    // long thirdAngle = 300*1.08;
    // long thirdAngle = 500*1.08;
    // long flowDuration = 5000; //time in ms for one way
    // Serial.println("Starting angle sweep from "+String(startAngle)+" to "+String(endAngle)+" then back to " + String(thirdAngle) + " over "+String(2*flowDuration)+" ms...");
    
    // potTest();
    // servoTest();
    
<<<<<<< HEAD
    // angleSweep(startAngle, endAngle, flowDuration, 0);
    // angleSweep(endAngle, thirdAngle, flowDuration, 5000);
    // ptTest();
    // exit(0);
=======
    //angleSweep(startAngle, endAngle, sweep.flowDuration, 0);
    //angleSweep(endAngle, thirdAngle, sweep.flowDuration, 5000);
    //exit(0);
>>>>>>> a4a9d3d7c4127e0e327824ef44f00aef11cd2d29
    t2 = micros();
    start_time = micros();

}

long lastPrint = 0;
// Start in closed position, sweep.angle should be 0





void loop() {

    angle = encoder.read();
<<<<<<< HEAD
    motorAngle = control.encoderToAngle(angle);
    potAngle = control.readPot();
    HPpsi = control.voltageToHighPressure(analogRead(HP_PT));
    LPpsi = control.voltageToPressure(analogRead(LP_PT));
    InjectorPT = control.voltageToPressure(analogRead(INJECTOR_PT));
=======
    ereg.motorAngle = ereg.encoderToAngle(angle);
    ereg.potAngle = ereg.readPot();
    ereg.HPpsi = ereg.voltageToPressure(analogRead(HP_PT));
    ereg.LPpsi = ereg.voltageToPressure(analogRead(LP_PT));
    ereg.InjectorPT = ereg.voltageToPressure(analogRead(INJECTOR_PT));
>>>>>>> a4a9d3d7c4127e0e327824ef44f00aef11cd2d29
    
    // LPpsi = analogRead(POTPIN)/1024.0*360;

    dt=micros()-t2;
    t2+=dt;
    isAngleUpdate=(angle!=oldPosition);
    e=angle-angle_setpoint;

    //Compute Inner PID Servo loop
    control.updateAngle(angle);

<<<<<<< HEAD
    //Compute Outer Pressure Control Loop and constrain angles and speeds
    control.updatePressure(30, 30.0e-6, 2.5, 130);

    control.runMotor();

    if (t2 - control.lastPrint > 1000) {
        #ifndef USE_DASHBOARD
        Serial.println( String(t2) + "\t"+ String(angle_setpoint) + "\t"+ String(pressure_setpoint) +"\t" + String(speed) + "\t" + String(motorAngle) + "\t" + String(HPpsi) + "\t" + String(LPpsi) + "\t" + String(InjectorPT) + "\t" + String(p_buff->get_slope()) + "\t" + String(pressure_errorInt) );     
        #else
        Comms::Packet packet = {.id = 1};
        // Comms::packetAddFloat(&packet, sin(t2/1e6));
        Comms::packetAddFloat(&packet, float(control.angle_setpoint));
        Comms::packetAddFloat(&packet, control.pressure_setpoint); //not sure how to send, possibly store these variables in class when calling updatePressure method
        Comms::packetAddFloat(&packet, float(control.speed));
        Comms::packetAddFloat(&packet, control.motorAngle);
        Comms::packetAddFloat(&packet, control.HPpsi);
        Comms::packetAddFloat(&packet, control.LPpsi);
        Comms::packetAddFloat(&packet, control.InjectorPT);
        Comms::packetAddFloat(&packet, control.p_buff->get_slope());
        Comms::packetAddFloat(&packet, control.pressure_errorInt); //not sure how to send, possibly store these variables in class when calling updatePressure method
        Comms::emitPacket(&packet);
        #endif
        lastPrint = micros();
=======
    //Compute Outer Pressure Control Loop
    pressure_e = ereg.LPpsi - pressure_setpoint;
    p_buff->insert(t2/1.0e6, ereg.LPpsi);
    double rawAngle = -( kp_outer*pressure_e + kd_outer*(p_buff->get_slope()) );
    if(rawAngle<MAX_ANGLE && (rawAngle>MIN_ANGLE || pressure_errorInt<0)){
        pressure_errorInt += pressure_e * dt;
        rawAngle -= ki_outer * pressure_errorInt;
    }
    pressure_e_old = pressure_e;

    // Constrain angles and speeds
    angle_setpoint = min(MAX_ANGLE, max(MIN_ANGLE, rawAngle));
    ereg.speed = min(max(MIN_SPD,rawSpd),MAX_SPD);

    ereg.runMotor();

    if (t2 - lastPrint > 50) {
        Serial.println( String(t2) + "\t"+ String(angle_setpoint) + "\t"+ String(pressure_setpoint) +"\t" + String(ereg.speed) + "\t" + String(ereg.motorAngle) + "\t" + String(ereg.HPpsi) + "\t" + String(ereg.LPpsi) + "\t" + String(ereg.InjectorPT) + "\t" + String(p_buff->get_slope()) + "\t" + String(pressure_errorInt) );     
        lastPrint = millis();
>>>>>>> a4a9d3d7c4127e0e327824ef44f00aef11cd2d29
    }
    
    while (Serial.available() > 0) {
        //Read incoming commands
        int inChar = Serial.read();
        if (inChar == '\n') {
            int new_setpt = inString.toInt(); //use class version?
            if (new_setpt >= 0 && new_setpt <= PRESSURE_MAXIMUM) {
                pressure_setpoint=inString.toInt();
            }
            inString = "";
        } else {
            inString += (char)inChar;
        }
        
    }

    // if ((micros()-start_time) > 30e6) {
    //     pressure_setpoint = 0;
    // }

}







