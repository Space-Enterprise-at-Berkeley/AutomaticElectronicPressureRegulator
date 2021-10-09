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
//   avoid using pins with LEDs attached
Encoder encoder(ENC1, ENC2);


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
    return 45.0 + (encoderValue/3200.0)*360*26/48.0;
}

double voltageToPressure(double voltage) {
    //1024 bits in analog read
    //PT voltage frange .4-4.5
    //PT reads from 0-1000
    //Arduino measures voltage from 0 to 5 V
    return (voltage/1024.0*5-0.5)*1000/4.0;
}

double readPot(){
    return (analogRead(POTPIN)/1024.0)*90.0;
}



//TESTING FUNCTIONS

void motorDirTest() {
    // check encoder and motor directions match (essential for PID loop)
    // run motors in positive direction for 2 seconds (encoder count should increase)
    // run motors in negative direction for 2 seconds (encoder count should decrease)
    long startTime = millis(); 
    long theta0 = encoder.read();

    Serial.println("Starting motor/encoder direction test...");

    speed = 50;
    runMotor();
    while (millis() - startTime < 2000) {}
    long theta1 = encoder.read();
    String msg = ((theta1-theta0) > 0) ? "\tPASS":"\tFAIL";
    Serial.println("Running motors + direction. t0, t1" + String(theta0) + "\t" +  String(theta1) + msg);

    startTime = millis();
    speed = -50;
    runMotor();
    while (millis() - startTime < 2000) {}
    long theta2 = encoder.read();
    msg = ((theta2-theta1) < 0) ? "\tPASS":"\tFAIL";
    Serial.println("Running motors + direction. t1, t2: " + String(theta1) + "\t" + String(theta2) + msg);

}

void ptTest() {
    // print PT reading 6 times, at 0.5s intervals
    Serial.println("Starting PT test...");
    Serial.print("Low Pressure: \t");
    for (int i=0; i<6; i++) {
        Serial.print( String(voltageToPressure(analogRead(LP_PT))) );
        delay(500);
    }
    Serial.print("\n");
    Serial.print("High Pressure: \t");
    for (int i=0; i<6; i++) {
        Serial.print( String(voltageToPressure(analogRead(HP_PT))) );
        delay(500);
    }
    Serial.print("\n");
}

void potTest() {
    Serial.println("Starting potentiometer test...");
    Serial.print("Set pot to 0.");
    while (readPot() > 1){}
    Serial.println("Finished \t" + String(readPot()));
    Serial.print("Set pot to 90.");
    while (readPot() < 89){
        Serial.print(" " + String(readPot()));
        delay(250);
    }
    Serial.println("Finished \t" + String(readPot()));
}

void servoTest() {
    Serial.println("Starting servo test...");
    long angle;
    bool isAngleUpdate;
    long oldPosition=-999;
    long e=0;
    long oldError=0;

    long setPoint=100;
    float kp=11.5;
    float ki=1.5e-6;
    float kd=0.1665e6;

    long errorInt=0;
    unsigned long t2;
    unsigned long dt;
    bool isPrint = false;

    String inString="";

    while (true) {
        dt=micros()-t2;
        t2+=dt;
        angle = encoder.read();
        isAngleUpdate=(angle!=oldPosition);
        e=angle-setPoint;
        //PI control
        float rawSpd=-(kp*e+kd*(e-oldError)/float(dt));
        if(rawSpd<MAX_SPD && rawSpd>MIN_SPD){ //anti-windup
            errorInt+=e*dt;
            rawSpd-=ki*errorInt;
        }
        else{errorInt=0;}
        
        speed=min(max(MIN_SPD,rawSpd),MAX_SPD);
        runMotor();
        if (isPrint){
            Serial.println(String(speed)+"\t"+String(angle)+"\t"+String(setPoint));
        }
        
        while (Serial.available() > 0) {
            //Read incoming commands
            int inChar = Serial.read();
            inString += (char)inChar;
            if (inChar == '\n') {
                if (inString == "fin"){
                    return;
                } else if (inString == "quiet"){
                    isPrint = false;
                } else if (inString == "loud"){
                    isPrint = true;
                } else{
                    setPoint=inString.toInt();
                }
            }
            inString = "";
        }
        if (isAngleUpdate) {
            oldPosition = angle;
            
        }
        oldError=e;
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
    motorDirTest();
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

long lastPrint = 0;

void loop() {
    return;
    motorAngle = encoderToAngle(encoder.read());
    potAngle = readPot();
    HPpsi = voltageToPressure(analogRead(HP_PT));
    LPpsi = voltageToPressure(analogRead(LP_PT));

    currentTime = millis();
    dt = currentTime - lastTime;
    lastTime = currentTime;

    currentError = setpoint-LPpsi;
    
    p = currentError * kP;

    //Anti Integral    
    dI = (currentError - lastError)*dt*kI;;

    if ((prevI > max_i && dI > 0) || (prevI < -max_i && dI > 0)) {
        dI = 0;
    }
    i = prevI + dI;
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

    if (currentTime - lastPrint > 500) {
        Serial.println( String(potAngle) + "\t" + String(motorAngle) + "\t" + String(HPpsi) + "\t" + String(LPpsi) ); // "\t" + String(speed) + ;
    }
}







