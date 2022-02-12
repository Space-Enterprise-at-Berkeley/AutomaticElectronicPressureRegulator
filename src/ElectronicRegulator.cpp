#include <Arduino.h>
#include <Encoder.h>
#include <data_buff.h>
#include <Comms.h>

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
Encoder encoder(ENC1, ENC2);


// Note: 1 rev on main shaft is 3200 counts
// Encoder itself is 64CPM (including all edges)
int speed=0;

double motorAngle;
double potAngle;
double HPpsi;
double LPpsi;
double InjectorPT;

void runMotor(){
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

    speed = 250;
    runMotor();
    while (millis() - startTime < 1000) {}
    long theta1 = encoder.read();
    String msg = ((theta1-theta0) > 0) ? "\tPASS":"\tFAIL";
    Serial.println("Running motors + direction. t0, t1" + String(theta0) + "\t" +  String(theta1) + msg);
    startTime = millis();
    speed = -speed;
    runMotor();
    while (millis() - startTime < 1000) {}
    long theta2 = encoder.read();
    msg = ((theta2-theta1) < 0) ? "\tPASS":"\tFAIL";
    Serial.println("Running motors + direction. t1, t2: " + String(theta1) + "\t" + String(theta2) + msg);
    speed = 0;
    runMotor();
}

void motorPowerTest() {
    String inString = "";
    speed = 0;
    unsigned long lastPrint = 0;
    long angle = 0;

    while (true){
        runMotor();

        if (millis()-lastPrint > 200){
            angle = encoder.read();
            Serial.println(String(speed)+"\t"+String(angle));
            lastPrint = millis();
        }

        while (Serial.available() > 0) {
            //Read incoming commands
            int inChar = Serial.read();
            
            if (inChar == '\n') {
                if (inString == "fin"){
                    return;
                }else{
                    speed = inString.toInt();
                }
                inString = "";
            } else {
                inString += (char)inChar;
            }
            
        }

    }
}

void ptTest() {
    // print PT reading 6 times, at 0.5s intervals
    #ifndef USE_DASHBOARD
    Serial.println("Starting PT test...");
    #endif
    long lastPrint = 0;
    Buffer p_buff(BUFF_SIZE);
    float old_p = 0;
    float p = 0;
    long t = 0;
    long old_t = 0;
    int count = 0;
    while (true) {
        count++;
        String inString="";
        p = voltageToPressure(analogRead(LP_PT)); 
        t = micros();
        p_buff.insert(double(t)/1.0e6, p);
        if (millis() - lastPrint > 100) {
            
            lastPrint = millis();
            count = 0;

            #ifndef USE_DASHBOARD
            Serial.println(String(count) + "\t Injector: \t" + String(voltageToPressure(analogRead(INJECTOR_PT))) + "\t HP: \t" + String(voltageToPressure(analogRead(HP_PT))) + "\t LP: \t" + String(p) + "\t" + String((p-old_p)/((t-old_t)/1e6)) + "\t" + String(p_buff.get_slope()));
            #else
            Comms::Packet packet = {.id = 1};
            Comms::packetAddFloat(&packet, sin(t/1e6));
            // Comms::packetAddFloat(&packet, 0.0);
            Comms::packetAddFloat(&packet, 0.0);
            Comms::packetAddFloat(&packet, float(speed));
            Comms::packetAddFloat(&packet, motorAngle);
            Comms::packetAddFloat(&packet, HPpsi);
            Comms::packetAddFloat(&packet, LPpsi);
            Comms::packetAddFloat(&packet, InjectorPT);
            Comms::packetAddFloat(&packet, p_buff.get_slope());
            Comms::packetAddFloat(&packet, 0);
            Comms::emitPacket(&packet);
            #endif
            
        }
        while (Serial.available() > 0) {
            //Read incoming commands
            int inChar = Serial.read();
            if (inChar == '\n') {
                if (inString == "start") {
                    return;
                }
                inString = "";
            }
            else{
                inString += (char)inChar;
            }
        }
        old_p = p;
        old_t = t;
    }
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
    // float kp=11.5;
    // float ki=1.5e-6;
    // float kd=0.1665e6;

    float kp=11.5;
    float ki=1.5e-6;
    float kd=0.21e6;

    long errorInt=0;
    unsigned long t2;
    unsigned long dt;
    bool isPrint = true;
    unsigned long lastPrint = 0;

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
        rawSpd += ((rawSpd<0) ? -STATIC_SPD : STATIC_SPD);
        speed=min(max(MIN_SPD,rawSpd),MAX_SPD);
        runMotor();
        if (isPrint && (millis()-lastPrint > 200)){
            Serial.println(String(speed)+"\t"+String(angle)+"\t"+String(setPoint) + "\t" + String(voltageToPressure(analogRead(HP_PT))) + "\t" + String(voltageToPressure(analogRead(LP_PT))));
            lastPrint = millis();
        }
        
        while (Serial.available() > 0) {
            //Read incoming commands
            int inChar = Serial.read();
            
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
                inString = "";
            } else {
                inString += (char)inChar;
            }
            
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

void servoCharacterization() {
    Serial.println("Starting servo-based characterization...");
    long angle;
    bool isAngleUpdate;
    long oldPosition=-999;
    long e=0;
    long oldError=0;

    long setPoint=0;
    // float kp=11.5;
    // float ki=1.5e-6;
    // float kd=0.1665e6;

    float kp=11.5;
    float ki=1.5e-6;
    float kd=0.21e6;

    long errorInt=0;
    unsigned long t2;
    unsigned long dt;
    bool isPrint = true;
    unsigned long lastPrint = 0;

    unsigned long flowStart = millis(); // in millis
    unsigned long flowDuration = 2500;

    unsigned int printFreq = 50; // in millis

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
        rawSpd += ((rawSpd<0) ? -STATIC_SPD : STATIC_SPD);
        speed=min(max(MIN_SPD,rawSpd),MAX_SPD);
        runMotor();
        if (isPrint && (millis()-lastPrint > printFreq)){
            Serial.println(String(millis()) + "\t" + String(speed)+"\t"+String(angle)+"\t"+String(setPoint) + "\t" + String(voltageToPressure(analogRead(HP_PT))) + "\t" + String(voltageToPressure(analogRead(LP_PT))));
            lastPrint = millis();
        }

        if (millis()-flowStart > flowDuration) {
            setPoint = 0;
            printFreq = 5000;
        }
        
        while (Serial.available() > 0) {
            //Read incoming commands
            int inChar = Serial.read();
            
            if (inChar == '\n') {
                if (inString == "fin"){
                    return;
                } else if (inString == "quiet"){
                    isPrint = false;
                } else if (inString == "loud"){
                    isPrint = true;
                } else{
                    // Start a flow
                    isPrint = true;
                    setPoint=inString.toInt();
                    flowStart = millis();
                    printFreq = 50;
                }
                inString = "";
            } else {
                inString += (char)inChar;
            }
            
        }
        if (isAngleUpdate) {
            oldPosition = angle;
            
        }
        oldError=e;
    }
}

void angleSweep(long startAngle, long endAngle, unsigned long flowDuration, long extraTime) {
    
    long angle;
    bool isAngleUpdate;
    long oldPosition=-999;
    long e=0;
    long oldError=0;

    long setPoint=startAngle;

    // float kp=11.5;
    // float ki=1.5e-6;
    // float kd=0.1665e6;

    float kp=11.5;
    float ki=1.5e-6;
    float kd=0.21e6;

    long errorInt=0;
    unsigned long t2;
    unsigned long dt;
    bool isPrint = true;
    unsigned long lastPrint = 0;

    unsigned long flowStart = millis(); // in millis
    // unsigned long flowDuration = 10000; 
    // long startAngle=500; // in encoder counts
    // long endAngle=1000;

    unsigned int printFreq = 20; // in millis

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
        rawSpd += ((rawSpd<0) ? -STATIC_SPD : STATIC_SPD);
        speed=min(max(MIN_SPD,rawSpd),MAX_SPD);
        runMotor();
        if (isPrint && (millis()-lastPrint > printFreq)){
            Serial.println(String(millis()) + "\t" + String(speed)+"\t"+String(angle)+"\t"+String(setPoint) + "\t" + String(voltageToPressure(analogRead(HP_PT))) + "\t" + String(voltageToPressure(analogRead(LP_PT))));
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
       
        if (isAngleUpdate) {
            oldPosition = angle;
            
        }
        oldError=e;
    }
}

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
// ki = 25e-6
// kd = 2.0e6
// perhaps run filtering on pressure derivative
double kp_outer = 40; // encoder counts per psi
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
    #endif

    speed = -150;
    runMotor();
    delay(2000);
    speed = 0;
    runMotor();
    // zero encoder value (so encoder readings range from -x (open) to 0 (closed))
    encoder.write(-20);

    #ifndef USE_DASHBOARD
    waitConfirmation();
    #endif
    // motorDirTest();
    // exit(0);
    ptTest();
    // servoTest();
    // motorPowerTest();
    #ifndef USE_DASHBOARD
    Serial.println("Next input will start servo loop, starting setpoint = "+String(pressure_setpoint));
    waitConfirmation();
    #endif
    // long startAngle = 600*1.08;
    // long endAngle = 1200*1.08;
    // long thirdAngle = 500*1.08;
    // long flowDuration = 4000; //time in ms for one way
    // Serial.println("Starting angle sweep from "+String(startAngle)+" to "+String(endAngle)+" then back to " + String(thirdAngle) + " over "+String(2*flowDuration)+" ms...");
    
    // potTest();
    // servoTest();
    
    //angleSweep(startAngle, endAngle, flowDuration, 0);
    //angleSweep(endAngle, thirdAngle, flowDuration, 5000);
    //exit(0);
    t2 = micros();
    start_time = micros();

    while (pressurize_tank());

    double pressurize_tank_time = 0e6;
    double curr_time = micros();
    while ((micros()-curr_time)>pressurize_tank_time) {
        pressurize_tank();
    }

}

long lastPrint = 0;
// Start in closed position, angle should be 0


double kp_outer_pressurize = 40/10; // encoder counts per psi
double ki_outer_pressurize = 30.0e-6*5; // time in micros
double kd_outer_pressurize = 2.5*4; // time in s

boolean pressurize_tank() {
    angle = encoder.read();
    motorAngle = encoderToAngle(angle);
    potAngle = readPot();
    HPpsi = voltageToPressure(analogRead(HP_PT));
    LPpsi = voltageToPressure(analogRead(LP_PT));
    InjectorPT = voltageToPressure(analogRead(INJECTOR_PT));
    
    // LPpsi = analogRead(POTPIN)/1024.0*360;

    dt=micros()-t2;
    t2+=dt;
    isAngleUpdate=(angle!=oldPosition);
    e=angle-angle_setpoint;

    //Compute Inner PID Servo loop
    float rawSpd=-(kp*e+kd*(e-oldError)/float(dt));
    if(rawSpd<MAX_SPD && rawSpd>MIN_SPD){ //anti-windup
        angle_errorInt+=e*dt;
        rawSpd-=ki*angle_errorInt;
    }
    else{angle_errorInt=0;}
    
    if (isAngleUpdate) {
        oldPosition = angle;
    }
    oldError=e;

    //Compute Outer Pressure Control Loop
    pressure_e = LPpsi - pressure_setpoint;
    p_buff->insert(t2/1.0e6, LPpsi);
    double rawAngle = -( kp_outer_pressurize*pressure_e + kd_outer_pressurize*(p_buff->get_slope()) );
    if(rawAngle<MAX_ANGLE && (rawAngle>MIN_ANGLE || pressure_errorInt<0)){
        pressure_errorInt += pressure_e * dt;
        rawAngle -= ki_outer_pressurize * pressure_errorInt;
    }
    pressure_e_old = pressure_e;

    // Constrain angles and speeds
    angle_setpoint = min(MAX_ANGLE, max(MIN_ANGLE, rawAngle));
    speed = min(max(MIN_SPD,rawSpd),MAX_SPD);

    runMotor();

    if (t2 - lastPrint > 1000) {
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
        Comms::packetAddFloat(&packet, p_buff->get_slope());
        Comms::packetAddFloat(&packet, pressure_errorInt);
        Comms::emitPacket(&packet);
        #endif
        lastPrint = micros();
    }
    
    while (Serial.available() > 0) {
        //Read incoming commands
        int inChar = Serial.read();
        if (inChar == '\n') {
            int new_setpt = inString.toInt();
            if (new_setpt >= 0 && new_setpt <= PRESSURE_MAXIMUM) {
                pressure_setpoint=inString.toInt();
            }
            inString = "";
        } else {
            inString += (char)inChar;
        }
        
    }

    if (LPpsi > pressure_setpoint*0.95) {
        return true;
    }
    return false;

    // if ((micros()-start_time) > 30e6) {
    //     pressure_setpoint = 0;
    // }
}


void loop() {

    angle = encoder.read();
    motorAngle = encoderToAngle(angle);
    potAngle = readPot();
    HPpsi = voltageToPressure(analogRead(HP_PT));
    LPpsi = voltageToPressure(analogRead(LP_PT));
    InjectorPT = voltageToPressure(analogRead(INJECTOR_PT));
    
    // LPpsi = analogRead(POTPIN)/1024.0*360;

    dt=micros()-t2;
    t2+=dt;
    isAngleUpdate=(angle!=oldPosition);
    e=angle-angle_setpoint;

    //Compute Inner PID Servo loop
    float rawSpd=-(kp*e+kd*(e-oldError)/float(dt));
    if(rawSpd<MAX_SPD && rawSpd>MIN_SPD){ //anti-windup
        angle_errorInt+=e*dt;
        rawSpd-=ki*angle_errorInt;
    }
    else{angle_errorInt=0;}
    
    if (isAngleUpdate) {
        oldPosition = angle;
    }
    oldError=e;

    //Compute Outer Pressure Control Loop
    pressure_e = LPpsi - pressure_setpoint;
    p_buff->insert(t2/1.0e6, LPpsi);
    double rawAngle = -( kp_outer*pressure_e + kd_outer*(p_buff->get_slope()) );
    if(rawAngle<MAX_ANGLE && (rawAngle>MIN_ANGLE || pressure_errorInt<0)){
        pressure_errorInt += pressure_e * dt;
        rawAngle -= ki_outer * pressure_errorInt;
    }
    pressure_e_old = pressure_e;

    // Constrain angles and speeds
    angle_setpoint = min(MAX_ANGLE, max(MIN_ANGLE, rawAngle));
    speed = min(max(MIN_SPD,rawSpd),MAX_SPD);

    runMotor();

    if (t2 - lastPrint > 1000) {
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
        Comms::packetAddFloat(&packet, p_buff->get_slope());
        Comms::packetAddFloat(&packet, pressure_errorInt);
        Comms::emitPacket(&packet);
        #endif
        lastPrint = micros();
    }
    
    while (Serial.available() > 0) {
        //Read incoming commands
        int inChar = Serial.read();
        if (inChar == '\n') {
            int new_setpt = inString.toInt();
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







