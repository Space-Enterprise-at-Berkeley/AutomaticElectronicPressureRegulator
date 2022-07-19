#include <Arduino.h>
#include <Encoder.h>
#include <data_buff.h>
#include <Comms.h>

// #define MOTOR1 9
// #define MOTOR2 10
// #define MOTOR3 5
// #define MOTOR4 6
// #define ENC1 2
// #define ENC2 3

#define MOTOR1 10
#define MOTOR2 9
#define MOTOR3 6
#define MOTOR4 5
#define ENC1 3
#define ENC2 2

#define PRESSURE_MAXIMUM 1000

#define MAX_SPD 255
#define MIN_SPD -255
// #define STATIC_SPD 60
#define STATIC_SPD 0

#define MAX_ANGLE 1296


#define MIN_ANGLE 0
#define AW_ANGLE_THRESH 150 // in encoder counts 

#define POTPIN A1
#define HP_PT A4
#define LP_PT A0
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
    analogWrite(MOTOR3,-min(0,speed));
    analogWrite(MOTOR4,max(0,speed));
}

double encoderToAngle(double encoderValue) {
    //convert encoder angle to degrees
    return encoderValue/1120.0*360/3.0;
    //return (encoderValue/3200.0)*360*26/48.0;
}

//new encoder angle conversion:
// 45 + encoderValue/1680.0*360

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
            //Serial.println(String(encoder.read()) + "\tInjector:\t" + String(voltageToPressure(analogRead(INJECTOR_PT))) + "\tHP:\t" + String(voltageToHighPressure(analogRead(HP_PT))) + "\tLP:\t" + String(p) + "\t" + String((p-old_p)/((t-old_t)/1e6)) + "\t" + String(p_buff.get_slope()));
            Serial.print("HP: ");
            Serial.print(voltageToHighPressure(analogRead(HP_PT)));
            Serial.print("\t");
            Serial.print("LP: ");
            Serial.println(voltageToPressure(analogRead(LP_PT)));

            #else
            Comms::Packet packet = {.id = 1};
            Comms::packetAddFloat(&packet, sin(t/1e6));
            // Comms::packetAddFloat(&packet, 0.0);
            Comms::packetAddFloat(&packet, 0.0);
            Comms::packetAddFloat(&packet, float(speed));
            Comms::packetAddFloat(&packet, motorAngle);
            Comms::packetAddFloat(&packet, voltageToHighPressure(analogRead(HP_PT)));
            Comms::packetAddFloat(&packet, voltageToPressure(analogRead(LP_PT)));
            Comms::packetAddFloat(&packet, -sin(t/1e6));
            Comms::packetAddFloat(&packet, p_buff.get_slope());
            Comms::packetAddFloat(&packet, cos(t/1e6));
            Comms::emitPacket(&packet);
            #endif
            
        }
        while (Serial.available() > 0) {
            //Read incoming commands
            int inChar = Serial.read();
            delay(1);
            if (inChar == '\n') {
                // Serial.println("Received a thing: " + inString);
                
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
    #ifndef USE_DASHBOARD
    Serial.println("Starting servo test...");
    #endif
    long angle;
    bool isAngleUpdate;
    long oldPosition=-999;
    long e=0;
    long oldError=0;

    long setPoint=  100;
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
        #ifndef USE_DASHBOARD
        if (isPrint && (millis()-lastPrint > 200)){
            Serial.println(String(speed)+"\t"+String(angle)+"\t"+String(setPoint) + "\t" + String(voltageToHighPressure(analogRead(HP_PT))) + "\t" + String(voltageToPressure(analogRead(LP_PT))));
            lastPrint = millis();
        }
        #endif
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
    float kd=0.60e6;

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
            Serial.println(String(millis()) + "\t" + String(speed)+"\t"+String(angle)+"\t"+String(setPoint) + "\t" + String(voltageToHighPressure(analogRead(HP_PT))) + "\t" + String(voltageToPressure(analogRead(LP_PT))));
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

void angleSweep(long startAngle, long endAngle, unsigned long flowDuration, long extraTime, long waitTime) {
    
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
    float kd=0.60e6;

    long errorInt=0;
    unsigned long t2;
    unsigned long dt;
    bool isPrint = true;
    unsigned long lastPrint = 0;

    unsigned long flowStart = millis(); // in millis
    // unsigned long flowDuration = 10000; 
    // long startAngle=500; // in encoder counts
    // long endAngle=1000;

    unsigned int printFreq = 2; // in millis

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
            if (millis()-flowStart-flowDuration > waitTime) {
                setPoint = 0;
                printFreq = 5000;
            }
            
        } else {
            float prog = float(millis()-flowStart)/float(flowDuration);
            setPoint = prog * endAngle + (1-prog) * startAngle;
        }

        if (millis()-flowStart > flowDuration + extraTime + waitTime) { // flow has ended, close valves
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
float kd=0.60e6;
long angle_setpoint=0;
long angle_errorInt=0;

// Constants should be tuned
// kp = 35
// ki = 25e-6
// kd = 2.0e6
// perhaps run filtering on pressure derivative

double kp_outer = 0.20;//0.75;//30; // encoder counts per psi
double ki_outer = 0;//1.125e-6*15;//30.0e-6; // time in micros
double kd_outer = 0;//0.0625;//2.5; // time in s

double final_pressure_setpoint = 520; //130
double pressure_setpoint = 200;
double pressure_e = 0;
double pressure_e_old = 0;
double pressure_errorInt = 0;

Buffer* p_buff;

unsigned long t2;
unsigned long dt;
unsigned long start_time;

double press = 0;
double pressure_scalar = 0;
double last_pressure_time = 0;

boolean pressurize_tank() {
    double pressure_errorInt = 0;
    long t2 = micros();
    long start_time = millis();
    
    long lastPrint = 0;
    double kp_outer_pressurize = 0.5; // encoder counts per psi
    double ki_outer_pressurize = 1.0e-6; // time in micros
    double kd_outer_pressurize = 0.0; // time in s
    Buffer* p_buff_2; // use for pressurize_tank
    p_buff_2 = new Buffer(BUFF_SIZE);
    unsigned long endFlow = 0;

    while (true) {
        angle = encoder.read();
        motorAngle = encoderToAngle(angle);
        potAngle = readPot();
        HPpsi = voltageToHighPressure(analogRead(HP_PT));
        LPpsi = voltageToPressure(analogRead(LP_PT));
        InjectorPT = voltageToPressure(analogRead(INJECTOR_PT));
        
        // LPpsi = analogRead(POTPIN)/1024.0*360;

        if (pressure_scalar < 1)
        {
            if (millis() - last_pressure_time > 20)
            {
                pressure_scalar += .05;
            }
        }
        if (pressure_scalar > 1)
        {
            pressure_scalar = 1;
        }
        pressure_setpoint = final_pressure_setpoint * pressure_scalar;

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
        p_buff_2->insert(t2/1.0e6, LPpsi);
        double rawAngle = -( kp_outer_pressurize*pressure_e + kd_outer_pressurize*(p_buff_2->get_slope()) );
        if(rawAngle<MAX_ANGLE && (rawAngle>MIN_ANGLE || pressure_errorInt<0)){
            pressure_errorInt += pressure_e * dt;
            rawAngle -= ki_outer_pressurize * pressure_errorInt;
        }
        pressure_e_old = pressure_e;

        // Constrain angles and speeds
        angle_setpoint = min(MAX_ANGLE, max(MIN_ANGLE, rawAngle));
        speed = min(max(MIN_SPD,rawSpd),MAX_SPD);

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
            Comms::packetAddFloat(&packet, -kp_outer * pressure_e);
            Comms::packetAddFloat(&packet, -kd_outer * p_buff_2->get_slope());
            Comms::packetAddFloat(&packet, -ki_outer * pressure_errorInt);
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

        if (millis() - start_time > 10000) {
            return false;
        }

    }
    
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
    delay(1000);
    speed = 0;
    runMotor();
    // zero encoder value (so encoder readings range from -x (open) to 0 (closed))
    encoder.write(-20);

    #ifndef USE_DASHBOARD
    waitConfirmation();
    #endif
    //motorDirTest();
    // ptTest();
    //delay(500);
    //waitConfirmation();
    //servoTest();
    //pressurize_tank();
    
    ptTest();
    //exit(0);
    #ifndef USE_DASHBOARD
    Serial.println("Next input will start servo loop, starting setpoint = "+String(pressure_setpoint));
    waitConfirmation();
    #endif

    // pressurize_tank();
    // long startAngle = 300;
    // long endAngle = 1300;
    // long thirdAngle = 300;
    // long flowDuration = 3000; //time in ms for one way
    // Serial.println("Starting angle sweep from "+String(startAngle)+" to "+String(endAngle)+" then back to " + String(thirdAngle) + " over "+String(2*flowDuration)+" ms...");
    
    // potTest();
    //servoTest();
    
    // angleSweep(startAngle, endAngle, flowDuration, 0, 200);
    // angleSweep(endAngle, thirdAngle, flowDuration, 5000, 500);
    // ptTest();
    // exit(0);
    t2 = micros();
    start_time = micros();
    pressure_errorInt = 0;
    
}

long lastPrint = 0;
// Start in closed position, angle should be 0
double slow_set_point = 0;

void loop() {
    
    slow_set_point = (micros()-start_time)/((float) 1.0e6)*pressure_setpoint;

    pressure_setpoint = min(pressure_setpoint, slow_set_point);

    angle = encoder.read();
    motorAngle = encoderToAngle(angle);
    potAngle = readPot();
    HPpsi = voltageToHighPressure(analogRead(HP_PT));
    LPpsi = voltageToPressure(analogRead(LP_PT));
    InjectorPT = voltageToPressure(analogRead(INJECTOR_PT));


    
    // LPpsi = analogRead(POTPIN)/1024.0*360;

    dt=micros()-t2;
    t2+=dt;
    isAngleUpdate=(angle!=oldPosition);
    e=angle-angle_setpoint;

    //Compute Inner PID Servo loop
    float rawSpd=-(kp*e+kd*(e-oldError)/float(dt));
    
    // double updated_i_term = ki*(angle_errorInt + e*dt);
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
    //rawAngle += compute_feedforward(pressure_setpoint, HPpsi);

    if(rawAngle<MAX_ANGLE && (rawAngle>MIN_ANGLE || pressure_errorInt<0)){ // antiwindup check #1
        if(((angle >= angle_setpoint-AW_ANGLE_THRESH) || pressure_e > 0) && ((angle <= angle_setpoint+AW_ANGLE_THRESH) || pressure_e < 0)){
            pressure_errorInt += pressure_e * dt;
        }
        rawAngle -= ki_outer * pressure_errorInt;
    }
    pressure_e_old = pressure_e;

    // Constrain angles and speeds
    angle_setpoint = min(MAX_ANGLE, max(MIN_ANGLE, rawAngle));
    speed = min(max(MIN_SPD,rawSpd),MAX_SPD);

    runMotor();

    if (t2 - lastPrint > 1000) {
        #ifndef USE_DASHBOARD
        Serial.println( String(t2) + "\t"+ String(angle_setpoint) + "\t"+ String(pressure_setpoint) +"\t" + String(speed) + "\t" + String(angle) + "\t" + String(HPpsi) + "\t" + String(LPpsi) + "\t" + String(InjectorPT) + "\t" + String(p_buff->get_slope()) + "\t" + String(pressure_errorInt) );     
        // Serial.print("HP: ");
        // Serial.print(HPpsi);
        // Serial.print("\t");
        // Serial.print("LP: ");
        // Serial.println(LPpsi);
        #else
        Comms::Packet packet = {.id = 1};
        // Comms::packetAddFloat(&packet, sin(t2/1e6));
        Comms::packetAddFloat(&packet, float(angle_setpoint));
        Comms::packetAddFloat(&packet, pressure_setpoint);
        Comms::packetAddFloat(&packet, float(speed));
        Comms::packetAddFloat(&packet, motorAngle);
        Comms::packetAddFloat(&packet, HPpsi);
        Comms::packetAddFloat(&packet, LPpsi);
        Comms::packetAddFloat(&packet, -kp_outer * pressure_e);
        Comms::packetAddFloat(&packet, -kd_outer * p_buff->get_slope());
        Comms::packetAddFloat(&packet, -ki_outer * pressure_errorInt);
        Comms::emitPacket(&packet);
        #endif
        lastPrint = micros();
    }
    
    while (Serial.available() > 0) {
        //Read incoming commands
        int inChar = Serial.read();
        if (inChar == '\n') {
            if (inString == "q") {
                speed = -150;
                runMotor();
                delay(1000);
                speed = 0;
                runMotor();
                exit(0);
            }
            int new_setpt = inString.toInt();
            if (new_setpt >= 0 && new_setpt <= PRESSURE_MAXIMUM) {
                pressure_setpoint=inString.toInt();
            }
            inString = "";
        } else {
            inString += (char)inChar;
        }
        
    }

    if ((micros()-start_time) > 30e6) {
        pressure_setpoint = 0;
    }

}







