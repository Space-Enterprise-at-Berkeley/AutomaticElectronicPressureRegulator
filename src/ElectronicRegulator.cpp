#include <Arduino.h>

#include <Encoder.h>
#include <data_buff.h>
#include <Comms.h>
#include <PIDController.h>
#include <HAL.h>
#include <Util.h>

#define PRESSURE_MAXIMUM 1000

#define MAX_SPD 255
#define MIN_SPD -255
// #define STATIC_SPD 60
#define STATIC_SPD 0
//old max angle = 1296
//3200*48/26
#define MAX_ANGLE 363
#define MIN_ANGLE 0

#define BUFF_SIZE 5

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   avoid using pins with LEDs attached
Encoder encoder(HAL::enc1, HAL::enc2);


// Note: 1 rev on main shaft is 3200 counts
// Encoder itself is 64CPM (including all edges)
int speed=0;
bool startFlow = false;
long lastPrint = 0;

double motorAngle;
double potAngle;
double HPpsi;
double LPpsi;
double InjectorPT;

void runMotor(){
    analogWrite(HAL::motor1,-min(0,speed));
    analogWrite(HAL::motor2,max(0,speed));
    analogWrite(HAL::motor3,-min(0,speed));
    analogWrite(HAL::motor4,max(0,speed));
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
// float kd=0.1665e6;
float kd=0.35e6;
long angle_setpoint=0;
long angle_errorInt=0;

// Constants should be tuned
// kp = 35
// ki = 25e-6
// kd = 2.0e6
// perhaps run filtering on pressure derivative

// 2k psi
double kp_outer = 1.5;//30; // encoder counts per psi
double ki_outer = 2.25e-6;//30.0e-6; // time in micros
double kd_outer = 0.125;//2.5; // time in s

// double kp_outer = 0.75;//30; // encoder counts per psi
// double ki_outer = 1.125e-6;//30.0e-6; // time in micros
// double kd_outer = 0.0625;//2.5; // time in s

double pressure_setpoint = 600; //130
double pressure_e = 0;
double pressure_e_old = 0;
double pressure_errorInt = 0;

Buffer* p_buff;

unsigned long t2;
unsigned long dt;
unsigned long start_time;



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
        motorAngle = Util::encoderToAngle(angle);
        potAngle = Util::readPot();
        HPpsi = Util::voltageToHighPressure(analogRead(HP_PT));
        LPpsi = Util::voltageToPressure(analogRead(LP_PT));
        InjectorPT = Util::voltageToPressure(analogRead(INJECTOR_PT));
        
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
    return 196 + (pressure_setpoint/hp) * 39; // computed value for ff constant is 140

}

void sendToAC(Comms::Packet packet) {
    Serial1.write(packet.id);
    Serial1.write(packet.len);
    Serial1.write(packet.timestamp, 4);
    Serial1.write(packet.checksum, 2);
    Serial1.write(packet.data, packet.len);
    Serial1.write('\n');
}


//TODO fill out command methods
void zero(Comms::Packet packet) {
    speed = -150;
    runMotor();
    delay(1000);
    speed = 0;
    runMotor();
    // zero encoder value (so encoder readings range from -x (open) to 0 (closed))
    encoder.write(-20);}

void setPressureSetpoint(Comms::Packet packet) {
    pressure_setpoint = Comms::packetGetFloat(&packet, 0);
}

void regulation(Comms::Packet packet) {

}

void flow(Comms::Packet packet) {
    startFlow = packet.data[0];
}

void fullOpen(Comms::Packet packet) {

}

void acMessage(Comms::Packet packet) {

}

void setup() {
    Serial.begin(9600);
    Serial1.begin(115200);

    PIDController outerController(1.50, 2.25e-6, 0.125);
    PIDController innerController(11.5, 1.5e-6, 0.35e-6);

    Comms::registerCallback(0, zero);
    Comms::registerCallback(1, setPressureSetpoint);
    Comms::registerCallback(2, regulation);
    Comms::registerCallback(3, flow);
    Comms::registerCallback(4, fullOpen);
}

void loop() {
    Comms::processWaitingPackets();

    if (!startFlow) {
        //reset
        t2 = micros();
        start_time = micros();
        pressure_errorInt = 0;
    } else {
        // start controller
        angle = encoder.read();
        motorAngle = Util::encoderToAngle(angle);
        potAngle = Util::readPot();
        HPpsi = Util::voltageToHighPressure(analogRead(HP_PT));
        LPpsi = Util::voltageToPressure(analogRead(LP_PT));
        InjectorPT = Util::voltageToPressure(analogRead(INJECTOR_PT));
        
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
        rawAngle += compute_feedforward(pressure_setpoint, HPpsi);
        if(rawAngle<MAX_ANGLE && (rawAngle>MIN_ANGLE || pressure_errorInt<0)){
            pressure_errorInt += pressure_e * dt;
            rawAngle -= ki_outer * pressure_errorInt;
        }
        pressure_e_old = pressure_e;

        // Constrain angles and speeds
        angle_setpoint = min(MAX_ANGLE, max(MIN_ANGLE, rawAngle));
        speed = min(max(MIN_SPD,rawSpd),MAX_SPD);

        runMotor();
    }

    
    //send data to AC
    if (micros() - lastPrint > 1000) {
        Comms::Packet packet = {.id = 85};
        Comms::packetAddFloat(&packet, HPpsi);
        Comms::packetAddFloat(&packet, LPpsi);
        Comms::packetAddFloat(&packet, InjectorPT);
        Comms::packetAddFloat(&packet, motorAngle);
        
        sendToAC(packet);
        lastPrint = micros();
    }
}







