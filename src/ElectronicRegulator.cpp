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


// valve angle based on pressure setpoint
PIDController outerController(1.50, 2.25e-6, 0.125, MIN_ANGLE, MAX_ANGLE);
// motor angle based on valve setpoint

// pressure against motor during flow different than tuning seperately? 
PIDController innerController(11.5, 1.5e-6, 0.35e-6, MIN_SPD, MAX_SPD);

void setup() {
    Serial.begin(9600);
    Serial1.begin(115200);

    Comms::registerCallback(0, zero);
    Comms::registerCallback(1, setPressureSetpoint);
    Comms::registerCallback(2, regulation);
    Comms::registerCallback(3, flow);
    Comms::registerCallback(4, fullOpen);
}

void loop() {
    Comms::processWaitingPackets();

    if (startFlow) {
        // start controller
        angle = encoder.read();
        motorAngle = Util::encoderToAngle(angle);
        potAngle = Util::readPot();
        HPpsi = Util::voltageToHighPressure(analogRead(HAL::hpPT));
        LPpsi = Util::voltageToPressure(analogRead(HAL::lpPT));
        InjectorPT = Util::voltageToPressure(analogRead(HAL::injectorPT));
        
        //Compute Inner PID Servo loop
        float speed = innerController.update(angle-angle_setpoint);

        //Compute Outer Pressure Control Loop
        pressure_e = LPpsi - pressure_setpoint;
        p_buff->insert(t2/1.0e6, LPpsi);
        double rawAngle = -( kp_outer*pressure_e + kd_outer*(p_buff->get_slope()) );
        rawAngle += Util::compute_feedforward(pressure_setpoint, HPpsi);
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

