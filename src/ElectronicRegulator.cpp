#include <Arduino.h>

#include <Encoder.h>
#include <data_buff.h>
#include <Comms.h>
#include <PIDController.h>
#include <HAL.h>
#include <Util.h>

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
bool startFlow = false;
long lastPrint = 0;

float pressure_setpoint;
float angle_setpoint;

void runMotors(float speed){
    analogWrite(HAL::motor1,-min(0,speed));
    analogWrite(HAL::motor2,max(0,speed));
    analogWrite(HAL::motor3,-min(0,speed));
    analogWrite(HAL::motor4,max(0,speed));
}

Buffer* p_buff;


//TODO fill out command methods
void zero(Comms::Packet packet) {
    runMotors(-150);
    delay(1000);
    runMotors(0);
    // zero encoder value (so encoder readings range from -x (open) to 0 (closed))
    encoder.write(-20);
}

void setPressureSetpoint(Comms::Packet packet) {
    pressure_setpoint = Comms::packetGetFloat(&packet, 0);
}

void flow(Comms::Packet packet) {
    startFlow = packet.data[0];
}

void fullOpen(Comms::Packet packet) {

}

void setPIDConstants(Comms::Packet packet) {

}


// valve angle based on pressure setpoint
PIDController outerController(1.50, 2.25e-6, 0.125, MIN_ANGLE, MAX_ANGLE);
// motor angle based on valve setpoint

// pressure against motor during flow different than tuning seperately? 
PIDController innerController(11.5, 1.5e-6, 0.35e-6, MIN_SPD, MAX_SPD);

void setup() {
    Serial.begin(9600);

    Comms::registerCallback(0, zero);
    Comms::registerCallback(1, setPressureSetpoint);
    Comms::registerCallback(2, flow);
    Comms::registerCallback(3, fullOpen);
}

void loop() {
    Comms::processWaitingPackets();

    float angle = encoder.read();
    float motorAngle = Util::encoderToAngle(angle);
    float potAngle = Util::readPot();
    float HPpsi = Util::voltageToHighPressure(analogRead(HAL::hpPT));
    float LPpsi = Util::voltageToPressure(analogRead(HAL::lpPT));
    float InjectorPT = Util::voltageToPressure(analogRead(HAL::injectorPT));

    if (startFlow) {
        //Compute Inner PID Servo loop
        float speed = innerController.update(motorAngle - angle_setpoint);

        //Compute Outer Pressure Control Loop
        float angle_setpoint = outerController.update(LPpsi - pressure_setpoint);
        angle_setpoint += Util::compute_feedforward(pressure_setpoint, HPpsi);

        runMotors(speed);
    }

    //send data to AC
    if (micros() - lastPrint > 1000) {
        Comms::Packet packet = {.id = 85};
        Comms::packetAddFloat(&packet, HPpsi);
        Comms::packetAddFloat(&packet, LPpsi);
        Comms::packetAddFloat(&packet, InjectorPT);
        Comms::packetAddFloat(&packet, motorAngle);
        
        Comms::emitPacket(&packet);
        lastPrint = micros();
    }
}

