#include <Arduino.h>

#include <Encoder.h>
#include <PIDController.h>
#include "HAL.h"
#include "Util.h"
#include "Comms.h"
#include "Config.h"

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   avoid using pins with LEDs attached
Encoder encoder(HAL::enc1, HAL::enc2);

PIDController *innerController = Util::getInnerController();
PIDController *outerController = Util::getOuterController();


// Note: 1 rev on main shaft is 3200 counts
// Encoder itself is 64CPM (including all edges)
bool startFlow = false;
long lastPrint = 0;

float pressure_setpoint = 0;
float angle_setpoint = 0;

//TODO fill out command methods
void zero(Comms::Packet packet) {
    Util::runMotors(-150);
    delay(2000);
    Util::runMotors(0);
    // zero encoder value (so encoder readings range from -x (open) to 0 (closed))
    encoder.write(-20);
}

//TODO need to implement rest of commands
void setPressureSetpoint(Comms::Packet packet) {
    pressure_setpoint = Comms::packetGetFloat(&packet, 0);
}

void flow(Comms::Packet packet) {
    startFlow = packet.data[0];
}

void stopFlow(Comms::Packet packet) {

}

void setPIDConstants(Comms::Packet packet) {

}

void abort(Comms::Packet packet) {

}

void setup() {

    Comms::registerCallback(0, zero);
    Comms::registerCallback(1, setPressureSetpoint);
    Comms::registerCallback(2, flow);
    Comms::registerCallback(3, stopFlow);
    Comms::registerCallback(4, setPIDConstants);
    Comms::registerCallback(5, abort);
}

void loop() {
    Comms::processWaitingPackets();

    float motorAngle = encoder.read();
    float potAngle = Util::readPot();
    float HPpsi = Util::voltageToHighPressure(analogRead(HAL::hpPT));
    float LPpsi = Util::voltageToLowPressure(analogRead(HAL::lpPT));
    float InjectorPT = Util::voltageToLowPressure(analogRead(HAL::injectorPT));

    float speed = 0;

    if (startFlow) {
        //Compute Inner PID Servo loop
        speed = innerController->update(motorAngle - angle_setpoint);

        //Compute Outer Pressure Control Loop
        angle_setpoint = outerController->update(LPpsi - pressure_setpoint);
        angle_setpoint += Util::compute_feedforward(pressure_setpoint, HPpsi);

        Util::runMotors(speed);
    }

    //send data to AC
    if (micros() - lastPrint > 1000) {
        Comms::Packet packet = {.id = 85};
        //TODO split into two temelemtry packets
        Comms::packetAddFloat(&packet, Config::p_outer);
        Comms::packetAddFloat(&packet, Config::i_outer);
        Comms::packetAddFloat(&packet, Config::d_outer);
        Comms::packetAddFloat(&packet, pressure_setpoint);
        Comms::packetAddFloat(&packet, angle_setpoint);
        Comms::packetAddFloat(&packet, HPpsi);
        Comms::packetAddFloat(&packet, LPpsi);
        Comms::packetAddFloat(&packet, InjectorPT);
        Comms::packetAddFloat(&packet, motorAngle);
        Comms::packetAddFloat(&packet, speed);
        
        Comms::emitPacket(&packet);
        lastPrint = micros();
    }
}

