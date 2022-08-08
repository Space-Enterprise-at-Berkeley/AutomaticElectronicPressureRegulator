#include <Arduino.h>

#include <Encoder.h>
#include <PIDController.h>
#include "HAL.h"
#include "Util.h"
#include "Comms.h"
#include "Config.h"
#include "StateMachine.h"

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   avoid using pins with LEDs attached
Encoder *encoder = Util::getEncoder();
StateMachine::State currentState = StateMachine::idleClosedState;

StateMachine::FlowState flowState = StateMachine::FlowState();
StateMachine::IdleClosedState idleClosedState = StateMachine::IdleClosedState();
StateMachine::PartiallyOpenState partiallyOpenState = StateMachine::PartiallyOpenState();

void zero() {
    Util::runMotors(-150);
    delay(2000);
    Util::runMotors(0);
    // zero encoder value (so encoder readings range from -x (open) to 0 (closed))
    encoder->write(-20);
}

void zero(Comms::Packet packet) {
    zero();
}

void enterFlowState() {
    currentState = StateMachine::flowState;
    flowState.init();
}

void enterIdleClosedState() {
    currentState = StateMachine::idleClosedState;
    idleClosedState.init();
}

//TODO need to implement rest of commands
void setPressureSetpoint(Comms::Packet packet) {
    // pressure_setpoint = Comms::packetGetFloat(&packet, 0);
}

void flow(Comms::Packet packet) {
    // startFlow = packet.data[0];
    enterFlowState();
}

void stopFlow(Comms::Packet packet) {
    enterIdleClosedState();
}

void setPIDConstants(Comms::Packet packet) {

}

void abort(Comms::Packet packet) {
    enterIdleClosedState();
}



void setup() {
    Comms::registerCallback(0, zero);
    Comms::registerCallback(1, setPressureSetpoint);
    Comms::registerCallback(2, flow);
    Comms::registerCallback(3, stopFlow);
    Comms::registerCallback(4, setPIDConstants);
    Comms::registerCallback(5, abort);
    zero();
    enterIdleClosedState();
}

void loop() {
    Comms::processWaitingPackets();

    switch (currentState) {
        case StateMachine::idleClosedState:
        idleClosedState.update();
        break;
        
        case StateMachine::partiallyOpenState:
        partiallyOpenState.update();
        break;

        case StateMachine::flowState:
        flowState.update();
        break;
    };
}

