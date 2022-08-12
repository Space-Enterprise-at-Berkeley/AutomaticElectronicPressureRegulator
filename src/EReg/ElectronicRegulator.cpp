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
StateMachine::FlowState *flowState = StateMachine::getFlowState();
StateMachine::IdleClosedState *idleClosedState = StateMachine::getIdleClosedState();
StateMachine::PartiallyOpenState *partiallyOpenState = StateMachine::getPartiallyOpenState();
StateMachine::DiagnosticState *diagnosticState = StateMachine::getDiagnosticState();

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

//TODO need to implement rest of commands
void setPressureSetpoint(Comms::Packet packet) {
    // pressure_setpoint = Comms::packetGetFloat(&packet, 0);
}

void flow(Comms::Packet packet) {
    // startFlow = packet.data[0];
    StateMachine::enterFlowState();
}

void stopFlow(Comms::Packet packet) {
    StateMachine::enterIdleClosedState();
}

void partialOpen(Comms::Packet packet) {
    StateMachine::enterPartialOpenState(Comms::packetGetFloat(&packet, 0));
}

void runDiagnostics(Comms::Packet packet) {
    StateMachine::enterDiagnosticState();
}

void setup() {
    Comms::registerCallback(0, flow);
    Comms::registerCallback(1, stopFlow);
    Comms::registerCallback(2, partialOpen);
    // Comms::registerCallback(3, pressurize);
    Comms::registerCallback(4, runDiagnostics);
    Comms::registerCallback(5, zero);
    zero();
    StateMachine::enterIdleClosedState();
}

void loop() {
    Comms::processWaitingPackets();

    switch (StateMachine::getCurrentState()) {
        case StateMachine::IDLE_CLOSED:
        idleClosedState->update();
        break;
        
        case StateMachine::PARTIAL_OPEN:
        partiallyOpenState->update();
        break;

        case StateMachine::FLOW:
        flowState->update();
        break;

        case StateMachine::DIAGNOSTIC:
        diagnosticState->update();
        break;
    };
}

