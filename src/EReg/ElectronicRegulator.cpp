#include <Arduino.h>
#include "HAL.h"
#include "Util.h"
#include "Comms.h"
#include "Config.h"
#include "StateMachine.h"
#include "Packets.h"

StateMachine::FlowState *flowState = StateMachine::getFlowState();
StateMachine::IdleClosedState *idleClosedState = StateMachine::getIdleClosedState();
StateMachine::PartiallyOpenState *partiallyOpenState = StateMachine::getPartiallyOpenState();
StateMachine::DiagnosticState *diagnosticState = StateMachine::getDiagnosticState();
StateMachine::PressurizeState *pressurizeState = StateMachine::getPressurizeState();

void zero() {
    DEBUGLN("starting zero command");
    Util::runMotors(-150);
    delay(2000);
    Util::runMotors(0);
    // zero encoder value (so encoder readings range from -x (open) to 0 (closed))
    delay(400);
    HAL::encoder.setCount(-20);
    DEBUG("encoder position after zero: ");
    DEBUGLN(HAL::encoder.getCount());
}

void zero(Comms::Packet packet, uint8_t ip) {
    zero();
}

void flow(Comms::Packet packet, uint8_t ip) {
    StateMachine::enterFlowState();
}

void stopFlow(Comms::Packet packet, uint8_t ip) {
    StateMachine::enterIdleClosedState();
}

void partialOpen(Comms::Packet packet, uint8_t ip) {
    StateMachine::enterPartialOpenState(Comms::packetGetFloat(&packet, 0));
}

void runDiagnostics(Comms::Packet packet, uint8_t ip) {
    StateMachine::enterDiagnosticState();
}

void pressurize(Comms::Packet packet, uint8_t ip) {
    StateMachine::enterPressurizeState();
}

void actuateMainValve(Comms::Packet packet, uint8_t ip) {
    StateMachine::enterMainValveState(Comms::packetGetUint8(&packet, 0));
}

void setup() {
    HAL::init();
    Comms::initComms();
    StateMachine::enterIdleClosedState();
    zero();
    Comms::registerCallback(0, flow);
    Comms::registerCallback(1, stopFlow);
    Comms::registerCallback(2, partialOpen);
    Comms::registerCallback(3, pressurize);
    Comms::registerCallback(4, runDiagnostics);
    Comms::registerCallback(5, zero);
    Comms::registerCallback(6, actuateMainValve);

    Packets::sendConfig();
}

void loop() {
    Serial.println("in loop");
    delay(100);
    Comms::processWaitingPackets();

    switch (StateMachine::getCurrentState()) {
        case StateMachine::IDLE_CLOSED:
        idleClosedState->update();
        break;
        
        case StateMachine::PARTIAL_OPEN:
        partiallyOpenState->update();
        break;

        case StateMachine::PRESSURIZE:
        pressurizeState->update();
        break;

        case StateMachine::FLOW:
        flowState->update();
        break;

        case StateMachine::DIAGNOSTIC:
        diagnosticState->update();
        break;
    };
}

