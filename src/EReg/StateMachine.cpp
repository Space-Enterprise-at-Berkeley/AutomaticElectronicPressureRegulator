#include "StateMachine.h"

namespace StateMachine {

    State currentState = IDLE_CLOSED;
    ValveAction currentMainValveState = MAIN_VALVE_CLOSE;

    void enterFlowState() {
        if (currentState == IDLE_CLOSED) {
            currentState = FLOW;
            getFlowState()->init();
        } else {
            // Illegal state transition
            Packets::sendStateTransitionError(0);
        }
    }

    void enterIdleClosedState() {
        currentState = IDLE_CLOSED;
        getIdleClosedState()->init();
    }

    void enterPartialOpenState(float angle) {
        if (currentState == IDLE_CLOSED || currentState == PARTIAL_OPEN) {
            if (angle > (MIN_ANGLE - 100) && angle < (MAX_ANGLE + 100)) {
                currentState = PARTIAL_OPEN;
                getPartiallyOpenState()->init(angle);
            } else {
                // Illegal parameters
                Packets::sendStateTransitionError(1);
            }
        } else {
            // Illegal state transition
            Packets::sendStateTransitionError(2);
        }
    }

    void enterDiagnosticState() {
        if (currentState == IDLE_CLOSED) {
            currentState = DIAGNOSTIC;
            getDiagnosticState()->init();
        } else {
            // Illegal state transition
            Packets::sendStateTransitionError(3);
        }
    }

    void enterPressurizeState() {
        if (currentState == IDLE_CLOSED) {
            currentState = PRESSURIZE;
            getPressurizeState()->init();
        } else {
            // Illegal state transition
            Packets::sendStateTransitionError(4);
        }
    }

    void enterMainValveState(uint8_t actionByte) {
        ValveAction action = actionByte ? MAIN_VALVE_OPEN : MAIN_VALVE_CLOSE;
        if (currentState == IDLE_CLOSED || currentState == PARTIAL_OPEN) {
            actuateMainValve(action);
        } else {
            // Illegal state transition
            Packets::sendStateTransitionError(5);
        }
    }

    State getCurrentState() {
        return currentState;
    }

    /**
     * Helper function to actuate main valve. DO NOT have outside code calling this function - 
     * This function does not validate the current state
     * @param action Desired valve state
     */
    void actuateMainValve(ValveAction action) {
        switch (action) {
            case MAIN_VALVE_OPEN:
                digitalWrite(HAL::mainValve1, HIGH);
                digitalWrite(HAL::mainValve2, LOW);
            break;
            case MAIN_VALVE_CLOSE:
                digitalWrite(HAL::mainValve1, LOW);
                digitalWrite(HAL::mainValve2, LOW);
            break;
        }
        currentMainValveState = action;
    }

    void checkAbortPressure(float currentPressure, float abortPressure) {
        if (currentPressure > abortPressure) {
            Packets::sendFlowState(0);
            Packets::broadcastAbort();
            StateMachine::enterIdleClosedState();
        }
    }

}
