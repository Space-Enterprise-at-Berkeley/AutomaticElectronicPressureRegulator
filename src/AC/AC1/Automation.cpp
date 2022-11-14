#include "Automation.h"
#include "Toggles.h"

namespace Automation {
    Task *flowTask = nullptr;
    Task *abortFlowTask = nullptr;
    Task *autoventFuelTask = nullptr;
    Task *autoventLoxTask = nullptr;
    
    bool tcAbortEnabled = false;
    bool lcAbortEnabled = false;

    bool igniterEnabled = true;
    bool breakwireEnabled = true;

    bool igniterTriggered = false;
    bool breakwireBroken = false;

    bool loxGemValveAbovePressure = false;
    bool fuelGemValveAbovePressure = false;

    Comms::Packet lcAbortPacket = {.id = 100};

    void initAutomation(Task *flowTask, Task *abortFlowTask, Task *autoventFuelTask, Task *autoventLoxTask) {
        Automation::flowTask = flowTask;
        Automation::abortFlowTask = abortFlowTask;
        Automation::autoventFuelTask = autoventFuelTask;
        Automation::autoventLoxTask = autoventLoxTask;

        Comms::registerCallback(9, beginFlow);
        Comms::registerCallback(201, beginManualAbortFlow);
        
        Comms::registerCallback(30, checkForTCAbort);
        Comms::registerCallback(31, checkForLCAbort);
    }

    Comms::Packet flowPacket = {.id = 50};
    int step = 0;

    void beginFlow(Comms::Packet packet, uint8_t ip) {
        if(!flowTask->enabled) {
            DEBUGLN("Beginning flow");
            step = 0;
            //reset values
            flowTask->nexttime = micros();
            flowTask->enabled = true;

            igniterTriggered = false;
            breakwireBroken = false;

            autoventFuelTask->enabled = false;
            autoventLoxTask->enabled = false;

            tcAbortEnabled = true;
        }
    }

    void sendFlowStatus(uint8_t status) {
        // DEBUG("FLOW STATUS: ");
        // DEBUG(status);
        // DEBUG("\n");
        flowPacket.len = 1;
        flowPacket.data[0] = status;
        Comms::emitPacket(&flowPacket);
    }

    uint32_t flow() {
        // DEBUG("STEP: ");
        // DEBUG(step);
        // DEBUG("\n");
        switch(step) {
            case 0: // enable and start igniter
                if(Toggles::breakWireVoltage > breakWireThreshold || !breakwireEnabled) {
                    Actuators::extendAct3(); //igniter enable
                    Toggles::startIgniter();
                    sendFlowStatus(STATE_ACTIVATE_IGNITER);
                    step++;

                    return 0.5 * 1e6; // delay 0.5s
                } else {
                    sendFlowStatus(STATE_BREAKWIRE_FAIL_CONTINUITY_ABORT);
                    beginAbortFlow();
                    return 0;
                }
            case 1: // turn off igniter
                Actuators::retractAct3();
                Toggles::stopIgniter();
                sendFlowStatus(STATE_IGNITER_ACTIVATED);
                step++;

                return 1.5 * 1e6; //delay by 1.5 seconds
            case 2: //check igniter trigger and breakwire to open arming valve
                if (breakwireBroken || !breakwireEnabled) {
                    Actuators::extendAct4(); //two way
                    sendFlowStatus(STATE_ARMED_VALVES);
                    step++;

                    return .5 * 1e6; //delay .5 seconds
                } else {
                    sendFlowStatus(STATE_BREAKWIRE_FAIL_DISCONNECT_ABORT);
                    beginAbortFlow();
                    return 0;
                }
            case 3: // start ereg flow
                if (Actuators::act4Current > twoWayCurrentThreshold) {
                    EReg::startFlow();
                    sendFlowStatus(STATE_START_FLOW);
                    step++;
                    //TODO get from config packet
                    return 2 * 1e6; //delay over burn time to close main valves
                } else {
                    sendFlowStatus(STATE_ARMING_VALVE_FAIL_CURRENT);
                    beginAbortFlow();
                    return 0;
                }
            case 4: //enable load cell abort
                lcAbortPacket.data[0] = 1;
                Comms::emitPacket(&lcAbortPacket);
                lcAbortEnabled = true;
                step++;
                return (burnTime - 2 * 1e6) + (3 * 1e6);
            case 5: // end config
                Actuators::retractAct4(); //two way
                sendFlowStatus(STATE_DISABLE_ARMING_VALVE);
                step++;
                return 0;
            default: // end
                flowTask->enabled = false;
                autoventFuelTask->enabled = true;
                autoventLoxTask->enabled = true;
                sendFlowStatus(STATE_END_FLOW);
                return 0;
        }
    }


    void beginManualAbortFlow(Comms::Packet packet, uint8_t ip) {
        DEBUGLN("Beginning manual abort");
        sendFlowStatus(STATE_MANUAL_ABORT);
        beginAbortFlow();
    }

    void beginAbortFlow() {
        if(!abortFlowTask->enabled) {
            step = 0;
            flowTask->enabled = false;
            abortFlowTask->nexttime = micros() + 1500; // 1500 is a dirty hack to make sure flow status gets recorded. Ask @andy
            abortFlowTask->enabled = true;
            autoventFuelTask->enabled = true;
            autoventLoxTask->enabled = true;

            //disable load cell abort
            lcAbortPacket.data[0] = 0;
            Comms::emitPacket(&lcAbortPacket);

            tcAbortEnabled = false;
            lcAbortEnabled = false;
        }
    }

    uint32_t abortFlow() {
        // DEBUG("ABORT STEP: ");
        // DEBUG(step);
        // DEBUG("\n");
        switch(step) {
            case 0: // deactivate igniter and vent pneumatics and tanks
                EReg::abort();
                Actuators::retractAct1(); //open all vents
                Actuators::retractAct2();
                Actuators::extendAct6();
                Actuators::extendAct7();
                step++;
                return 1 * 1e6;
            default: // end
                Actuators::retractAct4();
                abortFlowTask->enabled = false;
                sendFlowStatus(STATE_ABORT_END);
                return 0;
        }
    }

    Comms::Packet autoventPacket = {.id = 51};
    uint32_t autoventFuelGemValveTask() {
        float fuelPresure = EReg::fuelTankPTValue;

        if (fuelPresure > autoVentUpperThreshold) {
            Actuators::extendAct6();
            fuelGemValveAbovePressure = true;

            autoventPacket.len = 1;
            autoventPacket.data[0] = 1;
            Comms::emitPacket(&autoventPacket);
        } else if (fuelPresure < autoVentLowerThreshold && fuelGemValveAbovePressure) {
            Actuators::retractAct6();
            fuelGemValveAbovePressure = false;
        }

        return 0.25 * 1e6;
    }

    uint32_t autoventLoxGemValveTask() {
        float loxPressure = EReg::loxTankPTValue;

        if (loxPressure > autoVentUpperThreshold) {
            Actuators::extendAct7();
            loxGemValveAbovePressure = true;

            autoventPacket.len = 1;
            autoventPacket.data[0] = 0;
            Comms::emitPacket(&autoventPacket);
        } else if (loxPressure < autoVentLowerThreshold && loxGemValveAbovePressure) {
            Actuators::retractAct7();
            loxGemValveAbovePressure = false;
        }

        return 0.25 * 1e6;
    }

    void checkForLCAbort(Comms::Packet packet, uint8_t ip) {
        if (lcAbortEnabled) {
            sendFlowStatus(STATE_LC_ABORT);
            beginAbortFlow();
        }
    }

    void checkForTCAbort(Comms::Packet packet, uint8_t ip) {
        if (tcAbortEnabled) {
            sendFlowStatus(STATE_TC_ABORT);
            beginAbortFlow();
        }
    }

    uint32_t checkIgniter() {
        igniterTriggered = Toggles::igniterCurrent > igniterTriggerThreshold || igniterTriggered;
        breakwireBroken = Toggles::breakWireVoltage < breakWireThreshold || breakwireBroken;
        return Toggles::toggleCheckPeriod;
    }
};
