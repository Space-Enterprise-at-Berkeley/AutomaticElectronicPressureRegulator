#include "Automation.h"
#include "Toggles.h"

namespace Automation {
    Task *flowTask = nullptr;
    Task *abortFlowTask = nullptr;
    Task *autoventFuelTask = nullptr;
    Task *autoventLoxTask = nullptr;

    // uint32_t loxLead = Util::millisToMicros(165);
    // uint32_t burnTime = Util::secondsToMicros(25);
    // uint32_t ventTime = Util::millisToMicros(200);

    bool loxGemValveAbovePressure = false;
    bool fuelGemValveAbovePressure = false;

    void initAutomation(Task *flowTask, Task *abortFlowTask, Task *autoventFuelTask, Task *autoventLoxTask) {
        Automation::flowTask = flowTask;
        Automation::abortFlowTask = abortFlowTask;
        Automation::autoventFuelTask = autoventFuelTask;
        Automation::autoventLoxTask = autoventLoxTask;

        Comms::registerCallback(2, beginFlow);
        Comms::registerCallback(3, beginManualAbortFlow);
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
        }
    }

    inline void sendFlowStatus(uint8_t status) {
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
            case 0:
                Actuators::extendAct1(); //fuel tank vent rbv
                Actuators::extendAct2(); //lox tank vent rbv
                Actuators::retractAct6(); //fuel gems
                Actuators::retractAct7(); //lox gems

                autoventFuelTask->enabled = false;
                autoventLoxTask->enabled = false;
                step++;
                return 4 * 1e6;
            case 1: // step 0 (open arming valve)
                Actuators::extendAct4(); //two way
                Toggles::startIgniter();
                sendFlowStatus(STATE_ACTIVATE_TWO_WAY);
                step++;
                return 300 * 1e3;
            case 2: // start ereg flow
                Toggles::stopIgniter();
                EReg::startFlow();
                sendFlowStatus(STATE_EREG_BEGIN);
                step++;
                //TODO get from config packet
                return 20 * 1e6 + 1e6; // delay 20 seconds
            case 3: // end config
                Actuators::retractAct4(); //two way
            default: // end
                flowTask->enabled = false;
                autoventFuelTask->enabled = true;
                autoventLoxTask->enabled = true;
                sendFlowStatus(STATE_END_FLOW);
                return 0;
        }
    }


    void beginManualAbortFlow(Comms::Packet packet, uint8_t ip) {
        DEBUG("Beginning manual abort");
        flowTask->enabled = false;
        Actuators::retractAct1(); //fuel tank vent
        Actuators::retractAct2(); //lox tank vent
        Actuators::extendAct6(); //fuel gems
        Actuators::extendAct7(); //lox gems
        EReg::abort();
        autoventFuelTask->enabled = true;
        autoventLoxTask->enabled = true;
        delay(100);
        Actuators::retractAct4();
        sendFlowStatus(STATE_MANUAL_ABORT);
    }

    void beginAbortFlow() {
        if(!abortFlowTask->enabled) {
            step = 0;
            flowTask->enabled = false;
            abortFlowTask->nexttime = micros() + 1500; // 1500 is a dirty hack to make sure flow status gets recorded. Ask @andy
            abortFlowTask->enabled = true;
            autoventFuelTask->enabled = true;
            autoventLoxTask->enabled = true;
        }
    }

    uint32_t abortFlow() {
        // DEBUG("ABORT STEP: ");
        // DEBUG(step);
        // DEBUG("\n");
        switch(step) {
            case 0: // deactivate igniter and vent pneumatics and tanks
                Actuators::retractAct1();
                Actuators::retractAct2();
                Actuators::extendAct6();
                Actuators::extendAct7();
                EReg::abort();
                step++;
                return 500;
            default: // end
                Actuators::retractAct4();
                abortFlowTask->enabled = false;
                sendFlowStatus(STATE_EREG_ABORT);
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
};
