#include "Packets.h"
#include "Config.h"
#include "Comms.h"
#include "StateMachine.h"

namespace Packets {
    /**
     * Send telemetry packet:
     * - high pressure
     * - low pressure 
     * - injector pressure 
     * - encoder reading 
     * - angle setpoint 
     * - pressure setpoint
     * - motor power 
     * - pressure control loop P term
     * - pressure control loop I term
     * - pressure control loop D term
     */
    void sendTelemetry(
        float highPressure,
        float lowPressure,
        float injectorPressure,
        float encoderAngle,
        float angleSetpoint,
        float pressureSetpoint,
        float motorPower,
        float pressureControlP,
        float pressureControlI,
        float pressureControlD
    ) {
        #ifdef DEBUG_MODE
        DEBUG(StateMachine::getCurrentState()); DEBUG("\t");
        DEBUG(encoderAngle); DEBUG("\t");
        DEBUG(angleSetpoint); DEBUG("\t");
        DEBUG(motorPower); DEBUGLN("\t");
        #else
        Comms::Packet packet = {.id = TELEMETRY_ID};
        Comms::packetAddFloat(&packet, highPressure);
        Comms::packetAddFloat(&packet, lowPressure);
        Comms::packetAddFloat(&packet, encoderAngle);
        Comms::packetAddFloat(&packet, angleSetpoint);
        Comms::packetAddFloat(&packet, pressureSetpoint);
        Comms::packetAddFloat(&packet, motorPower);
        Comms::packetAddFloat(&packet, pressureControlP);
        Comms::packetAddFloat(&packet, pressureControlI);
        Comms::packetAddFloat(&packet, pressureControlD);
        Comms::emitPacket(&packet);
        Comms::emitPacket(&packet, HAL::acEndIp); //TODO not working right now, need for forwarding pressures for aborts
        #endif
    }

    /**
     * Send config packet:
     * - target downstream pressure
     * - outer control loop k_p
     * - outer control loop k_i
     * - outer control loop k_d
     * - inner control loop k_p
     * - inner control loop k_i
     * - inner control loop k_d
     */
    void sendConfig() {
        Comms::Packet packet = {.id = CONFIG_ID};
        Comms::packetAddFloat(&packet, Config::pressureSetpoint);
        Comms::packetAddFloat(&packet, Config::p_outer_nominal);
        Comms::packetAddFloat(&packet, Config::i_outer_nominal);
        Comms::packetAddFloat(&packet, Config::d_outer_nominal);
        Comms::packetAddFloat(&packet, Config::p_inner);
        Comms::packetAddFloat(&packet, Config::i_inner);
        Comms::packetAddFloat(&packet, Config::d_inner);
        Comms::packetAddFloat(&packet, (float) (Config::flowDuration / 1e6));
        Comms::emitPacket(&packet);
    }

    /**
     * Send diagnostic test report packet:
     * - success / failure message
     */
    void sendDiagnostic(uint8_t motorDirPass, uint8_t servoPass) {
        #ifdef DEBUG_MODE
        DEBUGF("Motor Dir Test: %i \t Servo Test: %i \n", motorDirPass, servoPass);
        #else
        Comms::Packet packet = {.id = DIAGNOSTIC_ID};
        packet.len = 0;
        Comms::packetAddUint8(&packet, motorDirPass);
        Comms::packetAddUint8(&packet, servoPass);
        Comms::emitPacket(&packet);
        #endif
    }

    /**
     * Send state transition failure packet:
     * - failure message
     */
    void sendStateTransitionError(uint8_t errorCode) {
        #ifdef DEBUG_MODE
        DEBUG("State Transition Error: ");
        DEBUGLN(errorCode);
        #else
        Comms::Packet packet = {.id = STATE_TRANSITION_FAIL_ID};
        packet.len = 0;
        Comms::packetAddUint8(&packet, errorCode);
        Comms::emitPacket(&packet);
        #endif
    }

    /**
     * @brief sends flow state updates to the AC
     * flowStates: 0 = abort
     * @param flowState 
     */
    void sendFlowState(uint8_t flowState) {
        Comms::Packet packet = {.id = FLOW_STATE};
        packet.len = 0;
        Comms::packetAddUint8(&packet, flowState);
        Comms::emitPacket(&packet);
    }


    /**
     * Sends an abort command to all 4 ESPs
     */
    void broadcastAbort() {
        Comms::Packet packet = {.id = ABORT_ID};
        packet.len = 0;

        //send to all ESPs including to itself
        Comms::emitPacket(&packet, ESP_ADDRESS_1);
        Comms::emitPacket(&packet, ESP_ADDRESS_2);
        Comms::emitPacket(&packet, ESP_ADDRESS_3);
        Comms::emitPacket(&packet, ESP_ADDRESS_4);
        Comms::emitPacket(&packet, HAL::acEndIp);
        Comms::emitPacket(&packet, HAL::daqEndIp);
    }

}