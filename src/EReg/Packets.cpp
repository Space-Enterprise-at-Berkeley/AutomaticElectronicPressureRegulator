#include "Packets.h"
#include "Config.h"
#include "Comms.h"

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
        Comms::Packet packet = {.id = TELEMETRY_ID};
        Comms::packetAddFloat(&packet, highPressure);
        Comms::packetAddFloat(&packet, lowPressure);
        Comms::packetAddFloat(&packet, injectorPressure);
        Comms::packetAddFloat(&packet, encoderAngle);
        Comms::packetAddFloat(&packet, angleSetpoint);
        Comms::packetAddFloat(&packet, motorPower);
        Comms::packetAddFloat(&packet, pressureSetpoint);
        Comms::packetAddFloat(&packet, pressureControlP);
        Comms::packetAddFloat(&packet, pressureControlI);
        Comms::packetAddFloat(&packet, pressureControlD);
        Comms::emitPacket(&packet);
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
        Comms::packetAddFloat(&packet, Config::p_outer);
        Comms::packetAddFloat(&packet, Config::i_outer);
        Comms::packetAddFloat(&packet, Config::d_outer);
        Comms::packetAddFloat(&packet, Config::p_inner);
        Comms::packetAddFloat(&packet, Config::i_inner);
        Comms::packetAddFloat(&packet, Config::d_inner);
        Comms::emitPacket(&packet);
    }
}