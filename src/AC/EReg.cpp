#include "Ereg.h"

namespace EReg {

    uint32_t samplePeriod = 12.5 * 1000; // 80 Hz
    char packetBuffer[sizeof(Comms::Packet)];

    float hpPT = 0;
    float lpPT = 0;
    float injectorPT = 0;
    float motorAngle = 0;

    void initEReg() {
        // EReg board connected to Serial8 of AC Teensy
        Serial8.begin(115200);

        Comms::registerCallback(0, startLoxFlow);
        Comms::registerCallback(1, startFuelFlow);
        Comms::registerCallback(2, startFlow);
        Comms::registerCallback(3, abort);
        Comms::registerCallback(4, setLoxPosition);
        Comms::registerCallback(5, setFuelPosition);
        Comms::registerCallback(6, staticPressurizeLox);
        Comms::registerCallback(6, staticPressurizeFuel);
        Comms::registerCallback(6, activateIgniter);
    }

    void startLoxFlow(Comms::Packet tmp, uint8_t ip) {

    }

    void startFuelFlow(Comms::Packet tmp, uint8_t ip) {

    }

    void startFlow(Comms::Packet tmp, uint8_t ip) {

    }

    void abort(Comms::Packet tmp, uint8_t ip) {

    }

    void setLoxPosition(Comms::Packet tmp, uint8_t ip) {

    }

    void setFuelPosition(Comms::Packet tmp, uint8_t ip) {

    }

    void staticPressurizeLox(Comms::Packet tmp, uint8_t ip) {

    }

    void staticPressurizeFuel(Comms::Packet tmp, uint8_t ip) {
        
    }

    void activateIgniter(Comms::Packet tmp, uint8_t ip) {

    }

    uint32_t sampleTelemetry() {
        if(Serial8.available()) {
            int cnt = 0;
            while(Serial8.available() && cnt < sizeof(Comms::Packet)) {
                packetBuffer[cnt] = Serial8.read();
                cnt++;
            }
            Comms::Packet *packet = (Comms::Packet *)&packetBuffer;
            //TODO retrive data from ereg packet
        }
        return samplePeriod;
    }

    void sendToEReg(Comms::Packet *packet) {
        Serial8.write(packet->id);
        Serial8.write(packet->len);
        Serial8.write(packet->timestamp, 4);
        Serial8.write(packet->checksum, 2);
        Serial8.write(packet->data, packet->len);
        Serial8.write('\n');
    }

}