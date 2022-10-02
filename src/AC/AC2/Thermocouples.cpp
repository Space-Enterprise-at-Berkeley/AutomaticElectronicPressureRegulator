#include "Thermocouples.h"

namespace Thermocouples {
    uint32_t tcUpdatePeriod = 100 * 1000;
    Comms::Packet tcPacket;

    uint8_t hysteresisValues[5] = {0};

    Comms::Packet eregAbortPacket = {.id = 1};

    float tc0Value;
    float tc1Value;
    float tc2Value;
    float tc3Value;
    float tc4Value;

    float tc0ROC;
    float tc1ROC;
    float tc2ROC;
    float tc3ROC;
    float tc4ROC;

    float tc0ROCValues[10] = {0};
    float tc1ROCValues[10] = {0};
    float tc2ROCValues[10] = {0};
    float tc3ROCValues[10] = {0};
    float tc4ROCValues[10] = {0};

    void initThermocouples() {
    }

    uint32_t tcSample(MAX31855 *amp, uint8_t packetID, float *value, float *thermocoupleValues, float *ROCValue) {
        float reading = amp->readCelsius();

        // DEBUGF("THERMOCOUPLE READING %i: %f \n", packetID, reading);

        //calculate ROC TC value
        for (int i = 1; i < 10; i++) {
            thermocoupleValues[i] = thermocoupleValues[i-1];
        }
        thermocoupleValues[0] = (reading - *value) / ((float)tcUpdatePeriod / 1e6);

        float sum = 0;
        for (int i = 0; i < 10; i++) {
            sum += thermocoupleValues[i];
        }
        *ROCValue = sum / 10;

        *value = reading;

        tcPacket.id = packetID;
        tcPacket.len = 0;
        Comms::packetAddFloat(&tcPacket, *value);
        Comms::emitPacket(&tcPacket);

        return tcUpdatePeriod;
    }

    uint32_t tc0Sample() { return tcSample(&HAL::tcAmp0, 110, &tc0Value, tc0ROCValues, &tc0ROC); }
    uint32_t tc1Sample() { return tcSample(&HAL::tcAmp1, 111, &tc1Value, tc1ROCValues, &tc1ROC); }
    uint32_t tc2Sample() { return tcSample(&HAL::tcAmp2, 112, &tc2Value, tc2ROCValues, &tc2ROC); }
    uint32_t tc3Sample() { return tcSample(&HAL::tcAmp3, 113, &tc3Value, tc3ROCValues, &tc3ROC); }
    uint32_t tc4Sample() { return tcSample(&HAL::tcAmp4, 114, &tc4Value, tc4ROCValues, &tc4ROC); }

    uint32_t checkForAbort() {
        //check thermocouple temperatures to be below a threshold
        float maxThermocoupleValue = max(max(Thermocouples::tc0Value, Thermocouples::tc1Value), 
                                        max(Thermocouples::tc2Value, Thermocouples::tc3Value));

        if (maxThermocoupleValue > thermocoupleAbsoluteThreshold) {
            hysteresisValues[0] += 1;
            if (hysteresisValues[0] >= hysteresisThreshold) {
                abortAll();
            }
        } else {
            hysteresisValues[0] = 0;
        }

        if (tc0Value > thermocoupleThreshold && tc0ROC > thermocoupleRateThreshold) {
            hysteresisValues[1] += 1;
            if (hysteresisValues[1] >= hysteresisThreshold) {
                abortAll();

            }
        } else {
            hysteresisValues[1] = 0;
        }

        if (tc1Value > thermocoupleThreshold && tc1ROC > thermocoupleRateThreshold) {
            hysteresisValues[2] += 1;
            if (hysteresisValues[2] >= hysteresisThreshold) {
                abortAll();
            }
        } else {
            hysteresisValues[2] = 0;
        }

        if (tc2Value > thermocoupleThreshold && tc2ROC > thermocoupleRateThreshold) {
            hysteresisValues[3] += 1;
            if (hysteresisValues[3] >= hysteresisThreshold) {
                abortAll();
            }
        } else {
            hysteresisValues[3] = 0;
        }

        if (tc3Value > thermocoupleThreshold && tc3ROC > thermocoupleRateThreshold) {
            hysteresisValues[4] += 1;
            if (hysteresisValues[4] >= hysteresisThreshold) {
                abortAll();
            }
        } else {
            hysteresisValues[4] = 0;
        }

        return tcUpdatePeriod;
    }

    bool abortAll() {
        uint8_t[4] ip_addresses = [31, 32, 33, 34];
        for (uint8_t ip : ip_addresses)
            Comms::emitPacket(&eregAbortPacket, ip);
    }
};
