#include "IdleClosedState.h"

namespace StateMachine {

    IdleClosedState idleClosedState = IdleClosedState();

    IdleClosedState* getIdleClosedState() {
        return &idleClosedState;
    }

    IdleClosedState::IdleClosedState() {
        this->init();
    }

    /**
     * Initialization for closing ereg valve
     * Note that this call begins valve closing action immediately, without waiting for update()
     */
    void IdleClosedState::init() {
        timeStarted_ = millis();
        lastPrint_ = 0;
        Util::runMotors(closeSpeed_);
        actuateMainValve(MAIN_VALVE_CLOSE);
    }

    /**
     * IdleClosed update function
     * If sufficient time has passed, turn off motors
     * Transmit telemetry
     */
    void IdleClosedState::update() {
        float motorAngle = HAL::encoder.getCount()
;
        float HPpsi = Util::voltageToHighPressure(HAL::adc.readADC(HAL::hpPT));
        float LPpsi = Util::voltageToLowPressure(HAL::adc.readADC(HAL::lpPT));
        float InjectorPT = Util::voltageToLowPressure(HAL::adc.readADC(HAL::injectorPT));

        //Compute Inner PID Servo loop
        float speed = 0;
        if (TimeUtil::timeInterval(timeStarted_, millis()) <= runTime_) {
            speed = closeSpeed_;
        }

        Util::runMotors(speed);

        //send data to AC
        Serial.println("senidng data");
        if (TimeUtil::timeInterval(lastPrint_, micros()) > Config::telemetryIntervalIdle) {
            Packets::sendTelemetry(
                sin(millis()/1000.0),
                LPpsi,
                InjectorPT,
                motorAngle,
                0,
                0,
                speed,
                0,
                0,
                0
            );
            lastPrint_ = micros();
        }
    }

}