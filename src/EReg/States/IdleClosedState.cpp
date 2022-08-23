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
        float motorAngle = encoder_->read();
        float HPpsi = Util::voltageToHighPressure(analogRead(HAL::hpPT));
        float LPpsi = Util::voltageToLowPressure(analogRead(HAL::lpPT));
        float InjectorPT = Util::voltageToLowPressure(analogRead(HAL::injectorPT));

        //Compute Inner PID Servo loop
        float speed = 0;
        if (TimeUtil::timeInterval(timeStarted_, millis()) <= runTime_) {
            speed = closeSpeed_;
        }

        Util::runMotors(speed);

        //send data to AC
        if (TimeUtil::timeInterval(lastPrint_, micros()) > Config::telemetryInterval) {
            Packets::sendTelemetry(
                HPpsi,
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