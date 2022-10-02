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
        float motorAngle = HAL::encoder.getCount();

        float HPpsi = Ducers::readHPPT();
        float LPpsi = Ducers::readLPPT();
        float InjectorPT = Ducers::readInjectorPT(); // TODO remove

        //Compute Inner PID Servo loop
        float speed = 0;
        if (TimeUtil::timeInterval(timeStarted_, millis()) <= runTime_) {
            speed = closeSpeed_;
        }

        Util::runMotors(speed);

        // send data to AC
        if (TimeUtil::timeInterval(lastPrint_, micros()) > Config::telemetryIntervalIdle) {
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