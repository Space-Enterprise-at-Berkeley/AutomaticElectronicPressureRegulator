#include "PIDController.h"
#include "StateMachine.h"
#include "Config.h"
#include "TimeUtil.h"

namespace StateMachine {

    State currentState = IDLE_CLOSED;
    FlowState flowState = FlowState();
    IdleClosedState idleClosedState = IdleClosedState();
    PartiallyOpenState partiallyOpenState = PartiallyOpenState();

    void enterFlowState() {
        currentState = FLOW;
        flowState.init();
    }

    void enterIdleClosedState() {
        currentState = IDLE_CLOSED;
        idleClosedState.init();
    }

    FlowState* getFlowState(){
        return &flowState;
    }

    IdleClosedState* getIdleClosedState() {
        return &idleClosedState;
    }

    PartiallyOpenState* getPartiallyOpenState() {
        return &partiallyOpenState;
    }

    State getCurrentState() {
        return currentState;
    }

    FlowState::FlowState() {
        encoder_ = Util::getEncoder();
        innerController = Util::getInnerController();
        outerController = Util::getOuterController();
        this->init();
    }

    /**
     * Prepare for start of flow
     */
    void FlowState::init() {
        lastPrint_ = 0;
        timeStarted_ = micros();
        pressureSetpoint_ = Config::pressureSetpoint; // TODO: support changing pressure setpoint at runtime
        angleSetpoint_ = 0;
        innerController->reset();
        outerController->reset();
    }

    /**
     * Perform single iteration of flow control loop 
     */
    void FlowState::update() {
        float motorAngle = encoder_->read();
        float HPpsi = Util::voltageToHighPressure(analogRead(HAL::hpPT));
        float LPpsi = Util::voltageToLowPressure(analogRead(HAL::lpPT));
        float InjectorPT = Util::voltageToLowPressure(analogRead(HAL::injectorPT));

        float speed = 0;

        //Compute Inner PID Servo loop
        speed = innerController->update(motorAngle - angleSetpoint_);

        //Compute Outer Pressure Control Loop
        angleSetpoint_ = outerController->update(LPpsi - pressureSetpoint_);
        angleSetpoint_ += Util::compute_feedforward(pressureSetpoint_, HPpsi);

        Util::runMotors(speed);

        //send data to AC
        if (TimeUtil::timeInterval(lastPrint_, micros()) > 1000) {
            Comms::Packet packet = {.id = 85};
            //TODO split into two temelemtry packets
            //TODO update packet definition with full telemetry
            Comms::packetAddFloat(&packet, Config::p_outer);
            Comms::packetAddFloat(&packet, Config::i_outer);
            Comms::packetAddFloat(&packet, Config::d_outer);
            Comms::packetAddFloat(&packet, pressureSetpoint_);
            Comms::packetAddFloat(&packet, angleSetpoint_);
            Comms::packetAddFloat(&packet, HPpsi);
            Comms::packetAddFloat(&packet, LPpsi);
            Comms::packetAddFloat(&packet, InjectorPT);
            Comms::packetAddFloat(&packet, motorAngle);
            Comms::packetAddFloat(&packet, speed);
            
            Comms::emitPacket(&packet);
            lastPrint_ = micros();
        }

        if (TimeUtil::timeInterval(timeStarted_, micros()) > Config::flowDuration) {
            enterIdleClosedState();
        }
    }

    PartiallyOpenState::PartiallyOpenState() {
        encoder_ = Util::getEncoder();
        innerController = Util::getInnerController();
        this->init(0);
    }

    /**
     * Prepare controllers to enter angle servo mode (partially open ereg)
     */
    void PartiallyOpenState::init(float angleSetpoint) {
        lastPrint_ = 0;
        angleSetpoint_ = angleSetpoint;
        innerController->reset();
    }

    /**
     * Perform single iteration of valve angle servo loop
     */
    void PartiallyOpenState::update() {
        float motorAngle = encoder_->read();
        float HPpsi = Util::voltageToHighPressure(analogRead(HAL::hpPT));
        float LPpsi = Util::voltageToLowPressure(analogRead(HAL::lpPT));
        float InjectorPT = Util::voltageToLowPressure(analogRead(HAL::injectorPT));

        float speed = 0;

        //Compute Inner PID Servo loop
        speed = innerController->update(motorAngle - angleSetpoint_);

        Util::runMotors(speed);

        //send data to AC
        if (TimeUtil::timeInterval(lastPrint_, micros()) > 1000) {
            Comms::Packet packet = {.id = 85};
            //TODO split into two temelemtry packets
            //TODO update packet definition with full telemetry
            Comms::packetAddFloat(&packet, Config::p_outer);
            Comms::packetAddFloat(&packet, Config::i_outer);
            Comms::packetAddFloat(&packet, Config::d_outer);
            Comms::packetAddFloat(&packet, 0);
            Comms::packetAddFloat(&packet, angleSetpoint_);
            Comms::packetAddFloat(&packet, HPpsi);
            Comms::packetAddFloat(&packet, LPpsi);
            Comms::packetAddFloat(&packet, InjectorPT);
            Comms::packetAddFloat(&packet, motorAngle);
            Comms::packetAddFloat(&packet, speed);
            
            Comms::emitPacket(&packet);
            lastPrint_ = micros();
        }
    }

    IdleClosedState::IdleClosedState() {
        timeStarted_ = millis();
        lastPrint_ = 0;
    }

    /**
     * Initialization for closing ereg valve
     * Note that this call begins valve closing action immediately, without waiting for update()
     */
    void IdleClosedState::init() {
        timeStarted_ = millis();
        lastPrint_ = 0;
        Util::runMotors(closeSpeed_);
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
        if (TimeUtil::timeInterval(lastPrint_, micros()) > 1000) {
            Comms::Packet packet = {.id = 85};
            //TODO split into two temelemtry packets
            //TODO update packet definition with full telemetry
            Comms::packetAddFloat(&packet, Config::p_outer);
            Comms::packetAddFloat(&packet, Config::i_outer);
            Comms::packetAddFloat(&packet, Config::d_outer);
            Comms::packetAddFloat(&packet, 0);
            Comms::packetAddFloat(&packet, 0);
            Comms::packetAddFloat(&packet, HPpsi);
            Comms::packetAddFloat(&packet, LPpsi);
            Comms::packetAddFloat(&packet, InjectorPT);
            Comms::packetAddFloat(&packet, motorAngle);
            Comms::packetAddFloat(&packet, speed);
            
            Comms::emitPacket(&packet);
            lastPrint_ = micros();
        }
    }
}
