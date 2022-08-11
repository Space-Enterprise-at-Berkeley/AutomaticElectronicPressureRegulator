#include "PIDController.h"
#include "StateMachine.h"
#include "Config.h"
#include "TimeUtil.h"

namespace StateMachine {

    State currentState = IDLE_CLOSED;
    FlowState flowState = FlowState();
    IdleClosedState idleClosedState = IdleClosedState();
    PartiallyOpenState partiallyOpenState = PartiallyOpenState();
    DiagnosticState diagnosticState = DiagnosticState();

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

    DiagnosticState* getDiagnosticState() {
        return &diagnosticState;
    }

    State getCurrentState() {
        return currentState;
    }

    FlowState::FlowState() {
        this->init();
    }

    /**
     * Prepare for start of flow
     */
    void FlowState::init() {
        Util::runMotors(0);
        lastPrint_ = 0;
        timeStarted_ = micros();
        pressureSetpoint_ = Config::pressureSetpoint; // TODO: support changing pressure setpoint at runtime
        angleSetpoint_ = 0;
        innerController_->reset();
        outerController_->reset();
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
        speed = innerController_->update(motorAngle - angleSetpoint_);

        //Compute Outer Pressure Control Loop
        angleSetpoint_ = outerController_->update(LPpsi - pressureSetpoint_);
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
        this->init(0);
    }

    /**
     * Prepare controllers to enter angle servo mode (partially open ereg)
     */
    void PartiallyOpenState::init(float angleSetpoint) {
        Util::runMotors(0);
        lastPrint_ = 0;
        angleSetpoint_ = angleSetpoint;
        innerController_->reset();
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
        speed = innerController_->update(motorAngle - angleSetpoint_);

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

    DiagnosticState::DiagnosticState() {
        this->init();
    }

    void DiagnosticState::init() {
        currentTest_ = TEST_BEGIN;
        motorDirAngle0_, motorDirAngle1_, motorDirAngle2_ = 0, 0, 0;
        motorDirStage_ = 0;
        servoSetpoint_ = 0;
        servoPassed_ = true;
        startNextTest();
    }

    void DiagnosticState::update() {
        float motorAngle = encoder_->read();
        float HPpsi = Util::voltageToHighPressure(analogRead(HAL::hpPT));
        float LPpsi = Util::voltageToLowPressure(analogRead(HAL::lpPT));
        float InjectorPT = Util::voltageToLowPressure(analogRead(HAL::injectorPT));

        switch (currentTest_) {
            case MOTOR_DIR:
            motorDirTestUpdate();
            break;
            case SERVO:
            servoTestUpdate();
            break;
            default:
            // all tests completed
            enterIdleClosedState();
            return;
        }
    }

    void DiagnosticState::motorDirTestUpdate() {

        float motorAngle = encoder_->read();
        float HPpsi = Util::voltageToHighPressure(analogRead(HAL::hpPT));
        float LPpsi = Util::voltageToLowPressure(analogRead(HAL::lpPT));
        float InjectorPT = Util::voltageToLowPressure(analogRead(HAL::injectorPT));
        
        unsigned long testTime = TimeUtil::timeInterval(timeTestStarted_, micros());
        float speed;

        if (testTime < 500*1000) { // do nothing for 0.5s
            speed = 0;
            motorDirAngle0_ = motorAngle;
        } else if (testTime < 1500*1000) { // open valve for 1s
            speed = testSpeed_;
        } else if (testTime < 2000*1000) { // stop for 0.5s
            speed = 0;
            motorDirAngle1_ = motorAngle;
        } else if (testTime < 3000*1000) { // close valve for 1s
            speed = -testSpeed_;
        } else if (testTime < 3500*1000) { // stop for 0.5s
            speed = 0;
            motorDirAngle2_ = motorAngle;
        } else{
            boolean isPassed = (motorDirAngle1_ > motorDirAngle0_ + Config::minAngleMovement)
            && (motorDirAngle1_ > motorDirAngle2_ + Config::minAngleMovement);
            // TODO send test results to AC
            this->startNextTest();
        }
        Util::runMotors(speed);
        //send data to AC
        if (TimeUtil::timeInterval(lastPrint_, micros()) > 1000) {
            Comms::Packet packet = {.id = 85};
            //TODO split into two telemetry packets
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

    void DiagnosticState::servoTestUpdate() {

        float motorAngle = encoder_->read();
        float HPpsi = Util::voltageToHighPressure(analogRead(HAL::hpPT));
        float LPpsi = Util::voltageToLowPressure(analogRead(HAL::lpPT));
        float InjectorPT = Util::voltageToLowPressure(analogRead(HAL::injectorPT));

        unsigned long testTime = TimeUtil::timeInterval(timeTestStarted_, micros());
        
        float speed = 0;

        // Compute Inner PID Servo loop
        if (testTime < totalTime_) {
            unsigned long intervalNumber = (testTime / servoInterval_);
            servoSetpoint_ = Config::servoTravelInterval * intervalNumber;
            speed = innerController_->update(motorAngle - servoSetpoint_);
            Util::runMotors(speed);
            
            unsigned long intervalTime = TimeUtil::timeInterval(intervalNumber * servoInterval_, testTime);
            if (intervalTime > Config::servoSettleTime) {
                if (abs(servoSetpoint_ - motorAngle) > servoInterval_) {
                    servoPassed_ = false;
                }
            }
        } else {
            // check and report test failure/success
            this->startNextTest();
        }

        // send data to AC
        if (TimeUtil::timeInterval(lastPrint_, micros()) > 1000) {
            Comms::Packet packet = {.id = 85};
            //TODO split into two telemetry packets
            //TODO update packet definition with full telemetry
            Comms::packetAddFloat(&packet, Config::p_outer);
            Comms::packetAddFloat(&packet, Config::i_outer);
            Comms::packetAddFloat(&packet, Config::d_outer);
            Comms::packetAddFloat(&packet, 0);
            Comms::packetAddFloat(&packet, servoSetpoint_);
            Comms::packetAddFloat(&packet, HPpsi);
            Comms::packetAddFloat(&packet, LPpsi);
            Comms::packetAddFloat(&packet, InjectorPT);
            Comms::packetAddFloat(&packet, motorAngle);
            Comms::packetAddFloat(&packet, speed);
            
            Comms::emitPacket(&packet);
            lastPrint_ = micros();
        }
    }

    void DiagnosticState::startNextTest() {
        timeTestStarted_ = micros();
        Util::runMotors(0);
        currentTest_ = currentTest_ + 1;
        if (currentTest_ == SERVO) {
            innerController_->reset();
        }
    }
}
