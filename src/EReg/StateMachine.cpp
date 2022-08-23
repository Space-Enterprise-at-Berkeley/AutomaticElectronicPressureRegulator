#include "PIDController.h"
#include "StateMachine.h"
#include "Config.h"
#include "Packets.h"
#include "TimeUtil.h"
#include "FlowProfiles.h"

namespace StateMachine {

    State currentState = IDLE_CLOSED;
    ValveAction currentMainValveState = MAIN_VALVE_CLOSE;
    FlowState flowState = FlowState();
    IdleClosedState idleClosedState = IdleClosedState();
    PartiallyOpenState partiallyOpenState = PartiallyOpenState();
    DiagnosticState diagnosticState = DiagnosticState();
    PressurizeState pressurizeState = PressurizeState();

    void enterFlowState() {
        if (currentState == IDLE_CLOSED) {
            currentState = FLOW;
            flowState.init();
        } else {
            // Illegal state transition
            Packets::sendStateTransitionError(0);
        }
    }

    void enterIdleClosedState() {
        currentState = IDLE_CLOSED;
        idleClosedState.init();
    }

    void enterPartialOpenState(float angle) {
        if (currentState == IDLE_CLOSED || currentState == PARTIAL_OPEN) {
            if (angle > (MIN_ANGLE - 100) && angle < (MAX_ANGLE + 100)) {
                currentState = PARTIAL_OPEN;
                partiallyOpenState.init(angle);
            } else {
                // Illegal parameters
                Packets::sendStateTransitionError(1);
            }
        } else {
            // Illegal state transition
            Packets::sendStateTransitionError(2);
        }
    }

    void enterDiagnosticState() {
        if (currentState == IDLE_CLOSED) {
            currentState = DIAGNOSTIC;
            diagnosticState.init();
        } else {
            // Illegal state transition
            Packets::sendStateTransitionError(3);
        }
    }

    void enterPressurizeState() {
        if (currentState == IDLE_CLOSED) {
            currentState = PRESSURIZE;
            pressurizeState.init();
        } else {
            // Illegal state transition
            Packets::sendStateTransitionError(4);
        }
    }

    void enterMainValveState(uint8_t actionByte) {
        ValveAction action = actionByte ? MAIN_VALVE_OPEN : MAIN_VALVE_CLOSE;
        if (currentState == IDLE_CLOSED || currentState == PARTIAL_OPEN) {
            actuateMainValve(action);
        } else {
            // Illegal state transition
            Packets::sendStateTransitionError(5);
        }
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

    PressurizeState* getPressurizeState() {
        return &pressurizeState;
    }

    State getCurrentState() {
        return currentState;
    }

    /**
     * Helper function to actuate main valve. DO NOT have outside code calling this function - 
     * This function does not validate the current state
     * @param action Desired valve state
     */
    void actuateMainValve(ValveAction action) {
        switch (action) {
            case MAIN_VALVE_OPEN:
                digitalWrite(HAL::mainValve1, HIGH);
                digitalWrite(HAL::mainValve2, LOW);
            break;
            case MAIN_VALVE_CLOSE:
                digitalWrite(HAL::mainValve1, LOW);
                digitalWrite(HAL::mainValve2, LOW);
            break;
        }
        currentMainValveState = action;
    }

    void checkAbortPressure(float currentPressure, float abortPressure) {
        if (currentPressure > abortPressure) {
            Packets::sendFlowState(0);
            StateMachine::enterIdleClosedState();
        }
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
        pressureSetpoint_ = 0;
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
        unsigned long flowTime = TimeUtil::timeInterval(timeStarted_, micros());
        pressureSetpoint_ = FlowProfiles::linearRampup(flowTime);

        float speed = 0;

        //Use dynamic PID Constants
        Util::PidConstants dynamicPidConstants = Util::computeDynamicPidConstants(HPpsi, LPpsi);
        outerController_->updateConstants(dynamicPidConstants.k_p, dynamicPidConstants.k_i, dynamicPidConstants.k_d);

        //Compute Outer Pressure Control Loop
        angleSetpoint_ = outerController_->update(LPpsi - pressureSetpoint_);
        angleSetpoint_ += Util::compute_feedforward(pressureSetpoint_, HPpsi);
        angleSetpoint_ = Util::clip(angleSetpoint_, MIN_ANGLE, MAX_ANGLE);

        //Compute Inner PID Servo loop
        speed = innerController_->update(motorAngle - angleSetpoint_);

        Util::runMotors(speed);
        actuateMainValve(MAIN_VALVE_OPEN);

        //send data to AC
        if (TimeUtil::timeInterval(lastPrint_, micros()) > Config::telemetryInterval) {
            Packets::sendTelemetry(
                HPpsi,
                LPpsi,
                InjectorPT,
                motorAngle,
                angleSetpoint_,
                pressureSetpoint_,
                speed,
                outerController_->getPTerm(),
                outerController_->getITerm(),
                outerController_->getDTerm()
            );
            lastPrint_ = micros();
        }

        if (flowTime > Config::flowDuration) {
            enterIdleClosedState();
        }

        checkAbortPressure(LPpsi, Config::abortPressureThresh);
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
        if (TimeUtil::timeInterval(lastPrint_, micros()) > Config::telemetryInterval) {
            Packets::sendTelemetry(
                HPpsi,
                LPpsi,
                InjectorPT,
                motorAngle,
                angleSetpoint_,
                0,
                speed,
                0,
                0,
                0
            );
            lastPrint_ = micros();
        }

        checkAbortPressure(LPpsi, Config::abortPressureThresh);
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

    DiagnosticState::DiagnosticState() {
        highPressureAbortBuffer_ = new Buffer(DIAGNOSTIC_BUFFER_SIZE);
        lowPressureAbortBuffer_ = new Buffer(DIAGNOSTIC_BUFFER_SIZE);
        this->init();
    }

    void DiagnosticState::init() {
        // Packets::sendDiagnostic(true, "Diagnostic state init");
        currentTest_ = TEST_BEGIN;
        motorDirAngle0_ = 0, motorDirAngle1_ = 0, motorDirAngle2_ = 0;
        motorDirStage_ = 0;
        servoSetpoint_ = 0;
        longestSettleTime_ = 0;
        servoPassed_ = true;
        motorDirPassed_ = true;
        startNextTest();
    }

    void DiagnosticState::update() {

        switch (currentTest_) {
            case MOTOR_DIR:
            motorDirTestUpdate();
            break;
            case SERVO:
            servoTestUpdate();
            break;
            default:
            // all tests completed
            Packets::sendDiagnostic(motorDirPassed_, servoPassed_);
            enterIdleClosedState();
            return;
        }
    }

    void DiagnosticState::motorDirTestUpdate() {

        // Packets::sendDiagnostic(true, "starting motor dir test");

        float motorAngle = encoder_->read();
        float HPpsi = Util::voltageToHighPressure(analogRead(HAL::hpPT));
        float LPpsi = Util::voltageToLowPressure(analogRead(HAL::lpPT));
        float InjectorPT = Util::voltageToLowPressure(analogRead(HAL::injectorPT));
        
        unsigned long testTime = TimeUtil::timeInterval(timeTestStarted_, micros());
        float speed;


        if (testTime < 500UL*1000UL) { // do nothing for 0.5s
            speed = 0;
            motorDirAngle0_ = motorAngle;
        } else if (testTime < 1500UL*1000UL) { // open valve for 1s
            speed = testSpeed_;
        } else if (testTime < 2000UL*1000UL) { // stop for 0.5s
            speed = 0;
            motorDirAngle1_ = motorAngle;
        } else if (testTime < 3000UL*1000UL) { // close valve for 1s
            speed = -testSpeed_;
        } else if (testTime < 3500UL*1000UL) { // stop for 0.5s
            speed = 0;
            motorDirAngle2_ = motorAngle;
        } else{
            motorDirPassed_ = (motorDirAngle1_ > motorDirAngle0_ + Config::minAngleMovement)
            && (motorDirAngle1_ > motorDirAngle2_ + Config::minAngleMovement);            
            this->startNextTest();
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
        highPressureAbortBuffer_->insert(testTime/1.0e6, HPpsi);
        lowPressureAbortBuffer_->insert(testTime/1.0e6, LPpsi);
        checkAbortPressure(highPressureAbortBuffer_->getAverage(), Config::stopDiagnosticPressureThresh);
        checkAbortPressure(lowPressureAbortBuffer_->getAverage(), Config::stopDiagnosticPressureThresh);
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
            
            if (abs(servoSetpoint_ - motorAngle) > Config::servoSettleThresh) {
                if (intervalTime > Config::servoSettleTime) {
                    servoPassed_ = false;
                }
                longestSettleTime_ = max(longestSettleTime_, intervalTime);
            }
        } else {
            this->startNextTest();
        }

        // send data to AC
        if (TimeUtil::timeInterval(lastPrint_, micros()) > Config::telemetryInterval) {
            Packets::sendTelemetry(
                HPpsi,
                LPpsi,
                InjectorPT,
                motorAngle,
                servoSetpoint_,
                0,
                speed,
                0,
                0,
                0
            );
            lastPrint_ = micros();
        }
        highPressureAbortBuffer_->insert(testTime/1.0e6, HPpsi);
        lowPressureAbortBuffer_->insert(testTime/1.0e6, LPpsi);
        checkAbortPressure(highPressureAbortBuffer_->getAverage(), Config::stopDiagnosticPressureThresh);
        checkAbortPressure(lowPressureAbortBuffer_->getAverage(), Config::stopDiagnosticPressureThresh);
    }

    void DiagnosticState::startNextTest() {
        timeTestStarted_ = micros();
        Util::runMotors(0);
        currentTest_ = static_cast<DiagnosticTest>((int)currentTest_ + 1);
        if (currentTest_ == SERVO) {
            innerController_->reset();
        }
        highPressureAbortBuffer_->clear();
        lowPressureAbortBuffer_->clear();
    }

    PressurizeState::PressurizeState() {
        this->init();
    }

    void PressurizeState::init() {
        Util::runMotors(0);
        lastPrint_ = 0;
        timeStarted_ = micros();
        angleSetpoint_ = 0;
        innerController_->reset();
        outerController_->reset();
    }

    void PressurizeState::update() {
        float motorAngle = encoder_->read();
        float HPpsi = Util::voltageToHighPressure(analogRead(HAL::hpPT));
        float LPpsi = Util::voltageToLowPressure(analogRead(HAL::lpPT));
        float InjectorPT = Util::voltageToLowPressure(analogRead(HAL::injectorPT));
        unsigned long flowTime = TimeUtil::timeInterval(timeStarted_, micros());
        pressureSetpoint_ = FlowProfiles::pressurizationRamp(flowTime);

        float speed = 0;

        //Compute Inner PID Servo loop
        speed = innerController_->update(motorAngle - angleSetpoint_);

        //Compute Outer Pressure Control Loop
        angleSetpoint_ = outerController_->update(LPpsi - pressureSetpoint_);
        angleSetpoint_ += 100;
        angleSetpoint_ = Util::clip(angleSetpoint_, MIN_ANGLE, MAX_ANGLE);
        Util::runMotors(speed);

        //send data to AC
        if (TimeUtil::timeInterval(lastPrint_, micros()) > Config::telemetryInterval) {
            Packets::sendTelemetry(
                HPpsi,
                LPpsi,
                InjectorPT,
                motorAngle,
                angleSetpoint_,
                pressureSetpoint_,
                speed,
                outerController_->getPTerm(),
                outerController_->getITerm(),
                outerController_->getDTerm()
            );
            lastPrint_ = micros();
        }

        if (LPpsi > Config::pressurizationCutoff) {
            enterIdleClosedState();
        }
    }
}
