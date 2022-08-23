
#include "DiagnosticState.h"

namespace StateMachine {

    DiagnosticState diagnosticState = DiagnosticState();

    DiagnosticState* getDiagnosticState() {
        return &diagnosticState;
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
}