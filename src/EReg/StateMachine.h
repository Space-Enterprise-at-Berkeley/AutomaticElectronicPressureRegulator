#pragma once

#include <Arduino.h>
#include <data_buff.h>
#include <PIDController.h>
#include <Encoder.h>
#include "HAL.h"
#include "Util.h"
#include "Comms.h"
#include "Config.h"

namespace StateMachine {

    enum State { IDLE_CLOSED, PARTIAL_OPEN, VENT, PRESSURIZE, FLOW }; 
    
    void enterFlowState();
    void enterIdleClosedState();
    State getCurrentState();

    class FlowState {
        private:
        Encoder *encoder_ = Util::getEncoder();
        PIDController *innerController_ = Util::getInnerController();
        PIDController *outerController_ = Util::getOuterController();
        // Note: 1 rev on main shaft is 3200 counts
        // Encoder itself is 64CPM (including all edges)
        unsigned long timeStarted_;
        unsigned long lastPrint_;
        float pressureSetpoint_;
        float angleSetpoint_;

        public:
        FlowState();
        void init();
        void update();
    };

    class PartiallyOpenState {
        private:
        Encoder *encoder_ = Util::getEncoder();
        PIDController *innerController_ = Util::getInnerController();
        // Note: 1 rev on main shaft is 3200 counts
        // Encoder itself is 64CPM (including all edges)
        unsigned long lastPrint_;
        float angleSetpoint_;

        public:
        PartiallyOpenState();
        void init(float angleSetpoint);
        void setAngle(float angleSetpoint);
        void update();
    };

    class IdleClosedState {
        private:
        Encoder *encoder_ = Util::getEncoder();
        // Note: 1 rev on main shaft is 3200 counts
        // Encoder itself is 64CPM (including all edges)
        const float closeSpeed_ = -OPEN_LOOP_SPEED;
        const float runTime_ = 3000; // in millis
        unsigned long timeStarted_;
        unsigned long lastPrint_;

        public:
        IdleClosedState();
        void init();
        void update();
    };

    class DiagnosticState {
        private:
        Encoder *encoder_ = Util::getEncoder();
        PIDController *innerController_ = Util::getInnerController();

        const float testSpeed_ = OPEN_LOOP_SPEED;
        const unsigned long servoInterval_ = Config::servoSettleTime * 2;
        const unsigned long totalTime_ = (unsigned long)Config::servoTestPoints * servoInterval_;

        unsigned long lastPrint_;
        unsigned long timeTestStarted_;
        enum DiagnosticTest:int { TEST_BEGIN = 0, MOTOR_DIR = 1, SERVO = 2, TEST_COMPLETE = 3 };
        DiagnosticTest currentTest_;

        float motorDirAngle0_, motorDirAngle1_, motorDirAngle2_;
        int motorDirStage_;
        float servoSetpoint_;
        bool servoPassed_;

        public:
        DiagnosticState();
        void init();
        void update();
        void motorDirTestUpdate();
        void servoTestUpdate();
        void startNextTest();
    };
    // TODO: Write classes for the rest of the states

    FlowState* getFlowState();
    PartiallyOpenState* getPartiallyOpenState();
    IdleClosedState* getIdleClosedState();
    DiagnosticState* getDiagnosticState();

};