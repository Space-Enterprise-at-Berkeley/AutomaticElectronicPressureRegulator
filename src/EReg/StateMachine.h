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

    enum State { IDLE_CLOSED, PARTIAL_OPEN, PRESSURIZE, FLOW, DIAGNOSTIC }; // TODO implement vent and pressurize
    
    void enterFlowState();
    void enterIdleClosedState();
    void enterPartialOpenState(float angle);
    void enterDiagnosticState();
    void enterPressurizeState();
    State getCurrentState();
    void checkAbortPressure(float currentPressure, float abortPressure);

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
        const float runTime_ = Config::closeTime; // in millis
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
        unsigned long longestSettleTime_;

        public:
        DiagnosticState();
        void init();
        void update();
        void motorDirTestUpdate();
        void servoTestUpdate();
        void startNextTest();
    };

    class PressurizeState {
        private:
        Encoder *encoder_ = Util::getEncoder();
        PIDController *innerController_ = Util::getInnerController(); 
        PIDController *outerController_ = Util::getOuterController(); //TODO Use special I-only controller!
        // Note: 1 rev on main shaft is 3200 counts
        // Encoder itself is 64CPM (including all edges)
        unsigned long timeStarted_;
        unsigned long lastPrint_;
        float pressureSetpoint_ = Config::pressureSetpoint;;
        float angleSetpoint_;

        public:
        PressurizeState();
        void init();
        void update();
    };
    // TODO: Write classes for the rest of the states

    FlowState* getFlowState();
    PartiallyOpenState* getPartiallyOpenState();
    IdleClosedState* getIdleClosedState();
    DiagnosticState* getDiagnosticState();
    PressurizeState* getPressurizeState();
};