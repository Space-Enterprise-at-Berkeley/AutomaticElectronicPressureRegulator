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
        PIDController *innerController = Util::getInnerController();
        PIDController *outerController = Util::getOuterController();
        // Note: 1 rev on main shaft is 3200 counts
        // Encoder itself is 64CPM (including all edges)
        unsigned long timeStarted_; //TODO Overflow detection
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
        PIDController *innerController = Util::getInnerController();
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
        const float closeSpeed_ = -200;
        const float runTime_ = 3000; // in millis
        unsigned long timeStarted_;
        unsigned long lastPrint_;

        public:
        IdleClosedState();
        void init();
        void update();
    };

    // TODO: Write classes for the rest of the states

    FlowState* getFlowState();
    PartiallyOpenState* getPartiallyOpenState();
    IdleClosedState* getIdleClosedState();

};