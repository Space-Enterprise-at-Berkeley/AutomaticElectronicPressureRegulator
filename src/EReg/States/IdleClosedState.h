#pragma once

#include <Arduino.h>
#include <data_buff.h>
#include <PIDController.h>
#include <Encoder.h>
#include "EReg/StateMachine.h"
#include "Ereg/HAL.h"
#include "Ereg/Util.h"
#include "Ereg/Comms.h"
#include "Ereg/Config.h"

namespace StateMachine {

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
    
    IdleClosedState* getIdleClosedState();
}