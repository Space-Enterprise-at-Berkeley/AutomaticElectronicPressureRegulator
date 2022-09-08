#pragma once

#include <Arduino.h>
#include <PIDController.h>
#include <Encoder.h>
#include "EReg/StateMachine.h"
#include "Ereg/HAL.h"
#include "Ereg/Util.h"
#include "Ereg/Packets.h"
#include "Ereg/Config.h"

namespace StateMachine {

    class PressurizeState {
        private:
        Encoder *encoder_ = Util::getEncoder();
        PIDController *innerController_ = Util::getInnerController(); 
        PIDController *outerController_ = Util::getOuterController(); 
        unsigned long timeStarted_;
        unsigned long lastPrint_;
        float pressureSetpoint_ = Config::pressureSetpoint;;
        float angleSetpoint_;

        public:
        PressurizeState();
        void init();
        void update();
    };

    PressurizeState* getPressurizeState();
}