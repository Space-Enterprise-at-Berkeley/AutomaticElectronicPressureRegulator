#pragma once

#include <Arduino.h>
#include <data_buff.h>
#include <PIDController.h>
#include <TimeUtil.h>
#include "EReg/StateMachine.h"
#include "Ereg/HAL.h"
#include "Ereg/Util.h"
#include "Ereg/Comms.h"
#include "Ereg/Config.h"
#include "EReg/FlowProfiles.h"
#include "EReg/Packets.h"

namespace StateMachine {
    
    class FlowState {
        private:
        PIDController *innerController_ = Util::getInnerController();
        PIDController *outerController_ = Util::getOuterController();
        unsigned long timeStarted_;
        unsigned long lastPrint_;
        float pressureSetpoint_;
        float angleSetpoint_;

        float previousDownsteamPsiReadings[Config::numReadingsStored];
        float previousUpsteamPsiReadings[Config::numReadingsStored];

        public:
        FlowState();
        void init();
        void update();
        void updatePreviousReadingsArrays(float DownsteamPsi, float UpstreamPsi);
    };

    FlowState* getFlowState();

}