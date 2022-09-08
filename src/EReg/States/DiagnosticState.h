#pragma once

#include <Arduino.h>
#include <data_buff.h>
#include <PIDController.h>
#include <Encoder.h>
#include <TimeUtil.h>
#include "EReg/StateMachine.h"
#include "EReg/Packets.h"
#include "Ereg/HAL.h"
#include "Ereg/Util.h"
#include "Ereg/Config.h"

namespace StateMachine {

    class DiagnosticState {
        private:
        Encoder *encoder_ = Util::getEncoder();
        PIDController *innerController_ = Util::getInnerController();
        Buffer *highPressureAbortBuffer_;
        Buffer *lowPressureAbortBuffer_;

        const float testSpeed_ = 50;
        const unsigned long servoInterval_ = Config::servoSettleTime * 4;
        const unsigned long totalTime_ = (unsigned long)Config::servoTestPoints * servoInterval_;

        unsigned long lastPrint_;
        unsigned long timeTestStarted_;
        enum DiagnosticTest:int { TEST_BEGIN = 0, MOTOR_DIR = 1, SERVO = 2, MOCK_PRESSURIZE = 3, MOCK_FLOW = 4, TEST_COMPLETE = 5 };
        DiagnosticTest currentTest_;

        float motorDirAngle0_, motorDirAngle1_, motorDirAngle2_;
        int motorDirStage_;
        bool motorDirPassed_;
        float servoSetpoint_;
        bool servoPassed_;
        unsigned long longestSettleTime_;

        bool isMockInitialized_ = false;
        const unsigned long mockPressurizationDuration_ = Config::pressurizationRampDuration;
        const unsigned long mockFlowDuration_ = Config::flowDuration - (1000UL * 1000UL); // careful about underflows if flow < 1s

        public:
        DiagnosticState();
        void init();
        void update();
        void motorDirTestUpdate();
        void servoTestUpdate();
        void mockPressurizeTestUpdate();
        void mockFlowTestUpdate();
        void startNextTest();
    };

    DiagnosticState* getDiagnosticState();

}