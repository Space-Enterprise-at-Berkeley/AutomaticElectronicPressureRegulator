#include "PressurizeState.h"

namespace StateMachine {

    PressurizeState pressurizeState = PressurizeState();

    PressurizeState* getPressurizeState() {
        return &pressurizeState;
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
        float motorAngle = HAL::encoder.getCount()
;
        float upstreamPsi = HAL::readUpstreamPT();
        float downstreamPsi = HAL::readDownstreamPT();
        unsigned long flowTime = TimeUtil::timeInterval(timeStarted_, micros());
        pressureSetpoint_ = FlowProfiles::pressurizationRamp(flowTime);

        float speed = 0;

        //Compute Inner PID Servo loop
        speed = innerController_->update(motorAngle - angleSetpoint_);

        //Compute Outer Pressure Control Loop
        angleSetpoint_ = outerController_->update(downstreamPsi - pressureSetpoint_, 150);
        
        Util::runMotors(speed);

        //send data to AC
        if (TimeUtil::timeInterval(lastPrint_, micros()) > Config::telemetryInterval) {
            Packets::sendTelemetry(
                upstreamPsi,
                downstreamPsi,
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

        if (downstreamPsi > Config::pressurizationCutoff) {
            enterIdleClosedState();
        }
    }
    
}