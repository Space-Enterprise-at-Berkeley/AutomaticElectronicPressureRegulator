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
        float HPpsi = Util::voltageToHighPressure(HAL::adc.readADC(HAL::hpPT));
        float LPpsi = Util::voltageToLowPressure(HAL::adc.readADC(HAL::lpPT));
        float InjectorPT = Util::voltageToLowPressure(HAL::adc.readADC(HAL::injectorPT));
        unsigned long flowTime = TimeUtil::timeInterval(timeStarted_, micros());
        pressureSetpoint_ = FlowProfiles::pressurizationRamp(flowTime);

        float speed = 0;

        //Compute Inner PID Servo loop
        speed = innerController_->update(motorAngle - angleSetpoint_);

        //Compute Outer Pressure Control Loop
        angleSetpoint_ = outerController_->update(LPpsi - pressureSetpoint_, 150);
        
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