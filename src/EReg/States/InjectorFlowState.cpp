#include "InjectorFlowState.h"

namespace StateMachine {

    InjectorFlowState injectorFlowState = InjectorFlowState();

    InjectorFlowState* getInjectorFlowState(){
        return &injectorFlowState;
    }

    InjectorFlowState::InjectorFlowState() {
        this->init();
    }

    /**
     * Prepare for start of flow
     */
    void InjectorFlowState::init() {
        Util::runMotors(0);
        lastPrint_ = 0;
        timeStarted_ = micros();
        pressureSetpoint_ = 0;
        angleSetpoint_ = 0;
        innerController_->reset();
        outerController_->reset();
    }

    /**
     * Perform single iteration of flow control loop 
     */
    void InjectorFlowState::update() {
        float motorAngle = HAL::encoder.getCount();
        float LPpsi = Ducers::readLPPT();
        float InjectorPT = Ducers::readInjectorPT();
        unsigned long flowTime = TimeUtil::timeInterval(timeStarted_, micros());
        float speed = 0;

        if (flowTime > Config::loxLead) {
            pressureSetpoint_ = FlowProfiles::constantPressure(flowTime - Config::loxLead);

            //Compute Outer Pressure Control Loop
            angleSetpoint_ = outerController_->update(InjectorPT - pressureSetpoint_, Util::compute_injector_feedforward());

            //Compute Inner PID Servo loop
            speed = innerController_->update(motorAngle - angleSetpoint_);

            Util::runMotors(speed);
        } else {
            innerController_->reset();
            outerController_->reset();
        }

        //send data to AC
        if (TimeUtil::timeInterval(lastPrint_, micros()) > Config::telemetryInterval) {
            Packets::sendTelemetry(
                0,
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

        if (flowTime > Config::flowDuration) {
            enterIdleClosedState();
        }

        checkAbortPressure(LPpsi, Config::abortPressureThresh);
    }

}