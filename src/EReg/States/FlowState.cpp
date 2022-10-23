#include "FlowState.h"

namespace StateMachine {

    FlowState flowState = FlowState();

    FlowState* getFlowState(){
        return &flowState;
    }

    FlowState::FlowState() {
        this->init();
    }

    /**
     * Prepare for start of flow
     */
    void FlowState::init() {
        Util::runMotors(0);
        lastPrint_ = 0;
        timeStarted_ = micros();
        pressureSetpoint_ = 0;
        angleSetpoint_ = 0;
        innerController_->reset();
        outerController_->reset();
        for(int i = 0; i < Config::numReadingsStored; i++){
            previousDownsteamPsiReadings[i] = 1;
            previousUpsteamPsiReadings[i] = 1;
        }

    }

    /**
     * Perform single iteration of flow control loop 
     */
    void FlowState::update() {
        float motorAngle = HAL::encoder.getCount();
        float UpstreamPsi = HAL::readUpstreamPT();
        float DownstreamPsi = HAL::readDownstreamPT();
        unsigned long flowTime = TimeUtil::timeInterval(timeStarted_, micros());
        float speed = 0;

        if (flowTime > Config::loxLead) {
            pressureSetpoint_ = FlowProfiles::flowPressureProfile(flowTime - Config::loxLead);

            //Use dynamic PID Constants
            // Util::PidConstants dynamicPidConstants = Util::computeDynamicPidConstants(UpstreamPsi, DownstreamPsi);
            // outerController_->updateConstants(dynamicPidConstants.k_p, dynamicPidConstants.k_i, dynamicPidConstants.k_d);
            double feedforward = Util::compute_feedforward(pressureSetpoint_, UpstreamPsi, flowTime);

            //Compute Outer Pressure Control Loop
            angleSetpoint_ = outerController_->update(DownstreamPsi - pressureSetpoint_, feedforward);

            //Compute Inner PID Servo loop
            speed = innerController_->update(motorAngle - angleSetpoint_);

            //Add downstreamPSI and upstreamPSI readings to previous-Readings arrays
            updatePreviousReadingsArrays(DownstreamPsi, UpstreamPsi);

            Util::runMotors(speed);
            actuateMainValve(MAIN_VALVE_OPEN);
        } else {
            innerController_->reset();
            outerController_->reset();
        }

        //send data to AC
        if (TimeUtil::timeInterval(lastPrint_, micros()) > Config::telemetryInterval) {
            Packets::sendTelemetry(
                UpstreamPsi,
                DownstreamPsi,
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

        bool downsteamIsAllZeros = true;
        bool upsteamIsAllZeros = true;
        for(int i = 0; i < Config::numReadingsStored; i++){
            if(previousDownsteamPsiReadings[i] != 0){
                downsteamIsAllZeros = false;
                break;
            }
            if(previousUpsteamPsiReadings[i] != 0){
                upsteamIsAllZeros = false;
                break;
            }
        }
        if(downsteamIsAllZeros || upsteamIsAllZeros){
            checkAbortPressure(DownstreamPsi, -1);
            checkAbortPressure(UpstreamPsi, -1);
        }

        checkAbortPressure(DownstreamPsi, Config::abortPressureThresh);
    }

    void FlowState::updatePreviousReadingsArrays(float DownsteamPsi, float UpstreamPsi){
        // Add to previousDownsteamPSIReadings
        for(int i = 0; i < Config::numReadingsStored - 1; i++){
            previousDownsteamPsiReadings[i] = previousDownsteamPsiReadings[i + 1];
        }
        previousDownsteamPsiReadings[Config::numReadingsStored - 1] =  DownsteamPsi;
        // Add to previousUpsteamPSIReadings
        for(int i = 0; i < Config::numReadingsStored - 1; i++){
            previousUpsteamPsiReadings[i] = previousUpsteamPsiReadings[i + 1];
        }
        previousUpsteamPsiReadings[Config::numReadingsStored - 1] =  UpstreamPsi;
    }

}