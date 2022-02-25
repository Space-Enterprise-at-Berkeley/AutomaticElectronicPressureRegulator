#pragma once

#include <Arduino.h>
#include <Encoder.h>
#include <data_buff.h>

#define MOTOR1 6
#define MOTOR2 5
#define ENC1 2
#define ENC2 3

#define PRESSURE_MAXIMUM 1000

#define MAX_SPD 255
#define MIN_SPD -255
// #define STATIC_SPD 60
#define STATIC_SPD 0

#define MAX_ANGLE 1296
#define MIN_ANGLE 0

#define POTPIN A0
#define HP_PT A1
#define LP_PT A2
#define INJECTOR_PT A3

#define BUFF_SIZE 5

#define USE_DASHBOARD 1

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   avoid using pins with LEDs attached
Encoder encoder(ENC1, ENC2);

class PID {

    //servo variables
    //long angle; -> input for servo fn?
    bool isAngleUpdate;
    long oldPosition=-999;

    //PID data
    long e=0;
    long oldError=0;
    long setPoint;
    long errorInt=0;
    float kp;
    float ki;
    float kd;
    // float kp=11.5;
    // float ki=1.5e-6;
    // float kd=0.21e6;
    // float kd=0.1665e6;

    //inner loop constants
    //long angle_setpoint=0;
    long angle_errorInt=0;

    //PT variables
    float old_pressure = 0;
    float pressure = 0;
    long time = 0;
    long old_time = 0;
    float rawSpd;
    //int count = 0;

    unsigned long t2;
    unsigned long dt;
    
    //unsigned long lastPrint = 0;

    //Servo Characterization
    unsigned long flowStart = millis(); // in millis
    unsigned long flowDuration;
    unsigned int printFreq; // in millis

    //Outer Loop constants
    double outer_e = 0;
    double outer_e_old = 0;
    double outer_errorInt =0;
    //Buffer* p_buff;


    public:
        // Note: 1 rev on main shaft is 3200 counts
        // Encoder itself is 64CPM (including all edges)
        int speed=0;
        double motorAngle;
        double potAngle;
        double HPpsi;
        double LPpsi;
        double InjectorPT;
        long angle;
        unsigned long lastPrint = 0;
        bool isPrint = true;
        String inString="";

        //loop constants
        long angle_setpoint=0;
        Buffer* p_buff;


        //create a constructor with inputs for kp,ki,kd,angle, setpoint, angle_setpoint?
        PID (float kp, float ki, float kd, long angle, long setPoint);
        void runMotor();
        double encoderToAngle(double encoderValue);
        double voltageToPressure(double voltage);
        double voltageToHighPressure(double voltage);
        double readPot();
        void updatePT();
        void updateAngle(long angle); //Is setpoint an input?
        int waitConfirmation();
        void angleSweep(long startAngle, long endAngle, unsigned long flowDuration, long extraTime);
        void updatePressure(double kp_outer, double ki_outer, double kd_outer, double pressure_setpoint);
        boolean pressurize_tank(double kp_outer, double ki_outer, double kd_outer, double pressure_setpoint);

};
