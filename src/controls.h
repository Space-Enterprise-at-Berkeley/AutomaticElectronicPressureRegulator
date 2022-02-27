#include <utility.h>



class PID {

    //servo variables
    //long angle; -> input for servo fn?
    // bool isAngleUpdate;
    // long oldPosition=-999;

    //PID data
    long error;
    long oldError;
    long setPoint;
    long errorInt;
    float rawValue;
    long time;
    int maxSpd = 255;
    int minSpd = 255;
    int maxAngle = 1296;
    int minAngle = 0;
    int staticSpd = 0;
    //staticSpd = 60;
    float kp;
    float ki;
    float kd;
    
    // float kp=11.5;
    // float ki=1.5e-6;
    // float kd=0.21e6;
    // float kd=0.1665e6;

    //anti-windup selector
    bool method;


    //inner loop constants
    //long angle_setpoint=0;
    long angle_errorInt=0;

    //PT variables
    // float old_pressure = 0;
    // float pressure = 0;
    // long old_time = 0;
    // float rawSpd;
    //int count = 0;

    unsigned long t2;
    unsigned long dt;
    
    //unsigned long lastPrint = 0;

    //Servo Characterization
    // unsigned long flowStart = millis(); // in millis
    // unsigned long flowDuration;
    // unsigned int printFreq; // in millis
    //Buffer* p_buff;


    public:
        // Note: 1 rev on main shaft is 3200 counts
        // Encoder itself is 64CPM (including all edges)
        double motorAngle;
        double potAngle;
        double HPpsi;
        double LPpsi;
        double InjectorPT;
        unsigned long lastPrint = 0;
        bool isPrint = true;
        String inString="";

        //loop constants
        long angle_setpoint=0;


        //create a constructor with inputs for kp,ki,kd,angle, setpoint, angle_setpoint?
        PID (float kp, float ki, float kd, double setPoint, bool method);
        //void updatePT();
        double update(double input);

};
