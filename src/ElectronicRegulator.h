#include <Arduino.h>
#include <Encoder.h>
#include <data_buff.h>

#define MOTOR1 6
#define MOTOR2 5
#define ENC1 2
#define ENC2 3

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

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   avoid using pins with LEDs attached
Encoder encoder(ENC1, ENC2);



//Create Ereg class that motor and Servo will inherit from?
class Ereg {
    //TODO build a constructor or not
   
    public:
        // Note: 1 rev on main shaft is 3200 counts
        // Encoder itself is 64CPM (including all edges)


        //implement a value for millis() - lastPrint?
        //Read incoming comands
            //bool readComm = Serial.available() > 0

        //int inChar = Serial.read() 
        int speed=0;
        //add unsinged long lastPrint
        //add String inString = ""
        //implement a value for millis() - lastPrint?
        //Read incoming comands
            //bool readComm = Serial.available() > 0

        //int inChar = Serial.read() 
        int speed=0;

        double motorAngle;
        double potAngle;
        double HPpsi;
        double LPpsi;
        double InjectorPT;

        long angle;
        unsigned long lastPrint = 0;
        String inString="";

        void runMotor(){
            analogWrite(MOTOR1,-min(0,speed));
            analogWrite(MOTOR2,max(0,speed));
        }

        double encoderToAngle(double encoderValue) {
            //convert encoder angle to degrees
            // return 45.0 + (encoderValue/3200.0)*360*26/48.0;
            return (encoderValue/3200.0)*360*26/48.0;
        }

        double voltageToPressure(double voltage) {
            //1024 bits in analog read
            //PT voltage frange .4-4.5
            //PT reads from 0-1000
            //Arduino measures voltage from 0 to 5 V
            return (voltage/1024.0*5-0.5)*1000/4.0;
        }

        double readPot(){
            return (analogRead(POTPIN)/1024.0)*90.0;
        }


};

//motor class? maybe not req
class Motor : public Ereg {
    public:
    long startTime = millis();
    long theta0 = encoder.read();

};

//Create Servo class?
    //Think we can use constructor
class Servo : public Ereg {
public:
//Create Servo class?
    //Think we can use constructor
    long angle;
    bool isAngleUpdate;
    long oldPosition=-999;
    long e=0;
    long oldError=0;

    long setPoint=100;

    float kp=11.5;
    float ki=1.5e-6;
    float kd=0.21e6;
    // float kd=0.1665e6;
    long angle_setpoint=0;
    long angle_errorInt=0;

    long errorInt=0;
    unsigned long t2;
    unsigned long dt;
    bool isPrint = true;
    
    unsigned long lastPrint = 0;

    unsigned long flowStart = millis(); // in millis
    unsigned long flowDuration;
    unsigned int printFreq; // in millis

    

    float rawSpd = -(kp*e+kd*(e-oldError)/float(dt));
    
    // user has some error/measurement
    // user wants the control signal
    float update(float measurement) {
        // do all the computations to get control signal
    }
    String inString="";
        

};