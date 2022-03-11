#include <Comms.h>
#include <controls.h>
#include <test.h>
#include <utility.h>
#include <config.h>

#define USE_DASHBOARD 1

Encoder encoder(ENC1, ENC2);

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   avoid using pins with LEDs attached
//PID control = PID(11.5, 1.5e-6, 0.1665e6, 0, 100);


// Note: 1 rev on main shaft is 3200 counts
// Encoder itself is 64CPM (including all edges)
// int speed=0;

// double motorAngle;
// double potAngle;
// double HPpsi;
// double LPpsi;
// double InjectorPT;


//start of main loop

// long angle;
// bool isAngleUpdate;
// long oldPosition=-999;
// long e=0;
// long oldError=0;
// String inString = "";

// Inner loop constants
// float kp=11.5;
// float ki=1.5e-6;
// float kd=0.1665e6;
// long angle_setpoint=0;
// long angle_errorInt=0;

// Constants should be tuned
// kp = 35
// sweep.ki = 25e-6
// kd = 2.0e6
// perhaps run filtering on pressure derivative
// double kp_outer = 30; // encoder counts per psi
// double ki_outer = 30.0e-6; // time in micros
// double kd_outer = 2.5; // time in s
// double pressure_setpoint = 130; //100
// double pressure_e = 0;
// double pressure_e_old = 0;
// double pressure_errorInt = 0;

// unsigned long t2;
// unsigned long dt;
// unsigned long start_time;

Buffer* p_buff;
unsigned long startTime;


void setup() {
    //Start with valve line perpendicular to body (90 degrees)
    Serial.begin(115200);
    while(!Serial);
    p_buff = new Buffer(BUFF_SIZE);

    delay(500);

    #ifndef USE_DASHBOARD
    waitConfirmation();
    // move to close motors
    Serial.println("Zeroing valve");
    #endif

    //speed = -150;
    //will hard code the runMotor() here instead to force it to close
    //utility::runMotor();
    analogWrite(MOTOR1, -min(0, -150));
    analogWrite(MOTOR2, max(0, -150));
    delay(2000);
    utility::runMotor();
    // zero encoder value (so encoder readings range from -x (open) to 0 (closed))
    encoder.write(-20);

    #ifndef USE_DASHBOARD
    waitConfirmation();
    #endif
    // motorDirTest();
    tests::ptTest();
    delay(500);
    // servoTest();
    // motorPowerTest();
    tests::pressureize_tank();
    
    utility::waitConfirmation();

    #ifndef USE_DASHBOARD
    Serial.println("Next input will start servo loop, starting setpoint = "+String(pressure_setpoint));
    waitConfirmation();
    #endif

    // long startAngle = 300*1.08;
    // long endAngle = 1200*1.08;
    // long thirdAngle = 300*1.08;
    // long thirdAngle = 500*1.08;
    // long flowDuration = 5000; //time in ms for one way
    // Serial.println("Starting angle sweep from "+String(startAngle)+" to "+String(endAngle)+" then back to " + String(thirdAngle) + " over "+String(2*flowDuration)+" ms...");
    
    // potTest();
    // servoTest();
    
    // angleSweep(startAngle, endAngle, flowDuration, 0);
    // angleSweep(endAngle, thirdAngle, flowDuration, 5000);
    // ptTest();
    // exit(0);
    startTime = micros();

}

// long lastPrint = 0;
// // Start in closed position, sweep.angle should be 0




void loop() {

    double motorAngle = utility::encoderToAngle(encoder.read());
    double potAngle = utility::readPot();
    double HPpsi = utility::voltageToHighPressure(analogRead(HP_PT));
    double LPpsi = utility::voltageToPressure(analogRead(LP_PT));
    double InjectorPT = utility::voltageToPressure(analogRead(INJECTOR_PT));
    
    // LPpsi = analogRead(POTPIN)/1024.0*360;

    // isAngleUpdate=(angle!=oldPosition);
    // e=angle-angle_setpoint;

    //Compute Inner PID Servo loop
    long angle_setpoint = 0;
    PID inner = PID (11.5, 1.5e-6, 0.1665e6, angle_setpoint, false);
    inner.update(encoder.read());

    //Compute Outer Pressure Control Loop and constrain angles and speeds
    double pressure_setpoint = 130;
    PID outer = PID (30, 30.0e-6, 2.5, pressure_setpoint, true);
    outer.update(LPpsi);

    control.runMotor();

    if (t2 - control.lastPrint > 1000) {
        #ifndef USE_DASHBOARD
        Serial.println( String(t2) + "\t"+ String(angle_setpoint) + "\t"+ String(pressure_setpoint) +"\t" + String(speed) + "\t" + String(motorAngle) + "\t" + String(HPpsi) + "\t" + String(LPpsi) + "\t" + String(InjectorPT) + "\t" + String(p_buff->get_slope()) + "\t" + String(pressure_errorInt) );     
        #else
        Comms::Packet packet = {.id = 1};
        // Comms::packetAddFloat(&packet, sin(t2/1e6));
        Comms::packetAddFloat(&packet, float(control.angle_setpoint));
        Comms::packetAddFloat(&packet, pressure_setpoint); //not sure how to send, possibly store these variables in class when calling updatePressure method
        Comms::packetAddFloat(&packet, float(control.speed));
        Comms::packetAddFloat(&packet, control.motorAngle);
        Comms::packetAddFloat(&packet, control.HPpsi);
        Comms::packetAddFloat(&packet, control.LPpsi);
        Comms::packetAddFloat(&packet, control.InjectorPT);
        Comms::packetAddFloat(&packet, control.p_buff->get_slope());
        Comms::packetAddFloat(&packet, pressure_errorInt); //not sure how to send, possibly store these variables in class when calling updatePressure method
        Comms::emitPacket(&packet);
        #endif
        lastPrint = micros();
    }
    
    while (Serial.available() > 0) {
        //Read incoming commands
        int inChar = Serial.read();
        if (inChar == '\n') {
            int new_setpt = inString.toInt(); //use class version?
            if (new_setpt >= 0 && new_setpt <= PRESSURE_MAXIMUM) {
                pressure_setpoint=inString.toInt();
            }
            inString = "";
        } else {
            inString += (char)inChar;
        }
        
    }

    // if ((micros()-start_time) > 30e6) {
    //     pressure_setpoint = 0;
    // }

}







