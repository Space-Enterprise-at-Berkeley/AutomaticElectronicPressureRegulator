#include <test.h>

namespace tests {
    Encoder encoder(ENC1, ENC2);
    int speed = 0;

    void motorDirTest() {
        // check encoder and motor directions match (essential for PID loop)
        // run motors in positive direction for 2 seconds (encoder count should increase)
        // run motors in negative direction for 2 seconds (encoder count should decrease)
        long startTime = millis(); 
        long theta0 = encoder.read();

        Serial.println("Starting motor/encoder direction test...");

        speed = 250;
        utility::runMotor(speed); //test for new class
        while (millis() - startTime < 1000) {}
        long theta1 = encoder.read();
        String msg = ((theta1-theta0) > 0) ? "\tPASS":"\tFAIL";
        Serial.println("Running motors + direction. t0, t1" + String(theta0) + "\t" +  String(theta1) + msg);
        startTime = millis();
        speed = -speed;
        utility::runMotor(speed);
        while (millis() - startTime < 1000) {}
        long theta2 = encoder.read();
        msg = ((theta2-theta1) < 0) ? "\tPASS":"\tFAIL";
        Serial.println("Running motors + direction. t1, t2: " + String(theta1) + "\t" + String(theta2) + msg);
        speed = 0;
        utility::runMotor(speed);
    }

//if anything breaks it might be because I stored 
//the common values of speed, motorAngle etc in common class variables

    void motorPowerTest() {
        String inString = "";
        // int speed = 0;
        unsigned long lastPrint = 0;
        long angle = 0;

        while (true){
            utility::runMotor(speed);

            if (millis() - lastPrint > 200){
                angle = encoder.read();
                Serial.println(String(speed)+"\t"+String(angle));
                lastPrint = millis();
            }

            while (Serial.available() > 0) {
                //Read incoming commands
                int inChar = Serial.read();
                
                if (inChar == '\n') {
                    if (inString == "fin"){
                        return;
                    }else{
                        speed = inString.toInt();
                    }
                    inString = "";
                } else {
                    inString += (char)inChar;
                }
                
            }

        }
    }

    void ptTest() {
        unsigned long lastPrint = 0;
        String inString = "";
        // print PT reading 6 times, at 0.5s intervals
        #ifndef USE_DASHBOARD
        Serial.println("Starting PT test...");
        #endif
        long p;
        long old_p;
        int count = 0;
        Buffer p_buff(BUFF_SIZE);
        long t;
        while (true) {
            count++;
            double pressure = utility::voltageToPressure(analogRead(LP_PT));
            t = micros();
            p_buff.insert(double(t)/1.0e6, pressure);
            double old_pressure = pressure;
            long old_t = t;
            double motorAngle;
            
            if (millis() - lastPrint > 100) {
                
                lastPrint = millis();
                count = 0;

                #ifndef USE_DASHBOARD
                Serial.println(String(count) + "\t Injector: \t" + String(utility::voltageToPressure(analogRead(INJECTOR_PT))) + "\t HP: \t" + String(utility::voltageToPressure(analogRead(HP_PT))) + "\t LP: \t" + String(p) + "\t" + String((p-old_p)/((t-old_t)/1e6)) + "\t" + String(p_buff.get_slope()));
                #else
                Comms::Packet packet = {.id = 1};
                Comms::packetAddFloat(&packet, sin(t/1e6));
                // Comms::packetAddFloat(&packet, 0.0);
                Comms::packetAddFloat(&packet, 0.0);
                Comms::packetAddFloat(&packet, float(speed));
                Comms::packetAddFloat(&packet, motorAngle);
                Comms::packetAddFloat(&packet, utility::voltageToHighPressure(analogRead(HP_PT)));
                Comms::packetAddFloat(&packet, utility::voltageToHighPressure(analogRead(LP_PT)));
                Comms::packetAddFloat(&packet, utility::voltageToHighPressure(analogRead(INJECTOR_PT)));
                Comms::packetAddFloat(&packet, p_buff.get_slope());
                Comms::packetAddFloat(&packet, 0);
                Comms::emitPacket(&packet);
                #endif
                
            }
            while (Serial.available() > 0) {
                //Read incoming commands
                int inChar = Serial.read();
                if (inChar == '\n') {
                    // Serial.println("Received a thing: " + inString);
                    delay(5);
                    if (inString == "start") {
                        return;
                    }
                    inString = "";
                }
                else{
                    inString += (char)inChar;
                }
            }
        }
    }

    void potTest() {
        Serial.println("Starting potentiometer test...");
        Serial.print("Set pot to 0.");
        while (utility::readPot() > 1){}
        Serial.println("Finished \t" + String(utility::readPot()));
        Serial.print("Set pot to 90.");
        while (utility::readPot() < 89){
            Serial.print(" " + String(utility::readPot()));
            delay(250);
        }
        Serial.println("Finished \t" + String(utility::readPot()));
    }

    void servoTest() {
    Serial.println("Starting servo test...");
    PID test = PID(11.5, 1.5e-6, 0.21e6, 100, false);
    long setPoint=100;
    speed = 0;
    // float kp=11.5;
    // float ki=1.5e-6;
    // float kd=0.1665e6;
    bool isPrint = true;
    unsigned long lastPrint = 0;
    long angle;

    String inString="";

    while (true) {
        speed = test.update(encoder.read());
        utility::runMotor(speed);
        if (isPrint && (millis()-lastPrint > 200)){
            Serial.println(String(speed)+"\t"+String(angle)+"\t"+String(setPoint) + "\t" + String(utility::voltageToHighPressure(analogRead(HP_PT))) + "\t" + String(utility::voltageToHighPressure(analogRead(LP_PT))));
            lastPrint = millis();
        }
        
        while (Serial.available() > 0) {
            //Read incoming commands
            int inChar = Serial.read();
            
            if (inChar == '\n') {
                if (inString == "fin"){
                    return;
                } else if (inString == "quiet"){
                    isPrint = false;
                } else if (inString == "loud"){
                    isPrint = true;
                } else{
                    setPoint=inString.toInt();
                }
                inString = "";
            } else {
                inString += (char)inChar;
            }
            
        }
    }

}

    void servoCharacterization() {
    Serial.println("Starting servo-based characterization...");
    PID test = PID(11.5, 1.5e-6, 0.21e6, 0, false);
    speed = 0;
    //long setPoint=0;
    // float kp=11.5;
    // float ki=1.5e-6;
    // float kd=0.1665e6;
    bool isPrint = true;
    unsigned long lastPrint = 0;
    long setPoint=100;
    long angle;

    unsigned long flowStart = millis(); // in millis
    unsigned long flowDuration = 2500;

    unsigned int printFreq = 50; // in millis

    String inString="";

    while (true) {
        speed = test.update(encoder.read());
        utility::runMotor(speed);
        if (isPrint && (millis()-lastPrint > printFreq)){
            Serial.println(String(millis()) + "\t" + String(speed)+"\t"+String(angle)+"\t"+String(setPoint) + "\t" + String(utility::voltageToHighPressure(analogRead(HP_PT))) + "\t" + String(utility::voltageToHighPressure(analogRead(LP_PT))));
            lastPrint = millis();
        }

        if (millis()-flowStart > flowDuration) {
            setPoint = 0;
            printFreq = 5000;
        }
        
        while (Serial.available() > 0) {
            //Read incoming commands
            int inChar = Serial.read();
            
            if (inChar == '\n') {
                if (inString == "fin"){
                    return;
                } else if (inString == "quiet"){
                    isPrint = false;
                } else if (inString == "loud"){
                    isPrint = true;
                } else{
                    // Start a flow
                    isPrint = true;
                    setPoint=inString.toInt(); //Unable to Abstract because of this setPoint Value
                    flowStart = millis();
                    printFreq = 50;
                }
                inString = "";
            } else {
                inString += (char)inChar;
            }
            
        }
    }
    }


    void angleSweep(long startAngle, long endAngle, unsigned long flowDuration, long extraTime) {
        PID test = PID(11.5, 1.5e-6, 0.21e6, 130, false);
        long setPoint = 130;
        unsigned long lastPrint = 0;
        long angle;
        unsigned long flowStart = millis(); // in millis
        bool isPrint = true;
        unsigned int printFreq = 20;
        while (true) {
            speed = test.update(encoder.read());
            utility::runMotor(speed);
            if (isPrint && (millis()-lastPrint > printFreq)){ 
                #ifndef USE_DASHBOARD
                Serial.println(String(millis()) + "\t" + String(speed)+"\t"+String(angle)+"\t"+String(setPoint) + "\t" + String(utility::voltageToHighPressure(analogRead(HP_PT))) + "\t" + String(utility::voltageToPressure(analogRead(LP_PT))));
                #else
                Comms::Packet packet = {.id = 1};
                // Comms::packetAddFloat(&packet, sin(t2/1e6));
                Comms::packetAddFloat(&packet, float(setPoint));
                Comms::packetAddFloat(&packet, 0);
                Comms::packetAddFloat(&packet, float(speed));
                Comms::packetAddFloat(&packet, angle);
                Comms::packetAddFloat(&packet, utility::voltageToHighPressure(analogRead(HP_PT)));
                Comms::packetAddFloat(&packet, utility::voltageToPressure(analogRead(LP_PT)));
                Comms::packetAddFloat(&packet, 0);
                Comms::packetAddFloat(&packet, 0);
                Comms::packetAddFloat(&packet, 0);
                Comms::emitPacket(&packet);
                #endif
            
                lastPrint = millis();
            }

            if (millis()-flowStart > flowDuration) { // flow has ended, close valves
                setPoint = 0;
                printFreq = 5000;
            } else {
                float prog = float(millis()-flowStart)/float(flowDuration);
                setPoint = prog * endAngle + (1-prog) * startAngle;
            }

            if (millis()-flowStart > flowDuration + extraTime) { // flow has ended, close valves
                return;
            }
        }
    }

    boolean pressurize_tank() {
        long dt;
        long t2 = micros();
        long start_time = micros();
        long lastPrint = 0;
        double pressure_e = 0;
        double pressure_e_old = 0;
        double pressure_errorInt = 0;
        Buffer* p_buff_2; //use for pressureize_tank
        p_buff_2 = new Buffer(BUFF_SIZE);
        unsigned long endFlow = 0;

        long angle;
        double motorAngle;
        double potAngle;
        double HPpsi;
        double LPpsi;
        double InjectorPT;

        while (true) {
            angle = encoder.read();
            motorAngle = utility::encoderToAngle(angle);
            potAngle = utility::readPot();
            HPpsi = utility::voltageToHighPressure(analogRead(HP_PT));
            LPpsi = utility::voltageToPressure(analogRead(LP_PT));
            InjectorPT = utility::voltageToPressure(analogRead(INJECTOR_PT));

            // LPpsi = analogRead(POTPIN)/1024.0*360

            long angle_setpoint = 0;
            //Compute Outer Pressure Control Loop
            long pressure_setpoint = 130;
            PID outer = PID (1.0, 5.0e-6, 0.0, pressure_setpoint, true);
            angle_setpoint = outer.update(LPpsi);


            //Compute Inner PID Servo loop
            PID inner = PID (11.5, 25e-6, 0.1665e6, angle_setpoint, false);
            inner.set_setPoint(angle_setpoint);
            speed = inner.update(angle);
        


            if (endFlow > 0) {
                angle_setpoint = 0;
            }

            utility::runMotor(speed);

            if (t2 - lastPrint > 50) {
                #ifndef USE_DASHBOARD
                //used p_buff_2 instead of p_buff as p_buff is not accessible
                Serial.println( String(t2) + "\t"+ String(angle_setpoint) + "\t"+ String(pressure_setpoint) +"\t" + String(speed) + "\t" + String(motorAngle) + "\t" + String(HPpsi) + "\t" + String(LPpsi) + "\t" + String(InjectorPT) + "\t" + String(p_buff_2->get_slope()) + "\t" + String(pressure_errorInt) );     
                #else
                Comms::Packet packet = {.id = 1};
                // Comms::packetAddFloat(&packet, sin(t2/1e6));
                Comms::packetAddFloat(&packet, float(angle_setpoint));
                Comms::packetAddFloat(&packet, pressure_setpoint);
                Comms::packetAddFloat(&packet, float(speed));
                Comms::packetAddFloat(&packet, motorAngle);
                Comms::packetAddFloat(&packet, HPpsi);
                Comms::packetAddFloat(&packet, LPpsi);
                Comms::packetAddFloat(&packet, InjectorPT);
                Comms::packetAddFloat(&packet, p_buff_2->get_slope());
                Comms::packetAddFloat(&packet, pressure_errorInt);
                Comms::emitPacket(&packet);
                #endif
                lastPrint = micros();
            }

            if (LPpsi > pressure_setpoint*0.95 && endFlow <=0) {
                endFlow = millis();
            }

            if (endFlow > 0 && (millis() > (endFlow + 3000))) {
            // finish flow
                speed = 0;
                utility::runMotor(speed);
                return true;
            }

        }
    }

};