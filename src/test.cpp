#include <test.h>

namespace tests {

    PID test = PID(11.5, 1.5e-6, 0.21e6, 0, 100);
    //Encoder encoder(ENC1, ENC2);

    void motorDirTest() {
        // check encoder and motor directions match (essential for PID loop)
        // run motors in positive direction for 2 seconds (encoder count should increase)
        // run motors in negative direction for 2 seconds (encoder count should decrease)
        long startTime = millis(); 
        long theta0 = enc1.read();

        Serial.println("Starting motor/encoder direction test...");

        test.speed = 250;
        test.runMotor(); //test for new class
        while (millis() - startTime < 1000) {}
        long theta1 = enc1.read();
        String msg = ((theta1-theta0) > 0) ? "\tPASS":"\tFAIL";
        Serial.println("Running motors + direction. t0, t1" + String(theta0) + "\t" +  String(theta1) + msg);
        startTime = millis();
        test.speed = -test.speed;
        test.runMotor();
        while (millis() - startTime < 1000) {}
        long theta2 = enc1.read();
        msg = ((theta2-theta1) < 0) ? "\tPASS":"\tFAIL";
        Serial.println("Running motors + direction. t1, t2: " + String(theta1) + "\t" + String(theta2) + msg);
        test.speed = 0;
        test.runMotor();
    }

//if anything breaks it might be because I stored 
//the common values of speed, motorAngle etc in common class variables

    void motorPowerTest() {
        // String inString = "";
        // int speed = 0;
        // unsigned long lastPrint = 0;
        //long angle = 0;

        while (true){
            test.runMotor();

            if (millis()-test.lastPrint > 200){
                test.angle = enc1.read();
                Serial.println(String(test.speed)+"\t"+String(test.angle));
                test.lastPrint = millis();
            }

            while (Serial.available() > 0) {
                //Read incoming commands
                int inChar = Serial.read();
                
                if (inChar == '\n') {
                    if (test.inString == "fin"){
                        return;
                    }else{
                        test.speed = test.inString.toInt();
                    }
                    test.inString = "";
                } else {
                    test.inString += (char)inChar;
                }
                
            }

        }
    }

    void ptTest() {
        // print PT reading 6 times, at 0.5s intervals
        #ifndef USE_DASHBOARD
        Serial.println("Starting PT test...");
        #endif
        test.lastPrint = 0;
        int count = 0;
        Buffer p_buff(BUFF_SIZE);
        long t;
        while (true) {
            count++;
            test.updatePT();
            t = micros();
            if (millis() - test.lastPrint > 100) {
                
                test.lastPrint = millis();
                count = 0;

                #ifndef USE_DASHBOARD
                Serial.println(String(count) + "\t Injector: \t" + String(voltageToPressure(analogRead(INJECTOR_PT))) + "\t HP: \t" + String(voltageToPressure(analogRead(HP_PT))) + "\t LP: \t" + String(p) + "\t" + String((p-old_p)/((t-old_t)/1e6)) + "\t" + String(p_buff.get_slope()));
                #else
                Comms::Packet packet = {.id = 1};
                Comms::packetAddFloat(&packet, sin(t/1e6));
                // Comms::packetAddFloat(&packet, 0.0);
                Comms::packetAddFloat(&packet, 0.0);
                Comms::packetAddFloat(&packet, float(test.speed));
                Comms::packetAddFloat(&packet, test.motorAngle);
                Comms::packetAddFloat(&packet, test.voltageToHighPressure(analogRead(HP_PT)));
                Comms::packetAddFloat(&packet, test.voltageToHighPressure(analogRead(LP_PT)));
                Comms::packetAddFloat(&packet, test.voltageToHighPressure(analogRead(INJECTOR_PT)));
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
                    if (test.inString == "start") {
                        return;
                    }
                    test.inString = "";
                }
                else{
                    test.inString += (char)inChar;
                }
            }
        }
    }

    void potTest() {
        Serial.println("Starting potentiometer test...");
        Serial.print("Set pot to 0.");
        while (test.readPot() > 1){}
        Serial.println("Finished \t" + String(test.readPot()));
        Serial.print("Set pot to 90.");
        while (test.readPot() < 89){
            Serial.print(" " + String(test.readPot()));
            delay(250);
        }
        Serial.println("Finished \t" + String(test.readPot()));
    }

    void servoTest() {
    Serial.println("Starting servo test...");
    test.speed = 0;
    long angle;
    bool isAngleUpdate;
    long oldPosition=-999;
    long e=0;
    long oldError=0;

    long setPoint=100;
    // float kp=11.5;
    // float ki=1.5e-6;
    // float kd=0.1665e6;

    float kp=11.5;
    float ki=1.5e-6;
    float kd=0.21e6;

    long errorInt=0;
    unsigned long t2;
    unsigned long dt;
    bool isPrint = true;
    unsigned long lastPrint = 0;

    String inString="";

    while (true) {
        dt=micros()-t2;
        t2+=dt;
        angle = enc1.read();
        isAngleUpdate=(angle!=oldPosition);
        e=angle-setPoint;
        //PI control
        float rawSpd=-(kp*e+kd*(e-oldError)/float(dt));
        if(rawSpd<MAX_SPD && rawSpd>MIN_SPD){ //anti-windup
            errorInt+=e*dt;
            rawSpd-=ki*errorInt;
        }
        else{errorInt=0;}
        rawSpd += ((rawSpd<0) ? -STATIC_SPD : STATIC_SPD);
        test.speed=min(max(MIN_SPD,rawSpd),MAX_SPD);
        test.runMotor();
        if (isPrint && (millis()-lastPrint > 200)){
            Serial.println(String(test.speed)+"\t"+String(angle)+"\t"+String(setPoint) + "\t" + String(test.voltageToHighPressure(analogRead(HP_PT))) + "\t" + String(test.voltageToHighPressure(analogRead(LP_PT))));
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
                    setPoint=inString.toInt(); //Could not abstract away using class because of this setPoint value
                }
                inString = "";
            } else {
                inString += (char)inChar;
            }
            
        }
        if (isAngleUpdate) {
            oldPosition = angle;
            
        }
        oldError=e;
    }

}

    void servoCharacterization() {
    Serial.println("Starting servo-based characterization...");
    test.speed = 0;
    long angle;
    bool isAngleUpdate;
    long oldPosition=-999;
    long e=0;
    long oldError=0;

    long setPoint=0;
    // float kp=11.5;
    // float ki=1.5e-6;
    // float kd=0.1665e6;

    float kp=11.5;
    float ki=1.5e-6;
    float kd=0.21e6;

    long errorInt=0;
    unsigned long t2;
    unsigned long dt;
    bool isPrint = true;
    unsigned long lastPrint = 0;

    unsigned long flowStart = millis(); // in millis
    unsigned long flowDuration = 2500;

    unsigned int printFreq = 50; // in millis

    String inString="";

    while (true) {
        dt=micros()-t2;
        t2+=dt;
        angle = enc1.read();
        isAngleUpdate=(angle!=oldPosition);
        e=angle-setPoint;
        //PI control
        float rawSpd=-(kp*e+kd*(e-oldError)/float(dt));
        if(rawSpd<MAX_SPD && rawSpd>MIN_SPD){ //anti-windup
            errorInt+=e*dt;
            rawSpd-=ki*errorInt;
        }
        else{errorInt=0;}
        rawSpd += ((rawSpd<0) ? -STATIC_SPD : STATIC_SPD);
        test.speed=min(max(MIN_SPD,rawSpd),MAX_SPD);
        test.runMotor();
        if (isPrint && (millis()-lastPrint > printFreq)){
            Serial.println(String(millis()) + "\t" + String(test.speed)+"\t"+String(angle)+"\t"+String(setPoint) + "\t" + String(test.voltageToHighPressure(analogRead(HP_PT))) + "\t" + String(test.voltageToHighPressure(analogRead(LP_PT))));
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
        if (isAngleUpdate) {
            oldPosition = angle;
            
        }
        oldError=e;
    }
}

};