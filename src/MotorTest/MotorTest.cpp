#include <Arduino.h>
#include <Encoder.h>

#define MOTOR1 6
#define MOTOR2 5
#define ENC1 2
#define ENC2 3

#define MAX_SPD 255
#define MIN_SPD -255

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(ENC1, ENC2);
//   avoid using pins with LEDs attached

// Note: 1 rev on main shaft is 3200 counts
// Encoder itself is 64CPM (including all edges)

int spd=0;
String inString="";

long angle;
bool isAngleUpdate;
long oldPosition=-999;
long e=0;
long oldError=0;

long setPoint=100;
float kp=11.5;
float ki=1.5e-6;
float kd=0.1665e6;

long errorInt=0;
unsigned long t2;
unsigned long dt;
unsigned long lastPrint=0;

void vroom(){
  analogWrite(MOTOR1,-min(0,spd));
  analogWrite(MOTOR2,max(0,spd));
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  analogWrite(MOTOR1,0);
  analogWrite(MOTOR2,0);
  t2=micros();
  e=angle-setPoint;
  oldError=e;
}



void loop() {
  dt=micros()-t2;
  t2+=dt;
  angle = myEnc.read();
  isAngleUpdate=(angle!=oldPosition);
  e=angle-setPoint;
  //PI control
  float rawSpd=-(kp*e+kd*(e-oldError)/float(dt));
  if(rawSpd<MAX_SPD && rawSpd>MIN_SPD){ //anti-windup
    errorInt+=e*dt;
    rawSpd-=ki*errorInt;
  }
  else{errorInt=0;}
  
  spd=min(max(MIN_SPD,rawSpd),MAX_SPD);
  vroom();
  Serial.println(String(spd)+"\t"+String(angle)+"\t"+String(setPoint));

//   if(t2-lastPrint>0.5e6){
// //    Serial.println(String(t2/1000)+"\t"+String(e)+"\t"+String(rawSpd));
//     setPoint=random(0,10001);
//     lastPrint=t2;
//   }
  
  while (Serial.available() > 0) {
    //Read incoming commands
    int inChar = Serial.read();
    if (isDigit(inChar)|| inChar=='-') {
      inString += (char)inChar;
    }
    if (inChar == '\n') {
        setPoint=inString.toInt();
      
//      Serial.print("Motor Speeds: ");
//      spd=min(max(-255,inString.toInt()),255);
//      analogWrite(MOTOR1,-min(0,spd));
//      analogWrite(MOTOR2,max(0,spd));
//      Serial.println(spd);
//      Serial.print(-min(0,spd));Serial.print('\t');Serial.println(max(0,spd));
      inString = "";
    }
  }

  if (isAngleUpdate) {
    oldPosition = angle;
    // Serial.println(String(millis())+"\t"+String(angle));
//    if(angle%2000==0){
//      Serial.println(String(millis())+"\t"+String(angle/2000));
//    }   
  }
  oldError=e;
}

