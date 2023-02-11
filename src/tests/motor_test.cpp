#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Ethernet.h>


uint8_t motor1 = 4;
uint8_t motor2 = 2;


void setup(void) 
{
    Serial.begin(115200); 

    pinMode(motor1, OUTPUT);
    pinMode(motor2, OUTPUT);

    ledcSetup(0, 5000, 8);
    ledcSetup(1, 5000, 8);

    ledcAttachPin(motor1, 0);
    ledcAttachPin(motor2, 1);

}

void loop(void) 
{
    ledcWrite(0, -255);
    ledcWrite(1, 255);
    delay(2000);
    ledcWrite(0, 0);
    ledcWrite(1, 0); 
    delay(2000);

}