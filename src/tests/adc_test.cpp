#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include <ADS1X15.h>
#include <Wire.h>
#include <SPI.h>
#include <Ethernet.h>

// Adafruit_ADS1115 adc;

// Adafruit_ADS1115 ads;

ADS1115 ads;

int16_t adc0;
int16_t adc1;
int16_t adc2;
int16_t adc3;
float Voltage0 = 0.0;
float Voltage1 = 0.0;
float Voltage2 = 0.0;
float Voltage3 = 0.0;
float pressure0 = 0.0;
float pressure1 = 0.0;
float pressure2 = 0.0;
float pressure3 = 0.0;

TaskHandle_t Task1;

void readADC(void * parameter) {
  for(;;) {
    adc0 = ads.readADC(0);
    adc1 = ads.readADC(1);
    adc0 = ads.readADC(2);
    adc0 = ads.readADC(3);
  }
}

void setup(void) 
{
  Wire.begin(5, 32);
  Serial.begin(115200); 
  Serial.println("please work");
  ads.begin();
  ads.setDataRate(7);
  Serial.println("worked");

  xTaskCreatePinnedToCore(
    readADC,
    "Task 1",
    10000,
    NULL,
    0,
    &Task1,
    0
  );

}

long lastMicros = micros();

void loop(void) 
{
// adc0 = ads.readADC(0);
// adc1 = ads.readADC(1);
// adc0 = ads.readADC(2);
// adc0 = ads.readADC(3);

Voltage0 = (adc0 * 0.1875)/1000;
Voltage1 = (adc1 * 0.1875)/1000;
Voltage2 = (adc2 * 0.1875)/1000;
Voltage3 = (adc3 * 0.1875)/1000;

Serial.print(Voltage0);
Serial.print("  ");
Serial.print(Voltage1);
Serial.print("  ");
Serial.print(Voltage2);
Serial.print("  ");
Serial.print(Voltage3);
Serial.println("  ");

}