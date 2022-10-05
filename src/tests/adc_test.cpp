#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include <Wire.h>
#include <SPI.h>
#include <Ethernet.h>

Adafruit_ADS1115 adc;
int count;

Adafruit_ADS1115 ads;
float Voltage0 = 0.0;
float Voltage1 = 0.0;
float Voltage2 = 0.0;
float Voltage3 = 0.0;
float pressure0 = 0.0;
float pressure1 = 0.0;
float pressure2 = 0.0;
float pressure3 = 0.0;

void setup(void) 
{
  Wire.begin(5, 32, 40000000);
  Serial.begin(115200); 
  Serial.println("please work");
  ads.begin();
  ads.setDataRate(RATE_ADS1115_860SPS);
  Serial.println("worked");
}

void loop(void) 
{
int16_t adc0;
int16_t adc1;
int16_t adc2;
int16_t adc3;



adc0 = ads.readADC_SingleEnded(0);
adc1 = ads.readADC_SingleEnded(1);
adc2 = ads.readADC_SingleEnded(2);
adc3 = ads.readADC_SingleEnded(3);
Voltage0 = (adc0 * 0.1875)/1000;
Voltage1 = (adc1 * 0.1875)/1000;
Voltage2 = (adc2 * 0.1875)/1000;
Voltage3 = (adc3 * 0.1875)/1000;

pressure0 = 222.22 * Voltage0 - 111.1;
pressure1 = 222.22 * Voltage1 - 111.1;
pressure2 = (1111.1 * Voltage2) + 15;


Serial.print(pressure1);
Serial.print(",");
Serial.println(pressure2);
Serial.print(Voltage0);
Serial.print(",");
Serial.print(Voltage1);
Serial.print(",");
Serial.print(Voltage2);
Serial.print(",");
Serial.println(Voltage3);


}

// Adafruit_ADS1115 adc;
// int count;

// void setup() {
//     Serial.begin(115200);
//     Wire.begin(5, 32);
//     Wire.setClock(400000);

//     adc.begin(0x48, &Wire);
//     adc.setDataRate(RATE_ADS1115_860SPS);
//     adc.startADCReading(0, true);

    pinMode(2, OUTPUT);
    pinMode(4, OUTPUT);
}
   
void loop() {
// }

// void loop() {


    digitalWrite(2, HIGH);
    digitalWrite(4, LOW);
//     adc.startADCReading(0, true);
//     Serial.print(count);
//     Serial.print(" : ");
//     Serial.print(adc.getLastConversionResults());
//     adc.startADCReading(1, true);
//     Serial.print(" : ");
//     Serial.println(adc.getLastConversionResults());
//     count ++;
}
//     delay(500);
// }