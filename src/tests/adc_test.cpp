#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include <Wire.h>
#include <SPI.h>
#include <Ethernet.h>

Adafruit_ADS1115 adc;
int count;

void setup() {
    // Serial.begin(115200);
    // Wire.begin(5, 32);
    // Wire.setClock(400000);

    // adc.begin(0x48, &Wire);
    // adc.setDataRate(RATE_ADS1115_860SPS);
    // adc.startADCReading(0, true);

    pinMode(2, OUTPUT);
    pinMode(4, OUTPUT);
}

void loop() {

    digitalWrite(2, HIGH);
    digitalWrite(4, LOW);
    // adc.startADCReading(0, true);
    // Serial.print(count);
    // Serial.print(" : ");
    // Serial.print(adc.getLastConversionResults());
    // adc.startADCReading(1, true);
    // Serial.print(" : ");
    // Serial.println(adc.getLastConversionResults());
    // count ++;
}