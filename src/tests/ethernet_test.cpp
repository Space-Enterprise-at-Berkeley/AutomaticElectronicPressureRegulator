#include <Arduino.h>
#include <SPI.h>
#include <EthernetUdp.h>
#include <Ethernet.h>

int count;
EthernetUDP Udp;
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 21 };
IPAddress groundStation1(10, 0, 0, 69);
IPAddress ip(10, 0, 0, 21);
int port = 42069;


void setup() {
    Serial.begin(115200);
    Ethernet.init(13);
    Ethernet.begin((uint8_t *)mac, ip);
    Udp.begin(port);
}

void loop() {
    Udp.beginPacket(groundStation1, port);
    Udp.write(0xFF);
    Udp.endPacket();

    Serial.println(count);
    count += 1;
}