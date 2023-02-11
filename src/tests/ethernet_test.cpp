#include <Arduino.h>
#include <SPI.h>
#include <EthernetUdp.h>
#include <Ethernet.h>

int count;
EthernetUDP Udp;
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 21 };
IPAddress groundStation1(10, 0, 0, 69);
IPAddress groundStation2(10, 0, 0, 70);
IPAddress ip(10, 0, 0, 21);
int port = 42069;


void setup() {
    Serial.begin(115200);
    Ethernet.init(13);
    Ethernet.begin((uint8_t *)mac, ip);
    Ethernet.setRetransmissionCount(0);
    Ethernet.setRetransmissionTimeout(0);
    Udp.begin(port);
}

void loop() {
    Serial.println("before begin1");
    Serial.flush();
    Udp.beginPacket(groundStation1, port);
    Serial.println("after begin1");
    Serial.flush();
    Udp.write("A");
    Udp.endPacket();

    Serial.println("before begin2");
    Serial.flush();
    Udp.beginPacket(groundStation2, port);
    Serial.println("after begin2");
    Serial.flush();
    Udp.write("B");
    Serial.println("beofore endpacket2");
    Serial.flush();
    Udp.endPacket();
    Serial.println("after endpacket2");
    Serial.flush();

    Serial.println(count);
    count += 1;
}