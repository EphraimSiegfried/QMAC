#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>                       
#include <QMAC.h>

#define SCK  5   // GPIO5  -- SX1278's SCK
#define MISO 19  // GPIO19 -- SX1278's MISnO
#define MOSI 27  // GPIO27 -- SX1278's MOSI
#define SS   18  // GPIO18 -- SX1278's CS
#define RST  14  // GPIO14 -- SX1278's RESET
#define DI0  26  // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND 868E6

#define GPS_RX_PIN 34
#define GPS_TX_PIN 12

TinyGPSPlus gps;                            
HardwareSerial GPSSerial1(1);                 

void setup() {
  // Setup Serial
  Serial.begin(115200);
  while (!Serial);

  // Setup LoRa
  LoRa.setPins(SS, RST, RST);// set CS, reset, IRQ pin
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  // Setup GPS
  GPSSerial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);   //17-TX 18-RX
  delay(1500);

  Serial.println("Setup done");

  QMAC.send("HelloWorld");
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }

    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }
}