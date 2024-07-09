#include <Arduino.h>
#include <LoRa.h>
#include <Wire.h>

#define SCK  5   // GPIO5  -- SX1278's SCK
#define MISO 19  // GPIO19 -- SX1278's MISnO
#define MOSI 27  // GPIO27 -- SX1278's MOSI
#define SS   18  // GPIO18 -- SX1278's CS
#define RST  14  // GPIO14 -- SX1278's RESET
#define DI0  26  // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND 868E6

unsigned int counter = 0;

void setup() {
    Serial.begin(9600);
    while (!Serial)
        ;

    SPI.begin(SCK, MISO, MOSI, SS);
    LoRa.setPins(SS, RST, DI0);
    if (!LoRa.begin(BAND)) {
        Serial.println("Starting LoRa failed!");
        while (1)
            ;
    }
    Serial.println("Succesfully Initialized");
}

// the loop function runs over and over again forever
void loop() {
    LoRa.beginPacket();
    String packet = "hello " + String(counter);
    Serial.println("Sending packet: " + packet);
    LoRa.print(packet);
    LoRa.endPacket();
    counter++;
    delay(3000);
}
