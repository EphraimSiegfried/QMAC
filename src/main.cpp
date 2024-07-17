#include <Arduino.h>
#include <Debug.h>
#include <LoRa.h>
#include <QMAC.h>
#include <SPI.h>
#include <TinyGPS++.h>

#include <List.hpp>

#include "esp_timer.h"

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
bool lastState;

String getTime();

void setup() {
    // Setup Serial
    Serial.begin(115200);
    while (!Serial);

    // Setup LoRa
    SPI.begin(SCK, MISO, MOSI, SS);
    LoRa.setPins(SS, RST, DI0);  // set CS, reset, IRQ pin
    if (!LoRa.begin(BAND)) {
        LOG("Starting LoRa failed!");
        while (1);
    }

    // Setup GPS
    GPSSerial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);  // 17-TX 18-RX
    delay(1500);

    QMAC.begin(5000, 5000);
    LOG("Setup done");
}

void loop() {
    if (Serial.available()) {
        String msg = Serial.readStringUntil('\n');
        QMAC.push(msg);
    }

    bool currentState = QMAC.active;

    // Only log when the state changes
    if (currentState != lastState) {
        Serial.println();
        String state = currentState ? "active" : "sleeping";
        LOG(state + " time: " + String(millis()));
        lastState = currentState;
    }
    QMAC.run();

    for (size_t i = 0; i < QMAC.receptionQueue.getSize(); i++) {
        Packet p = QMAC.receptionQueue[i];
        QMAC.receptionQueue.remove(i);
        LOG("Received packet:");
        for (size_t i = 0; i < p.payloadLength; i++) {
            Serial.print((char)p.payload[i]);
        }
        Serial.println();
    }
}

String getTime() {
    while (GPSSerial1.available()) gps.encode(GPSSerial1.read());
    return String(gps.time.minute()) + ":" + String(gps.time.second());
}