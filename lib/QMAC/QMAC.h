#pragma once

#include <Arduino.h>
#include <Debug.h>
#include <LoRa.h>
#include <cppQueue.h>
#include <esp_timer.h>

typedef struct QMACPacket {
    byte destination;
    byte localAddress;
    byte msgCount;
    byte payloadLength;
    byte payload[5];
} Packet;

class QMACClass {
   public:
    byte localAddress;
    // 60 seconds sleep time, 5 seconds active time
    bool begin(int64_t wakeUpInterval = 60000,
               int64_t activeDuration = 5000, byte localAddress = 0xFF);
    bool send(String payload, byte destination = 0xFF);
    bool push(String payload, byte destination = 0xFF);
    void run();
    static void onReceiveWrapper(int packetSize);
    void receive(int packetSize);
    cppQueue receptionQueue{sizeof(Packet)};
    cppQueue sendQueue{sizeof(String)};
    volatile bool isActivePeriod;

   private:
    byte msgCount;
    int64_t lastTimeActive;
    int64_t wakeUpInterval;
    int64_t activeDuration;
    esp_timer_handle_t timer_handle;
    bool sendAck(byte destination, byte msgCount);
    static void timerCallback(void* arg);
};
extern QMACClass QMAC;