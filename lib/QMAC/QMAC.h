#pragma once

#include <Arduino.h>
#include <Debug.h>
#include <KickSort.h>
#include <LoRa.h>
#include <esp_timer.h>

#include <List.hpp>

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
    bool begin(int64_t wakeUpInterval = 60000, int64_t activeDuration = 5000,
               byte localAddress = 0xFF);
    bool push(String payload, byte destination = 0xFF);
    void run();
    int amountAvailable();
    bool receive(int packetSize);
    List<Packet> receptionQueue;
    List<Packet> sendQueue;
    bool isActivePeriod();

   private:
    byte msgCount;
    int64_t lastTimeActive;
    int64_t wakeUpInterval;
    int64_t activeDuration;
    esp_timer_handle_t timer_handle;
    List<Packet> unackedQueue;
    bool sendAck(Packet p);
    bool sendPacket(Packet p);
};
extern QMACClass QMAC;