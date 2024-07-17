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
    uint16_t nextWakeUpTime;
    byte payloadLength;
    byte payload[5];

    bool isAck() const { return payloadLength == 0; }
    bool isSyncPacket() const { return msgCount == 0; }

    String toString() const {
        String result = "destination: 0x" + String(destination, HEX) + "\n";
        result += "localAddress: 0x" + String(localAddress, HEX) + "\n";
        result += "msgCount: 0x" + String(msgCount, HEX) + "\n";

        if (isSyncPacket()) {
            result += "nextWakeUpTime: " + String(nextWakeUpTime) + "\n";
            return result;
        }
        result += "payloadLength: 0x" + String(payloadLength, HEX) + "\n";
        if (!isAck()) {
            result += "payload: ";
            for (byte i = 0; i < payloadLength; i++) {
                result += (char)payload[i];
            }
            result += "\n";
        }
        return result;
    }
} Packet;

class QMACClass {
   public:
    byte localAddress;
    // 60 seconds sleep time, 5 seconds active time
    bool begin(int64_t sleepingDuration = 60000, int64_t activeDuration = 5000,
               byte localAddress = 0xFF);
    bool push(String payload, byte destination = 0xFF);
    void run();
    int amountAvailable();
    bool receive(Packet *p);
    List<Packet> receptionQueue;
    List<Packet> sendQueue;
    bool active = true;
    static void timerCallback(void *arg);
    uint16_t nextActiveTime();

   private:
    void synchronize();
    void updateTimer(uint64_t timeUntilActive);
    byte msgCount;
    int64_t lastTimeActive;
    int64_t sleepingDuration;
    int64_t activeDuration;
    int64_t startCounter = millis();
    esp_timer_handle_t timer_handle;
    List<Packet> unackedQueue;
    bool sendAck(Packet p);
    bool sendSyncPacket(byte destination);
    bool sendPacket(Packet p);
};
extern QMACClass QMAC;