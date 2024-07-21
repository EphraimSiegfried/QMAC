#pragma once

#include <Arduino.h>
#include <CRC.h>
#include <CRC16.h>
#include <Debug.h>
#include <KickSort.h>
#include <LoRa.h>
#include <esp_timer.h>

#include <List.hpp>

#define BCADDR 0xFF  

typedef struct QMACPacket {
    byte destination;
    byte source;
    byte msgCount;
    uint16_t nextActiveTime;
    byte payloadLength;
    byte payload[5];

    bool isAck() const { return payloadLength == 0; }
    bool isSyncPacket() const { return msgCount == 0; }

    String toString() const {
        String result = "destination: 0x" + String(destination, HEX) + "\n";
        result += "localAddress: 0x" + String(source, HEX) + "\n";
        result += "msgCount: 0x" + String(msgCount, HEX) + "\n";

        if (isSyncPacket()) {
            result += "nextWakeUpTime: " + String(nextActiveTime) + "\n";
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
    // 60 seconds sleep time, 5 seconds active time
    bool begin(int64_t sleepingDuration = 60000, int64_t activeDuration = 5000,
               int8_t periodsUntilSync = 5, byte localAddress = 0xFF);
    bool push(String payload, byte destination = 0xFF);
    void run();

    int amountAvailable();
    bool receive(Packet *p);
    static void timerCallback( void *arg);  
    uint16_t nextActiveTime();
    List<Packet> receptionQueue;
    List<Packet> sendQueue;
    bool active = true;
    byte localAddress;
    static constexpr double PACKET_UNACKED_THRESHOLD = 0.3;

   private:
    void synchronize();
    void updateTimer(
        uint64_t timeUntilActive);
    bool sendAck(Packet p); 
    bool sendSyncPacket(byte destination);
    bool sendPacket(Packet p);
    int64_t sleepingDuration;
    int64_t activeDuration;
    byte msgCount;
    uint8_t periodsUntilSync;
    uint8_t periodsSinceSync = 0;
    esp_timer_handle_t timer_handle;
    List<Packet> unackedQueue; 
};
extern QMACClass QMAC;