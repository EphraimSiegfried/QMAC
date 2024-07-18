#pragma once

#include <Arduino.h>
#include <Debug.h>
#include <KickSort.h>
#include <LoRa.h>
#include <esp_timer.h>

#include <List.hpp>

// Definition of a packet in the QMAC protocol
typedef struct QMACPacket {
    byte destination; // Destination address
    byte localAddress; // Source address
    byte msgCount; // Number of messages sent including this one
    uint16_t nextWakeUpTime; // Next time the sender will start its
        // next active period
    byte payloadLength; // Length of data from upper layers
    byte payload[5]; // Data from upper layers

    bool isAck() const { return payloadLength == 0; } // Checks if it's an
        // ACK packet
    bool isSyncPacket() const { return msgCount == 0; } // Checks if it's a
        // sync packet

    // Prints packet in an human readable way
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

// The class of the QMAC protocol
class QMACClass {
   public:
    // 60 seconds sleep time, 5 seconds active time
    bool begin(int64_t sleepingDuration = 60000, int64_t activeDuration = 5000,
               byte localAddress = 0xFF); // Initializes the QMAC protocol
    bool push(String payload, byte destination = 0xFF); // Creates a packet with
        // the given payload and destination, and add it to the send queue
    void run(); // Main procedure of the protocol
    int amountAvailable(); // Length of the reception queue
    bool receive(Packet *p); // Puts all information of the received data in
        // the given pointer to a packet
    static void timerCallback(void *arg); // Procedure called when
        // the activity timer is over : resets the timer, set it to the next
        // active/sleeping period depending on the current activity status,
        // then switches activity status
    uint16_t nextActiveTime(); // Returns the remaining time before
        // the next activity period
    List<Packet> receptionQueue; // List of received packets, for upper layers
    List<Packet> sendQueue; // List of packets to send
    bool active = true; // True if node is active, false if sleeping
    byte localAddress; // MAC address of this node
    static constexpr double PACKET_UNACKED_THRESHOLD = 0.3;

   private:
    void synchronize(); // Sends current next active time to the other nodes,
        // and calculate the average next active time from the answers
        // from the other nodes, and set this average as this node's
        // next active time
    void updateTimer(uint64_t timeUntilActive); // Resets the
        // activity switching timer, then set it to the given next active time
    bool sendAck(Packet p); // Sends an ACK as answer of the given packet
    bool sendSyncPacket(byte destination); // Sends a sync packet
    bool sendPacket(Packet p); // Sends the given packet
    byte msgCount; // Number of messages sent including this one
    // int64_t lastTimeActive; // LEGACY
    int64_t sleepingDuration; // Duration of the sleeping period
    int64_t activeDuration; // Duration of the active period
    // int64_t startCounter = millis(); // LEGACY
    esp_timer_handle_t timer_handle; // Handler of the activity switching timer
    List<Packet> unackedQueue; // List of packets that have been sent
        // by this node but not ACKed by the receiver yet
    
};
extern QMACClass QMAC;