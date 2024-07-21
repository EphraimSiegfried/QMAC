#pragma once

#include <Arduino.h>
#include <CRC.h>
#include <CRC16.h>
#include <Debug.h>
#include <KickSort.h>
#include <LoRa.h>
#include <LoRaAirtime.h>
#include <esp_timer.h>

#include <List.hpp>

#define BCADDR             0xFF
#define ACK_PACKET_SIZE    6
#define SYNC_PACKET_SIZE   7
#define NORMAL_HEADER_SIZE 6
#define PREAMBLE_LENGTH    7
// 235 (max lora packet length) - 8 (preamble length) - 6 (normal header size) =
// 221
#define PAYLOAD_SIZE 221
// following calculated with https://www.loratools.nl/#/airtime
#define SYNC_AIRTIME 28.93
#define ACK_AIRTIME  28.93

class QMACClass {
   public:
    /**
     * Initialize the QMACClass with the given local address.
     * It will try synchronize with other devices and run until devices were
     * reached. This will be run for a minimum of a cycle duration (=
     * sleepDuration + activeDuration). This function should ideally be called
     * until it returns true.
     * @param localAddress The local address of the device.
     * @return true if other nodes were reached, false otherwise.
     */
    bool begin(byte localAddress = 0xFF);

    /**
     * Run the QMAC protocol, handling packet transmission, synchronization and
     * reception. Should be run frequently.
     * @return true if devices are reachable, false otherwise.
     */
    bool run();

    /**
     * Add a packet which should be sent in the next active time period.
     * @param payload The payload to be sent.
     * @param payloadSize Number of bytes set in the payload
     * @param destination The destination address of the packet.
     */
    void push(byte payload[PAYLOAD_SIZE], byte payloadSize,
              byte destination = 0xFF);

    /**
     * Get the number of packets which were succesfully sent to this device.
     * @return The number of available packets in the reception queue.
     */
    int numPacketsAvailable();

    /**
     * Remove a packet from the reception queue and retrieve it.
     * @return The packet at the front of the reception queue.
     */
    Packet pop();

    /**
     * Check if the device is currently sending or receiving packets.
     * @return true if active, false otherwise.
     */
    boolean isActive();

    /**
     * Set the duration where the LoRa module is inactive.
     * @param duration The sleeping duration in milliseconds (default is 60000).
     */
    void setSleepingDuration(uint64_t duration = 60000);

    /**
     * Set the duration in which packets can be sent and received.
     * @param duration The active duration in milliseconds (default is 5000).
     */
    void setActiveDuration(uint64_t duration = 5000);

    /**
     * Set the number of cycles (activeDuration + sleepingDuration) until
     * synchronization is required.
     * @param periods The number of periods until synchronization (default is
     * 50).
     */
    void setPeriodsUntilSync(uint8_t periods = 50);

    /**
     * Set the maximum number of retries for resending packets until packets are
     * dropped.
     * @param maxTries The maximum number of retries (default is 3).
     */
    void setMaxPacketsResendTries(uint16_t maxTries = 3);

    /**
     * Set the threshold for unacknowledged packets before synchronization is
     * triggered.
     * @param threshold The threshold as a percentage (0.0 to 1.0) (default is
     * 0.8).
     */
    void setUnackedPacketThreshold(float threshold = 0.8);

    /**
     * Get the next active time based on the current timer and durations.
     * @return The next active time in milliseconds.
     */
    uint16_t nextActiveTime();

    byte localAddress;

   private:
    bool synchronize();
    void updateTimer(uint64_t timeUntilActive);
    bool sendAck(Packet p);
    bool sendSyncPacket(byte destination);
    bool send(Packet p);
    bool receive(Packet *p);
    static void timerCallback(void *arg);
    esp_timer_handle_t timer_handle;
    List<Packet> receptionQueue;
    List<Packet> sendQueue;
    List<Packet> resendQueue;
    double availableAirtime;
    double availableAirtimePerCycle;
    uint64_t sleepDuration = 60000;
    uint64_t activeDuration = 5000;
    byte msgCount = 1;
    uint8_t periodsUntilSync = 50;
    uint8_t periodsSinceSync = 0;
    uint16_t maxPacketResendTries = 3;
    bool active = true;
    float unackedPacketThreshold = 0.8;
};

typedef struct QMACPacket {
    // Packet Headers:
    byte destination;
    byte source;
    byte packetID;
    // only in sync packets
    uint16_t nextActiveTime;
    byte payloadLength;
    byte payload[PAYLOAD_SIZE];
    // byte crc[2];
    uint16_t sendRetryCount;

    bool isAck() const { return payloadLength == 0; }
    bool isSyncPacket() const { return packetID == 0; }

    String toString() const {
        String result = "destination: 0x" + String(destination, HEX) + "\n";
        result += "localAddress: 0x" + String(source, HEX) + "\n";
        result += "msgCount: 0x" + String(packetID, HEX) + "\n";

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

extern QMACClass QMAC;
