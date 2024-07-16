#include <QMAC.h>

bool QMACClass::begin(int64_t sleepingDuration, int64_t activeDuration,
                      byte localAddress) {
    // assign random address if address not specified
    this->localAddress = localAddress == 0xFF ? random(254) : localAddress;
    this->msgCount = 0;
    this->sleepingDuration = sleepingDuration;
    this->activeDuration = activeDuration;

    esp_timer_create_args_t timer_args = {.callback = &QMACClass::timerCallback,
                                          .arg = this,
                                          .name = "duty_cycle_timer"};
    esp_timer_create(&timer_args, &this->timer_handle);
    esp_timer_start_once(timer_handle, sleepingDuration * 1000);
    synchronize();

    LOG("Local Address: " + String(this->localAddress));
    return true;
}

void QMACClass::run() {
    if (!this->active) return;

    // Schedule in which time slots packets in the queue should be sent
    int slotTime = 100;  // TODO: move
    int activeSlots[sendQueue.getSize()];
    int numSlots = activeDuration / slotTime;
    for (size_t i = 0; i < sendQueue.getSize(); i++) {
        activeSlots[i] = random(numSlots);
    }
    KickSort<int>::quickSort(activeSlots, sendQueue.getSize());

    // Start listening and sending packets
    int64_t startTime = millis();
    size_t idx = 0;
    unackedQueue.clear();
    while (this->active) {
        if (!sendQueue.isEmpty() &&
            millis() >= activeSlots[idx] * slotTime + startTime) {
            Packet nextPacket = sendQueue[0];
            sendPacket(nextPacket);
            sendQueue.removeFirst();
            unackedQueue.add(nextPacket);
            idx++;
        }

        QMAC.receive(LoRa.parsePacket());
    }

    // Go to sleep when active time is over
    LoRa.sleep();
    if (!unackedQueue.isEmpty())
        LOG("Number of UNACKED Packets: " + String(unackedQueue.getSize()));
    sendQueue.addAll(unackedQueue);
    return;
}

int QMACClass::amountAvailable() { return receptionQueue.getSize(); }

bool QMACClass::push(String payload, byte destination) {
    Packet p;
    p.destination = destination;
    p.localAddress = localAddress;
    p.msgCount = this->msgCount++;
    p.payloadLength = payload.length();
    payload.getBytes(p.payload, sizeof(p.payload));
    sendQueue.add(p);
    return true;
}

bool QMACClass::sendAck(Packet p) {
    p.destination = p.localAddress;
    p.localAddress = this->localAddress;
    p.payloadLength = 0;
    LOG("Sent ACK for message ID: " + String(p.msgCount));
    return sendPacket(p);
}

bool QMACClass::sendPacket(Packet p) {
    if (!LoRa.beginPacket()) {
        LOG("LoRa beginPacket failed");
        return false;
    }
    LoRa.write(p.destination);   // add destination address
    LoRa.write(p.localAddress);  // add sender address
    LoRa.write(p.msgCount);      // add message ID
    LOG("Next active period: " + String(esp_timer_get_next_alarm() / 1000 +
                                        this->sleepingDuration - millis()));
    if (p.msgCount == 0)
        LoRa.print(esp_timer_get_next_alarm() / 1000 + this->sleepingDuration -
                   millis());
    LoRa.write(p.payloadLength);  // add payload length
    for (size_t i = 0; i < p.payloadLength; i++) {
        LoRa.write(p.payload[i]);
    }

    if (!LoRa.endPacket()) {
        LOG("LoRa endPacket failed");
        return false;
    }
    LOG("Sent Packet with ID:" + String(p.msgCount));
    return true;
}

bool QMACClass::receive(int packetSize) {
    if (!packetSize) return false;
    Packet p;
    p.destination = LoRa.read();
    p.localAddress = LoRa.read();
    p.msgCount = LoRa.read();
    // If it is a sync packet, answer:
    if (p.msgCount == 0) {
        LoRa.print(esp_timer_get_next_alarm() / 1000 + this->sleepingDuration -
                   millis());
    }
    // It isn't:
    else {
        p.payloadLength = LoRa.read();
        for (size_t i = 0; i < p.payloadLength; i++) {
            p.payload[i] = LoRa.read();
        }

        // ignore packet if it is not for this device
        if (p.destination != this->localAddress && p.destination != 0xff) {
            return true;
        }

        // Check if received packet is an ACK
        if (p.payloadLength == 0) {
            for (size_t i = 0; i < unackedQueue.getSize(); i++) {
                if (unackedQueue[i].msgCount == p.msgCount) {
                    unackedQueue.remove(i);
                    LOG("Packet " + String(p.msgCount) + " got succesfully ACKED");
                    return true;
                }
            }
        } else {
            LOG("Received packet with ID " + String(p.msgCount));
            receptionQueue.add(p);
            if (!sendAck(p)) LOG("Failed to send ACK");
        }
    }

    return true;
}
void QMACClass::timerCallback(void* arg) {
    QMACClass* self = static_cast<QMACClass*>(arg);
    esp_timer_start_once(self->timer_handle, self->active
                                                 ? self->sleepingDuration * 1000
                                                 : self->activeDuration * 1000);
    self->active = !self->active;
}

void QMACClass::synchronize() {
    LOG("Start synchronization");
    Packet synchronizationPacket{.destination = 0xFF,
                                 .localAddress = this->localAddress,
                                 .msgCount = 0,
                                 .payloadLength = 0};
    List<int> receivedTimestamps;
    List<uint64_t> transmissionDelays;
    List<uint64_t> receptionTimestamps;

    while (1) {
        long transmissionStartTime = millis();
        sendPacket(synchronizationPacket);

        // listen for sync responses for some time
        long listeningStartTime = millis();
        while (millis() - listeningStartTime < 1000) {
            int packetSize = LoRa.parsePacket();
            if (!packetSize) continue;
            byte destination = LoRa.read();
            byte localAddress = LoRa.read();
            byte msgCount = LoRa.read();
            LOG(msgCount);
            // msgCount == 0 is a sync response
            if (msgCount == 0) {
                // TODO: Handle error when receiving
                LOG("RECEIVED sync packet");
                LOG(packetSize);
                byte t[4];
                LoRa.readBytes(t, 4);
                // convert byte array to int
                int timestamp = *((int*)t);

                transmissionDelays.add(transmissionStartTime - millis());
                receivedTimestamps.add(timestamp);
                receptionTimestamps.add(millis());
            }
        }

        // stop if response received
        if (!receivedTimestamps.isEmpty()) break;

        // go to sleep if no responses received
        LoRa.sleep();
        delay(3000);
    }

    int numResponses = receivedTimestamps.getSize();
    uint64_t averageNextActiveTime = 0;
    for (size_t i = 0; i < numResponses; i++) {
        averageNextActiveTime += receivedTimestamps[i] -
                                 0.5 * transmissionDelays[i] -
                                 (millis() - receptionTimestamps[i]);
    }
    averageNextActiveTime = averageNextActiveTime / numResponses;
    LOG("Average next time active " + averageNextActiveTime);
    updateTimer(averageNextActiveTime);
}

void QMACClass::updateTimer(uint64_t timeUntilActive) {
    esp_timer_stop(this->timer_handle);
    this->active = false;
    esp_timer_start_once(this->timer_handle, timeUntilActive);
}

QMACClass QMAC;
