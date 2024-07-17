#include <QMAC.h>

bool QMACClass::begin(int64_t sleepingDuration, int64_t activeDuration,
                      byte localAddress) {
    // assign random address if address not specified
    this->localAddress = localAddress == 0xFF ? random(254) : localAddress;
    this->msgCount = 1;
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

        Packet p = {};
        // start listening and ignore if nothing received
        if (!QMAC.receive(&p)) continue;
        // ignore packet if it is not for this device
        if (p.destination != this->localAddress && p.destination != 0xff)
            continue;
        // react according to packet type
        if (p.isSyncPacket()) {
            sendSyncPacket(p.localAddress);
        } else if (p.isAck()) {
            LOG("Received ACK for ID " + String(p.msgCount));
            for (size_t i = 0; i < unackedQueue.getSize(); i++) {
                if (unackedQueue[i].msgCount == p.msgCount) {
                    unackedQueue.remove(i);
                    break;
                }
            }
        } else {
            // Normal Packet
            LOG("Received Packet with ID " + String(p.msgCount));
            bool isAlreadyReceived = false;
            for (size_t i = 0; i < receptionQueue.getSize(); i++) {
                if (receptionQueue[i].msgCount == p.msgCount) {
                    isAlreadyReceived = true;
                    break;
                }
            }
            if (!isAlreadyReceived) {
                receptionQueue.add(p);
            }
            sendAck(p);
        }
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
    Packet ackPacket = {
        .destination = p.localAddress,
        .localAddress = this->localAddress,
        .msgCount = p.msgCount,
        .payloadLength = 0,
    };
    LOG("Sending ACK for ID " + String(p.msgCount));
    return sendPacket(ackPacket);
}

bool QMACClass::sendSyncPacket(byte destination) {
    Packet syncResponse = {
        .destination = destination,
        .localAddress = this->localAddress,
        .msgCount = 0,
        .nextWakeUpTime = nextActiveTime(),
        .payloadLength = 0,
    };
    LOG("Sending Sync Packet with timestamp " +
        String(syncResponse.nextWakeUpTime));
    return sendPacket(syncResponse);
}

bool QMACClass::sendPacket(Packet p) {
    if (!LoRa.beginPacket()) {
        LOG("LoRa beginPacket failed");
        return false;
    }
    LoRa.write(p.destination);   // add destination address
    LoRa.write(p.localAddress);  // add sender address
    LoRa.write(p.msgCount);      // add message ID
    if (p.isSyncPacket()) {
        LoRa.write(p.nextWakeUpTime & 0xff);
        LoRa.write(p.nextWakeUpTime >> 8);
    } else {
        LoRa.write(p.payloadLength);  // add payload length
        for (size_t i = 0; i < p.payloadLength; i++) {
            LoRa.write(p.payload[i]);
        }
    }
    if (!LoRa.endPacket()) {
        LOG("LoRa endPacket failed");
        return false;
    }
    return true;
}

bool QMACClass::receive(Packet* p) {
    if (!LoRa.parsePacket()) return false;
    p->destination = LoRa.read();
    p->localAddress = LoRa.read();
    p->msgCount = LoRa.read();
    if (p->isSyncPacket()) {
        byte t[4];
        LoRa.readBytes(t, 4);
        p->nextWakeUpTime = *((int*)t);
    } else {
        p->payloadLength = LoRa.read();
        for (size_t i = 0; i < p->payloadLength; i++) {
            p->payload[i] = LoRa.read();
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

uint16_t QMACClass::nextActiveTime() {
    int64_t nextTimeout = esp_timer_get_next_alarm() / 1000 - millis();
    return active ? nextTimeout + this->sleepingDuration : nextTimeout;
}

void QMACClass::synchronize() {
    LOG("Start synchronization");
    List<uint16_t> receivedTimestamps;
    List<uint64_t> transmissionDelays;
    List<uint64_t> receptionTimestamps;

    while (1) {
        long transmissionStartTime = millis();
        sendSyncPacket(0xFF);

        // listen for sync responses for some time
        long listeningStartTime = millis();
        while (millis() - listeningStartTime < 1000) {
            Packet p = {};
            if (!receive(&p)) continue;
            if (p.isSyncPacket()) {
                LOG(p.toString());
                transmissionDelays.add(millis() - transmissionStartTime);
                receivedTimestamps.add(p.nextWakeUpTime);
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
    LOG("NUM RESPONSES " + String(numResponses));
    for (size_t i = 0; i < numResponses; i++) {
        LOG("RECEIVED TIMESTAMP " + String(receivedTimestamps[i]));
        LOG("DELAY " + String(transmissionDelays[i]));
        LOG("TIME RECEIVED " + String(receptionTimestamps[i]));
        LOG("MILLIS " + String(millis()));
        averageNextActiveTime += receivedTimestamps[i] -
                                 0.5 * transmissionDelays[i] -
                                 (millis() - receptionTimestamps[i]);
    }
    averageNextActiveTime += nextActiveTime();
    averageNextActiveTime = averageNextActiveTime / (numResponses + 1);
    LOG("Own Schedule " + String(nextActiveTime()));
    LOG("Average next time active " + String(averageNextActiveTime));

    updateTimer(averageNextActiveTime);
}

void QMACClass::updateTimer(uint64_t timeUntilActive) {
    esp_timer_stop(this->timer_handle);
    this->active = false;
    esp_timer_start_once(this->timer_handle, timeUntilActive);
}

QMACClass QMAC;
