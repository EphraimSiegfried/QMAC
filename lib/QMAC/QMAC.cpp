#include <QMAC.h>

bool QMACClass::begin(int64_t sleepingDuration, int64_t activeDuration,
                      int8_t periodsUntilSync, byte localAddress) {
    this->localAddress =
        localAddress == BCADDR
            ? random(254)
            : localAddress;  // assign random address if address not specified
    this->msgCount = 1;
    this->sleepingDuration = sleepingDuration;
    this->activeDuration = activeDuration;
    this->periodsUntilSync = periodsUntilSync;

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

    if (this->periodsSinceSync >= this->periodsUntilSync) {
        synchronize();
        this->periodsSinceSync = 0;
        return;
    }

    // Schedule in which time slots packets in the queue should be sent
    // We assign a random time slot for every packets to send to avoid collision
    int slotTime = 100;  // TODO: move
    int numSlots = activeDuration / slotTime;
    int numPacketsReady = sendQueue.getSize();
    int activeSlots[numPacketsReady];
    bool receivedSync = false;
    for (size_t i = 0; i < numPacketsReady; i++) {
        activeSlots[i] = random(numSlots);
    }
    KickSort<int>::quickSort(activeSlots, numPacketsReady);

    // Start listening and sending packets
    int64_t startTime = millis();
    size_t idx = 0;
    unackedQueue.clear();
    while (this->active) {
        // For each packet, we send it only when it's its turn:
        if (!sendQueue.isEmpty() &&
            millis() >= activeSlots[idx] * slotTime + startTime) {
            Packet nextPacket = sendQueue[0];
            sendPacket(nextPacket);
            sendQueue.removeFirst();
            // Nobody ACKs a broadcast message, so we shouldn't expect an answer:
            if (nextPacket.destination != BCADDR) {
                unackedQueue.add(nextPacket);
            }
            idx++;
        }

        Packet p = {};
        // start listening and ignore if nothing received
        if (!QMAC.receive(&p)) continue;
        // ignore packet if it is not for this device
        if (p.destination != this->localAddress && p.destination != BCADDR)
            continue;
        // react according to packet type
        if (p.isSyncPacket()) {
            receivedSync = true;
            sendSyncPacket(p.source);
        } else if (p.isAck()) {
            // When ACK for a packet is received, we can remove the packet
            // from the unacked queue:
            LOG("Received ACK for ID " + String(p.msgCount));
            for (size_t i = 0; i < unackedQueue.getSize(); i++) {
                if (unackedQueue[i].msgCount == p.msgCount) {
                    unackedQueue.remove(i);
                    break;
                }
            }
        } else {
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
            if (p.destination != BCADDR) {
                sendAck(p);
            }
        }
    }

    // Go to sleep when active time is over
    LoRa.sleep();
    // synchronize if a percentage of packets didn't arrive
    double unackedRatio = (double)unackedQueue.getSize() / numPacketsReady;
    if (numPacketsReady > 0) {
        LOG("PERCENTAGE of UNACKED packets: " + String(100 * unackedRatio) +
            "%");
    }
    if (numPacketsReady > 0 && unackedRatio >= PACKET_UNACKED_THRESHOLD &&
        !receivedSync) {
        synchronize();
    }

    // Putting all unacked packets to the send packets queue, so they will be
    // sent during the next active period:
    sendQueue.addAll(unackedQueue);

    this->periodsSinceSync++;

    return;
}

int QMACClass::amountAvailable() { return receptionQueue.getSize(); }

bool QMACClass::push(String payload, byte destination) {
    // Creating a new packets with all required fields:
    Packet p;
    p.destination = destination;
    p.source = localAddress;
    p.msgCount = this->msgCount++;
    p.payloadLength = payload.length();
    payload.getBytes(p.payload, sizeof(p.payload));
    sendQueue.add(p);
    return true;
}

bool QMACClass::sendAck(Packet p) {
    // Just switches sender and receiver address, and setting the payload length
    // to 0, to identify it as an ACK packet (chosen arbitrarily):
    Packet ackPacket = {
        .destination = p.source,
        .source = this->localAddress,
        .msgCount = p.msgCount,
        .payloadLength = 0,
    };
    LOG("Sending ACK for ID " + String(p.msgCount));
    return sendPacket(ackPacket);
}

bool QMACClass::sendSyncPacket(byte destination) {
    // Just switches sender and receiver address, and setting the message count
    // to 0, to identify it as a sync packet (chosen arbitrarily).
    // Also adds the next active time:
    Packet syncResponse = {
        .destination = destination,
        .source = this->localAddress,
        .msgCount = 0,
        .nextActiveTime = nextActiveTime(),
        .payloadLength = 0,
    };
    LOG("Sending Sync Packet with timestamp " +
        String(syncResponse.nextActiveTime));
    return sendPacket(syncResponse);
}

bool QMACClass::sendPacket(Packet p) {
    if (!LoRa.beginPacket()) {
        LOG("LoRa beginPacket failed");
        return false;
    }
    CRC16 crc;
    // Sends all fields of the packet in order:
    LoRa.write(p.destination);  
    crc.add(p.destination);
    LoRa.write(p.source); 
    crc.add(p.source);
    LoRa.write(p.msgCount); 
    crc.add(p.msgCount);
    if (p.isSyncPacket()) {
        // convert nextActiveTime to byte array
        byte b[2] = {p.nextActiveTime & 0xff, p.nextActiveTime >> 8};
        LoRa.write(b, 2);
        crc.add(b, 2);
    } else {
        LoRa.write(p.payloadLength);
        crc.add(p.payloadLength);
        LoRa.write(p.payload, p.payloadLength);
        crc.add(p.payload, p.payloadLength);
    }
    uint16_t checksum = crc.calc();
    byte c[2] = {checksum & 0xff, checksum >> 8};
    LoRa.write(c, 2);
    if (!LoRa.endPacket()) {
        LOG("LoRa endPacket failed");
        return false;
    }
    return true;
}

bool QMACClass::receive(Packet* p) {
    if (!LoRa.parsePacket()) return false;
    // Parses the received data as a packet:
    CRC16 crc;
    p->destination = LoRa.read();
    crc.add(p->destination);
    p->source = LoRa.read();
    crc.add(p->source);
    p->msgCount = LoRa.read();
    crc.add(p->msgCount);
    if (p->isSyncPacket()) {
        byte t[2];
        LoRa.readBytes(t, 2);
        p->nextActiveTime = *((int*)t);
        crc.add(t, 2);
    } else {
        p->payloadLength = LoRa.read();
        crc.add(p->payloadLength);
        LoRa.readBytes(p->payload, p->payloadLength);
        crc.add(p->payload, p->payloadLength);
    }
    byte checksum[2];
    LoRa.readBytes(checksum, 2);
    // return true if CRC check is successfull
    // TODO: should the checksum be added to the Packet struct? should checking
    // be done outside of the receive function?
    return crc.calc() == *((uint16_t*)checksum);
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
    List<uint8_t> addresses; // TODO: Use better data structure
    uint64_t cycleDuration = this->activeDuration + this->sleepingDuration;
    int minimumListeningDuration = 200;

    // Sending sync packets and waiting until a response is received:
    while (receivedTimestamps.isEmpty()) {
        uint64_t syncStartTime = millis();
        while ((millis() - syncStartTime) < cycleDuration) {
            uint64_t period = random(minimumListeningDuration, this->activeDuration);

            long transmissionStartTime = millis();
            sendSyncPacket(BCADDR);

            // listen for sync responses for some time
            long listeningStartTime = millis();
            while (millis() - listeningStartTime < period) {
                Packet p = {};
                if (!receive(&p)) continue;
                boolean knownAddress = false;
                for (size_t i = 0; i < addresses.getSize(); i++) {
                    if (p.source == addresses[i]){
                        knownAddress = true;
                        break;
                    }
                }
                if (p.isSyncPacket() && !knownAddress) {
                    LOG("Received sync packet");
                    transmissionDelays.add(millis() - transmissionStartTime);
                    receivedTimestamps.add(p.nextActiveTime);
                    receptionTimestamps.add(millis());
                    addresses.add(p.source);
                }
            }
            LoRa.sleep();
            delay(period);
        }
    }

    int numResponses = receivedTimestamps.getSize();
    uint64_t averageNextActiveTime = 0;
    for (size_t i = 0; i < numResponses; i++) {
        // Results show removing the delay from the calculation improves the synchronization
        // uint64_t timeResponseSent = receptionTimestamps[i] - 0.5 * transmissionDelays[i];
        uint64_t timeResponseSent = receptionTimestamps[i];
        uint64_t timeSinceResponseSent = millis() - timeResponseSent;

        averageNextActiveTime += receivedTimestamps[i] - (millis() - timeResponseSent);
        if (timeSinceResponseSent > receivedTimestamps[i]){
            averageNextActiveTime += cycleDuration;
        }
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
    esp_timer_start_once(this->timer_handle, timeUntilActive * 1000);
}

QMACClass QMAC;
