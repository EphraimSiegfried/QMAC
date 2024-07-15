#include <QMAC.h>

bool QMACClass::begin(int64_t wakeUpInterval, int64_t activeDuration,
                      byte localAddress) {
    // assign random address if address not specified
    this->localAddress = localAddress == 0xFF ? random(254) : localAddress;
    this->msgCount = 0;
    this->wakeUpInterval = wakeUpInterval;
    this->activeDuration = activeDuration;

    LOG("Local Address: " + String(this->localAddress));
    return true;
}

bool QMACClass::isActivePeriod() {
    return millis() % this->wakeUpInterval < this->activeDuration;
}

void QMACClass::run() {
    if (!isActivePeriod()) return;

    // Schedule in which time slots packets in the queue should be sent
    int slotTime = 40;  // TODO: move
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
    while (isActivePeriod()) {
        if (!sendQueue.isEmpty() &&
            millis() >= activeSlots[idx] * slotTime + startTime) {
            Packet nextPacket = sendQueue[sendQueue.getSize() - 1];
            sendPacket(nextPacket);
            sendQueue.removeLast();
            unackedQueue.add(nextPacket);
            idx++;
        }

        QMAC.receive(LoRa.parsePacket());
        delay(50);
    }

    // Go to sleep when active time is over
    LoRa.sleep();
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
    LoRa.write(p.destination);    // add destination address
    LoRa.write(p.localAddress);   // add sender address
    LoRa.write(p.msgCount);       // add message ID
    LoRa.write(p.payloadLength);  // add payload length
    LOG("SENT PAYLOADLENGTH " + String(p.payloadLength));
    for (size_t i = 0; i < p.payloadLength; i++) {
        LoRa.write(p.payload[i]);
    }

    if (!LoRa.endPacket()) {
        LOG("LoRa endPacket failed");
        return false;
    }
    LOG("Sent Packet " + String(p.msgCount));
    return true;
}

bool QMACClass::receive(int packetSize) {
    if (!packetSize) return false;
    LOG("RECEIVED");
    Packet p;
    p.destination = LoRa.read();
    p.localAddress = LoRa.read();
    p.msgCount = LoRa.read();
    p.payloadLength = LoRa.read();
    LOG("Payload length " + String(p.payloadLength));
    for (size_t i = 0; i < p.payloadLength; i++) {
        p.payload[i] = LoRa.read();
    }

    // Check if received packet is an ACK
    if (p.payloadLength == 0) {
        for (size_t i = 0; i < unackedQueue.getSize(); i++) {
            if (unackedQueue[i].msgCount == p.msgCount) {
                unackedQueue.remove(i);
                LOG("Packet " + String(unackedQueue[i].msgCount) +
                    " got succesfully ACKED");
            }
        }
    } else {
        LOG("Received packet with ID " + String(p.msgCount));
        receptionQueue.add(p);
        if (!sendAck(p)) LOG("Failed to send ACK");
    }

    return true;
}

QMACClass QMAC;
