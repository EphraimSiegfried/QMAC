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
    while (isActivePeriod()) {
        if (!sendQueue.isEmpty() &&
            millis() >= activeSlots[idx] * slotTime + startTime) {
            send(sendQueue[sendQueue.getSize() - 1]);
            sendQueue.removeLast();
            idx++;
        }

        QMAC.receive(LoRa.parsePacket());
    }

    // Go to sleep when active time is over
    LoRa.sleep();
    return;
}

int QMACClass::amountAvailable() { return receptionQueue.getSize(); }

bool QMACClass::push(String payload, byte destination) {
    sendQueue.add(payload);
    return true;
}

bool QMACClass::sendAck(byte destination, byte msgCount) {
    if (!LoRa.beginPacket()) {
        return false;
    }
    LoRa.write(destination);
    LoRa.write(this->localAddress);
    LoRa.write(msgCount);
    LoRa.write(0);  // add payload length (0 for ack)
    if (!LoRa.endPacket()) {
        return false;
    }
    LOG("Sent ACK for message ID: " + String(msgCount));
    return true;
}

bool QMACClass::send(String payload, byte destination) {
    if (!LoRa.beginPacket()) {
        LOG("LoRa beginPacket failed");
        return false;
    }
    LoRa.write(destination);         // add destination address
    LoRa.write(this->localAddress);  // add sender address
    LoRa.write(this->msgCount);      // add message ID
    LoRa.write(payload.length());    // add payload length
    LoRa.print(payload);             // add payload
    if (!LoRa.endPacket()) {
        LOG("LoRa endPacket failed");
        return false;
    }
    this->msgCount++;
    LOG("Sent packet");
    return true;
}

bool QMACClass::receive(int packetSize) {
    if (!packetSize) return false;
    Packet p;
    p.destination = LoRa.read();
    p.localAddress = LoRa.read();
    p.msgCount = LoRa.read();
    p.payloadLength = LoRa.read();
    for (size_t i = 0; i < p.payloadLength; i++) {
        p.payload[i] = LoRa.read();
    }

    // Check if received packet is an ACK
    if (p.payloadLength == 0) {
        LOG("Received ACK");
        return true;
    }
    LOG("Received packet");
    receptionQueue.add(p);
    if (!sendAck(p.localAddress, p.msgCount)) LOG("Failed to send ACK");
    return true;
}

QMACClass QMAC;
