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

    LOG("Local Address: " + String(this->localAddress));
    return true;
}

void QMACClass::run() {
    if (!this->active) return;

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
    p.payloadLength = LoRa.read();
    for (size_t i = 0; i < p.payloadLength; i++) {
        p.payload[i] = LoRa.read();
    }

    // ignore packet if it is not for us
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

    return true;
}
void QMACClass::timerCallback(void* arg) {
    QMACClass* self = static_cast<QMACClass*>(arg);
    esp_timer_start_once(self->timer_handle, self->active
                                                 ? self->sleepingDuration * 1000
                                                 : self->activeDuration * 1000);
    self->active = !self->active;
}

// void QMACClass::updateTimer(int timeUntilActivePeriod) {
//     esp_timer_stop(this->timer_handle);
//     this->active = false;
//     esp_timer_start_once(this->timer_handle, timeUntilActivePeriod);
// }

QMACClass QMAC;
