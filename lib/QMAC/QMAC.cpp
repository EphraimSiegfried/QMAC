#include <QMAC.h>

bool QMACClass::begin(int64_t wakeUpInterval, int64_t activeDuration,
                      byte localAddress) {
    // assign random address if address not specified
    this->localAddress = localAddress == 0xFF ? random(254) : localAddress;
    this->msgCount = 0;
    this->wakeUpInterval = 1000 * wakeUpInterval;
    this->activeDuration = 1000 * activeDuration;
    this->isActivePeriod = false;

    esp_timer_create_args_t timer_args = {.callback = &QMACClass::timerCallback,
                                          .arg = this,
                                          .name = "duty_cycle_timer"};
    esp_timer_create(&timer_args, &this->timer_handle);
    esp_timer_start_once(timer_handle, wakeUpInterval);

    LOG("Local Address: " + String(this->localAddress));
    return true;
}

void QMACClass::run() {
    while (this->isActivePeriod) {
        for (size_t i = 0; i < sendQueue.getCount(); i++) {
            String payload;
            sendQueue.pop(&payload);
            send(payload);
        }
    }
    return;
}

bool QMACClass::push(String payload, byte destination) {
    sendQueue.push(&payload);
    return true;
}

bool QMACClass::sendAck(byte destination, byte msgCount) {
    if (!LoRa.beginPacket()) {
        // LOG("LoRa beginPacket failed");
        return false;
    }
    LoRa.write(destination);
    LoRa.write(this->localAddress);
    LoRa.write(msgCount);
    LoRa.write(0);  // add payload length (0 for ack)
    if (!LoRa.endPacket()) {
        // LOG("LoRa endPacket failed");
        return false;
    }
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
    return true;
}

void QMACClass::receive(int packetSize) {
    Packet p;
    p.destination = LoRa.read();
    p.localAddress = LoRa.read();
    p.msgCount = LoRa.read();
    p.payloadLength = LoRa.read();
    for (size_t i = 0; i < p.payloadLength; i++) {
        p.payload[i] = LoRa.read();
    }
    receptionQueue.push(&p);
    sendAck(p.localAddress, p.msgCount);
}

void QMACClass::timerCallback(void* arg) {
    QMACClass* self = static_cast<QMACClass*>(arg);

    if (self->isActivePeriod) {
        LoRa.sleep();
        self->isActivePeriod = false;
        esp_timer_start_once(self->timer_handle, self->wakeUpInterval);
    } else {
        LoRa.receive();
        self->isActivePeriod = true;
        esp_timer_start_once(self->timer_handle, self->activeDuration);
    }
}

QMACClass QMAC;
