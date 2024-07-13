#include <QMAC.h>


bool QMACClass::begin(byte localAddress) {
    // assign random address if not address not specified
    this->localAddress = localAddress == 0xFF ? random(254) : localAddress;
    LOG("Local Address: " + String(this->localAddress));
    this->msgCount = 0;
    LoRa.onReceive(onReceiveWrapper);  // set the static wrapper as the callback
    return true;
}

bool QMACClass::send(String payload, byte destination) {
    if (!LoRa.beginPacket()) {
        LOG("LoRa beginPacket failed");
        return false;
    }
    LoRa.write(destination);              // add destination address
    LoRa.write(this->localAddress);       // add sender address
    LoRa.write(this->msgCount);           // add message ID
    LoRa.write(payload.length());         // add payload length
    LoRa.print(payload);                  // add payload
    if (!LoRa.endPacket()) {
        LOG("LoRa endPacket failed");
        return false;
    }
    this->msgCount++;
    return true;
}

void QMACClass::onReceiveWrapper(int packetSize) {
    QMAC.onReceive(packetSize);  // call the non-static member function
}

void QMACClass::onReceive(int packetSize) {
    if (packetSize == 0) return;  // if there's no packet, return

    // read packet header bytes:
    int recipient = LoRa.read();          // recipient address
    byte sender = LoRa.read();            // sender address
    byte incomingMsgId = LoRa.read();     // incoming msg ID
    byte incomingLength = LoRa.read();    // incoming msg length

    String incoming = "";

    while (LoRa.available()) {
        incoming += (char)LoRa.read();
    }

    if (incomingLength != incoming.length()) {   // check length for error
        Serial.println("error: message length does not match length");
        return;                             // skip rest of function
    }

    // if the recipient isn't this device or broadcast,
    if (recipient != localAddress && recipient != 0xFF) {
        Serial.println("This message is not for me.");
        return;                             // skip rest of function
    }

    // if message is for this device, or broadcast, print details:
    Serial.println("Received from: 0x" + String(sender, HEX));
    Serial.println("Sent to: 0x" + String(recipient, HEX));
    Serial.println("Message ID: " + String(incomingMsgId));
    Serial.println("Message length: " + String(incomingLength));
    Serial.println("Message: " + incoming);
    Serial.println("RSSI: " + String(LoRa.packetRssi()));
    Serial.println("Snr: " + String(LoRa.packetSnr()));
    Serial.println();
}

QMACClass QMAC;
