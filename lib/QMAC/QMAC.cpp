#include <QMAC.h>


bool QMACClass::send(String payload) {
  byte msgCount = 0;            // count of outgoing messages
  byte localAddress = 0xBB;     // address of this device
  byte destination = 0xFF;      // destination to send to
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(payload.length());        // add payload length
  LoRa.print(payload);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
}

bool QMACClass::receive(){
    return true;
}

QMACClass QMAC;