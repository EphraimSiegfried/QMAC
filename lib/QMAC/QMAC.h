#pragma once

#include <Arduino.h>
#include <LoRa.h>

class QMACClass {
    public:
        bool begin(LoRaClass LoRa);
        bool send(String payload);
        bool receive();

};
extern QMACClass QMAC;