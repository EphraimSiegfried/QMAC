#pragma once

#include <Arduino.h>
#include <Debug.h>
#include <LoRa.h>

class QMACClass {
    public:
        byte localAddress;
        bool begin(byte localAddress = 0xFF);
        bool send(String payload, byte destination = 0xFF);
        static void onReceiveWrapper(int packetSize);
        void onReceive(int packetSize);

    
    private:
        byte msgCount;
        LoRaClass _LoRa;

};
extern QMACClass QMAC;