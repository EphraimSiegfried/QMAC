#pragma once
#include <Arduino.h>

class LoRaAirtime {
   private:
    uint8_t sf;
    uint16_t bw;
    uint8_t cr;
    boolean h;
    boolean crc;
    boolean de;
    uint8_t preambleLength;

   public:
    LoRaAirtime() {
        this->sf = 7;
        this->bw = 125;
        this->cr = 4;
        this->preambleLength = 8;
        this->h = false;
        this->crc = false;
        this->de = false;
    }

    void setSpreadingFactor(uint8_t spreadingFactor) {
        this->sf = spreadingFactor;
    }

    void setBandwidth(uint16_t bandWidth) { this->bw = bandWidth; }

    void setCodeRate(uint8_t codeRate) { this->cr = 4 / (codeRate + 4); }

    void setHasExplicitHeader(boolean hasExplicitHeader) {
        this->h = hasExplicitHeader;
    }

    void setCRC(boolean hasCRC) { this->crc = hasCRC; }

    void setLowDataRateEnabled(boolean lowDataRateEnabled) {
        this->de = lowDataRateEnabled;
    }

    void setPreambleLength(uint8_t preambleLength) {
        this->preambleLength = preambleLength;
    }

    // returns airtime in ms
    float getAirtime(uint8_t payloadLength) {
        // Calculation based on:
        // https://www.openhacks.com/uploadsproductos/loradesignguide_std.pdf
        const float tSym = pow(2, sf) / bw;
        const float tPreamble = (preambleLength + 4.25) * tSym;
        float payloadSymbNb = ceil((8 * payloadLength - 4 * sf + 44 - 20 * h) /
                                   (4 * (sf - 2 * de))) *
                              (cr + 4);
        payloadSymbNb = payloadSymbNb > 0 ? payloadSymbNb : 0;
        payloadSymbNb += 8;

        const float tPayload = payloadSymbNb * tSym;
        return tPayload + tPreamble;
    }
};
extern LoRaAirtime LoRaCalc;