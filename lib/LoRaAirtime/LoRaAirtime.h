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
    LoRaAirtime(uint8_t spreadingFactor, uint16_t bandWidth, uint8_t codeRate,
                uint8_t preambleLength, boolean hasExplicitHeader,
                boolean hasCRC, boolean lowDataRateEnabled) {
        this->sf = spreadingFactor;
        this->bw = bandWidth;
        this->cr = 4 / (codeRate + 4);
        this->h = hasExplicitHeader;
        this->crc = hasCRC;
        this->de = lowDataRateEnabled;
        this->preambleLength = preambleLength;
    }

    // returns airtime in ms
    float calcAirtime(uint8_t payloadLength) {
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