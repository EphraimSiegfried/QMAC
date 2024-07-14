#include <LoRa.h>

#define SS   18  // GPIO18 -- SX1278's CS
#define RST  14  // GPIO14 -- SX1278's RESET
#define DI0  26  // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND 868E6

void setup() {
    Serial.begin(9600);
    while (!Serial);

    LoRa.setPins(SS, RST, DI0);  // set CS, reset, IRQ pin
    if (!LoRa.begin(BAND)) {
        Serial.println("Starting LoRa failed!");
        while (1);
    }
    // Uncomment the next line to disable the default AGC and set LNA gain,
    // values between 1 - 6 are supported LoRa.setGain(6);

    // put the radio into receive mode
    LoRa.receive();
    Serial.println("Listening...");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.println("Received packet:");

    while (LoRa.available()) {
      int byteReceived = LoRa.read();
      char buffer[10];
      sprintf(buffer, "0x%02X %c", byteReceived, (char)byteReceived);
      Serial.println(buffer);
    }
    Serial.print("RSSI: ");
    Serial.println(LoRa.packetRssi());
  }
}
