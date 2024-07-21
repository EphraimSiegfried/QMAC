# Sleepy

## How to run

- Install platformio or use the nix dev shell by running `nix develop`
- Flash the device with `pio run -t upload`
- Observe the output with `pio device monitor`

## Common Mac Protocols

### Synchronous

These protocols require time synchronization

- [S-MAC](https://en.wikipedia.org/wiki/Sensor_Media_Access_Control)
- [T-MAC](https://dl.acm.org/doi/10.1145/958491.958512)
- [TSMP](https://web.archive.org/web/20061007051054/http://www.dustnetworks.com/docs/TSMP_Whitepaper.pdf)

### Asynchronous

No synchronization between nodes needed

#### Sender initiated

Nodes wake-up periodically and sense the channel for ongoing transmission

- Contiki-Mac

  ...

#### Receiver initiated

The responsibility of initiating the transmission is given to the receiver.

- Koala

  ...

## Useful Links

### Code

- [TTGO-T-BEAM Repository with code examples](https://github.com/LilyGO/TTGO-T-Beam)
- [LoRa Library](https://github.com/sandeepmistry/arduino-LoRa/tree/master)

### Theory

- [Contiki-Mac Duty Cycling (gives great overview over MAC protocols)](https://manuscriptlink-society-file.s3-ap-northeast-1.amazonaws.com/kics/conference/icufn2021/abs/5B-3.pdf)
- [TTGO-T-BEAM Specs](https://www.lilygo.cc/products/t-beam-v1-1-esp32-lora-module)
- [LoRa Duty Cycle explained](https://www.thethingsnetwork.org/docs/lorawan/duty-cycle/)
