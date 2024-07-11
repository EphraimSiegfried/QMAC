/*****************************************
* ESP32 GPS VKEL 9600 Bds
******************************************/

#include <TinyGPS++.h>                       
#define GPS_RX_PIN 34
#define GPS_TX_PIN 12

TinyGPSPlus gps;                            
HardwareSerial GPSSerial1(1);                 

static void smartDelay(unsigned long ms);

void setup()
{
  Serial.begin(115200);
  GPSSerial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);   //17-TX 18-RX
  delay(1500);
}

void loop()
{
  Serial.print("Latitude  : ");
  Serial.println(gps.location.lat(), 5);
  Serial.print("Longitude : ");
  Serial.println(gps.location.lng(), 4);
  Serial.print("Satellites: ");
  Serial.println(gps.satellites.value());
  Serial.print("Altitude  : ");
  Serial.print(gps.altitude.feet() / 3.2808);
  Serial.println("M");
  Serial.print("Time      : ");
  Serial.print(gps.time.hour());
  Serial.print(":");
  Serial.print(gps.time.minute());
  Serial.print(":");
  Serial.println(gps.time.second());
  Serial.println("**********************");

  smartDelay(1000);                                      

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}

static void smartDelay(unsigned long ms)                
{
  unsigned long start = millis();
  do {
    while (GPSSerial1.available())
      gps.encode(GPSSerial1.read());
  } while (millis() - start < ms);
}
