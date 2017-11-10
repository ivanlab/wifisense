// based on Ray Burnette 20161013 work
// added XBEE transport by Ivan Padilla 20171001
// added MQTT-SN encapsulation by Ivan Padilla 20171001


#include <SPI.h>
#include <WiFi.h>
#include <TinyGPS++.h>                                  // Tiny GPS Plus Library
#include <SoftwareSerial.h>
#include "./functions2.h"

#define disable 0
#define enable  1

unsigned int channel = 1;



static const uint32_t GPSBaud = 9600;                   // Ublox GPS default Baud Rate is 9600
//gpio.mode(5, gpio.OUTPUT);
//gpio.write(5, gpio.HIGH);
unsigned long gpsTimer = millis();



void setup() {
  Serial.begin(57600);
  Serial.printf("\n\nSDK version:%s\n\r", system_get_sdk_version());
  Serial.println(F("ESP8266 mini-sniff - MQTT version"));
  Serial.println(F("Type:   /-------MAC------/-----WiFi Access Point SSID-----/  /----MAC---/  Chnl  RSSI"));

  Serial1.begin(9600);
  ss.begin(GPSBaud);

  wifi_set_opmode(STATION_MODE);            // Promiscuous works only with station mode
  wifi_set_channel(channel);
  wifi_promiscuous_enable(disable);
  wifi_set_promiscuous_rx_cb(promisc_cb);   // Set up promiscuous callback
  wifi_promiscuous_enable(enable);
}

void loop() {

  channel = 1;
  wifi_set_channel(channel);
  while (true) {
    nothing_new++;                          // Array is not finite, check bounds and adjust if required
    if (nothing_new > 200) {
      nothing_new = 0;
      channel++;
      if (channel == 15) break;             // Only scan channels 1 to 14
      wifi_set_channel(channel);
    }
    delay(1);  // critical processing timeslice for NONOS SDK! No delay(0) yield()
    // Press keyboard ENTER in console with NL active to repaint the screen

    readGps(500);

    if (millis()-gpsTimer>15000) {
    lat=gps.location.lat();
    lon=gps.location.lng();
    gpsTimer=millis();
    Serial.print("Position: ");
    Serial.print(lon);
    Serial.print("/");
    Serial.println(lat);
    }

  }
}
