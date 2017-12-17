// based on Ray Burnette 20161013 work
// added XBEE transport by Ivan Padilla 20171001
// added MQTT-SN encapsulation by Ivan Padilla 20171001


#include <SPI.h>
#include <WiFi.h>
#include "./functions2.h"
#define disable 0
#define enable  1

unsigned int channel = 1;
unsigned long time = millis();                    // Counter to reset client count


void setup() {

  Serial.begin(115200);                  // Console debug
  Serial1.begin(9600);                   // Link to Particle LTE
  Serial.printf("\n\nSDK version:%s\n\r", system_get_sdk_version());
  Serial.println(F("ESP8266 mini-sniff - MQTT version"));

  byte message[15]="NodeMCU up";
  sendMessage("C1", message, 10);

  // Define WiFi scanning parameters
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
    nothing_new++;                     // Array is not finite, check bounds and adjust if required
    if (nothing_new > 200) {
      nothing_new = 0;
      channel++;
      if (channel == 15) break;        // Only scan channels 1 to 14
      wifi_set_channel(channel);
    }
    delay(1);                         // critical processing timeslice for NONOS SDK! No delay(0) yield()
    if (millis()>time+30e3) {
      Serial.println("Sending...");
      sendArrayOfClients();
      time = millis();
    }
  }
}
// test
