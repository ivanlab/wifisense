// based on Ray Burnette 20161013 work ( using Arduino 1.6.12 )
// added XBEE transport by Ivan Padilla 20171001
// added MQTT-SN encapsulation by Ivan Padilla 20171001
// added GPS support by Ivan Padilla 20171101

// Expose Espressif SDK functionality
extern "C" {
#include "user_interface.h"
  typedef void (*freedom_outside_cb_t)(uint8 status);
  int  wifi_register_send_pkt_freedom_cb(freedom_outside_cb_t cb);
  void wifi_unregister_send_pkt_freedom_cb(void);
  int  wifi_send_pkt_freedom(uint8 *buf, int len, bool sys_seq);
  }

#include <WiFi.h>
#include <TinyGPS++.h>                                  // Tiny GPS Plus Library
#include <SoftwareSerial.h>
#include "./structures.h"

//Wifi Initialization Parameters
#define MAX_APS_TRACKED     100
#define MAX_CLIENTS_TRACKED 200
beaconinfo aps_known[MAX_APS_TRACKED];            // Array to save MACs of known APs
int aps_known_count = 0;                          // Number of known APs
int nothing_new = 0;
clientinfo clients_known[MAX_CLIENTS_TRACKED];    // Array to save MACs of known CLIENTs
int clients_known_count = 0;                      // Number of known CLIENTs
unsigned long time = millis();                    // Counter to reset client count

//MQTT-SN Initialization Parameters
#define MQTT_SN_MAX_PACKET_LENGTH     (255)
#define MQTT_SN_TYPE_PUBLISH          (0x0C)
#define MQTT_SN_FLAG_QOS_N1           (0x3 << 5)
#define MQTT_SN_FLAG_RETAIN           (0x1 << 4)
#define MQTT_SN_TOPIC_TYPE_SHORT      (0x02)

// GPS Initialization Parameters
static float lon = 0.0;
static float lat = 0.0;
TinyGPSPlus gps;                            //Create a GPS object
static const int RXPin = 12, TXPin = 13;    // Ublox  GPS module to pins 12 and 13
SoftwareSerial ss(RXPin, TXPin);            // The serial connection to the GPS device

// GPS Reading function
static void readGps(unsigned long ms)       // Smart delay function ensures that the gps object is being "fed".
{
  unsigned long start = millis();
    do {
      while (ss.available())gps.encode(ss.read());
    } while (millis() - start < ms);
}

// MQTT-SN Message encapsulation and sending
void sendMessage(const char topic[2], byte message[15], bool retain=false)
{
  uint8_t payload[24];                // Total size of MQTT-SN packet to be sent
  payload[0] = 24;                    // header +sizeof message
  payload[1] = MQTT_SN_TYPE_PUBLISH;
  payload[2] = MQTT_SN_FLAG_QOS_N1 | MQTT_SN_TOPIC_TYPE_SHORT;
  if (retain) {
    payload[2] |= MQTT_SN_FLAG_RETAIN;
  }
  payload[3] = topic[0];
  payload[4] = topic[1];
  payload[5] = 0x00;                  // Message ID High
  payload[6] = 0x00;                  // message ID Low
  payload[7] = 0x07;                  // size of header

  Serial.print("Topic= "); Serial.print(topic[0]); Serial.print("-");
  // Load the message in the MQTT-SN payload
    for (int i=8;i<24;i++){
      payload[i] = message[i-8];
    }

  // Serialize the MQTT-SN packet
    Serial1.write(254);               // startByte
    for (int i=0;i<24;i++){
      Serial.print(payload[i], HEX);  // Log HEX payload to console (debugging)
      Serial1.write(payload[i]);      //transmit to Particle via serial1
    }
    Serial1.write(255);               // stopByte
    Serial.println();
  }

// STORE Known APs
int register_beacon(beaconinfo beacon)
{
  int known = 0;                      // Clear known flag
  for (int u = 0; u < aps_known_count; u++){
    if (! memcmp(aps_known[u].bssid, beacon.bssid, ETH_MAC_LEN)) {
      known = 1;
      break;
    }                                 // AP known => Set known flag
  }
  if (! known){                       // AP is NEW, copy MAC to array and return it

    memcpy(&aps_known[aps_known_count], &beacon, sizeof(beacon));
    aps_known_count++;

    if ((unsigned int) aps_known_count >=
        sizeof (aps_known) / sizeof (aps_known[0]) ) {
      Serial.printf("exceeded max aps_known\n");
      aps_known_count = 0;
    }
  }
  return known;
}

//Store known clients
int register_client(clientinfo ci){
  int known = 0;   // Clear known flag

  // Clear known clients every minute
  if (millis()-time > 60000) {
    clients_known_count = 0;
    time = millis();
  }

  for (int u = 0; u < clients_known_count; u++){
    if (! memcmp(clients_known[u].station, ci.station, ETH_MAC_LEN)) {
      known = 1;
      break;
    }
  }
  if (! known){
    memcpy(&clients_known[clients_known_count], &ci, sizeof(ci));
    clients_known_count++;

    if ((unsigned int) clients_known_count >=
        sizeof (clients_known) / sizeof (clients_known[0]) ) {
      Serial.printf("exceeded max clients_known\n");
      clients_known_count = 0;
    }
  }
  return known;
}

// Print AP info on serial interface
void print_beacon(beaconinfo beacon)
{
  if (beacon.err != 0) {
    //Serial.printf("BEACON ERR: (%d)  ", beacon.err);
  } else {
    Serial.printf("BEACON: <=============== [%32s]  ", beacon.ssid);
    for (int i = 0; i < 6; i++) Serial.printf("%02x", beacon.bssid[i]);
    Serial.printf("   %2d", beacon.channel);
    Serial.printf("   %4d\r\n", beacon.rssi);
  }
}

// Print & Publish client info
void print_client(clientinfo ci)
{
  int u = 0;
  byte message [7+8];                   // MAC + RSSI + Lat/Lon
  int known = 0;                        // Clear known flag
  if (ci.err != 0) {
    // nothing
  } else {
    for (int i = 0; i < 6; i++) {
          message[i] = ci.station[i];   // Load MAC address into Message
    }
     message[6] = ci.rssi;              // Load RSSI into Message

    //add GPS position to Message - Serialize float
    for (int i=0;i<4;i++) message[7+i] = (*((int*)&lon) >> 8 * i) & 0xFF;
    for (int i=0;i<4;i++) message[11+i]= (*((int*)&lat) >> 8 * i) & 0xFF;

    sendMessage("S1",message);

  }
}

// WiFi Callback
void promisc_cb(uint8_t *buf, uint16_t len)
{
  int i = 0;
  uint16_t seq_n_new = 0;
  if (len == 12) {
    struct RxControl *sniffer = (struct RxControl*) buf;
  } else if (len == 128) {
    //Serial.printf("Becon received \n");
  } else {
    struct sniffer_buf *sniffer = (struct sniffer_buf*) buf;
    //Is data or QOS?
    if ((sniffer->buf[0] == 0x08) || (sniffer->buf[0] == 0x88)) {
      struct clientinfo ci = parse_data(sniffer->buf, 36, sniffer->rx_ctrl.rssi, sniffer->rx_ctrl.channel);
      if (memcmp(ci.bssid, ci.station, ETH_MAC_LEN)) {
        if (register_client(ci) == 0) {
          print_client(ci);
          nothing_new = 0;
        }
      }
    }
  }
}
