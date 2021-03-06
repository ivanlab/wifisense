// based on Ray Burnette 20161013 work ( using Arduino 1.6.12 )
// added XBEE transport by Ivan Padilla 20171001
// added MQTT-SN encapsulation by Ivan Padilla 20171001
//

// This-->tab == "functions.h"

// Expose Espressif SDK functionality
extern "C" {
#include "user_interface.h"
  typedef void (*freedom_outside_cb_t)(uint8 status);
  int  wifi_register_send_pkt_freedom_cb(freedom_outside_cb_t cb);
  void wifi_unregister_send_pkt_freedom_cb(void);
  int  wifi_send_pkt_freedom(uint8 *buf, int len, bool sys_seq);
}

#include <WiFi.h>
#include <XBee.h>
//#include <Printers.h>
#include "./structures.h"


//Wifi Sensor Parameters
#define MAX_APS_TRACKED 100
#define MAX_CLIENTS_TRACKED 200

XBee xbee = XBee();

//MQTT-SN Parameters

#define MQTT_SN_MAX_PACKET_LENGTH     (255)
#define MQTT_SN_TYPE_PUBLISH          (0x0C)
#define MQTT_SN_FLAG_QOS_N1           (0x3 << 5)
#define MQTT_SN_FLAG_RETAIN           (0x1 << 4)
#define MQTT_SN_TOPIC_TYPE_SHORT      (0x02)


// MQTT-SN Encapsulation

void sendMessage(const char topic[2], byte message[15], bool retain=false)
{
  uint8_t payload[24];

  XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x40a6a3ee);
  ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
  ZBTxStatusResponse txStatus = ZBTxStatusResponse();

  payload[0] = 24; // header +sizeof message
  payload[1] = MQTT_SN_TYPE_PUBLISH;
  payload[2] = MQTT_SN_FLAG_QOS_N1 | MQTT_SN_TOPIC_TYPE_SHORT;
  if (retain) {
    payload[2] |= MQTT_SN_FLAG_RETAIN;
  }
  payload[3] = topic[0];
  payload[4] = topic[1];
  payload[5] = 0x00;  // Message ID High
  payload[6] = 0x00;  // message ID Low
  payload[7] = 0x07;  // size of header

  for (int i=8;i<24;i++){
    payload[i] = message[i-8];
    //memcpy();
  }

  xbee.send(zbTx);
}

// Get GPS position

float lon = 3.1234;
float lat = 41.1234;


// WiFi Sense functions

beaconinfo aps_known[MAX_APS_TRACKED];                    // Array to save MACs of known APs
int aps_known_count = 0;                                  // Number of known APs
int nothing_new = 0;
clientinfo clients_known[MAX_CLIENTS_TRACKED];            // Array to save MACs of known CLIENTs
int clients_known_count = 0;                              // Number of known CLIENTs

unsigned long time = millis();



// STORE Known APs

int register_beacon(beaconinfo beacon)
{
  int known = 0;   // Clear known flag
  for (int u = 0; u < aps_known_count; u++)
  {
    if (! memcmp(aps_known[u].bssid, beacon.bssid, ETH_MAC_LEN)) {
      known = 1;
      break;
    }   // AP known => Set known flag
  }
  if (! known)  // AP is NEW, copy MAC to array and return it
  {
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

int register_client(clientinfo ci)
{
  int known = 0;   // Clear known flag
  //Serial.print(millis());
  //Serial.printf("%s","-" );
  //Serial.println(time);

// Clear known clients every minute

  if (millis()-time > 60000) {
    clients_known_count = 0;
    time = millis();
  }

  for (int u = 0; u < clients_known_count; u++)
  {
    if (! memcmp(clients_known[u].station, ci.station, ETH_MAC_LEN)) {
      known = 1;
      break;
    }
  }
  if (! known)
  {
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

// Get/PUBLISH/Print CLIENT INFO

void print_client(clientinfo ci)
{
  int u = 0;
  byte message [7+8];
  int known = 0;   // Clear known flag
  if (ci.err != 0) {
    // nothing
  } else {
    Serial.printf("DEVICE: ");
    for (int i = 0; i < 6; i++) {
          Serial.printf("%02x", ci.station[i]);
          message[i] = ci.station[i];
    }
     message[6] = ci.rssi;

    Serial.printf(" ==> ");

    //add GPS to payload

    for (int i=0;i<4;i++) message[7+i] = (*((int*)&lon) >> 8 * i) & 0xFF;
    for (int i=0;i<4;i++) message[11+i] = (*((int*)&lat) >> 8 * i) & 0xFF;

    union {
      byte asBytes[4];
      float asFloat;
    } longitude;

    union {
      byte asBytes[4];
      float asFloat;
    } latitude;

      for (int i=0;i<4;i++)longitude.asBytes[i]=message[7+i];
      for (int i=0;i<4;i++)latitude.asBytes[i]=message[11+i];


        Serial.print("lon=" );
        for (int i=0;i<4;i++)Serial.print(message[7+i], HEX);
        Serial.print("->");
        Serial.print(longitude.asFloat);


        Serial.print(",lat=" );
        for (int i=0;i<4;i++)Serial.print(message[11+i], HEX);
        Serial.print("->");
        Serial.print(latitude.asFloat);
        Serial.println("");


    for (int i=7; i<15; i++) Serial.print(message[i],HEX);

    //PUBLISH

    sendMessage("S1",message);


    // Check connected AP data

    for (u = 0; u < aps_known_count; u++)
    {
      if (! memcmp(aps_known[u].bssid, ci.bssid, ETH_MAC_LEN)) {
        Serial.printf("[%32s]", aps_known[u].ssid);
        known = 1;     // AP known => Set known flag
        break;
      }
    }

    if (! known)  {
      Serial.printf("   Unknown/Malformed packet \r\n");
      //  for (int i = 0; i < 6; i++) Serial.printf("%02x", ci.bssid[i]);
    } else {
      Serial.printf("%2s", " ");
      for (int i = 0; i < 6; i++) Serial.printf("%02x", ci.ap[i]);
      Serial.printf("  %3d", aps_known[u].channel);
      Serial.printf("   %4d\r\n", ci.rssi);
    }
  }
}

// CALLBACK

void promisc_cb(uint8_t *buf, uint16_t len)
{
  int i = 0;
  uint16_t seq_n_new = 0;
  if (len == 12) {
    struct RxControl *sniffer = (struct RxControl*) buf;
  } else if (len == 128) {
    struct sniffer_buf2 *sniffer = (struct sniffer_buf2*) buf;
    struct beaconinfo beacon = parse_beacon(sniffer->buf, 112, sniffer->rx_ctrl.rssi);
    if (register_beacon(beacon) == 0) {
      print_beacon(beacon);
      nothing_new = 0;
    }
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
