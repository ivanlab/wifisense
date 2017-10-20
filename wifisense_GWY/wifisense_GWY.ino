// This #include statement was automatically added by the Particle IDE.
#include <XBee.h>

// This #include statement was automatically added by the Particle IDE.
#include <MQTT.h>

// xbee object
XBee xbee = XBee();

// payload array for xbee transmission
byte payload[15] ;

// TX objects:

// SH + SL Address of receiving XBee
XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x40A945EC);
//TX
ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
// TX Status Response
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

// Response Objects:

//RX
XBeeResponse response = XBeeResponse();

//Response Status
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();


void setup() {

    Serial.begin(9600);
    Serial1.begin(9600);
    xbee.setSerial(Serial1);

    Serial.println ("Todo ok hasta aqui");

}

void loop() {


   xbee.readPacket();

    if (xbee.getResponse().isAvailable()) {
      // got something
      if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
        // got a zb rx packet
        Serial.println("got a new report");
        // now fill our zb rx class
        xbee.getResponse().getZBRxResponse(rx);

        if (rx.getOption() == ZB_PACKET_ACKNOWLEDGED) {
            // the sender got an ACK
            Serial.println ("Sender is informed about it");
        } else {
            // we got it (obviously) but sender didn't get an ACK
            Serial.println ("Sender didn't get an ACK");
        }
        // Extract Packet Containt
            Serial.print ("Reported by:");
            Serial.println(rx.getRemoteAddress16(), HEX);

            //int i = 0;
            for (int i=0;i<15;i++) {
                Serial.print(rx.getData(i));
                Serial.print(".");
                payload[i]=(rx.getData(i));
            }

            //char rssi = (~rx.getData(6)+1);
            //Serial.println (-rssi, DEC);

            Serial.println("\n");

      } else if (xbee.getResponse().getApiId() == MODEM_STATUS_RESPONSE) {
        xbee.getResponse().getModemStatusResponse(msr);
        // the local XBee sends this response on certain events, like association/dissociation

        if (msr.getStatus() == ASSOCIATED) {
          // yay this is great.  flash led
          Serial.print ("local Xbee got associated");
        } else if (msr.getStatus() == DISASSOCIATED) {
          // this is awful.. flash led to show our discontent
          Serial.print ("local Xbee got DISassociated :(");
        } else {
          // another status
          Serial.print ("Don't really know whats happening with the modem status");
        }
      } else {
      	// not something we were expecting
        Serial.print ("I got something not expected...moving on");
      }
    } else if (xbee.getResponse().isError()) {
      Serial.print("Error reading packet.  Error code: ");
      Serial.println(xbee.getResponse().getErrorCode());
    }


    delay(1000);

}

void flashLed(int pin, int times, int wait) {

  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(wait);
    digitalWrite(pin, LOW);

    if (i + 1 < times) {
      delay(wait);
    }
  }
}
