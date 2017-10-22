/*
 * Project WifiSense_GWY
 * Description:
 * Author:
 * Date:
 */

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


// MQTT

void callback(char* topic, byte* payload, unsigned int length);

MQTT client("broker.mqttdashboard.com", 1883, callback);

// recieve message
void callback(char* topic, byte* payload, unsigned int length) {
    char p[length + 1];
    memcpy(p, payload, length);
    p[length] = NULL;

}


 void setup() {

     Serial.begin(9600);
     Serial1.begin(9600);
     xbee.setSerial(Serial1);

     // connect to the MQTT server
     client.connect("ivanlab_particle_1");

     // publish/subscribe
     if (client.isConnected()) {
         client.publish("ivanlab/particle","Ivanlab_Particle 1 ON");
         client.subscribe("inTopic/particle");
     } else Serial.print("MQTT connection error");

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
                 payload[i]=(rx.getData(i));
             }

            //Publish Payload to MQTT
             if (!client.isConnected()) client.connect("ivanlab_particle_1");

               char t1 = rx.getData(3);
               char t2 = rx.getData(4);

               String topic ="ivanlab/";
               topic += t1;
               topic += t2;
               Serial.print ("Topic=");
               Serial.println (topic);

               String macString="";
               for (int i=8;i<14;i++) macString+=String(rx.getData(i), HEX);

               Serial.print ("mac=");
               Serial.println (macString);
               macString+=":";

               char rssi = (~rx.getData(14)+1);
               //Serial.println (-rssi, DEC);
               macString+=String (-rssi, DEC);
               Serial.println(macString);

             client.publish(topic,macString);

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

     if (client.isConnected())
         client.loop();


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
