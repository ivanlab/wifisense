/*
 * Project WifiSense_GWY
 * Description:
 * Author: Ivan Padilla
 * Date:2017.10
 */


 #include <XBee.h>
 #include <MQTT.h>
 #include <ArduinoJson.h>
 #include <SparkTime.h>
 #include <Particle.h>
 #include <CellularHelper.h>

 // Set your 3rd-party SIM APN here
 // https://docs.particle.io/reference/firmware/electron/#setcredentials-
 //STARTUP(cellular_credentials_set("isp.mymeteor.ie", "my", "isp", NULL));
 STARTUP(cellular_credentials_set("internet", "", "", NULL));

SerialLogHandler logHandler;


 XBee xbee = XBee();

 // payload array for xbee transmission
 byte payload[24] ;


 // SH + SL Address of receiving XBee, case we need to send something
 XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x40A945EC);
 ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));

 // XBEE TX Status Response
 ZBTxStatusResponse txStatus = ZBTxStatusResponse();

 //XBEE RX Message & Status
 ZBRxResponse rx = ZBRxResponse();
 ModemStatusResponse msr = ModemStatusResponse();


// MQTT

void callback(char* topic, byte* payload, unsigned int length);

MQTT client("ivanlab.org", 1883, callback);

// recieve message
void callback(char* topic, byte* payload, unsigned int length) {
    char p[length + 1];
    memcpy(p, payload, length);
    p[length] = NULL;

}


//RTC
UDP UDPClient;
SparkTime rtc;
String timeStr;
unsigned long currentTime;

//JSON
StaticJsonBuffer<200> jsonBuffer;
JsonObject& root = jsonBuffer.createObject();

 void setup() {

     // Serial for Debug, Serial1 for Xbee
     Serial.begin(9600);
     Serial1.begin(9600);

     Log.info("manufacturer=%s", CellularHelper.getManufacturer().c_str());
     Log.info("model=%s", CellularHelper.getModel().c_str());
     Log.info("firmware version=%s", CellularHelper.getFirmwareVersion().c_str());
     Log.info("ordering code=%s", CellularHelper.getOrderingCode().c_str());
     Log.info("IMEI=%s", CellularHelper.getIMEI().c_str());
     Log.info("IMSI=%s", CellularHelper.getIMSI().c_str());
     Log.info("ICCID=%s", CellularHelper.getICCID().c_str());

     // RTC
     rtc.begin(&UDPClient, "0.pool.ntp.org");
     rtc.setTimeZone(+1); // gmt offset

     // XBEE
     xbee.setSerial(Serial1);

     // MQTT
     client.connect("ivanlab_particle_1");

     if (client.isConnected()) {
         client.publish("ivanlab/particle_gwy","Gateway_Particle 1 Connected!");
         client.subscribe("ivanlab/particle_gwy");
     } else Serial.print("MQTT connection error");

 }

 void loop() {

    xbee.readPacket();

     //Get XBEE packet

     if (xbee.getResponse().isAvailable()) {
       // got something
       if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
         // got a zb rx packet
         Serial.println("got a new report");
         // now fill our zb class in 'rx'
         xbee.getResponse().getZBRxResponse(rx);
         // process rx to find content
         if (rx.getOption() == ZB_PACKET_ACKNOWLEDGED) {
             // the sender got an ACK
             Serial.println ("It has been correctly ACK'ed");
         } else {
             // we got it (obviously) but sender didn't get an ACK
             Serial.println ("Sender didn't get an ACK from us");
         }
         // Extract Packet Containt

             String remoteSensorString = String (rx.getRemoteAddress16(), HEX);
             char remoteSensorChar[5];

             remoteSensorString.toCharArray(remoteSensorChar,5);
             Serial.print ("Source sensor = ");
             Serial.println(remoteSensorString);

             /*for (int i=0;i<24;i++) {
                 payload[i]=(rx.getData(i));
             }
             */

            //Build Payload > JSON & Publish to MQTT
             if (!client.isConnected()) client.connect("ivanlab_particle_1");

               // 1 - Extract Topic from SN message
               char t1 = rx.getData(3);
               char t2 = rx.getData(4);
               //Compose Topic String
               String topic ="ivanlab/";
               topic += t1;
               topic += t2;
               Serial.print ("Topic = ");
               Serial.println (topic);

               // 2 - Extract and compose Mac address from SN
               String macString="";
               char mac[18];
               for (int i=8;i<14;i++) {
                 macString+=String(rx.getData(i), HEX);
                 macString+=":";
               }
               macString.toCharArray(mac,18);
               Serial.print ("mac = ");
               Serial.println (macString);

               // 3 - Extract and add RSSI
               char rssi = (~rx.getData(14)+1);
               char rssiChar[4];
               String rssiString = String (-(rssi), DEC);
               rssiString.toCharArray(rssiChar,4);
               Serial.print ("RSSI = ");
               Serial.println(rssiString);

               /*Get the Timestamp - NTP
               currentTime = rtc.now();
               timeStr = rtc.yearString(currentTime);
               timeStr+="-";
               timeStr+= rtc.monthString(currentTime);
               timeStr+="-";
               timeStr+= rtc.dayString(currentTime);
               timeStr+="T";
               timeStr+= rtc.hourString(currentTime);
               timeStr+=":";
               timeStr+= rtc.minuteString(currentTime);
               timeStr+=":";
               timeStr+= rtc.secondString(currentTime);
               timeStr+="+01:00";
               Serial.print ("TimeStamp = ");
               Serial.println(timeStr);
               char timeChar[26];
               timeStr.toCharArray(timeChar,26);
               */

               // Get the timestamp - Particle.Time -> UNIX
               //uint32_t now_time = tmConvert_t(Time.year(), Time.month(), Time.day(), Time.hour(), Time.minute(), Time.second());
               uint32_t now_time = rtc.now() - 2208988803; // 70 years in seconds, including leap yeas
               Serial.print ("TimeStamp UNIX = ");
               Serial.println(String(now_time));
               //Serial.print ("TimeStamp SPARK = ");
               //Serial.println(String(currentTime));

               // Extract GPS position
               char longitude[9];
               char latitude[9];

               union {
                byte asBytes[4];
                float asFloat;
              } longi;

              union {
                byte asBytes[4];
                float asFloat;
              } lati;

              for(int i=15;i<19;i++) {
                longi.asBytes[i-15]=rx.getData(i);
              }
              for(int i=19;i<23;i++) {
                lati.asBytes[i-19]=rx.getData(i);
              }

               String lon = String(longi.asFloat,4);
               String lat = String(lati.asFloat,4);
               lon.toCharArray(longitude,9);
               lat.toCharArray(latitude,9);

               Serial.print ("Position = ");
               Serial.print (lon);
               Serial.print (",");
               Serial.println (lat);

               //Encapsulate in JSON
               {
                 StaticJsonBuffer<300> jsonBuffer;
                 JsonObject& root = jsonBuffer.createObject();
                 root["sensor"] = remoteSensorChar;
                 root["timestamp"] = now_time;
                 root["lon"] = longitude;
                 root["lat"] = latitude;
                 root["mac"] = mac;
                 root["rssi"] = rssiChar;

                 root.printTo(Serial);
                 Serial.println();
                 root.prettyPrintTo(Serial);

                 //Publish the MQTT payload
                 char jsonChar[200];
                 root.printTo(jsonChar);
                 client.publish(topic,jsonChar);
               }

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
     //Check if there is any downstream MQTT message for me
     if (client.isConnected())
         client.loop();
 }

/*
 time_t tmConvert_t(int YYYY, byte MM, byte DD, byte hh, byte mm, byte ss)
{
  struct tm t;
  t.tm_year = YYYY-1900;
  t.tm_mon = MM - 1;
  t.tm_mday = DD;
  t.tm_hour = hh;
  t.tm_min = mm;
  t.tm_sec = ss;
  t.tm_isdst = 0;
  time_t t_of_day = mktime(&t);
  return t_of_day;
}
*/
