/*
 * Project WifiSense_GWY
 * Description:
 * Author: Ivan Padilla
 * Date:2017.10
 */


 #include <MQTT.h>
 #include <ArduinoJson.h>
 #include <SparkTime.h>
 #include <Particle.h>
 #include <CellularHelper.h>
 #include <google-maps-device-locator.h>


 // Set your 3rd-party SIM APN here
 // https://docs.particle.io/reference/firmware/electron/#setcredentials-
 //STARTUP(cellular_credentials_set("isp.mymeteor.ie", "my", "isp", NULL));
 STARTUP(cellular_credentials_set("internet", "", "", NULL));

  GoogleMapsDeviceLocator locator;
  SerialLogHandler logHandler;


const byte numChars = 40;
byte receivedChars[numChars];
boolean newData = false;
byte payload[24] ;
double batteryLife = 0;
FuelGauge fuel;

// MQTT

int keepalive = 5;
void callback(char* topic, byte* payload, unsigned int length);
MQTT client("broker.mqttdashboard.com", 1883, keepalive, callback);
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
     locator.withLocatePeriodic(30);

     Serial.println("Particle Up");

     //get battery life on startup
      batteryLife = fuel.getSoC();
      Particle.variable("batteryLife", batteryLife);
      Particle.function("getSoC", getBatteryLife);

      Serial.print("Battery level: ");
      Serial.print(getBatteryLife("get"));
      Serial.println(" % ");

     // MQTT
     client.connect("ivanlab_particle_1");

     if (client.isConnected()) {
         client.publish  ("ivanlab/particle_gwy","Gateway_Particle 1 Connected!");
         client.subscribe("ivanlab/particle_gwy");
         Serial.println("MQTT Up");
     } else Serial.print ("MQTT connection error");
     Serial.print ("keepalive: ");
     Serial.println(keepalive);

 }

 void loop() {

  recvWithStartEndMarkers();
  publishNewData();
  client.loop();
  if (!client.isConnected()) {
    client.connect("ivanlab_particle_1");
    Serial.print("...reconnecting MQTT");
    client.publish  ("ivanlab/particle_gwy","Particle1 reconnecting...");
  }

}

//function get battery life upon request
int getBatteryLife(String command) {
//Set Battery Life Variable with current battery life value
  batteryLife = fuel.getSoC();
  return (int)batteryLife;
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = 254;
    char endMarker = 255;
    byte rc;

 // if (Serial.available() > 0) {
    while (Serial1.available() > 0 && newData == false) {
        rc = Serial1.read();
        //Serial.print(rc, HEX);
        //Serial.print(".");

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void showNewData() {
    if (newData == true) {
        Serial.println("This just in ... ");
        for (int i=0; i<24;i++){
          Serial.print(receivedChars[i], HEX);
        }
        Serial.println("");

        newData = false;
    }
}

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

void publishNewData(){

  if (newData == true) {
    //Serial.print("MQTT Connected? ");
    //Serial.println(client.isConnected());
     // 1 - Extract Topic from SN message
     char t1 = receivedChars[3];
     char t2 = receivedChars[4];
     //Compose Topic String
     String topic ="ivanlab/";
     topic += t1;
     topic += t2;
     //Serial.print ("Topic = ");
     //Serial.println (topic);

     // 2 - Extract and compose Mac address from SN
     String macString="";
     char mac[18];
     for (int i=8;i<14;i++) {
         if (receivedChars[i]>9) {
           macString+=String(receivedChars[i], HEX);
       } else {
         macString+="0";
         macString+=String(receivedChars[i], HEX);
       }
     macString+=":";
     }
     macString.toCharArray(mac,18);
     //Serial.print ("mac = ");
     //Serial.println (macString);

     // 3 - Extract and add RSSI
     char rssi = (~receivedChars[14]+1);
     char rssiChar[4];
     String rssiString = String (-(rssi), DEC);
     rssiString.toCharArray(rssiChar,4);
     //Serial.print ("RSSI = ");
     //Serial.println(rssiString);

     // Get the timestamp - Particle.Time -> UNIX
     uint32_t now_time = tmConvert_t(Time.year(), Time.month(), Time.day(), Time.hour(), Time.minute(), Time.second());
     //Serial.print ("TimeStamp UNIX = ");
     //Serial.println(String(now_time));


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
      longi.asBytes[i-15]=receivedChars[i];
    }
    for(int i=19;i<23;i++) {
      lati.asBytes[i-19]=receivedChars[i];
    }

     String lon = String(longi.asFloat,4);
     String lat = String(lati.asFloat,4);
     lon.toCharArray(longitude,9);
     lat.toCharArray(latitude,9);

     //Serial.print ("Position = ");
     //Serial.print (lon);
     //Serial.print (",");
     //Serial.println (lat);

   //Encapsulate in JSON
    {
     StaticJsonBuffer<300> jsonBuffer;
     JsonObject& root = jsonBuffer.createObject();
     root["sensor"] = 1;
     root["timestamp"] = now_time;
     root["lon"] = longitude;
     root["lat"] = latitude;
     root["mac"] = mac;
     root["rssi"] = rssiChar;

     for (int i=0; i<24;i++) Serial.print(receivedChars[i], HEX);
     Serial.println();
     //root.printTo(Serial);
     //Serial.println();
     //root.prettyPrintTo(Serial);

     //Publish the MQTT payload

     char jsonChar[200];
     root.printTo(jsonChar);
     client.publish(topic,jsonChar);
     Particle.publish("wifisense",jsonChar);
    }

     //Serial.println("");
     newData = false;
  }
}
