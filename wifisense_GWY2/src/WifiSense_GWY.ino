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

// Google Maps integration with Particle Cloud
GoogleMapsDeviceLocator locator;

// Battery initialization
SerialLogHandler logHandler;                  // log debug messages to console (Serial)
double batteryLife = 0;                       // battery related
FuelGauge fuel;

// Serial link to NodeMCU initial parameters
const byte numChars = 40;                     // Size of message from NodeMCU
byte receivedChars[numChars];                 // Array holding message coming from NodeMCU
boolean newData = false;

// MQTT Functions
byte payload[24] ;
int keepalive = 5;
void callback(char* topic, byte* payload, unsigned int length);
MQTT client("ivanlab.org", 1883, keepalive, callback);

//RTC Parameters
UDP UDPClient;
SparkTime rtc;
String timeStr;
unsigned long currentTime;

//JSON Functions
StaticJsonBuffer<200> jsonBuffer;

void setup() {

 // initialization Serial for Debug, Serial1 for Xbee
 Serial.begin(9600);
 Serial1.begin(9600);
 Serial.println("Particle Up");

 locator.withLocatePeriodic(30);
 pinMode(D6, OUTPUT);                               //Reset line for NodeMCU
 digitalWrite(D6, HIGH);

 //get battery life on startup
  batteryLife = fuel.getSoC();
  Particle.variable("batteryLife", batteryLife);
  Particle.function("getSoC", getBatteryLife);
  String battery="Battery level: "+String(getBatteryLife("get"))+"% ";
  String batteryV="Battery Volt: "+String(fuel.getVCell()) + "V ";

 // MQTT initialization
 client.connect("ivanlab_particle_1");
 if (client.isConnected()) {
     client.publish  ("ivanlab/particle_gwy","Particle-1 Connected to MQTT");
     client.subscribe("ivanlab/particle_gwy");
     Serial.println("MQTT Up");
     Particle.publish("MQTT Connection UP");
 } else {
   Serial.print ("MQTT connection error");
   Particle.publish("MQTT Connection ERROR");
 }
 Serial.print ("MQTT keepalive: ");
 Serial.println(keepalive);

// Log battery status to Console, MQTT and Particle cloud
 Serial.print(battery);                             // Log battery level to Serial
 Serial.print(batteryV);
 Particle.publish(battery);                         // Log battery to Particle cloud
 Particle.publish(batteryV);
 client.publish  ("ivanlab/particle_gwy",battery);  // Log battery to MQTT cloud
 client.publish  ("ivanlab/particle_gwy",batteryV);

 digitalWrite(D6, LOW);                             // Reset NodeMCU
 delay(100);
 digitalWrite(D6, HIGH);
}

void loop() {
  recvWithStartEndMarkers();                        // Read Serial link from NodeMCU
  publishNewData();                                 // Write to MQTT
  client.loop();                                    // Listen and Keepalive MQTT

  if (!client.isConnected()) {                      // Check if MQTT still up
    client.connect("ivanlab_particle_1");
    Serial.print("...reconnecting MQTT");
    client.publish  ("ivanlab/particle_gwy","Particle1 reconnecting...");
  }
}

// ****** FUNCTIONS *********

// MQTT Callback for subscription received messages
void callback(char* topic, byte* payload, unsigned int length) {
    char p[length + 1];
    memcpy(p, payload, length);
    p[length] = NULL;

}

// Function get battery life upon request
int getBatteryLife(String command) {
//Set Battery Life Variable with current battery life value
  batteryLife = fuel.getSoC();
  return (int)batteryLife;
}

// Serial communications with NodeMCU functions
void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = 254;
  char endMarker = 255;
  byte rc;

  while (Serial1.available() > 0 && newData == false) {
    rc = Serial1.read();
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

// Time format conversion functions
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

// Read Serial line and publish new data if exists
void publishNewData(){
  if (newData == true) {
    // Get the timestamp - Particle.Time -> UNIX
    uint32_t now_time = tmConvert_t(Time.year(), Time.month(), Time.day(), Time.hour(), Time.minute(), Time.second());
    // print received message through console
    for (int i=0; i<24;i++) Serial.print(receivedChars[i], HEX);
    Serial.println();

     // 1 - Extract Topic from SN message
     char t1 = receivedChars[3];
     char t2 = receivedChars[4];
     String topic ="ivanlab/";
     topic += t1;
     topic += t2;

     Serial.print("topic=");
     Serial.println(t1);

     if(t1=='S'){       //if Message is written to the DATA topic

       // 2 - Extract Mac address from SN
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

       // 3 - Extract RSSI
       char rssi = (~receivedChars[14]+1);
       char rssiChar[4];
       String rssiString = String (-(rssi), DEC);
       rssiString.toCharArray(rssiChar,4);

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

     //Encapsulate in JSON
      {
       StaticJsonBuffer<200> jsonBuffer;
       JsonObject& root = jsonBuffer.createObject();
       root["sensor"] = 1;
       root["timestamp"] = now_time;
       root["lon"] = longitude;
       root["lat"] = latitude;
       root["mac"] = mac;
       root["rssi"] = rssiChar;
       //root.prettyPrintTo(Serial);

       //Publish the MQTT payload
       char jsonChar[200];
       root.printTo(jsonChar);
       client.publish(topic,jsonChar);
       Particle.publish("wifisense",jsonChar);
      }
    }else{                                // it is a control message?
      if(t1=='C'){
        char message[15];
        for (int i=8; i<23;i++) message[i-8]=receivedChars[i];
        message[23]='/n';
        Serial.print(message);
        StaticJsonBuffer<200> jsonBuffer;
        JsonObject& root = jsonBuffer.createObject();
        root["sensor"] = 1;
        root["timestamp"] = now_time;
        root["Message"] = message;
        root.prettyPrintTo(Serial);

        //Publish the MQTT payload
        char jsonChar[200];
        root.printTo(jsonChar);
        client.publish(topic,jsonChar);
        Particle.publish("wifisense",jsonChar);
      }
    }
   newData = false;
  }
}
