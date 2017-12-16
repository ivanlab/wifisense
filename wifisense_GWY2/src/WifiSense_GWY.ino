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
 #include <TinyGPS++.h>
 #include <Serial4/Serial4.h>

 // Set your 3rd-party SIM APN here
 // https://docs.particle.io/reference/firmware/electron/#setcredentials-
 //STARTUP(cellular_credentials_set("isp.mymeteor.ie", "my", "isp", NULL));
 STARTUP(cellular_credentials_set("internet", "", "", NULL));

// Google Maps integration with Particle Cloud
// GoogleMapsDeviceLocator locator;

//SerialLogHandler logHandler;                  // log debug messages to console (Serial)

// Battery initialization
double batteryLife = 0;                       // battery related
FuelGauge fuel;

// Serial link to NodeMCU initial parameters
const char numChars = 300;                  // Size of message from NodeMCU
char receivedChars[numChars];                 // Array holding message coming from NodeMCU
int lastNdx;
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
//StaticJsonBuffer<200> jsonBuffer;

// GPS parameters
static float lon = 0.0;
static float lat = 0.0;
unsigned long gpsTimer = millis();
TinyGPSPlus gps;


//                           ********* SETUP *********


void setup() {

  pinMode(D6, OUTPUT);                               //Reset line for NodeMCU
  digitalWrite(D6, LOW);
  pinMode(D5, OUTPUT);                               //Enable line for GPS
  digitalWrite(D5, LOW);

  Serial.begin(9600);                                // debug USB port
  Serial1.begin(115200);                             // NodeMCU comms port
  Serial4.begin(9600);                               // GPS Comms port
  Serial.println("Particle Up");
  Serial.println(TinyGPSPlus::libraryVersion());     // Print GPS lib version

 // locator.withLocatePeriodic(30);

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

  digitalWrite(D6, HIGH);                            // Enable NodeMCU
  digitalWrite(D5, HIGH);                            // Enable GPS
}

//                           ********* LOOP *********

void loop() {

  if (!client.isConnected()) {                      // Check if MQTT still up
    client.connect("ivanlab_particle_1");
    Serial.print("...reconnecting MQTT");
    client.publish  ("ivanlab/particle_gwy","Particle1 reconnecting...");
  }

  if(digitalRead(D5)==HIGH) {                       // Read and publish GPS
      readGps(500);
    if ( (gps.location.lat()!=0 && gps.location.lng()!=0) || (millis() - gpsTimer > 300e3)){
      lat=gps.location.lat();
      lon=gps.location.lng();
      Serial.print("Position: "); Serial.print(lon); // Log position to console
      Serial.print("/"); Serial.println(lat);
      sendPosition(lat,lon);
      digitalWrite(D5, LOW);                         // Turn off  GPS
    }
  }

  recvWithStartEndMarkers();                        // Read Serial link from NodeMCU
  if (newData) publishNewData();                                 // Write to MQTT
  client.loop();                                    // Allow to check for MQTT callback
  delay(1);
  //System.sleep(D1,RISING,300);
}

//                           ****** FUNCTIONS *********


static void readGps(unsigned long ms)       // Smart delay - gps object is being "fed".
{
  unsigned long start = millis();
    do {
      while (Serial4.available())gps.encode(Serial4.read());
    } while (millis() - start < ms);
}

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
  static char ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;
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
          lastNdx = ndx;
          ndx = 0;
          newData = true;
      }
    }
    else if (rc == startMarker) {
        recvInProgress = true;
    }
  }
}

// Time conversion functions

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

// Ascii to HEX helper
unsigned char h2d(unsigned char hex)
{
  if(hex > 0x39) hex -= 7; // adjust for hex letters upper or lower case
  return(hex & 0xf);
}

// Parse Serial line and publish new data
void publishNewData(){

  uint32_t now_time = tmConvert_t(Time.year(), Time.month(), Time.day(), Time.hour(), Time.minute(), Time.second());

  //Extract Payload size from first octect
  int payloadSize = (h2d(receivedChars[0]) << 4) | h2d(receivedChars[1]);  // convert to decimal
  Serial.print("payload size advertised="); Serial.println(payloadSize);

  // print received message to console
  Serial.print("Received chars="); Serial.println(lastNdx);
  for (int i=0; i < lastNdx; i=i+2) {
    Serial.print(receivedChars[i]);
    Serial.print(receivedChars[i+1]);
    Serial.print("|");
  }
  Serial.println();

   // 1 - Extract Topic from SN message
   char t1 = (h2d(receivedChars[6]) << 4) | h2d(receivedChars[7]);
   char t2 = (h2d(receivedChars[8]) << 4) | h2d(receivedChars[9]);
   String topic ="ivanlab/";
   topic += t1;
   topic += t2;

   Serial.print("topic=");
   Serial.println(topic);

   if(t1=='S'){       //if Message is written to the DATA topic

     int receivedClients = (payloadSize-8)/7; Serial.print("# de rcv.macs: "); Serial.println(receivedClients);

     // 2 - Extract Mac address from SN
     char mac[receivedClients][17];              //17 caracteres por cada Mac recibida
     for (int f=0;f<receivedClients;f++) {
       for (int i=0;i<17;i=i+3) {
         Serial.print("f=");Serial.print(f);Serial.print(" i=");Serial.print(i/3);
         Serial.print(" Char=");
         mac[f][i]=receivedChars[16+i/3+7*f];
         Serial.print(receivedChars[16+i/3+7*f]);
         mac[f][i+1]=receivedChars[16+i/3+7*f+1];
         Serial.println(receivedChars[16+i/3+7*f+1]);
         mac[f][i+2]=':';
        }
      }

     // 3 - Extract RSSI
     char rssiChar[receivedClients][4];
     char rssi;
     char rssiValue;
     String rssiString;
     for (int f=0; f<receivedClients;f++){
       rssiValue = (h2d(receivedChars[28+f*7]) << 4) | h2d(receivedChars[29+f*7]);
       rssi = (~rssiValue+1);
       rssiString = String (-(rssi), DEC);
       Serial.print("rssi=");Serial.println(rssiString);
       rssiString.toCharArray(rssiChar[f*7],4);
      }

   //Encapsulate in JSON
    {
     StaticJsonBuffer<300> jsonBuffer;
     JsonObject& root = jsonBuffer.createObject();
     root["sensor"] = 1;
     root["timestamp"] = now_time;
     /*
     for (int i=0; i<receivedClients;i++){
       macs="mac"+String(i);
       rssis="rssi"+String(i);
       root[macs] = mac[i][0];
       root[rssis] = rssiChar[i][0];
      }
      */

     root.prettyPrintTo(Serial);

     //Publish the MQTT payload
     char jsonChar[300];
     root.printTo(jsonChar);
     client.publish(topic,jsonChar);
     Particle.publish("wifisense",jsonChar);
    }
  }else{                                // it is a control message?
    if(t1=='C'){
      char message[15];
      for (int i=8; i<23;i++) message[i-8]=receivedChars[i];
      message[23]='/n';
      Serial.print(message);          //Log Message to console

      StaticJsonBuffer<200> jsonBuffer;
      char jsonChar[200];
      JsonObject& root = jsonBuffer.createObject();
      root["sensor"] = 1;
      root["timestamp"] = now_time;
      root["Message"] = message;
      root.printTo(jsonChar);
      root.prettyPrintTo(Serial);     //Log JSON to console

      //Publish the MQTT payload
      client.publish(topic,jsonChar);
      Particle.publish("wifisense",jsonChar);
    }
  }
 newData = false;
 lastNdx= 0;
}

void sendPosition(float lat, float lon){
  uint32_t now_time = tmConvert_t(Time.year(), Time.month(), Time.day(), Time.hour(), Time.minute(), Time.second());
  char longitude[9];
  char latitude[9];

  String longit = String(lon,4);
  String latit = String(lat,4);
  longit.toCharArray(longitude,9);
  latit.toCharArray(latitude,9);

  char jsonChar[200];
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["sensor"] = 1;
  root["timestamp"] = now_time;
  root["lon"] = longitude;
  root["lat"] = latitude;
  root.printTo(jsonChar);
  root.prettyPrintTo(Serial);

  client.publish("C1",jsonChar);
  Particle.publish("wifisense",jsonChar);
}
