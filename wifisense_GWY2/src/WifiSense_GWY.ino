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

SerialLogHandler logHandler;                  // log debug messages to console (Serial)

#define window              300                        // Secs between transmissions from NodeMCU
#define sensorID            2
#define controlTopic        "ivanlab/C2"
#define TOPIC_NAME           "ivanlab/"
#define sensorName          "Nelium-Particle-2"
#define CONTROL_ITERACTIONS 1
String controlID = String (TOPIC_NAME) + String ('C') + String(sensorID);

// Battery initialization
double batteryLife = 0;                       // battery related
FuelGauge fuel;

// Serial link to NodeMCU initial parameters
const int numChars = 500;                  // Size of message from NodeMCU
char receivedChars[500];                 // Array holding message coming from NodeMCU
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

unsigned long receiving;

// GPS parameters
static float lon = 0.0;
static float lat = 0.0;
unsigned long gpsTimer = millis();
TinyGPSPlus gps;

int iteractions = 0;


//                           ********* SETUP *********


void setup() {

  pinMode(D6, OUTPUT);                               //Reset line for NodeMCU
  digitalWrite(D6, LOW);
  pinMode(D5, OUTPUT);                               //Enable line for GPS
  digitalWrite(D5, LOW);

  Serial.begin(9600);                                // debug USB port
  Serial1.begin(9600);                             // NodeMCU comms port
  Serial4.begin(9600);                               // GPS Comms port
  Serial.println("Particle Up");
  Serial.println(TinyGPSPlus::libraryVersion());     // Print GPS lib version

 // locator.withLocatePeriodic(30);

 //get battery life on startup
  batteryLife = fuel.getSoC();
  Particle.variable("batteryLife", batteryLife);
  Particle.function("getSoC", getBatteryLife);
  String battery=String(getBatteryLife("get"))+"% ";
  String batteryV=String(fuel.getVCell()) + "V ";

 // MQTT initialization
  client.connect(sensorName);
  if (client.isConnected()) {
    client.publish  (controlID,sensorName);
    client.subscribe(controlID);
    Serial.println("MQTT Up");
    Particle.publish(String(sensorName) + " - MQTT UP");
  } else {
    Serial.print (String(sensorName) +"MQTT connection error");
    Particle.publish(String(sensorName) + "MQTT Connection ERROR");
  }
  Serial.print ("MQTT keepalive: ");
  Serial.println(keepalive);

 // Log battery status to Console, MQTT and Particle cloud
  Serial.print(battery);                             // Log battery level to Serial
  Serial.println(batteryV);
  Particle.publish(battery);                         // Log battery to Particle cloud
  Particle.publish(batteryV);

  digitalWrite(D6, HIGH);                            // Enable NodeMCU
  digitalWrite(D5, HIGH);                            // Enable GPS
}

//                           ********* LOOP *********

void loop() {

  if (!client.isConnected()) {
    client.connect(sensorName);
    Serial.print("...reconnecting MQTT");
    client.publish  (controlID,"Particle reconnecting...");
  }                     // Check if MQTT still up
  if(digitalRead(D5)==HIGH) {
    readGps(300);
    if ( (gps.location.lat()!=0 && gps.location.lng()!=0) || (millis() - gpsTimer > 300e3)){
      lat=gps.location.lat();
      lon=gps.location.lng();
      Serial.print("Position: "); Serial.print(lon); // Log position to console
      Serial.print("/"); Serial.println(lat);
      sendPosition(lat,lon);
      digitalWrite(D5, LOW);                         // Turn off  GPS
    }
  }                                           // Read GPS
  recvWithStartEndMarkers();                        // Read Serial link from NodeMCU
  if (newData) publishNewData();                    // Process received data and Write to MQTT
  client.loop();                                    // Allow to check for MQTT callback
  delay(1);
}

//                           ****** FUNCTIONS *********


void callback(char* topic, byte* payload, unsigned int length) {
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL;

}

int getBatteryLife(String command) {
  batteryLife = fuel.getSoC();
  return (int)batteryLife;
}

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static int ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;
  while (Serial1.available() > 0 && newData == false) {
    rc = Serial1.read();
    if (recvInProgress == true) {
      if ((millis()-receiving)> 10e3) ndx = 0;     // trash data in buffer if it's too old
      if (rc != endMarker) {
          receivedChars[ndx] = rc;
          Serial.print(rc);
          ndx++;

          if (ndx >= numChars) {
              ndx = numChars - 1;
          }

      }
      else {
          receivedChars[ndx] = '\0'; // terminate the string
          Serial.println();
          recvInProgress = false;
          lastNdx = ndx;
          ndx = 0;
          newData = true;
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
      receiving = millis();
    }
  }
}

void publishNewData() {

  uint32_t now_time = tmConvert_t(Time.year(), Time.month(), Time.day(), Time.hour(), Time.minute(), Time.second());

  int payloadSize = (h2d(receivedChars[0]) << 4) | h2d(receivedChars[1]);  // convert to decimal
  Serial.print("payload size advertised="); Serial.println(payloadSize);

  Serial.print("Received chars="); Serial.println(lastNdx+2);

  if (lastNdx<payloadSize*2) {
    Serial.println("Error de recepcion Serial");
    newData = false;
    lastNdx= 0;
    return;
  }

  for (int i=0; i < lastNdx; i=i+2) {
    Serial.print(receivedChars[i]);
    Serial.print(receivedChars[i+1]);
    Serial.print("|");
  }
  Serial.println();

  // 1 - Extract Topic from SN message
  char t1 = (h2d(receivedChars[6]) << 4) | h2d(receivedChars[7]);
  char t2 = (h2d(receivedChars[8]) << 4) | h2d(receivedChars[9]);
  String topic =TOPIC_NAME;
  topic += t1;
  topic += t2;
  Serial.print("topic=");
  Serial.println(topic);

  if(t1=='S'){       //if Message is written to the DATA topic

    int receivedClients = (payloadSize-8)/11; Serial.print("# de rcv.macs: "); Serial.println(receivedClients);

    // 2 - Extract Mac address from SN
    char mac[receivedClients][19];              //17 caracteres por cada Mac recibida
    for (int f=0;f<receivedClients;f++) {
      Serial.print(f+1); Serial.print(" client -> ");
      for (int i=0;i<17;i=i+3) {
       mac[f][i]=receivedChars[16+(2*i/3)+(22*f)];
       Serial.print(receivedChars[16+(2*i/3)+(22*f)]);
       mac[f][i+1]=receivedChars[17+(2*i/3)+(22*f)];
       Serial.print(receivedChars[17+(2*i/3)+(22*f)]);
       mac[f][i+2]=':'; Serial.print(":");
      }
      mac[f][18]='\0';
      Serial.println();
    }

    // 3 - Extract RSSI
    char rssiChar[receivedClients][4];
    char rssi;
    char rssiValue;
    String rssiString;
    for (int f=0; f<receivedClients;f++){
      rssiValue = (h2d(receivedChars[28+f*22]) << 4) | h2d(receivedChars[29+f*22]);
      rssi = (~rssiValue+1);
      rssiString = String (-(rssi), DEC);
      Serial.print("rssi=");Serial.println(rssiString);
      rssiString.toCharArray(rssiChar[f],4);
    }

    // 4 - Extract timestamp

    unsigned long times[receivedClients];
    for (int f=0; f<receivedClients;f++){
      union {
        char myByte[4];
        unsigned long myLong;
      } myUnion;

      for (int i=0; i<4;i++){
        char byteHexTime = (h2d(receivedChars[30+i*2+f*22]) << 4) | h2d(receivedChars[31+i*2+f*22]);
        myUnion.myByte[i] = byteHexTime;
      }
      times[f]=myUnion.myLong;
      Serial.print("millis="); Serial.println(myUnion.myLong);
    }


   //Encapsulate in JSON
    DynamicJsonBuffer jsonBuffer(200);
    for (int i=0; i<receivedClients;i++){
     JsonObject& root = jsonBuffer.createObject();
     root["sensor"] = sensorID;
     root["timestamp"] = now_time - window + times[i]/1000;
     root["mac"] = mac[i];
     root["rssi"]= rssiChar[i];
     root.prettyPrintTo(Serial);
     Serial.println();

     //Publish the MQTT payload
     char jsonChar[200];
     root.printTo(jsonChar);
     client.publish(topic,jsonChar);
    }

    // Control channel added data
    iteractions++;
    if(iteractions % CONTROL_ITERACTIONS == 0){
      sendPosition(lat,lon);
      iteractions=0;
    }


   Serial.println("=====>> Going for a nap...");
   if (digitalRead(D5)==LOW) System.sleep(D1,RISING,window-45);

  }else{                                // it is a control message?
    if(t1=='C'){
      char message[(payloadSize-8+1)];
      for (int i=8; i<payloadSize;i++) {
        message[i-8]= (h2d(receivedChars[i*2]) << 4) | h2d(receivedChars[i*2+1]);
      }
      message[(payloadSize-8+1)]='/n';

      DynamicJsonBuffer jsonBuffer(200);
      char jsonChar[200];
      JsonObject& root = jsonBuffer.createObject();
      root["sensor"] = sensorID;
      root["timestamp"] = now_time;
      root["Message"] = message;
      root.printTo(jsonChar);
      root.prettyPrintTo(Serial);     //Log JSON to console
      Serial.println("========================");
      //Publish the MQTT payload
      client.publish(topic,jsonChar);
    }
  }
 newData = false;
 lastNdx= 0;
 Serial.println("end of published data");
 return;
}

void sendPosition(float lat, float lon){
  uint32_t now_time = tmConvert_t(Time.year(), Time.month(), Time.day(), Time.hour(), Time.minute(), Time.second());
  char longitude[9];
  char latitude[9];
  char batteryChar[4];
  char batteryVChar[5];


  String longit = String(lon,4);
  String latit = String(lat,4);
  longit.toCharArray(longitude,9);
  latit.toCharArray(latitude,9);
  String battery=String(getBatteryLife("get"))+"% ";
  String batteryV=String(fuel.getVCell()) + "V ";
  battery.toCharArray(batteryChar,3);
  batteryV.toCharArray(batteryVChar,4);

  char jsonChar[200];
  DynamicJsonBuffer jsonBuffer(200);
  JsonObject& root = jsonBuffer.createObject();
  root["sensor"] = sensorID;
  root["timestamp"] = now_time;
  root["lon"] = longitude;
  root["lat"] = latitude;
  root["Battery"] = batteryChar;
  root["Voltage"] = batteryVChar;
  root.printTo(jsonChar);
  root.prettyPrintTo(Serial);
  Serial.println();
  client.publish(controlTopic,jsonChar);
  Particle.publish(sensorName,jsonChar);
  Particle.publish("Battery level" , battery);
  Particle.publish("Battery volt" , batteryV);
}

unsigned char h2d(unsigned char hex) {
  if(hex > 0x39) hex -= 7; // adjust for hex letters upper or lower case
  return(hex & 0xf);
}

static void readGps(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (Serial4.available())gps.encode(Serial4.read());
  } while (millis() - start < ms);
}

time_t tmConvert_t(int YYYY, byte MM, byte DD, byte hh, byte mm, byte ss) {
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
