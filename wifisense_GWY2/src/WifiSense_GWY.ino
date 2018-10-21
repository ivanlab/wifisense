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

 #define WINDOW               300                        // Secs between transmissions from NodeMCU
 #define SENSORID             "22"
 #define SENSOR_TYPE          "headcount"
 #define CONTROL_TOPIC        "ivanlab/headcount/C/22"
 #define DATA_TOPIC           "ivanlab/headcount/S/22"
 #define TOPIC_NAME           "ivanlab/"
 #define SENSOR_NAME          "Nelium-Particle-022"
 #define CONTROL_ITERACTIONS  1
 #define APN                  "isp.vodafone.ie"
 #define VERSION              "v1.0.13"
 #define DELAY_BEFORE_REBOOT (15 * 1000)


 // Set your 3rd-party SIM APN here
 // https://docs.particle.io/reference/firmware/electron/#setcredentials-
 //STARTUP(cellular_credentials_set("isp.mymeteor.ie", "my", "isp", NULL));
 STARTUP(cellular_credentials_set(APN, "", "", NULL));

// Google Maps integration with Particle Cloud
// GoogleMapsDeviceLocator locator;

SerialLogHandler logHandler;                  // log debug messages to console (Serial)

//String controlID = String (TOPIC_NAME) + String ('C') + String(SENSORID);

// Battery initialization
double batteryLife = 0;                       // battery related
FuelGauge fuel;

// Serial link to NodeMCU initial parameters
const int numChars = 2000;                  // Size of message from NodeMCU
char receivedChars[2000];                 // Array holding message coming from NodeMCU
int lastNdx;
boolean newData = false;


// MQTT Functions
byte payload[24] ;
int keepalive = 5;
void callback(char* topic, byte* payload, unsigned int length);
MQTT client("mqtt.ivanlab.org", 1883, keepalive, callback);

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

int enableGPS(String command);
int preSatDetected = 1;

unsigned int rebootDelayMillis = DELAY_BEFORE_REBOOT;
unsigned long rebootSync = millis();
boolean resetFlag = false;
int cloudResetFunction(String command);


//                           ********* SETUP *********


void setup() {

  pinMode(D6, OUTPUT);                               //Reset line for NodeMCU
  digitalWrite(D6, LOW);
  pinMode(D5, OUTPUT);                               //Enable line for GPS
  digitalWrite(D5, LOW);

  Serial.begin(9600);                                // debug USB port
  Serial1.begin(9600);                               // NodeMCU comms port
  Serial4.begin(9600);                               // GPS Comms port
  Serial.println("Particle Up");
  Serial.println(TinyGPSPlus::libraryVersion());     // Print GPS lib version

 // locator.withLocatePeriodic(30);
  Particle.function("EnableGPS", enableGPS);
  Particle.function("CloudReset", cloudResetFunction);

 //get battery life on startup
  batteryLife = fuel.getSoC();
  Particle.variable("batteryLife", batteryLife);
  Particle.function("getSoC", getBatteryLife);
  //String battery=String(getBatteryLife("get"))+"% ";
  //String batteryV=String(fuel.getVCell()) + "V ";

 // MQTT initialization
  client.connect(SENSOR_NAME);
  if (client.isConnected()) {
    client.publish  (CONTROL_TOPIC,SENSOR_NAME);
    client.subscribe(CONTROL_TOPIC);
    Serial.println("MQTT Up");
    Particle.publish(String(SENSOR_NAME) + " - MQTT UP");
  } else {
    Serial.print (String(SENSOR_NAME) +"MQTT connection error");
    Particle.publish(String(SENSOR_NAME) + "MQTT Connection ERROR");
  }
  Serial.print ("MQTT keepalive: ");
  Serial.println(keepalive);

 // Log battery status to Console, MQTT and Particle cloud
  sendPosition(0,0);

  digitalWrite(D6, HIGH);                            // Enable NodeMCU
  digitalWrite(D5, HIGH);                            // Enable GPS
}

//                           ********* LOOP *********

void loop() {

  if(digitalRead(D5)==HIGH) {
    readGps(500);
    if ( (gps.location.lat()!=0 && gps.location.lng()!=0) || (millis() - gpsTimer > 900e3)){
      lat=gps.location.lat();
      lon=gps.location.lng();
      Serial.print("Position: "); Serial.print(lon); // Log position to console
      Serial.print("/"); Serial.println(lat);
      sendPosition(lat,lon);
      digitalWrite(D5, LOW);                         // Turn off  GPS
    } else {
      if(gps.satellites.value()!=preSatDetected ) {
        preSatDetected = gps.satellites.value();
        String satNumber = String (SENSOR_NAME) + " -> Sats detected: " + String (gps.satellites.value());
        Serial.println(satNumber);
        client.publish(CONTROL_TOPIC,satNumber);
      }
    }
  }                                               // Read GPS
  recvWithStartEndMarkers();                        // Read Serial link from NodeMCU
  if (newData) publishNewData();                    // Process received data and Write to MQTT
  if ((resetFlag) && (millis() - rebootSync >=  rebootDelayMillis)) {
    // do things here  before reset and then push the button
    client.publish(CONTROL_TOPIC,"CloudReset requested...bye");
    System.reset();
  }
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


  while (Serial1.available() > 0 && newData == false) {
    rc = Serial1.read();

    if (recvInProgress == true) {
      if ((millis()-receiving)> 15e3) ndx = 0;     // trash data in buffer if it's too old
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
            ndx = numChars - 1;
        }

      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        Serial.println("Cadena recibida y terminada");
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

  if (!client.isConnected()) {
    client.connect(SENSOR_NAME);
    Serial.print("...reconnecting MQTT");
    String reconnect = String(SENSOR_NAME) + "reconnecting MQTT";
    client.publish  (CONTROL_TOPIC,reconnect);
  }

  uint32_t now_time = tmConvert_t(Time.year(), Time.month(), Time.day(), Time.hour(), Time.minute(), Time.second());
  Serial.print("Received chars="); Serial.println(lastNdx+2);

  int payloadSize = lastNdx/2;

/*
  Serial.print("payload size advertised="); Serial.println(payloadSize);

  if (lastNdx<payloadSize*2) {
    Serial.println("Error de recepcion Serial");
    newData = false;
    lastNdx= 0;
    return;
  }
  */

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
  topic += "/";
  topic += t2;
  Serial.print("topic=");
  Serial.println(topic);

  if(t1=='S'){       //if Message is written to the DATA topic

    int receivedClients = ((payloadSize)-8)/11; Serial.print("# de rcv.macs: "); Serial.println(receivedClients);

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

    char longitude[9];
    char latitude[9];
    String longit = String(lon,4);
    String latit = String(lat,4);
    longit.toCharArray(longitude,9);
    latit.toCharArray(latitude,9);

   //Encapsulate in JSON
    DynamicJsonBuffer jsonBuffer(1024);
    for (int i=0; i<receivedClients;i++){
     JsonObject& root = jsonBuffer.createObject();
     root["sensor"] = SENSORID;
     root["sensorType"]= SENSOR_TYPE;
     root["timestamp"] = now_time - WINDOW + times[i]/1000;
     root["mac"] = mac[i];
     root["rssi"]= rssiChar[i];
     root["lon"] = longitude;
     root["lat"] = latitude;

     root.prettyPrintTo(Serial);
     Serial.println();

     //Publish the MQTT payload
     char jsonChar[1024];
     root.printTo(jsonChar);
     client.publish(DATA_TOPIC,jsonChar);
     //Particle.publish("wifisense",jsonChar, PRIVATE);

    }

    // Control channel added data
    iteractions++;
    if(iteractions % CONTROL_ITERACTIONS == 0){
      sendPosition(lat,lon);
      iteractions=0;
    }

   if (digitalRead(D5)==LOW) {
     String nap = String (SENSOR_NAME)+ String (" going for a nap...");
     client.publish(CONTROL_TOPIC,nap);
     System.sleep(D1,RISING,WINDOW-45);
     String wakeup = String (SENSOR_NAME)+ String (" waking up...");
     client.connect(SENSOR_NAME);
     client.publish(CONTROL_TOPIC,wakeup);
    }
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
      root["sensor"] = SENSORID;
      root["sensorType"]= SENSOR_TYPE;
      root["timestamp"] = now_time;
      root["Message"] = message;
      root.printTo(jsonChar);
      root.prettyPrintTo(Serial);     //Log JSON to console
      Serial.println("========================");
      //Publish the MQTT payload
      client.publish(CONTROL_TOPIC,jsonChar);
      //Particle.publish("wifisense",jsonChar, PRIVATE);
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
  String battery1=String(getBatteryLife("get"))+"% ";
  String batteryV1=String(fuel.getVCell()) + "V ";
  battery1.toCharArray(batteryChar,3);
  batteryV1.toCharArray(batteryVChar,4);

  char jsonChar[300];
  DynamicJsonBuffer jsonBuffer(300);
  JsonObject& root = jsonBuffer.createObject();
  root["sensor"] = SENSORID;
  root["sensorType"]=SENSOR_TYPE;
  root["Name"] = SENSOR_NAME;
  root["Version"] = VERSION;
  root["timestamp"] = now_time;
  root["APN"] = APN;
  root["lon"] = longitude;
  root["lat"] = latitude;
  root["Battery"] = batteryChar;
  root["Voltage"] = batteryVChar;

  root.printTo(jsonChar);
  root.prettyPrintTo(Serial);
  Serial.println();
  client.publish(CONTROL_TOPIC,jsonChar);
  //Particle.publish("wifisense",jsonChar, PRIVATE);
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

int enableGPS(String data){
  digitalWrite(D5, HIGH);
  client.publish(CONTROL_TOPIC,"GPS activation requested...");
  gpsTimer = millis();
  return (1);
}

int cloudResetFunction(String command) {
 resetFlag = true;
 rebootSync=millis();
 return 0;
}
