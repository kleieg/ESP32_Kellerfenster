#include <Arduino.h>

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
//#include <Adafruit_Sensor.h>
#include <string>

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <Arduino_JSON.h>
#include <AsyncElegantOTA.h>

#include "WLAN_Credentials.h"

#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
#define SERIALINIT Serial.begin(115200);
#else
#define SERIALINIT
#endif

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<   Anpassungen !!!!
// set hostname used for MQTT tag and WiFi 
#define HOSTNAME "BME280"


// variables to connects to  MQTT broker
const char* mqtt_server = "192.168.178.15";
const char* willTopic = "tele/SR04/LWT";       // muss mit HOSTNAME passen !!!  tele/HOSTNAME/LWT    !!!

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<   Anpassungen Ende !!!!


Adafruit_BME280 bme; // I2C               


// for MQTT
byte willQoS = 0;
const char* willMessage = "Offline";
boolean willRetain = true;
std::string mqtt_tag;
int Mqtt_sendInterval = 120001;   // in milliseconds = 2 minutes
long Mqtt_lastScan = 0;
long lastReconnectAttempt = 0;

// Define NTP Client to get time
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 0;
int UTC_syncIntervall = 3600000; // in milliseconds = 1 hour
long UTC_lastSync;
time_t UTC_time;

// Initializes the espClient. 
WiFiClient myClient;
PubSubClient client(myClient);
// name used as Mqtt tag
std::string gateway = HOSTNAME ;   

/*
used GPIOs 
BMNE280: SDA = 21  SCL = 22

Fensterschalter = 18
Taster = 19

Motorsteuerung = 16 & 17

LEDs  25 = auto  26 = open  27 = close

*/
// variables for BME280
bool status;
int BME280_scanIntervall = 30000;  // in milliseconds
long BME280lastScan = 0;
float BME_Temperature;
float BME_Humidity;
float BME_Pressure;
long  BME_Time;
int BME_Mode = 1; // 1 = auto  2 = open  3 = closed  based on Button
// mode set by IP-Symcon
int Mode = 1;   // 1 = auto  2 = open  3 = closed

// variables for LED 
int LED_auto = 25;
int LED_open = 26;
int LED_close = 27;
bool led = 1;

// Fensterkontakt
bool WindowState;
int WindowGpio = 18;
int Window_scanIntervall = 1000;  // in milliseconds
long WindowlastScan = 0;

// Taster
int ButtonGpio = 19;
int Button_scanIntervall = 1000;  // in milliseconds
long ButtonlastScan = -1000;

// Motor Fenster
bool ButtonState;
int OpenGpio = 16;
int CloseGpio = 17;
int MotorDuration = 2500; // intervall in milliseconds
int MotorCount = 7;  // intervall count
int MotorPause = 500; // pause in milliseconds
long MotorStart = 0;
int MotorRun;
bool MotorClose;

// Timer
long now = millis();
char strtime[8];


// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create a WebSocket object
AsyncWebSocket ws("/ws");

// end of definitions -----------------------------------------------------

// Initialize SPIFFS
void initSPIFFS() {
  if (!SPIFFS.begin()) {
    // Serial.println("An error has occurred while mounting LittleFS");
  }
  // Serial.println("LittleFS mounted successfully");
}

String getOutputStates(){
  JSONVar myArray;
  
  myArray["cards"][0]["c_text"] = HOSTNAME;
  myArray["cards"][1]["c_text"] = willTopic;
  myArray["cards"][2]["c_text"] = String(WiFi.RSSI());
  myArray["cards"][3]["c_text"] = String(BME_Time);
  myArray["cards"][4]["c_text"] = String(Mqtt_sendInterval) + "ms";
  myArray["cards"][5]["c_text"] = String(Window_scanIntervall) + "ms";
  myArray["cards"][6]["c_text"] = String(Button_scanIntervall) + "ms";
  myArray["cards"][7]["c_text"] = String(BME280_scanIntervall) + "ms";
  myArray["cards"][8]["c_text"] = String(BME_Humidity) +"%";
  myArray["cards"][9]["c_text"] = String(BME_Pressure) ;
  myArray["cards"][10]["c_text"] = String(BME_Temperature) + "Grad";
  myArray["cards"][11]["c_text"] = String(Mode);
  
  String jsonString = JSON.stringify(myArray);
  log_i("%s",jsonString.c_str()); 
  return jsonString;
}

void notifyClients(String state) {
  ws.textAll(state);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    char help[30];
    int card;
    int value;
    
    for (int i = 0; i <= len; i++){
      help[i] = data[i];
    }

    log_i("Data received: ");
    log_i("%s\n",help);

    JSONVar myObject = JSON.parse(help);

    card =  myObject["card"];
    value =  myObject["value"];
    log_i("%d", card);
    log_i("%d",value);

    switch (card) {
      case 0:   // fresh connection
        notifyClients(getOutputStates());
        break;
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      log_i("WebSocket client connected");
      break;
    case WS_EVT_DISCONNECT:
      log_i("WebSocket client disconnected");
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

// init WiFi
void setup_wifi() {

  delay(10);
  digitalWrite(LED_auto, 1); 
  delay(500);
  digitalWrite(LED_auto, 0); 
  delay(500);
  digitalWrite(LED_open, 1);
  delay(500);
  digitalWrite(LED_open, 0);
  delay(500);
  digitalWrite(LED_close, 1);
  delay(500);
  digitalWrite(LED_close, 0);
  log_i("Connecting to ");
  log_i("%s",ssid);
  log_i("%s",password);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
     if(led == 0){
       digitalWrite(LED_auto, 0);  // LED aus
       led = 1;
     }else{
       digitalWrite(LED_auto, 1);  // LED ein
       led = 0;
     }
    log_i(".");
  }

  digitalWrite(LED_auto, 0);  // LED aus
  led = 1;
  log_i("WiFi connected - IP address: ");
  log_i("%s",WiFi.localIP().toString().c_str());

  // get time from NTP-server
  log_i("init NTP-server");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);  // update ESP-systemtime to UTC
  delay(100);                                                 // udate takes some time
  time(&UTC_time);
  itoa(UTC_time,strtime,10);
  log_i("%s","Unix-timestamp =" );
  log_i("%s",strtime);
}


// LED blink 
void LED_blink(int stat) {
    
    if (stat == 0) {
      if(led == 0){
        led = 1;
      }else{
        led = 0;
      }
    }else{
      led = 1;
    }

    switch (Mode) {
      case 1:
        digitalWrite(LED_auto, led);
        digitalWrite(LED_open, 0);
        digitalWrite(LED_close, 0);
      break;
      case 2:
        digitalWrite(LED_open, led);
        digitalWrite(LED_auto, 0);
        digitalWrite(LED_close, 0);
      break;
      case 3:
        digitalWrite(LED_close, led);
        digitalWrite(LED_open, 0);
        digitalWrite(LED_auto, 0);
      break;
    }
  }   


// reconnect to WiFi 
void reconnect_wifi() {
  log_i("%s\n","WiFi try reconnect"); 
  WiFi.begin();
  delay(500);
  if (WiFi.status() == WL_CONNECTED) {
    // Once connected, publish an announcement...
    log_i("%s\n","WiFi reconnected"); 
  }
}

// This functions reconnects your ESP32 to your MQTT broker

void reconnect_mqtt() {
  if (client.connect(gateway.c_str(), willTopic, willQoS, willRetain, willMessage)) {
    // Once connected, publish an announcement...
    log_i("%s\n","Mqtt connected"); 
    mqtt_tag = gateway + "/connect";
    client.publish(mqtt_tag.c_str(),"connected");
    log_i("%s",mqtt_tag.c_str());
    log_i("%s\n","connected");
    mqtt_tag = "tele/" + gateway  + "/LWT";
    client.publish(mqtt_tag.c_str(),"Online",willRetain);
    log_i("%s",mqtt_tag.c_str());
    log_i("%s\n","Online");

    mqtt_tag = "cmnd/" + gateway + "/#";
    client.subscribe(mqtt_tag.c_str());
  }
}

// receive MQTT messages
void MQTT_callback(char* topic, byte* message, unsigned int length) {
  
  log_i("%s","Message arrived on topic: ");
  log_i("%s\n",topic);
  log_i("%s","Data : ");

  String MQTT_message;
  for (int i = 0; i < length; i++) {
    MQTT_message += (char)message[i];
  }
  log_i("%s\n",MQTT_message);

  String Topic_Window = String("cmnd/");
  Topic_Window = String(Topic_Window + gateway.c_str() + "/Window");
  String Topic_Mode = String("cmnd/");
  Topic_Mode = String(Topic_Mode + gateway.c_str() + "/Mode");

  if (String(topic) == Topic_Window ){

    ButtonlastScan = ButtonlastScan + 30000;       // ignore pressed botton next 30 seconds

    if(MQTT_message == "open"){
      log_i("%s\n","open window");
      digitalWrite(OpenGpio, LOW);
      MotorStart = millis();
      MotorRun = MotorCount;
      MotorClose = false;
    }
    else if(MQTT_message == "close"){
      log_i("%s\n","open window");
      digitalWrite(CloseGpio, LOW);
      MotorStart = millis();
      MotorRun = MotorCount;
      MotorClose = true;
    }
  }

  if (String(topic) == Topic_Mode ){

    if(MQTT_message == "auto"){
      log_i("%s\n","Mode = auto");
      Mode = 1;
      BME_Mode = 1;
    }
    if(MQTT_message == "open"){
      log_i("%s\n","Mode = open");
      Mode = 2;
      BME_Mode = 2;
    }
    if(MQTT_message == "close"){
      log_i("%s\n","Mode = close");
      Mode = 3;
      BME_Mode = 3;
    }

    LED_blink(1);

  }
  notifyClients(getOutputStates());

}

// init BME280
void setup_BME280() {
  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin();  
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  log_i("%s\n","BME280 sensor connected");
}



// read BME280 values
void BME280_scan() {
  
  BME_Temperature = bme.readTemperature();
  BME_Humidity = bme.readHumidity();
  BME_Pressure = bme.readPressure() / 100.0F ;  // This forces floating point division

  time(&UTC_time);
  BME_Time = UTC_time;
  notifyClients(getOutputStates());
}

// send data using Mqtt 
void MQTTsend () {

 JSONVar mqtt_data; 
  
  mqtt_tag = "tele/" + gateway + "/SENSOR";
  log_i("%s",mqtt_tag.c_str());

  mqtt_data["Temp"] = round (BME_Temperature*10) / 10;
  mqtt_data["Hum"] = round (BME_Humidity*10) / 10;
  mqtt_data["Pres"] = round (BME_Pressure*10) / 10;
  mqtt_data["Mode"] = BME_Mode;

  if  ( WindowState ) {
    mqtt_data["Window"] = 1;
  }
  else{
    mqtt_data["Window"] = 0;
  }
  
  mqtt_data["RSSI"] = WiFi.RSSI();
  mqtt_data["Time"] = BME_Time;
  String mqtt_string = JSON.stringify(mqtt_data);

  log_i("%s",mqtt_string.c_str()); 

  client.publish(mqtt_tag.c_str(), mqtt_string.c_str());

/*
  static char Temperature_char[7];
  dtostrf(Temperature, 6, 2, Temperature_char);

  // Publishes Temperature and Humidity values only if value is not "nan"
  // dtostrf some times returns "nan" for unknown reason
  if (strcmp(Temperature_char , "nan") != 0 ) {
    client.publish("BME280/temperature", Temperature_char);
    log_i("%s","BME280/temperature=");
    log_i("%s",Temperature_char);
  }

*/
}

void setup() {

  SERIALINIT
  
  log_i("setup device\n");

  // initialise LED
  pinMode(LED_auto, OUTPUT);
  pinMode(LED_open, OUTPUT);
  pinMode(LED_close, OUTPUT);

  // initialise Fensterkontakt, Taster  und Motor
  pinMode(WindowGpio, INPUT);
  pinMode(ButtonGpio, INPUT);
  pinMode(OpenGpio, OUTPUT);
  pinMode(CloseGpio, OUTPUT);
  digitalWrite (OpenGpio, HIGH);
  digitalWrite (CloseGpio, HIGH);

  log_i("setup WiFi\n");
  setup_wifi();

  log_i("setup MQTT\n");
  client.setServer(mqtt_server, 1883);
  client.setCallback(MQTT_callback);

  log_i("setup BME280\n");
  setup_BME280() ;
  delay(10);
  BME280_scan();


  initSPIFFS();
  initWebSocket();

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html",false);
  });

  server.serveStatic("/", SPIFFS, "/");

  // Start ElegantOTA
  AsyncElegantOTA.begin(&server);
  
  // Start server
  server.begin();

}

void loop() {

  AsyncElegantOTA.loop();
  ws.cleanupClients();

  now = millis();


  // perform BME280 scan  
   if (now - BME280lastScan > BME280_scanIntervall) {
    BME280lastScan = now;
    BME280_scan();
  }    

    // check Fensterkontakt
   if (now - WindowlastScan > Window_scanIntervall) {
    WindowlastScan = now;
    
    bool OldState = WindowState;
    WindowState = digitalRead(WindowGpio);
    if ((WindowState == LOW) && (OldState == HIGH)){
      Mqtt_lastScan = now - Mqtt_sendInterval - 10;  // --> MQTT send !!
    }
    if ((WindowState == HIGH) && (OldState == LOW)){
      Mqtt_lastScan = now - Mqtt_sendInterval - 10;  // --> MQTT send !!
    }
  }    

    // check Taster
   if (now - ButtonlastScan > Button_scanIntervall) {
    ButtonlastScan = now;

    LED_blink(0);

    ButtonState = digitalRead(ButtonGpio);
    if (ButtonState == LOW) {
      BME_Mode = BME_Mode + 1;
      if ( BME_Mode == 4 ) BME_Mode = 1;
      Mqtt_lastScan = now - Mqtt_sendInterval - 10;  // --> MQTT send !!
      ButtonlastScan = ButtonlastScan + 10000;       // ignore pressed botton next 10 seconds
    }
  } 
  
  
  // check Motor
   if ((now - MotorStart > MotorDuration) && (MotorStart != 0 )){
     MotorRun = MotorRun - 1;
     if ( MotorRun == 0) {
      MotorStart = 0;
      digitalWrite (OpenGpio, HIGH);
      digitalWrite (CloseGpio, HIGH);
     } else {
      MotorStart = now;
      if (MotorClose) {
        digitalWrite (CloseGpio, HIGH);
      } else {
        digitalWrite (OpenGpio, HIGH);
      }
      delay (MotorPause);
      if (MotorClose) {
        digitalWrite (CloseGpio, LOW);
      } else {
        digitalWrite (OpenGpio, LOW);
      }
     }
  }    


  // check WiFi
  if (WiFi.status() != WL_CONNECTED  ) {
    // try reconnect every 5 seconds
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;              // prevents mqtt reconnect running also
      // Attempt to reconnect
      log_i("WiFi reconnect"); 
      reconnect_wifi();
    }
  }

  // perform UTC sync
  if (now - UTC_lastSync > UTC_syncIntervall) {
    UTC_lastSync = now;
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);  // update ESP-systemtime to UTC
    delay(100);                                                 // udate takes some time
    time(&UTC_time);
    log_i("%s","Re-sync ESP-time!! Unix-timestamp =");
    itoa(UTC_time,strtime,10);
    log_i("%s",strtime);
  }   

  // check if MQTT broker is still connected
  if (!client.connected()) {
    // try reconnect every 5 seconds
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      log_i("MQTT reconnect"); 
      reconnect_mqtt();
    }
  } else {
    // Client connected

    client.loop();

    // send data to MQTT broker
    if (now - Mqtt_lastScan > Mqtt_sendInterval) {
    Mqtt_lastScan = now;
    MQTTsend();
    } 
  }
}
