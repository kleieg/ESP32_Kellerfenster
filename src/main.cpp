#include <Arduino.h>


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
#include <NTPClient.h>
#include <WiFiUdp.h>

#include "WLAN_Credentials.h"
#include "config.h"
#include "wifi_mqtt.h"


// NTP
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
long My_time = 0;
long Start_time;
long Up_time;
long U_days;
long U_hours;
long U_min;
long U_sec;

// Timers auxiliar variables
long now = millis();
char strtime[8];


Adafruit_BME280 bme; // I2C               

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
int BME280_scanIntervall = 20000;  // in milliseconds
long BME280lastScan = 0;
float BME_Temp;
float BME_Hum;
float BME_Pres;
long  BME_Time;

// mode set by IP-Symcon or Button
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
int MotorDuration = 15000; //  in milliseconds
long MotorStart = 0;



// Create AsyncWebServer object on port 80
AsyncWebServer Asynserver(80);

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
  
  U_days = Up_time / 86400;
  U_hours = (Up_time % 86400) / 3600;
  U_min = (Up_time % 3600) / 60;
  U_sec = (Up_time % 60);

  myArray["cards"][0]["c_text"] = Hostname;
  myArray["cards"][1]["c_text"] = WiFi.dnsIP().toString() + "   /   " + String(VERSION);
  myArray["cards"][2]["c_text"] = String(WiFi.RSSI());
  myArray["cards"][3]["c_text"] = String(MQTT_INTERVAL) + "ms";
  myArray["cards"][4]["c_text"] = String(U_days) + " days " + String(U_hours) + ":" + String(U_min) + ":" + String(U_sec);
  myArray["cards"][5]["c_text"] = "WiFi = " + String(WiFi_reconnect) + "   MQTT = " + String(Mqtt_reconnect);
  myArray["cards"][6]["c_text"] = String(Mode);
  myArray["cards"][7]["c_text"] = " to reboot click ok";
  myArray["cards"][8]["c_text"] = String(BME_Hum) +"%";
  myArray["cards"][9]["c_text"] = String(BME_Pres) ;
  myArray["cards"][10]["c_text"] = String(BME_Temp) + "Grad";
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
    data[len] = 0;   // according to AsyncWebServer documentation this is ok

    log_i("Data received: ");
    log_i("%s\n",data);

    JSONVar myObject = JSON.parse((const char *)data);
//    int value;
    int card;
    
    card =  myObject["card"];
//    value =  myObject["value"];
    log_i("%d", card);
//    log_i("%d",value);

    switch (card) {
      case 0:   // fresh connection
        notifyClients(getOutputStates());
        break;
      case 7:
        log_i("Reset..");
        ESP.restart();
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
        digitalWrite(LED_auto, 0);
        digitalWrite(LED_open, led);
        digitalWrite(LED_close, 0);
      break;
      case 3:
        digitalWrite(LED_auto, 0);
        digitalWrite(LED_open, 0);
        digitalWrite(LED_close, led);
      break;
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

  String windowTopic = Hostname + "/CMD/Fensterauf";
  String modeTopic = Hostname + "/CMD/Mode";
  String strTopic = String(topic);

  if (strTopic == windowTopic ){

    LED_blink(1);
    ButtonlastScan = ButtonlastScan + MotorDuration;       // ignore pressed botton next MotorDuration seconds

    if(MQTT_message == "true"){
      log_i("%s\n","open window");
      digitalWrite(OpenGpio, LOW);
      MotorStart = millis();
    }
    if(MQTT_message == "false"){
      log_i("%s\n","close window");
      digitalWrite(CloseGpio, LOW);
      MotorStart = millis();
    }
  }

  if (strTopic == modeTopic ){

    if(MQTT_message == "1"){
      log_i("%s\n","Mode = auto");
      Mode = 1;
    }
    if(MQTT_message == "2"){
      log_i("%s\n","Mode = open");
      Mode = 2;
    }
    if(MQTT_message == "3"){
      log_i("%s\n","Mode = close");
      Mode = 3;
    }
  }
  notifyClients(getOutputStates());
  Mqtt_lastSend = now - MQTT_INTERVAL - 10;  // --> MQTT send !!

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
  bme.setSampling(Adafruit_BME280::MODE_FORCED, // takeForcedMeasurement must be called before each reading
  Adafruit_BME280::SAMPLING_X1, // Temp. oversampling
  Adafruit_BME280::SAMPLING_X1, // Pressure oversampling
  Adafruit_BME280::SAMPLING_X1, // Humidity oversampling
  Adafruit_BME280::FILTER_OFF,
  Adafruit_BME280::STANDBY_MS_1000);
  log_i("%s\n","BME280 sensor initialized");

  bme.takeForcedMeasurement();
  BME_Temp = bme.readTemperature();
  BME_Hum = bme.readHumidity();
  BME_Pres = bme.readPressure() / 100.0F;  // /100.0F  >> forces floating point division
  log_i("%s\n","BME280 Startwert für exponentielle Glättung erstellt");
}



// read BME280 values
void BME280_scan() {
  
  bme.takeForcedMeasurement();
  BME_Temp = BME_Temp + (bme.readTemperature() - BME_Temp) * 0.5;
  BME_Hum = BME_Hum + (bme.readHumidity() - BME_Hum) * 0.5;
  BME_Pres = BME_Pres + (bme.readPressure() / 100.0F  - BME_Pres) * 0.5;  // /100.0F  >> forces floating point division

  notifyClients(getOutputStates());
}

// send data using Mqtt 
void MQTTsend () {

  int i;
  JSONVar mqtt_data,  actuators;

  String mqtt_tag = Hostname + "/STATUS";
  log_i("%s\n", mqtt_tag.c_str());
  
  mqtt_data["Time"] = My_time;
  mqtt_data["RSSI"] = WiFi.RSSI();
  mqtt_data["Temp"] = round (BME_Temp*10) / 10;
  mqtt_data["Hum"] = round (BME_Hum*10) / 10;
  mqtt_data["Pres"] = round (BME_Pres*10) / 10;

  actuators["Mode"] = Mode;
   if  ( WindowState ) {
    actuators["Fensterauf"] = true;
  }
  else{
    actuators["Fensterauf"] = false;
  }
  mqtt_data["Actuators"] = actuators;


  String mqtt_string = JSON.stringify(mqtt_data);

  log_i("%s\n", mqtt_string.c_str());


  Mqttclient.publish(mqtt_tag.c_str(), mqtt_string.c_str());

  notifyClients(getOutputStates());

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
  initWiFi();

  log_i("setup MQTT\n");
  Mqttclient.setServer(MQTT_BROKER, 1883);
  Mqttclient.setCallback(MQTT_callback);

  log_i("setup BME280\n");
  setup_BME280() ;
  delay(10);
  BME280_scan();


  initSPIFFS();
 
  // init Websocket
  ws.onEvent(onEvent);
  Asynserver.addHandler(&ws);

  // Route for root / web page
  Asynserver.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html",false);
  });

  Asynserver.serveStatic("/", SPIFFS, "/");

  timeClient.begin();
  timeClient.setTimeOffset(0);
  // update UPCtime for Starttime
  timeClient.update();
  Start_time = timeClient.getEpochTime();

  // Start ElegantOTA
  AsyncElegantOTA.begin(&Asynserver);
  
  // Start server
  Asynserver.begin();

}

void loop() {

  ws.cleanupClients();

  // update UPCtime
    timeClient.update();
    My_time = timeClient.getEpochTime();
    Up_time = My_time - Start_time;

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
      MotorStart = 0;
      digitalWrite (CloseGpio, HIGH);             // window closed. switch off motor
      Mqtt_lastSend = now - MQTT_INTERVAL - 10;  // --> MQTT send !!
    }
    if ((WindowState == HIGH) && (OldState == LOW)){
      Mqtt_lastSend = now - MQTT_INTERVAL - 10;  // --> MQTT send !!
    }
  }    

    // check Taster
   if (now - ButtonlastScan > Button_scanIntervall) {
    ButtonlastScan = now;
    LED_blink(0);
    ButtonState = digitalRead(ButtonGpio);
    if (ButtonState == LOW) {
      Mode = Mode + 1;
      if ( Mode == 4 ) Mode = 1;
      Mqtt_lastSend = now - MQTT_INTERVAL - 10;  // --> MQTT send !!
      ButtonlastScan = ButtonlastScan + 2000;       // ignore pressed botton next 2 seconds
      LED_blink(1);
    }
  } 
  
  
  // check Motor
   if ((now - MotorStart > MotorDuration) && (MotorStart != 0 )){
      MotorStart = 0;
      digitalWrite (OpenGpio, HIGH);
      digitalWrite (CloseGpio, HIGH);
  }    

  // check WiFi
  if (WiFi.status() != WL_CONNECTED  ) {
    // try reconnect every 5 seconds
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;              // prevents mqtt reconnect running also
      // Attempt to reconnect
      reconnect_wifi();
    }
  }


  // check if MQTT broker is still connected
  if (!Mqttclient.connected()) {
    // try reconnect every 5 seconds
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      reconnect_mqtt();
    }
  } else {
    // Client connected

    Mqttclient.loop();

    // send data to MQTT broker
    if (now - Mqtt_lastSend > MQTT_INTERVAL) {
    Mqtt_lastSend = now;
    MQTTsend();
    } 
  }   
}
