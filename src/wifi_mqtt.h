#include <MQTT.h>
#include <WiFi.h>

// will be computed as "<HOSTNAME>_<MAC-ADDRESS>"
String Hostname;

int WiFi_reconnect = 0;

// for MQTT
long Mqtt_lastSend = 0;
long lastReconnectAttempt = 0;
int Mqtt_reconnect = -1;


// Initializes the espClient. 
WiFiClient ethClient;
MQTTClient mqttClient(256);

// Initialize WiFi
void initWiFi() {
    // dynamically determine hostname
  Hostname = HOSTNAME;
  Hostname += "_";
  Hostname += WiFi.macAddress();
  Hostname.replace(":", "");

  WiFi.hostname(Hostname);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  log_i("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    log_i(".");
    delay(1000);
  }
  log_i("%s",WiFi.localIP().toString().c_str());
}

  void initMQTT() {
  String willTopic = Hostname + "/LWT";
  
  log_i("setup MQTT\n");
  
  mqttClient.begin(ethClient);
  mqttClient.setHost(MQTT_BROKER, 1883);
  mqttClient.setWill(willTopic.c_str(), "Offline", true, 0);
}


// reconnect to WiFi 
void reconnect_wifi() {
  log_i("%s\n","WiFi try reconnect"); 
  WiFi_reconnect = WiFi_reconnect + 1;
  WiFi.disconnect();
  WiFi.reconnect();
  delay(500);
  if (WiFi.status() == WL_CONNECTED) {
    // Once connected, publish an announcement...
    log_i("%s\n","WiFi reconnected"); 
  }
}

// This functions reconnects your ESP32 to your MQTT broker
void reconnect_mqtt()
{
  String willTopic = Hostname + "/LWT";
  String cmdTopic = Hostname + "/CMD/+";

  log_i("%s\n", "MQTT try reconnect");

  Mqtt_reconnect = Mqtt_reconnect + 1;

  if (mqttClient.connect(Hostname.c_str()))
  {
    log_i("%s\n", "MQTT connected");

    mqttClient.publish(willTopic.c_str(), "Online", true, 0);
  
    mqttClient.subscribe(cmdTopic.c_str());
  } else {
    log_i("Failed to connect to broker; error: %d\n", mqttClient.lastError());
  }
}