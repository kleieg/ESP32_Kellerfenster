// set hostname used for MQTT tag and WiFi
#define HOSTNAME "ESP-BME280"
#define MQTT_BROKER "192.168.178.15"
#define VERSION "v 1.2.0"

#define MQTT_INTERVAL 120000
#define RECONNECT_INTERVAL 5000

#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
#define SERIALINIT Serial.begin(115200);
#else
#define SERIALINIT
#endif