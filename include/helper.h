#include <Arduino.h>
#include <ArduinoJson.h>
#include "esp_system.h"
// Konfiguration
#include <Preferences.h>
// Wifi
#include <WifiClient.h>
// MQTT
#include <PubSubClient.h>
#ifndef helper_h
#define helper_h

// Typen
typedef struct relaisss {
  int pin;
  int threshold;
} t_relais;

typedef struct power {
  unsigned long time;
  int requested_power;
} t_power_req;


typedef struct mqtt_werte {
  char mqttServer[1024]; 
  int  mqttPort; 
  char mqttUser[1024];
  char mqttPassword[1024];
  char mqttClientId[1024];
} t_mqtt_val;

// Externe Funktionen
extern t_mqtt_val* getMQTTSetup(void);
extern PubSubClient* getMQTTServer(void);

// Interne Funktionen
String getMacAddress();

boolean mqtt_reconnect();
#endif