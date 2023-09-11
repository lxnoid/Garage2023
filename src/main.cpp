#include <Arduino.h>
#include "LittleFS.h"
#include <ArduinoJson.h>
#include "esp_system.h"
// Wifi
#include <WifiClient.h>
#include <WiFiManager.h>
// MQTT
#include <PubSubClient.h>
#include <cppQueue.h>
// local resource
#include <helper.h>

char wifiSsid[1024]      = "";  
char wifiPassword[1024]  = "";
char mqttServer[1024]    = ""; 
int  mqttPort            = 0; 
char mqttUser[1024]      = "";
char mqttPassword[1024]  = "";
char mqttClientId[1024]  = "";

void mqtt_callback (char* topic, byte* payload, unsigned int length);
void IRAM_ATTR reedISR(void);
// ==================================== Globals ==========================================

// -> Zeiten
unsigned long current_millis = 0;
unsigned long prev_millis_led = 0;
unsigned long prev_millis_mqtt = 0;

// -> MQTT & Wifi
WiFiClient wifi_client;
t_mqtt_val mqtt_v;
PubSubClient mqtt_client(wifi_client);
int mqtt_status = (int)(20 * 1000); // in Millisekunden

enum Request {
  eUNDEFINED_EVENT = 0,
  ePUSH = 1
};

#define IMPLEMENTATION FIFO	
cppQueue qRequest(sizeof(enum Request), 5, IMPLEMENTATION);

//pins
int boardLEDPin = 2;
int pinRelais = 25;
int pinReed = 26; 


// ==================================== Setup ==========================================
void setup()
{
  Serial.begin(115200);
  //read config
  if (!LittleFS.begin()) {
    Serial.println("LITTLEFS Mount Failed");
  }
  
  File cfile = LittleFS.open("/config.json", "r");
  if (cfile) {
    String config_data;
    while (cfile.available()) {
      config_data += char(cfile.read());
    }
    Serial.print(config_data);   
    DynamicJsonDocument cjson(config_data.length());
    deserializeJson(cjson, config_data);

    const char* c_wifiSsid     = cjson["wifiSsid"];  
    const char* c_wifiPassword = cjson["wifiPassword"];
    const char* c_mqttServer   = cjson["mqttServer"]; 
    const char* c_mqttPort     = cjson["mqttPort"]; 
    const char* c_mqttUser     = cjson["mqttUser"]; 
    const char* c_mqttPassword = cjson["mqttPassword"];
    const char* c_mqttClientId = cjson["mqttClientID"];

    Serial.print(c_wifiSsid);

    if (strlen(c_wifiSsid)) { 
      sprintf(wifiSsid, "%s", c_wifiSsid); 
    }
    if (strlen(c_wifiPassword)) { 
      sprintf(wifiPassword, "%s", c_wifiPassword); 
      }
    if (strlen(c_mqttServer)) { 
      sprintf(mqtt_v.mqttServer, "%s", c_mqttServer); 
    }
    if (strlen(c_mqttPort)) { 
      mqtt_v.mqttPort = atoi(c_mqttPort);
    }
    if (strlen(c_mqttUser)) { 
      sprintf(mqtt_v.mqttUser, "%s", c_mqttUser);
    }
    if (strlen(c_mqttPassword)) { 
      sprintf(mqtt_v.mqttPassword, "%s", c_mqttPassword);
    }
    if (strlen(c_mqttClientId)) {
      sprintf(mqtt_v.mqttClientId, "%s", c_mqttClientId);
    }
  } else {
    Serial.print("Config File missing.");
    for (;;)
      delay(1);
  }
  delay(100);
  cfile.close();
  
  
  pinMode(boardLEDPin, OUTPUT);
  pinMode(pinRelais, OUTPUT);
  pinMode(pinReed, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinReed), reedISR, CHANGE);
  
  digitalWrite(pinRelais, HIGH);

  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  //WiFi.setOutputPower(10.0);
  WiFi.setTxPower(WIFI_POWER_11dBm);
  //WiFi.setPhyMode(WIFI_PHY_MODE_11N);
  WiFi.persistent(false);
  //Wifi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.setSleep(WIFI_PS_NONE);
  WiFi.setAutoReconnect(true);
  
  WiFi.begin(wifiSsid, wifiPassword);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("...Connecting to WiFi");
    delay(1000);
  }  
  Serial.println("Connected to WiFi");
  
  delay(100);
 
  char mac_address[13];
  getMacAddress().toCharArray(mac_address, 13);
  snprintf(mqtt_v.mqttClientId, 1024, "%s", mac_address);
  // MQTT ------------------------------------------------------------------------------
  mqtt_client.setServer(mqtt_v.mqttServer, mqtt_v.mqttPort);
  mqtt_client.setCallback(mqtt_callback);

  while (!mqtt_client.connected())
  {
    Serial.println("...Connecting to MQTT");
    if (mqtt_client.connect(mqtt_v.mqttClientId, mqtt_v.mqttUser, mqtt_v.mqttPassword))
    {
      Serial.println("Connected to MQTT");
    }
    else
    {
      Serial.print("Failed connecting MQTT with state: ");
      Serial.print(mqtt_client.state());
      delay(2000);
    }
  }

  mqtt_client.publish("garage/status", "Hello World.", false);
  mqtt_client.subscribe("garage/cmd");

  Serial.println("-- End of Setup --");
}

static bool triggerMQTT = false;
// ==================================== Loop ==========================================
void loop()
{
  static bool startup = false;
  static bool isBoardLEDon = false;
  static bool newRequest = false;
  int* ip;

  current_millis = millis();
  mqtt_client.loop();

  if (!startup)
  {
    // Check wi-fi is connected to wi-fi network
    Serial.println("WiFi connected successfully");
    Serial.print("Got IP: ");
    Serial.println(WiFi.localIP()); // Show ESP32 IP on serial
    startup = true;
  }

  if (current_millis - prev_millis_led >= 2500)
  {
    prev_millis_led = current_millis;
    if (isBoardLEDon)
    {
      digitalWrite(boardLEDPin, LOW);
      isBoardLEDon = false;
    }
    else
    {
      digitalWrite(boardLEDPin, HIGH);
      isBoardLEDon = true;
    }
  }

  while (qRequest.getCount() > 0) 
  {
    // just want the last one
    qRequest.pop(&ip);
    free(ip);
    newRequest = true;
  }


  if (newRequest) {
    newRequest = false;
    digitalWrite(pinRelais, LOW);
    delay(1000);
    digitalWrite(pinRelais, HIGH);
    delay(100);
    triggerMQTT = true;
  }

  if (((int)(current_millis - prev_millis_mqtt) >= mqtt_status) || triggerMQTT)
  {
    char mqtt_message[500];
    char mqtt_topic[120];
    delay(100);
    int door_status = digitalRead(pinReed);

    snprintf(mqtt_topic, 120, "garage/status");
    snprintf(mqtt_message, 500, "{ \"position\" : %d }", door_status);

    String temp_s = "";
    boolean status = mqtt_client.publish(mqtt_topic, mqtt_message, 80);
    if (!status)
    {
      Serial.println("MQTT - sending failed: " + status);
    }
    triggerMQTT = false;
    prev_millis_mqtt = current_millis;
  }


  // check if MQTT and Wifi connected (wifi is set to reconnect on auto)
  if (!mqtt_client.connected())
  {
    mqtt_reconnect();
    delay(200);
  }
}

//variables to keep track of the timing of recent interrupts
unsigned long reed_time = 0;  
unsigned long last_reed_time = 0; 

void IRAM_ATTR reedISR(void) {
  reed_time = millis();
  if (reed_time - last_reed_time > 300)
  {
    Serial.println("x");
    triggerMQTT = true;
    last_reed_time = reed_time;
  }

}

// ==================================== Getters ==========================================

t_mqtt_val *getMQTTSetup(void)
{
  return &mqtt_v;
}

PubSubClient *getMQTTServer(void)
{
  return &mqtt_client;
}

void mqtt_callback (char* topic, byte* payload, unsigned int length) 
{
  char c_payload[length + 1];
  Serial.print("MQTT_callback -> Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("MQTT_callback -> Message: ");
  for (unsigned int i = 0; i < length; i++) {
    c_payload[i] = (char)payload[i];
  }
  c_payload[length] = '\0';
  Serial.print(c_payload);
  Serial.println();

  if (strcmp(c_payload, "Push") == 0)
  {
    int* pi = (int*)malloc(sizeof(int));
    *pi = (int)ePUSH;
    qRequest.push(&pi);
    Serial.println("MQTT_callback -> Valid Command!");
  }
}
 
