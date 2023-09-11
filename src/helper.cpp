#include <helper.h>

String getMacAddress()
{
	uint8_t baseMac[6];
	// Get MAC address for WiFi station
	esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
	char baseMacChr[18] = {0};
	sprintf(baseMacChr, "%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
	return String(baseMacChr);
}

boolean mqtt_reconnect() {
  t_mqtt_val* mqtt_v = getMQTTSetup();
  PubSubClient* mqtt_client = getMQTTServer();
  if (mqtt_client->connect(mqtt_v->mqttClientId, mqtt_v->mqttUser, mqtt_v->mqttPassword )) {
    mqtt_client->publish("heaterme/heat", "Hello World, again", true);
    mqtt_client->subscribe("heaterme/cmd");
  }
  return mqtt_client->connected();
}

