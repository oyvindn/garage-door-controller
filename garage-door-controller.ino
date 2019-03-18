#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

#include "setup.h"

WiFiClientSecure wifiClientSecure;
PubSubClient mqttPubSubClient(mqtt_broker_host, mqtt_broker_port, &subscriptionCallback, wifiClientSecure);

void setup() {
  Serial.begin(115200);
  delay(10);
  connectToWifi();
  
  wifiClientSecure.setCACert(root_ca);
}

void loop() {
  ensureMqttConnection(mqttPubSubClient);
  mqttPubSubClient.loop();
}

void connectToWifi() {
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void ensureMqttConnection(PubSubClient &mqttClient) {
  if (!mqttClient.connected()) {
    connectToMqttBroker(mqttClient);
  }
}

void connectToMqttBroker(PubSubClient &mqttClient) {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    if (mqttClient.connect("garage-door-controller")) {
      Serial.println("connected");
      subscribeOnConnect(mqttClient);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" trying again in 5 seconds");
      delay(5000);
    }
  }
}

void subscribeOnConnect(PubSubClient &mqttClient) {
  mqttClient.subscribe("home/garage/door/trigger");
}

void subscriptionCallback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
}