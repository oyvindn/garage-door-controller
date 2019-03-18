#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

#include "setup.h"

WiFiClientSecure wifiClientSecure;
PubSubClient mqttClient(mqtt_broker_host, mqtt_broker_port, &callback, wifiClientSecure);

void setup() {
  Serial.begin(115200);
  delay(10);
  setup_wifi();
  
  wifiClientSecure.setCACert(root_ca);
}

void loop() {
    if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();

}

void setup_wifi() {
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

void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect("GarageDoor")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      mqttClient.publish("sensors","hello world");
      // ... and resubscribe
      mqttClient.subscribe("commands");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte* message, unsigned int length) {
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