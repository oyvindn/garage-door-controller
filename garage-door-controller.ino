#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

#include "setup.h"

WiFiClientSecure wifiClientSecure;
PubSubClient mqttPubSubClient(mqtt_broker_host, mqtt_broker_port, &handleIncomingMqttMessage, wifiClientSecure);

void setup() {
    Serial.begin(115200);
    delay(10);
    connectToWifi();

    wifiClientSecure.setCACert(root_ca);

    pinMode(garage_door_open_magnetic_sensor_gpio, INPUT);
    pinMode(garage_door_closed_magnetic_sensor_gpio, INPUT);
    pinMode(garage_door_opener_sensor_gpio, INPUT);
    pinMode(garage_door_opener_relay_switch_gpio, OUTPUT);
}

void loop() {
    //TODO: MQTT authentication
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
            mqttClient.subscribe(garage_door_operner_control_topic);
            Serial.print("Subscribed to topic ");
            Serial.println(garage_door_operner_control_topic);
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" trying again in 5 seconds");
            delay(5000);
        }
    }
}

void handleIncomingMqttMessage(char* topic, byte* message, unsigned int length) {
    Serial.print("Message arrived on topic ");
    Serial.print(topic);
    Serial.print(": ");
    String command;

    for (int i = 0; i < length; i++) {
        Serial.print((char)message[i]);
        command += (char)message[i];
    }
    Serial.println();

    if(topic == garage_door_operner_control_topic && command == "TRIGGER_DOOR") {
        triggerGarageDoorOpener();
    }
}

void triggerGarageDoorOpener() {
    Serial.println("Triggering garage door opener relay switch for 500ms");
    digitalWrite(garage_door_opener_relay_switch_gpio, HIGH);
    delay(500);
    digitalWrite(garage_door_opener_relay_switch_gpio, LOW);
}