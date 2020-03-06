#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

#include "Debug.h"
#include "Config.h"

WiFiClientSecure wifiClientSecure;
PubSubClient mqttClient(mqtt_broker_host, mqtt_broker_port, &handleIncomingMqttMessage, wifiClientSecure);

void setup() {
    Serial.begin(115200);
    delay(10);
    connectToWifi();

    wifiClientSecure.setCACert(root_ca_certificate);
    wifiClientSecure.setCertificate(client_certificate);
    wifiClientSecure.setPrivateKey(client_private_key);

    pinMode(garage_door_open_magnetic_sensor_gpio, INPUT_PULLDOWN);
    pinMode(garage_door_closed_magnetic_sensor_gpio, INPUT_PULLDOWN);
    pinMode(garage_door_opener_active_sensor_gpio, INPUT_PULLDOWN);
    pinMode(garage_door_opener_relay_switch_gpio, OUTPUT);
}

void loop() {
    connectToMqttBroker();

    readSensorValueAndPublishToMqtt(garage_door_opener_active_sensor_gpio, garage_door_opener_active_sensor_topic);
    readSensorValueAndPublishToMqtt(garage_door_open_magnetic_sensor_gpio, garage_door_open_sensor_topic);
    readSensorValueAndPublishToMqtt(garage_door_closed_magnetic_sensor_gpio, garage_door_closed_sensor_topic);

    mqttClient.loop();

    delay(500);
}

void connectToWifi() {
    DPRINT();
    DPRINT("Connecting to ");
    DPRINTLN(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        DPRINT(".");
    }

    DPRINTLN();
    DPRINTLN("WiFi connected");
    DPRINT("IP address: ");
    DPRINTLN(WiFi.localIP());
}

void connectToMqttBroker() {
    while (!mqttClient.connected()) {
        DPRINT("Attempting MQTT connection...");

        if (mqttClient.connect("garage-door-controller")) {
            DPRINTLN("connected");
            mqttClient.subscribe(garage_door_operner_control_topic);
            DPRINT("Subscribed to topic ");
            DPRINTLN(garage_door_operner_control_topic);
        } else {
            DPRINT("failed, rc=");
            DPRINT(mqttClient.state());
            DPRINTLN(" trying again in 5 seconds");
            delay(5000);
        }
    }
}

void handleIncomingMqttMessage(char* topic, byte* message, unsigned int length) {
    String command;

    for (int i = 0; i < length; i++) {
        command += (char)message[i];
    }

    DPRINT("Message arrived on topic ");
    DPRINT(topic);
    DPRINT(": ");
    DPRINTLN(command);

    if(String(topic) == String(garage_door_operner_control_topic) && command == "1") {
        triggerGarageDoorOpener();
    }
}

void triggerGarageDoorOpener() {
    DPRINTLN("Triggering garage door opener relay switch for 500ms");
    digitalWrite(garage_door_opener_relay_switch_gpio, HIGH);
    delay(500);
    digitalWrite(garage_door_opener_relay_switch_gpio, LOW);
}

void readSensorValueAndPublishToMqtt(const int &sensorGpio, const char* mqttTopic) {
    int state = digitalRead(sensorGpio);
    mqttClient.publish(mqttTopic, state == HIGH ? "1" : "0");
}