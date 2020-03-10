#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

#include "Config.h"
#include "Debug.h"


WiFiClientSecure wifiClientSecure;
PubSubClient mqttClient(mqtt_broker_host, mqtt_broker_port, &handleIncomingMqttMessage, wifiClientSecure);

enum DoorState {
    OPEN = 0,
    CLOSED = 1,
    OPENING = 2,
    CLOSING = 3,
    STOPPED = 4,
    UNKNOWN = 5
};

int lastDoorState = UNKNOWN;
int lastDoorMovementState = UNKNOWN;

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

    int currentDoorState = getCurrentDoorState();
    if(currentDoorState != lastDoorState) {
        lastDoorState = currentDoorState;
        if((currentDoorState == OPENING || currentDoorState == CLOSING) && lastDoorMovementState != currentDoorState) {
            lastDoorMovementState = currentDoorState;
        }

        mqttClient.publish(garage_door_current_state_topic, "" + currentDoorState, true);
    }

    mqttClient.loop();
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
            mqttClient.subscribe(garage_door_opener_control_topic);
            DPRINT("Subscribed to topic ");
            DPRINTLN(garage_door_opener_control_topic);
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

    if(String(topic) == String(garage_door_opener_control_topic) && command == "1") {
        triggerGarageDoorOpener();
    }
}

void triggerGarageDoorOpener() {
    DPRINTLN("Triggering garage door opener relay switch for 500ms");
    digitalWrite(garage_door_opener_relay_switch_gpio, HIGH);
    delay(500); //TODO: avoid delay?
    digitalWrite(garage_door_opener_relay_switch_gpio, LOW);
}

int getCurrentDoorState() {
    int doorOpenerRunning = digitalRead(garage_door_opener_active_sensor_gpio);
    int doorOpen = digitalRead(garage_door_open_magnetic_sensor_gpio);
    int doorClosed = digitalRead(garage_door_closed_magnetic_sensor_gpio);
    
    if(doorOpenerRunning == HIGH) {
        if(lastDoorState == OPENING || lastDoorState == CLOSED) {
            return OPENING;
        } else if(lastDoorState == CLOSING || lastDoorState == OPEN) {
            return CLOSING;
        } else if(lastDoorState == STOPPED) {
            if(lastDoorMovementState == OPENING) {
                return CLOSING;
            } else if(lastDoorMovementState == CLOSING) {
                return OPENING;
            } else {
                UNKNOWN;
            }
        } else {
           return UNKNOWN; 
        }
    } else if(doorOpen == HIGH) {
        return OPEN;
    } else if(doorClosed == HIGH) {
        return CLOSED;
    } else {
        return STOPPED;
    }
}
