#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

#include "Config.h"
#include "Debug.h"
#include "MillisDelay.h"

enum DoorState {
    OPEN = 0,
    CLOSED = 1,
    OPENING = 2,
    CLOSING = 3,
    STOPPED = 4,
    MOVING_IN_UNKNOWN_DIRECTION = 5
};

struct SensorValues {
    int doorInMotion, doorOpen, doorClosed;
};

WiFiClientSecure wifiClientSecure;
PubSubClient mqttClient(mqtt_broker_host, mqtt_broker_port, &handleIncomingMqttMessage, wifiClientSecure);

MillisDelay legacySensorPublishDelay;

int lastDoorState = -1;
int lastDoorMovementState = -1;

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

    legacySensorPublishDelay.start(500);
}

void loop() {
    connectToMqttBroker();

    DPRINTLN();

    SensorValues sensorValues = readSensors();
    DPRINT("Sensor readings - garage_door_open_magnetic_sensor: ");
    DPRINT(sensorValues.doorOpen);
    DPRINT(", garage_door_closed_magnetic_sensor: ");
    DPRINT(sensorValues.doorClosed);
    DPRINT(", garage_door_opener_active_sensor: ");
    DPRINTLN(sensorValues.doorOpenerRunning);

    int currentDoorState = determineCurrentDoorState(sensorValues, lastDoorState, lastDoorMovementState);
    DPRINT("Current door state: ");
    DPRINTLN(currentDoorState);
    if (currentDoorState != lastDoorState) {
        lastDoorState = currentDoorState;
        if (currentDoorState == OPENING || currentDoorState == CLOSING) {
            lastDoorMovementState = currentDoorState;
        } else if (currentDoorState == OPEN) {
            lastDoorMovementState = OPENING;
        } else if (currentDoorState = CLOSED) {
            lastDoorMovementState = CLOSING;
        }

        publishToMqtt(garage_door_current_state_topic, currentDoorState, true);
    }

    if (legacySensorPublishDelay.justFinished()) {
        legacySensorPublishDelay.repeat();
        publishToMqtt(garage_door_opener_active_sensor_topic, sensorValues.doorInMotion, false);
        publishToMqtt(garage_door_open_sensor_topic, sensorValues.doorOpen, false);
        publishToMqtt(garage_door_closed_sensor_topic, sensorValues.doorClosed, false);
    }

    mqttClient.loop();
}

void connectToWifi() {
    DPRINTLN();
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
        int numberOfFailedConnectionAttemts = 0;
        DPRINTLN();
        DPRINT("Attempting MQTT connection...");

        if (mqttClient.connect("garage-door-controller")) {
            DPRINTLN("connected");
            mqttClient.subscribe(garage_door_opener_control_topic);
            DPRINT("Subscribed to topic ");
            DPRINTLN(garage_door_opener_control_topic);
        } else {
            numberOfFailedConnectionAttemts += 1;
            int waitMilliseconds = 500;
            if (numberOfFailedConnectionAttemts > 5) {
                waitMilliseconds = 5000;
            }
            DPRINT("failed, rc=");
            DPRINT(mqttClient.state());
            DPRINTLN(" trying again in 1 seconds");
            delay(waitMilliseconds);
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

    if (String(topic) == String(garage_door_opener_control_topic) && command == "1") {
        triggerGarageDoorOpener();
    }
}

void triggerGarageDoorOpener() {
    DPRINTLN("Triggering garage door opener relay switch for 500ms");
    digitalWrite(garage_door_opener_relay_switch_gpio, HIGH);
    delay(500); //TODO: avoid delay?
    digitalWrite(garage_door_opener_relay_switch_gpio, LOW);
}

void publishToMqtt(const char* mqttTopic, int value, boolean retained) {
    DPRINTLN();
    DPRINT("Publishing to MQTT - topic: ");
    DPRINT(mqttTopic);
    DPRINT(" , value: ");
    DPRINT(String(value).c_str());
    DPRINTLN();
    mqttClient.publish(mqttTopic, String(value).c_str(), retained);
}

struct SensorValues readSensors() {
    int doorInMotion1 = digitalRead(garage_door_opener_active_sensor_gpio);
    int doorOpen = digitalRead(garage_door_open_magnetic_sensor_gpio);
    int doorInMotion2 = digitalRead(garage_door_opener_active_sensor_gpio);
    int doorClosed = digitalRead(garage_door_closed_magnetic_sensor_gpio);
    int doorInMotion3 = digitalRead(garage_door_opener_active_sensor_gpio);
    int doorInMotion = (doorInMotion1 + doorInMotion2 + doorInMotion3) == 3 ? HIGH : LOW;

    return {
        doorInMotion,
        doorOpen,
        doorClosed
    };
}

int determineCurrentDoorState(const struct SensorValues &sensorValues, const int &lastDoorState, const int &lastDoorMovementState) {
    if (sensorValues.doorInMotion == HIGH) {
        if (lastDoorState == OPENING || lastDoorState == CLOSED) {
            return OPENING;
        } else if (lastDoorState == CLOSING || lastDoorState == OPEN) {
            return CLOSING;
        } else if (lastDoorState == STOPPED) {
            if (lastDoorMovementState == OPENING) {
                return CLOSING;
            } else if (lastDoorMovementState == CLOSING) {
                return OPENING;
            } else {
                return MOVING_IN_UNKNOWN_DIRECTION;
            }
        } else {
           return MOVING_IN_UNKNOWN_DIRECTION; 
        }
    } else if (sensorValues.doorOpen == HIGH) {
        return OPEN;
    } else if (sensorValues.doorClosed == HIGH) {
        return CLOSED;
    } else {
        return STOPPED;
    }
}
