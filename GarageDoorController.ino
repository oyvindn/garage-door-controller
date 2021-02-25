#include <Arduino.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

#include "Config.h"
#include "MillisDelay.h"

enum DoorState {
    OPEN = 0,
    CLOSED = 1,
    OPENING = 2,
    CLOSING = 3,
    STOPPED = 4,
    MOVING_IN_UNKNOWN_DIRECTION = 5,
};

struct DoorSensorsReading {
    bool doorOpen, doorClosed, doorInMotion;

    DoorSensorsReading(bool doorOpen, bool doorClosed, bool doorInMotion) {
        this->doorOpen = doorOpen;
        this->doorClosed = doorClosed;
        this->doorInMotion = doorInMotion;
    }
};

void handleIncomingMqttMessage(char* topic, byte* message, unsigned int length);

WiFiClientSecure wifiClientSecure;
PubSubClient mqttClient(MQTT_BROKER_HOST, MQTT_BROKER_PORT, &handleIncomingMqttMessage, wifiClientSecure);

MillisDelay republishCurrentDoorStateDelay;
MillisDelay flakyDoorInMotionSensorDelay;
MillisDelay garageDoorOpenerRelaySwitchDelay;

DoorState lastDoorState = CLOSED;

void setup() {
    Serial.begin(115200);
    delay(10);

    initGPIO();

    ensureWifiConnection();

    wifiClientSecure.setCACert(root_ca_certificate);
    wifiClientSecure.setCertificate(client_certificate);
    wifiClientSecure.setPrivateKey(client_private_key);

    mqttClient.setSocketTimeout(10);

    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPasswordHash(OTA_PASSWORD_MD5_HASH);

    ArduinoOTA.onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {
        type = "filesystem";
      }
      Serial.println("[OTA] Start updating " + type);
    }).onEnd([]() {
      Serial.println("\n[OTA] End");
    }).onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("[OTA] Progress: %u%%\r", (progress / (total / 100)));
    }).onError([](ota_error_t error) {
      Serial.printf("[OTA] Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

    ArduinoOTA.begin();

    republishCurrentDoorStateDelay.start(MQTT_REPUBLISH_INTERVAL_MS);
}

void loop() {
    ensureGarageDoorOpenerTriggerRelease();
    ensureWifiConnection();
    ArduinoOTA.handle();

    if(connectedToMqttBroker()) {
        DoorSensorsReading doorSensorsState = readDoorSensors();
        DoorState currentDoorState = calculateDoorState(doorSensorsState, lastDoorState);

        if (currentDoorState != lastDoorState) {
            /*
            * The garage_door_opener_active_sensor use a output signal from the garage door opener ment for controlling a warning light.
            * This output signal is a bit flaky and will sporadically read as HIGH, even though the opener is inactive.
            * To remedy this we make sure the sinal is HIGH for å minimum time frame to make sure the opener is actual active and the door in motion.
            */
            if (doorSensorsState.doorInMotion && !flakyDoorInMotionSensorDelay.justFinished() && !flakyDoorInMotionSensorDelay.isRunning()) {
                flakyDoorInMotionSensorDelay.start(100);
            } else {
                publishToMqtt(GARAGE_DOOR_CURRENT_STATE_TOPIC, currentDoorState, true);
            }
        } else {
            flakyDoorInMotionSensorDelay.stop();
            if (republishCurrentDoorStateDelay.justFinished()) {
                republishCurrentDoorStateDelay.restart();
                publishToMqtt(GARAGE_DOOR_CURRENT_STATE_TOPIC, currentDoorState, true);
            }
        }

        mqttClient.loop();
        lastDoorState = currentDoorState;
    }
}

void initGPIO() {
    pinMode(GARAGE_DOOR_OPENER_RELAY_SWITCH_GPIO, OUTPUT);

    pinMode(GARAGE_DOOR_OPENER_ACTIVE_SENSOR_GPIO, INPUT_PULLDOWN);
    pinMode(GARAGE_DOOR_OPEN_MAGNETIC_SENSOR_GPIO, INPUT);
    pinMode(GARAGE_DOOR_CLOSED_MAGNETIC_SENSOR_GPIO, INPUT);

    pinMode(GARAGE_DOOR_OPEN_INDICATOR_GREEN_LED_GPIO, OUTPUT);
    pinMode(GARAGE_DOOR_OPENER_ACTIVE_INDICATOR_YELLOW_LED_GPIO, OUTPUT);
    pinMode(GARAGE_DOOR_CLOSED_INDICATOR_RED_LED_GPIO, OUTPUT);
}

void ensureWifiConnection() {
    if (WiFi.status() != WL_CONNECTED){
        Serial.println("[WIFI] Connecting");
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

        if (WiFi.waitForConnectResult() == WL_CONNECTED){
            Serial.println("[WIFI] Connected: " + WiFi.localIP());
        } else {
            Serial.println("[WIFI] Failed");
            delay(WIFI_RECOVER_TIME_MS);
            ESP.restart();
        }
    }
}

bool connectedToMqttBroker() {
    if (!mqttClient.connected()) {
        Serial.println("[MQTT] Connecting");

        if (mqttClient.connect(MQTT_CLIENT_ID)) {
            Serial.println("[MQTT] Connected to broker");
            mqttClient.subscribe(GARAGE_DOOR_OPENER_CONTROL_TOPIC);
            Serial.print("[MQTT] Subscribed to topic: ");
            Serial.println(GARAGE_DOOR_OPENER_CONTROL_TOPIC);
        } else {
            Serial.print("[MQTT] Connection failed, rc=");
            Serial.println(mqttClient.state());
            //TODO: replace delay() with non blocking delay
            delay(MQTT_RECOVER_TIME_MS);
        }
    }

    return mqttClient.connected();
}

void handleIncomingMqttMessage(char* topic, byte* message, unsigned int length) {
    String command;

    for (int i = 0; i < length; i++) {
        command += (char)message[i];
    }

    if (String(topic) == String(GARAGE_DOOR_OPENER_CONTROL_TOPIC) && command == "1") {
        triggerGarageDoorOpener();
    }
}

void triggerGarageDoorOpener() {
    digitalWrite(GARAGE_DOOR_OPENER_RELAY_SWITCH_GPIO, HIGH);
    garageDoorOpenerRelaySwitchDelay.start(500);
}

void ensureGarageDoorOpenerTriggerRelease() {
    if (garageDoorOpenerRelaySwitchDelay.justFinished() || !garageDoorOpenerRelaySwitchDelay.isRunning()) {
        digitalWrite(GARAGE_DOOR_OPENER_RELAY_SWITCH_GPIO, LOW);
    }
}

void publishToMqtt(const char* mqttTopic, int value, boolean retained) {
    mqttClient.publish(mqttTopic, String(value).c_str(), retained);
}

DoorSensorsReading readDoorSensors() {
    bool doorOpen = digitalRead(GARAGE_DOOR_OPEN_MAGNETIC_SENSOR_GPIO) == LOW;
    bool doorClosed = digitalRead(GARAGE_DOOR_CLOSED_MAGNETIC_SENSOR_GPIO) == LOW;
    bool doorInMotion = digitalRead(GARAGE_DOOR_OPENER_ACTIVE_SENSOR_GPIO) == HIGH;

    digitalWrite(GARAGE_DOOR_OPEN_INDICATOR_GREEN_LED_GPIO, doorOpen ? HIGH : LOW);
    digitalWrite(GARAGE_DOOR_OPENER_ACTIVE_INDICATOR_YELLOW_LED_GPIO, doorInMotion ? HIGH : LOW);
    digitalWrite(GARAGE_DOOR_CLOSED_INDICATOR_RED_LED_GPIO, doorClosed ? HIGH : LOW);

    return DoorSensorsReading(doorOpen, doorClosed, doorInMotion);
}

DoorState calculateDoorState(DoorSensorsReading doorSensorReading, DoorState lastDoorState) {
    if (doorSensorReading.doorInMotion) {
        if (lastDoorState == CLOSED) {
            return OPENING;
        } else if (lastDoorState == OPEN) {
            return CLOSING;
        } else if (lastDoorState == STOPPED) {
            return MOVING_IN_UNKNOWN_DIRECTION;
        } else {
            return lastDoorState;
        }
    } else if (doorSensorReading.doorOpen) {
        return OPEN;
    } else if (doorSensorReading.doorClosed) {
        return CLOSED;
    } else {
        return STOPPED;
    }
}
