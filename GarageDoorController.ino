#include <Arduino.h>
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

void handleIncomingMqttMessage(char* topic, byte* message, unsigned int length) {
    String command;

    for (int i = 0; i < length; i++) {
        command += (char)message[i];
    }

    if (String(topic) == String(GARAGE_DOOR_OPENER_CONTROL_TOPIC) && command == "1") {
        turnOnGarageDoorOpenerRelayTriggerSwitch();
    }
}

WiFiClientSecure wifiClientSecure;
PubSubClient mqttClient(MQTT_BROKER_HOST, MQTT_BROKER_PORT, &handleIncomingMqttMessage, wifiClientSecure);

MillisDelay republishCurrentDoorStateDelay;
MillisDelay flakyDoorInMotionSensorDelay;
MillisDelay garageDoorOpenerRelaySwitchDelay;

DoorState lastDoorState = CLOSED;

void setup() {
    Serial.begin(115200);
    delay(10);

    wifiClientSecure.setCACert(ROOT_CA_CERTIFICATE);
    wifiClientSecure.setCertificate(CLIENT_CERTIFICATE);
    wifiClientSecure.setPrivateKey(CLIENT_PRIVATE_KEY);

    mqttClient.setSocketTimeout(10);

    initGPIO();

    republishCurrentDoorStateDelay.start(MQTT_REPUBLISH_INTERVAL_MS);
}

void loop() {
    if (ensureWifiConnection() && ensureConnectionToMqttBroker()) {
        runScheduledTasks();

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

bool ensureWifiConnection(){
    if (WiFi.status() != WL_CONNECTED){
        Serial.println("[WIFI] Connecting");
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

        unsigned long startAttemptTime = millis();

        // Keep looping while we're not connected and haven't reached the timeout
        while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS){}

        // When we couldn't make a WiFi connection (or the timeout expired)
        // sleep for a while and then retry.
        if (WiFi.status() == WL_CONNECTED){
            Serial.println("[WIFI] Connected: " + WiFi.localIP());
        } else {
            Serial.println("[WIFI] FAILED");
            delay(WIFI_RECOVER_TIME_MS);
        }
    }

    return WiFi.isConnected();
}

bool ensureConnectionToMqttBroker() {
    if (!mqttClient.connected()) {
        int numberOfFailedConnectionAttemts = 0;
        Serial.println("[MQTT] Connecting");

        if (mqttClient.connect(MQTT_CLIENT_ID)) {
            Serial.println("[MQTT] Connected to broker");
            mqttClient.subscribe(GARAGE_DOOR_OPENER_CONTROL_TOPIC);
            Serial.print("[MQTT] Subscribed to topic: ");
            Serial.println(GARAGE_DOOR_OPENER_CONTROL_TOPIC);
        } else {
            Serial.print("[MQTT] Connection failed, rc=");
            Serial.print(mqttClient.state());
            Serial.print(" - trying again in ");
            Serial.print(MQTT_RECOVER_TIME_MS);
            Serial.println(" milliseconds");
            delay(MQTT_RECOVER_TIME_MS);
        }
    }

    return mqttClient.connected();
}

void turnOnGarageDoorOpenerRelayTriggerSwitch() {
    digitalWrite(GARAGE_DOOR_OPENER_RELAY_SWITCH_GPIO, HIGH);
    garageDoorOpenerRelaySwitchDelay.start(500);
}

void runScheduledTasks() {
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
