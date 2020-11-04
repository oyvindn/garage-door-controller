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
    MOVING_IN_UNKNOWN_DIRECTION = 5,
};

class DoorSensorsState {
    private:
    bool doorOpen, doorClosed, doorInMotion;

    public:

    DoorSensorsState(bool doorOpen, bool doorClosed, bool doorInMotion) {
        this->doorOpen = doorOpen;
        this->doorClosed = doorClosed;
        this->doorInMotion = doorInMotion;
    }

    DoorState calculateDoorState(DoorState lastDoorState) {
        if (doorInMotion) {
            if (lastDoorState == CLOSED) {
                return OPENING;
            } else if (lastDoorState == OPEN) {
                return CLOSING;
            } else if (lastDoorState == STOPPED) {
                return MOVING_IN_UNKNOWN_DIRECTION;
            } else {
                return lastDoorState; 
            }
        } else if (doorOpen) {
            return OPEN;
        } else if (doorClosed) {
            return CLOSED;
        } else {
            return STOPPED;
        }
    }
};

WiFiClientSecure wifiClientSecure;
PubSubClient mqttClient(config::mqtt_broker_host, config::mqtt_broker_port, &handleIncomingMqttMessage, wifiClientSecure);

MillisDelay republishCurrentDoorStateDelay;
MillisDelay flakyDoorOpenerSignalDelay;
MillisDelay garageDoorOpenerRelaySwitchDelay;

DoorState lastDoorState = CLOSED;

void setup() {
    Serial.begin(115200);
    delay(10);
    connectToWifi();

    wifiClientSecure.setCACert(config::root_ca_certificate);
    wifiClientSecure.setCertificate(config::client_certificate);
    wifiClientSecure.setPrivateKey(config::client_private_key);

    mqttClient.setSocketTimeout(10);

    initGPIO();
    
    republishCurrentDoorStateDelay.start(60000);
}

void loop() {
    ensureConnectionToMqttBroker();

    if(garageDoorOpenerRelaySwitchDelay.justFinished()) {
        digitalWrite(config::garage_door_opener_relay_switch_gpio, LOW);
    }

    DoorSensorsState doorSensorsState = readDoorSensors();
    DoorState currentDoorState = doorSensorsState.calculateDoorState(lastDoorState);
    
    DPRINTLN();
    DPRINT("Current door state: ");
    DPRINTLN(currentDoorState);

    if (currentDoorState == lastDoorState) {
        flakyDoorOpenerSignalDelay.stop();
        if(republishCurrentDoorStateDelay.justFinished()) {
            republishCurrentDoorStateDelay.restart();
            publishToMqtt(config::garage_door_current_state_topic, currentDoorState, true);
        }
    } else {
        /*
         * The garage_door_opener_active_sensor use a output signal from the garage door opener ment for controlling a warning light.
         * This output signal is a bit flaky and will sporadically read as HIGH, even though the opener is inactive.
         * To remedy this we make sure the sinal is HIGH for å minimum time frame to make sure the opener is actual active and the door in motion.
         */
        if ((currentDoorState == OPENING || currentDoorState == CLOSING || currentDoorState == MOVING_IN_UNKNOWN_DIRECTION) && !flakyDoorOpenerSignalDelay.justFinished()) {
            if(!flakyDoorOpenerSignalDelay.isRunning()) {
                flakyDoorOpenerSignalDelay.start(100);
            }
        } else {
            publishToMqtt(config::garage_door_current_state_topic, currentDoorState, true);
        }
    }

    mqttClient.loop();
    lastDoorState = currentDoorState;
}

void initGPIO() {
    pinMode(config::garage_door_opener_relay_switch_gpio, OUTPUT);

    pinMode(config::garage_door_opener_active_sensor_gpio, INPUT_PULLDOWN);
    pinMode(config::garage_door_open_magnetic_sensor_gpio, INPUT);
    pinMode(config::garage_door_closed_magnetic_sensor_gpio, INPUT);

    pinMode(config::garage_door_open_indicator_green_led_gpio, OUTPUT);
    pinMode(config::garage_door_opener_active_indicator_yellow_led_gpio, OUTPUT);
    pinMode(config::garage_door_closed_indicator_red_led_gpio, OUTPUT);
}

void connectToWifi() {
    DPRINTLN();
    DPRINT("Connecting to ");
    DPRINTLN(ssid);

    WiFi.begin(config::ssid, config::password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        DPRINT(".");
    }

    DPRINTLN();
    DPRINTLN("WiFi connected");
    DPRINT("IP address: ");
    DPRINTLN(WiFi.localIP());
}

void ensureConnectionToMqttBroker() {
    while (!mqttClient.connected()) {
        int numberOfFailedConnectionAttemts = 0;
        DPRINTLN();
        DPRINT("Attempting MQTT connection...");

        if (mqttClient.connect("garage-door-controller")) {
            DPRINTLN("connected");
            mqttClient.subscribe(config::garage_door_opener_control_topic);
            DPRINT("Subscribed to topic ");
            DPRINTLN(garage_door_opener_control_topic);
        } else {
            numberOfFailedConnectionAttemts += 1;
            int waitMilliseconds = numberOfFailedConnectionAttemts > 5 ? 5000 : 500;

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

    if (String(topic) == String(config::garage_door_opener_control_topic) && command == "1") {
        triggerGarageDoorOpener();
    }
}

void triggerGarageDoorOpener() {
    DPRINTLN("Triggering garage door opener relay switch for 500ms");
    digitalWrite(config::garage_door_opener_relay_switch_gpio, HIGH);
    garageDoorOpenerRelaySwitchDelay.start(500);
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

DoorSensorsState readDoorSensors() {
    bool doorOpen = digitalRead(config::garage_door_open_magnetic_sensor_gpio) == LOW;
    bool doorClosed = digitalRead(config::garage_door_closed_magnetic_sensor_gpio) == LOW;
    bool doorInMotion = digitalRead(config::garage_door_opener_active_sensor_gpio) == HIGH;

    digitalWrite(config::garage_door_open_indicator_green_led_gpio, doorOpen ? HIGH : LOW);
    digitalWrite(config::garage_door_opener_active_indicator_yellow_led_gpio, doorInMotion ? HIGH : LOW);
    digitalWrite(config::garage_door_closed_indicator_red_led_gpio, doorClosed ? HIGH : LOW);

    return DoorSensorsState(doorOpen, doorClosed, doorInMotion);
}
