#ifndef Setup_h
#define Setup_h

// WiFi settings
const char* ssid     = "WIFI_SSID";
const char* password = "WIFI_PASSWORD";

// IO pins settings
const int garage_door_open_magnetic_sensor_gpio   = 12;
const int garage_door_closed_magnetic_sensor_gpio = 27;
const int garage_door_opener_sensor_gpio          = 13;
const int garage_door_opener_relay_switch_gpio   = 33;

// MQTT broker settings
const char* mqtt_broker_host = "MQTT_HOST";
const int mqtt_broker_port   = 8883;
const char* mqtt_user        = "MQTT_USERNAME";
const char* mqtt_password    = "MQTT_PASSWORD";

// MQTT topics
const char* garage_door_operner_control_topic       = "home/garage/door-opener/control";
const char* garage_door_opener_active_sensor_topic  = "home/garage/door-opener/active";
const char* garage_door_open_sensor_topic           = "home/garage/door/open";
const char* garage_door_closed_sensor_topic         = "home/garage/door/closed";

// TLS settings
const char* root_ca = \
    "-----BEGIN CERTIFICATE-----\n" \
    "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\n" \
    "15T5+EGHs8ccHQWyYQ6gqYk8o4JgP4rSJqO1sMI=\n" \
    "-----END CERTIFICATE-----\n";

#endif //Setup_h