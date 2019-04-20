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
const char* root_ca_certificate = \
    "-----BEGIN CERTIFICATE-----\n" \
    "Root CA Certificate\n" \
    "-----END CERTIFICATE-----\n";

const char* client_certificate = \
    "-----BEGIN CERTIFICATE-----\n" \
    "Client Certificate\n" \
    "-----END CERTIFICATE-----\n";

const char* client_private_key = \
    "-----BEGIN RSA PRIVATE KEY-----\n" \
    "Client Private Key\n" \
    "-----END RSA PRIVATE KEY-----\n";


#endif //Setup_h