#ifndef Config_h
#define Config_h


    // WiFi settings
    #define WIFI_SSID "WIFI_SSID"
    #define WIFI_PASSWORD "WIFI_PASSWORD"

    #define WIFI_RECOVER_TIME_MS 10000 // Wait 10 seconds after a failed connection attempt

    // OTA Update settings
    #define OTA_HOSTNAME "gdc-esp32"
    #define OTA_PASSWORD_MD5_HASH "d20bd8d97c20493e7cee1630ba61d6f0"

    // IO pins settings
    #define GARAGE_DOOR_OPENER_RELAY_SWITCH_GPIO 13

    #define GARAGE_DOOR_OPEN_MAGNETIC_SENSOR_GPIO 17
    #define GARAGE_DOOR_CLOSED_MAGNETIC_SENSOR_GPIO 18
    #define GARAGE_DOOR_OPENER_ACTIVE_SENSOR_GPIO 19

    #define GARAGE_DOOR_OPEN_INDICATOR_GREEN_LED_GPIO 21
    #define GARAGE_DOOR_OPENER_ACTIVE_INDICATOR_YELLOW_LED_GPIO 22
    #define GARAGE_DOOR_CLOSED_INDICATOR_RED_LED_GPIO 23

    #define GARAGE_OUTSIDE_TEMP_SENSOR_GPIO 27

    #define UNUSED_GPIO_1 25
    #define UNUSED_GPIO_2 26
    #define UNUSED_GPIO_3 32
    #define UNUSED_GPIO_4 33

    // MQTT settings
    #define MQTT_BROKER_HOST "MQTT_HOST"
    #define MQTT_BROKER_PORT 8883

    #define MQTT_CLIENT_ID "garage-door-controller"

    #define GARAGE_DOOR_OPENER_CONTROL_TOPIC "home/garage/door-opener/control"
    #define GARAGE_DOOR_CURRENT_STATE_TOPIC "home/garage/door/current-state"

    #define MQTT_RECOVER_TIME_MS 10000
    #define MQTT_REPUBLISH_INTERVAL_MS 10000

    // TLS settings
    #define ROOT_CA_CERTIFICATE "-----BEGIN CERTIFICATE-----\nRoot CA Certificate\n-----END CERTIFICATE-----\n"
    #define CLIENT_CERTIFICATE "-----BEGIN CERTIFICATE-----\nClient Certificate\n-----END CERTIFICATE-----\n"
    #define CLIENT_PRIVATE_KEY "-----BEGIN RSA PRIVATE KEY-----\nClient Private Key\n-----END RSA PRIVATE KEY-----\n"


#endif //Config_h