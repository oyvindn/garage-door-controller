#ifndef Config_h
#define Config_h

namespace config {

    // WiFi settings
    const char *ssid = "WIFI_SSID";
    const char *password = "WIFI_PASSWORD";

    // IO pins settings
    const int garage_door_opener_relay_switch_gpio = 13;

    const int garage_door_open_magnetic_sensor_gpio = 17;
    const int garage_door_closed_magnetic_sensor_gpio = 18;
    const int garage_door_opener_active_sensor_gpio = 19;

    const int garage_door_open_indicator_green_led_gpio = 21;
    const int garage_door_opener_active_indicator_yellow_led_gpio = 22;
    const int garage_door_closed_indicator_red_led_gpio = 23;

    const int garage_outside_temp_sensor_gpio = 27;

    const int unused_gpio_1 = 25;
    const int unused_gpio_2 = 26;
    const int unused_gpio_3 = 32;
    const int unused_gpio_4 = 33;

    // MQTT broker settings
    const char *mqtt_broker_host = "MQTT_HOST";
    const int mqtt_broker_port = 8883;

    // MQTT topics
    const char *garage_door_opener_control_topic = "home/garage/door-opener/control";
    const char *garage_door_current_state_topic = "home/garage/door/current-state";

    // TLS settings
    const char *root_ca_certificate =
        "-----BEGIN CERTIFICATE-----\n"
        "Root CA Certificate\n"
        "-----END CERTIFICATE-----\n";

    const char *client_certificate =
        "-----BEGIN CERTIFICATE-----\n"
        "Client Certificate\n"
        "-----END CERTIFICATE-----\n";

    const char *client_private_key =
        "-----BEGIN RSA PRIVATE KEY-----\n"
        "Client Private Key\n"
        "-----END RSA PRIVATE KEY-----\n";

} // namespace config

#endif //Config_h