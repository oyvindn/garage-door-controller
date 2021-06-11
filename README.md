# Garage door controller

Arduino based garage door controller for ESP32.

Uses MQTT to publish sensor data and to receive the command for triggering the garage door opener.

The 5V relay is used to trigger the garage door opener and the two magnetic door sensors is used to monitor if the door is in the open or closed position.
My garage door opener also has a 5V output for a warning light, which I use to check if the door is in motion.

## Dependencies
* [Arduino core for the ESP32](https://github.com/espressif/arduino-esp32)
* [Arduino Client for MQTT](https://pubsubclient.knolleary.net/index.html)
* [One Wire](https://github.com/PaulStoffregen/OneWire)
* [Arduino Library for Maxim Temperature Integrated Circuits](https://github.com/milesburton/Arduino-Temperature-Control-Library)

## Hardware

* [ESP32-WROOM-32D](https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32d_esp32-wroom-32u_datasheet_en.pdf)
* 2x MC-38 Wired Door Window Sensor
* 5V relay
* Waterproof 1-Wire DS18B20 Compatible Digital temperature sensor
