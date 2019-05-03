# Garage door controller

Arduino based garage door controller for ESP32.

Uses MQTT to publish sensor data and to receive the command for triggering the garage door opener.

The 5V relay is used to trigger the garage door opener and the two magnetic door sensors is used to monitor if the door is in the open or closed position.
My garage door opener also has a 5V output for a warning light, which I use to check if the door is in motion.

## Dependencies

* [Arduino core for the ESP32](https://github.com/espressif/arduino-esp32)
* [Arduino Client for MQTT](https://pubsubclient.knolleary.net/index.html)

## Hardware

* [Adafruit HUZZAH32 ESP32 Feather](https://learn.adafruit.com/adafruit-huzzah32-esp32-feather/)
* [2x MC-38 Wired Door Window Sensor](https://www.ebay.com/sch/i.html?_nkw=MC-38+sensor)
* [5V relay](https://www.ebay.com/sch/i.html?_nkw=5v+relay)
