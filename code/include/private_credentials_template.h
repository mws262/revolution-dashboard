#ifndef _PRIVATE_CREDENTIALS_H_
#define _PRIVATE_CREDENTIALS_H_

#include <Arduino.h>

#define WIFI_SSID "your_ssid"
#define WIFI_PASSWORD "your_password"

const uint16_t PIN_MODE_NORMAL = 1234; // Unlock PIN for normal riding mode.
const uint16_t PIN_MODE_SPORT = 2345;  // Unlock PIN for sport riding mode.
const uint16_t PIN_MODE_THROTTLE_CAL = 1000; // code to run throttle calibration

#endif
