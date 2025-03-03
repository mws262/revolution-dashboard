#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <Arduino.h>
#include <TFT_eSPI.h>

#define PIN_BRAKE_SW 1
#define PIN_HEADLIGHT 2
#define PIN_REARLIGHT 3
#define PIN_THROTTLE 10 // signal from throttle (max 3,3V)

#define PIN_POWER_ON 15
#define PIN_LCD_RES 5
#define PIN_BUTTON_1 0
#define PIN_BUTTON_2 14
#define PIN_BAT_VOLT 4

#define PIN_TOUCH_SDA 18
#define PIN_TOUCH_SCL 17
#define PIN_TOUCH_IRQ 16
#define PIN_TOUCH_RST 21

// user setup
float thPercentage = 80; // limits max % of throttle for mode1, enter max. RPM in VESC Tool (Motor
                         // Settings - General - RPM)

/*
Driving modes are not saved in the VESC, only some values ​​are temporarily transferred to the
VESC (until restart). In this sketch only the max RPM is increased in sport mode. The smoother
acceleration in mode 1 is achieved by reducing the max throttle value.
*/

// Rear light brightnesses normal and while braking.
const uint8_t BACK_RUNNING_LIGHT_DUTY_CYCLE = 200;
const uint8_t BACK_BRAKING_LIGHT_DUTY_CYCLE = 255;

// This values are independent from VESC Tool, you have to set it too.
const float WHEEL_DIA_MM = 240.0f; // tyre Size in mm, tune this to get correct velocity (also set the
                                // same diameter in the VESC Tool)
const uint8_t MOT_POLE_PAIRS = 20; // motor pole pairs (Boosted Rev = 40 magnets)

const uint8_t BATT_SERIES_CELLS = 12; // number of cells in series of the battery pack

constexpr uint32_t TOUCH_DEBOUNCE_TIME_MS = 300; // delay to wait for next valid touch input
constexpr uint32_t ANTI_STUTTER_DELAY_MS = 1000; // throttle delay after mode change

bool useMiles = false; // km/mi toggle

float tachComp = 1.00; // if distance is different to GPS, compensate it with this multiplier

bool voltdropcomp = true;   /* The throttle reading is very sensitive on the given imput voltage.
  The 5V rail drops when turn on the headlight. This depends on the thin and long wires connected from
  the VESC to the dashboard.   1 = Turn on the compensation, this increases the reading of the throttle
  value. */
bool showThReading = false; // Turn this on to have a look at the input reading. This will be
                            // displayed above the text "Trip".
const int16_t thComp = 180; // This should be the difference of the input reading of the throlle if
                            // the headlight is turned on or off.

bool stopOnBrake = true; // 1 = the motors can not accelerate whie using the disc brake, 0 = motors
                         // can accelerate while braking

constexpr float MI_PER_KM = 0.621371f;
constexpr float MIN_PER_HOUR = 60.0f;
constexpr float KM_PER_MM = 1.0f / 1000000.0f;

// GUI settings
constexpr uint16_t BACKGROUND_COLOR = TFT_BLACK;

#endif // _CONFIG_H_