#ifndef _CONFIG_H_
#define _CONFIG_H_
//****************************************************************************
//user setup

const uint16_t mode1 = 1234;   //enter your own code to unlock mode 1, use this for regular driving mode
const uint16_t mode2 = 2345;   //enter your own code to unlock mode 2, use this for sport mode
const uint16_t throttleCal = 1000; // code to run throttle calibration
float thPercentage = 80;  //limits max % of throttle for mode1, enter max. RPM in VESC Tool (Motor Settings - General - RPM)

/*
Driving modes are not saved in the VESC, only some values ​​are temporarily transferred to the VESC (until restart).
In this sketch only the max RPM is increased in sport mode.
The smoother acceleration in mode 1 is achieved by reducing the max throttle value.
*/

const uint8_t dimmBL = 200; // 0-255 dimming rear light when not braking

bool setMi = false; // 0 = km / 1 = miles

//This values are independent from VESC Tool, you have to set it too.
const uint16_t wheelDia = 240; //tyre Size in mm, tune this to get correct velocity (also set the same diameter in the VESC Tool)
const uint8_t motPol = 20;   //motor pole pairs (Boosted Rev = 40 magnets)
float tachComp = 1.00; // if distance is different to GPS, compensate it with this multiplier

const uint8_t numbCell = 12; //number of cells in series of the battery pack

bool voltdropcomp = true; /* The throttle reading is very sensitive on the given imput voltage.
The 5V rail drops when turn on the headlight. This depends on the thin and long wires connected from the VESC to the dashboard.
1 = Turn on the compensation, this increases the reading of the throttle value. */
bool showThReading = false; //Turn this on to have a look at the input reading. This will be displayed above the text "Trip".
const int16_t thComp = 180; // This should be the difference of the input reading of the throlle if the headlight is turned on or off.

bool stopOnBrake = true; // 1 = the motors can not accelerate whie using the disc brake, 0 = motors can accelerate while braking

//****************************************************************************

#define PIN_BRAKE_SW         1
#define PIN_HEADLIGHT        2
#define PIN_REARLIGHT        3    
#define PIN_THROTTLE         10    // signal from throttle (max 3,3V)

#define PIN_POWER_ON         15
#define PIN_LCD_RES          5
#define PIN_BUTTON_1         0
#define PIN_BUTTON_2         14
#define PIN_BAT_VOLT         4

#endif // _CONFIG_H_