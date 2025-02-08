//****************************************************************************
//user setup

#define WIFI_SSID ""  // don't forget to provide your login data inside the quote signs to use over the air programming
#define WIFI_PASS ""  // programming is only possible at unlocked device, successful connection is displayed under the battery bar

const int mode1 = 1234;   //enter your own code to unlock mode 1, use this for regular driving mode
const int mode2 = 2345;   //enter your own code to unlock mode 2, use this for sport mode
const int throttleCal = 1000; // code to run throttle calibration
float thPercentage = 80;  //limits max % of throttle for mode1, enter max. RPM in VESC Tool (Motor Settings - General - RPM)

/*
Driving modes are not saved in the VESC, only some values ​​are temporarily transferred to the VESC (until restart).
In this sketch only the max RPM is increased in sport mode.
The smoother acceleration in mode 1 is achieved by reducing the max throttle value.
*/

const int dimmBL = 200; // 0-255 dimminig rear light when not braking

bool setMi = 0; // 0 = km / 1 = miles

//This values are independent from VESC Tool, you have to set it too.
const int wheelDia = 240; //tyre Size in mm, tune this to get correct velocity (also set the same diameter in the VESC Tool)
const int motPol = 20;   //motor pole pairs (Boosted Rev = 40 magnets)
float tachComp = 1.00; // if distance is different to GPS, compensate it with this multiplier

const int numbCell = 12; //number of cells in series of the battery pack

bool voltdropcomp = 1; /* The trottle reading is very sensitive on the given imput voltage.
The 5V rail drops when turn on the headlight. This depends on the thin and long wires connected from the VESC to the dashboard.
1 = Turn on the compensation, this increases the reading of the throttle value. */
bool showThReading = 0; //Turn this on to have a look at the input reading. This will be displayed above the text "Trip".
const int thComp = 180; // This schould be the difference of the input reading of the throlle if the headlight is turned on or off.

bool stopOnBrake = 1; // 1 = the motors can not accelerate whie using the disc brake, 0 = motors can accelerate while braking

//****************************************************************************

#pragma once

#define brakeSw              1
#define headlight            2
#define rearlight            3    
#define throttle             10    // signal from throttle (max 3,3V)

#define PIN_LCD_BL           38
#define PIN_LCD_D0           9
#define PIN_LCD_D1           40
#define PIN_LCD_D2           41
#define PIN_LCD_D3           42
#define PIN_LCD_D4           45
#define PIN_LCD_D5           46
#define PIN_LCD_D6           47
#define PIN_LCD_D7           48
#define PIN_POWER_ON         15
#define PIN_LCD_RES          5
#define PIN_LCD_CS           6
#define PIN_LCD_DC           7
#define PIN_LCD_WR           8
#define PIN_LCD_RD           9
#define PIN_BUTTON_1         0
#define PIN_BUTTON_2         14
#define PIN_BAT_VOLT         4

CST816S touch (18, 17, 21, 16);	// sda, scl, rst, irq