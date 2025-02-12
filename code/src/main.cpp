/*
Project: Revolution Dashboard 1.0
Author: Roland Stollnberger (stollnberger@gmx.at)
Created: 12.05.2024
Description: Control VESC based ESC, set Profiles, Display Data, Lockscreen, Over The Air
programming...

Included Libraries:
https://github.com/stolliroli/VescUart
LipoCheck: https://github.com/Peemouse/SmartRing // modified

Install Boards:
Lilygo T-Display S3 Touch: https://github.com/Xinyuan-LilyGO/T-Display-S3 // follow the README to
install https://github.com/espressif/arduino-esp32

Libraries to install (via Arduino IDE):
https://github.com/fbiego/CST816S
https://github.com/denyssene/SimpleKalmanFilter
https://github.com/Bodmer/TFT_eSPI
*/

#include "Arduino.h"
#include "LiPoCheck.h"
#include "TFT_eSPI.h"
#include "VescUart.h"
#include "Wire.h"
#include "config.h"
#include "driver/ledc.h" //for PWM
#include <CST816S.h>     //TouchLib

#include "wifi_credentials.h"
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <HardwareSerial.h>
#include <Preferences.h>
#include <SimpleKalmanFilter.h> //to smooth throttle value
#include <WiFi.h>
#include <WiFiUdp.h>
// images & font
#include "DSEG7.h"
#include "Esc.h"
#include "Light.h"
#include "Mot.h"
#include "Rev1.h"

const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;
bool WIFI = 0;

SimpleKalmanFilter thFilter(2, 2, 0.01);
Preferences pref;
HardwareSerial SerialVESC(2);
VescUart UART;
CST816S touch(18, 17, 21, 16); // sda, scl, rst, irq

// setup PWM for rearlight
const uint8_t PWM_CHANNEL = 1;
const uint32_t PWM_FREQ = 500;
const uint8_t PWM_RESOLUTION = 8;
const int BACKLIGHT_DUTY_CYCLE = dimmBL;
const int BRAKELIGHT_DUTY_CYCLE = 255; // 255 for max brightness = brakelight

float erpm = 0;
float rpm = 0;
float speed = 0;
int batt = 0;
int battPerc;
float trip;
int escT = 0;
int motT = 0;
bool profSet = 0;

bool lightF = 0;
bool lock = 1;
bool modeS = 0;
bool confMode = 0;

String entry = "";

int nunck = 127;
uint32_t filterTime = 0; // for Kalman Filter
bool filterDelay = true;

unsigned int maxVal = analogRead(PIN_THROTTLE);
unsigned int minVal = analogRead(PIN_THROTTLE);
unsigned int thMax;
unsigned int thZero;
unsigned int thMin;
unsigned int throttleRAW;

// debounce for touch
unsigned long lastTouchTime = 0;   // to manage debounce
unsigned long debounceDelay = 300; // delay in milliseconds to wait for next valid touch input

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite mainSprite = TFT_eSprite(&tft);

void lockscreen(int x, int y);

void setup() {
  // OTA
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  // Start mDNS with a hostname -- makes it easier to find the ESP32 on your network
  // Try to ping it if the OTA upload isn't working.
  while (!MDNS.begin("rev")) {
    Serial.println("Error starting mDNS");
    delay(2000);
  }
  ArduinoOTA.setHostname("rev");
  ArduinoOTA.begin();

  // data storage
  pref.begin("thValues", false); // "true" defines read-only access
  thMax = pref.getUInt("thMax", 0);
  thZero = pref.getUInt("thZero", 0);
  thMin = pref.getUInt("thMin", 0);
  pref.end();

  // serial VESC
  SerialVESC.begin(115200, SERIAL_8N1, 43,
                   44); // Lilygo Pin43=RX to VescTX, Lilygo Pin44=TX to VescRX
  while (!SerialVESC) {
    ;
  }

  UART.setSerialPort(&SerialVESC);
  UART.getFWversion();
  // setup the input & output pins
  pinMode(PIN_HEADLIGHT, OUTPUT);
  pinMode(PIN_BRAKE_SW, INPUT);
  pinMode(PIN_THROTTLE, INPUT);
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PIN_REARLIGHT, PWM_CHANNEL);
  // display
  tft.init();
  tft.setRotation(0);
  tft.setSwapBytes(true);
  tft.pushImage(0, 0, 170, 320, Rev1);
  mainSprite.createSprite(170, 320);
  mainSprite.setSwapBytes(true);
  // touch
  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);
  touch.begin();

  delay(2500); // waiting to start the VESC
  lockscreen(-1, -1);
}

void lockscreen(int x, int y) {

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(4);

  int xpos[3] = {3, 58, 113};
  int ypos[4] = {73, 128, 183, 238};
  char chars[4][3] = {{'1', '2', '3'}, {'4', '5', '6'}, {'7', '8', '9'}, {' ', '0', 'x'}};
  int sx = 10, sy = 10;
  for (int i = 0; i < 4; i++) {
    if (y >= ypos[i] && y <= ypos[i] + 54)
      sy = i;
    for (int j = 0; j < 3; j++) {
      if (x >= xpos[j] && x <= xpos[j] + 54)
        sx = j;
      if (sx == j && sy == i)
        tft.drawRoundRect(xpos[j], ypos[i], 54, 54, 2, TFT_DARKGREY);
      else
        tft.drawRoundRect(xpos[j], ypos[i], 54, 54, 2, TFT_WHITE);
      tft.drawRoundRect(3, 238, 54, 54, 2, TFT_BLACK);
      tft.drawRoundRect(113, 238, 54, 54, 2, TFT_BLACK);
      tft.drawString(String(chars[i][j]), xpos[j] + 27, ypos[i] + 30, 4);
      if (x >= xpos[j] && x <= xpos[j] + 54)
        sx = j;
    }
  }

  if (chars[sy][sx] == 'x') // clear entry
    entry = "";
  else
    entry = entry + String(chars[sy][sx]);

  if (entry.toInt() == mode1) {
    lock = 0;
  }

  if (entry.toInt() == mode2) {
    modeS = 1;
    lock = 0;
  }

  tft.drawString(entry, 85, 40, 6); // draw touched entry

  if (entry.toInt() == throttleCal) {
    confMode = 1;
  }
}

void drawScreen() {
  // Sprite
  mainSprite.fillSprite(TFT_BLACK);
  mainSprite.unloadFont(); // to draw all other txt before DSEG7 font
  // WIFI connection
  mainSprite.setTextColor(TFT_BLUE, TFT_BLACK);
  mainSprite.setTextDatum(4);
  if (WIFI == 1)
    mainSprite.drawString("WIFI connected", 110, 37, 2);
  // batt bar
  mainSprite.setTextColor(TFT_WHITE, TFT_BLACK);
  if (battPerc > 15) {
    mainSprite.fillRoundRect(60, 10, battPerc, 15, 2, TFT_GREEN);
  } else {
    mainSprite.fillRoundRect(60, 10, battPerc, 15, 2, TFT_RED);
  }
  mainSprite.drawRoundRect(60, 10, 100, 15, 2, TFT_WHITE);
  // batt txt
  mainSprite.drawString(String(batt) + " V", 30, 8, 2);
  mainSprite.drawString(String(battPerc) + " %", 30, 26, 2);
  // trip txt
  mainSprite.setTextDatum(0);
  mainSprite.drawString(String("Trip"), 10, 182, 2);
  mainSprite.setTextDatum(2);
  if (setMi == 1) {
    mainSprite.drawString(String(trip * 0.621371, 2) + " mi", 160, 175, 4);
  } else {
    mainSprite.drawString(String(trip, 2) + " km", 160, 175, 4);
  }
  // show throttle reading
  if (showThReading == 1) {
    mainSprite.setTextDatum(0);
    mainSprite.drawString(String(throttleRAW), 10, 162, 2);
  }
  // line
  mainSprite.drawLine(0, 210, 170, 210, TFT_DARKGREY);
  // ESCTemp txt
  mainSprite.setTextDatum(2);
  mainSprite.drawString(String(escT), 40, 290, 4);
  mainSprite.drawCircle(46, 293, 3, TFT_WHITE);
  mainSprite.pushImage(10, 230, 40, 40, Esc);
  // motTemp txt
  mainSprite.setTextDatum(2);
  mainSprite.drawString(String(motT), 152, 290, 4);
  mainSprite.drawCircle(158, 293, 3, TFT_WHITE);
  mainSprite.pushImage(120, 230, 40, 40, Mot);
  // mode
  if (modeS == 1) {
    mainSprite.drawRoundRect(75, 286, 22, 27, 3, TFT_RED);
  } else {
    mainSprite.drawRoundRect(75, 286, 22, 27, 3, TFT_DARKGREY);
  }
  mainSprite.setTextDatum(4);
  mainSprite.drawString(String("S"), 86, 299, 2);
  // light
  mainSprite.pushImage(65, 230, 40, 40, Light);
  if (lightF == 1) {
    mainSprite.drawRoundRect(62, 229, 46, 41, 3, TFT_BLUE);
  } else {
    mainSprite.drawRoundRect(62, 229, 46, 41, 3, TFT_DARKGREY);
  }
  // speed
  mainSprite.setTextDatum(4);
  mainSprite.loadFont(DSEG7);
  if (setMi == 1) {
    speed = speed * 0.621371;
  }
  if (speed < 0) {
    speed = 0;
  } else {
    mainSprite.drawString(String(speed, 0), 79, 102, 8);
  }
  // push Sprite to disp
  mainSprite.pushSprite(0, 0);
}

void loop() {
  // OTA
  if (WiFi.waitForConnectResult() == WL_CONNECTED) {
    WIFI = 1;
  } else {
    WIFI = 0;
  }
  ArduinoOTA.handle();

  // Lockscreen
  while (lock == 1 && confMode == 0) {
    if (touch.available()) {
      unsigned long currentTouchTime = millis();
      if (currentTouchTime - lastTouchTime > debounceDelay) {
        lastTouchTime = currentTouchTime;
        if (touch.data.y > 75 && touch.data.y < 290)
          lockscreen(touch.data.x, touch.data.y);
      }
    }
  }

  // calculate the estimated value with Kalman Filter
  throttleRAW = thFilter.updateEstimate(analogRead(PIN_THROTTLE));

  // calibrate throttle
  while (confMode == 1) {
    mainSprite.fillSprite(TFT_BLACK);
    mainSprite.setTextColor(TFT_WHITE, TFT_BLACK);
    mainSprite.setTextDatum(4);
    mainSprite.drawString("move throttle", 85, 20, 2);
    mainSprite.drawString("a few times from", 85, 40, 2);
    mainSprite.drawString("full throttle to full brake", 85, 60, 2);
    mainSprite.drawString("and press OK", 85, 80, 2);
    mainSprite.drawRoundRect(10, 245, 70, 50, 2, TFT_RED);
    mainSprite.drawString("X", 45, 272, 4);
    mainSprite.drawRoundRect(90, 245, 70, 50, 2, TFT_GREEN);
    mainSprite.drawString("OK", 125, 272, 4);

    unsigned int throttleRAW = thFilter.updateEstimate(analogRead(PIN_THROTTLE));

    if (throttleRAW > maxVal) {
      maxVal = throttleRAW;
    }

    if (throttleRAW < minVal) {
      minVal = throttleRAW;
    }

    mainSprite.setTextDatum(0);
    mainSprite.drawString("old Value", 5, 110, 2);
    mainSprite.drawString(String(thMax), 5, 130, 4);
    mainSprite.drawString(String(thZero), 5, 160, 4);
    mainSprite.drawString(String(thMin), 5, 190, 4);

    mainSprite.setTextDatum(2);
    mainSprite.drawString("new Value", 165, 110, 2);
    mainSprite.drawString(String(maxVal), 165, 130, 4);
    mainSprite.drawString(String(throttleRAW), 165, 160, 4);
    mainSprite.drawString(String(minVal), 165, 190, 4);

    mainSprite.pushSprite(0, 0);

    if (touch.available()) {
      unsigned long currentTouchTime = millis();
      if (currentTouchTime - lastTouchTime > debounceDelay) {
        lastTouchTime = currentTouchTime;
        if (touch.data.y > 245 && touch.data.y < 295 && touch.data.x > 85) {
          pref.begin("thValues", false);
          pref.putUInt(
              "thMax",
              maxVal); // at 5V input, the Hall Sensor Value schould be 4095 on full throttle
          pref.putUInt("thZero",
                       throttleRAW);     // schould be about 2880, depends on input Voltage ~ 5V
          pref.putUInt("thMin", minVal); // schould be about 2100, depends on input Voltage ~ 5V
          pref.end();
          mainSprite.fillSprite(TFT_BLACK);
          mainSprite.pushSprite(0, 0);
          delay(100);
          ESP.restart();
        }
        if (touch.data.y > 245 && touch.data.y < 295 && touch.data.x < 85) {
          mainSprite.fillSprite(TFT_BLACK);
          mainSprite.pushSprite(0, 0);
          ESP.restart();
        }
      }
    }
  }

  // set profile
  if (modeS == 1 && profSet == 0) {
    bool store = false;      // save persistently the new profile in vesc memory
    bool forward_can = true; // forward profile to slave Vesc through can bus
    bool ack = false;
    bool divide_by_controllers = false; // divied the watt limit by the number of VESC
    float current_min_rel =
        1.0; // this value is multiplied with the minimum current. It is a convenient method to
             // scale the current limits without forgetting the actual minimal value.  DEFAULT = 1
    float current_max_rel =
        1.0; // this value is multiplied with the maximum current. It is a convenient method to
             // scale the current limits without forgetting the actual maximum value.  DEFAULT = 1
    float speed_max_reverse =
        -15; // maximum reverse speed in m/s. In order, to use it make sure to set the wheel
             // diameter and gear ratio in VESC Tool   (m/s = (km/h / 3.6) )
    float speed_max = 15;   // maximum speed in m/s. In order, to use it make sure to set the wheel
                            // diameter and gear ratio in VESC Tool
    float duty_min = 0.005; // minimum duty cycle. DEFAULT = 0.005
    float duty_max = 1;     // maximum duty cycle. DEFAULT = 1
    float watt_min = -1500000.0; // minimum watt value. DEFAULT = - 1500000.0
    float watt_max = 1500000.0;  // maximum watt value. DEFAULT =  1500000.0

    UART.setLocalProfile(store, forward_can, divide_by_controllers, current_min_rel,
                         current_max_rel, speed_max_reverse, speed_max, duty_min, duty_max,
                         watt_min, watt_max);
    profSet = 1;
  }

  // switch headlight
  if (touch.available()) {
    unsigned long currentTouchTime = millis();
    if (currentTouchTime - lastTouchTime > debounceDelay) {
      lastTouchTime = currentTouchTime;
      //      if(touch.data.x == 85 && touch.data.y == 360) //only the touch button
      if (touch.data.y >= 212)
        lightF = !lightF;
    }
  }

  if (lightF == HIGH) {
    digitalWrite(PIN_HEADLIGHT, HIGH);
  } else {
    digitalWrite(PIN_HEADLIGHT, LOW);
  }

  // handling brakelight
  if (digitalRead(PIN_BRAKE_SW) == HIGH ||
      throttleRAW < thZero - 250) { // reduce -250 for brakelight deadband
    ledcWrite(PWM_CHANNEL, BRAKELIGHT_DUTY_CYCLE);
  } else {
    ledcWrite(PWM_CHANNEL, BACKLIGHT_DUTY_CYCLE);
  }

  // reading VESC data
  if (UART.getVescValues()) {
    erpm = UART.data.rpm;
    batt = UART.data.inpVoltage;
    escT = UART.data.tempFET;
    motT = UART.data.tempMotor;
    trip = UART.data.tachometer;
  }
  rpm = erpm / motPol;
  speed = erpm / motPol * wheelDia * 3.1415 * 0.00006;
  trip = trip / wheelDia / 1000 * tachComp;
  battPerc = CapCheckPerc(batt, numbCell);

  // calc nunchuck value
  float maxNunck;
  if (modeS == true) {
    maxNunck = 255;
  } else {
    maxNunck = thPercentage * 0.01 * 127 + 127;
  }

  if (lightF == HIGH &&
      voltdropcomp == 1) { // compensation of the voltage drop when headlight is turned on
    throttleRAW = throttleRAW + thComp;
  }

  if (throttleRAW > thZero) {
    nunck =
        map(throttleRAW, thZero, thMax + 100, 127, maxNunck); //+100 to avoid running out of range
  } else if (throttleRAW < thZero) {
    nunck = map(throttleRAW, thZero, thMin - 100, 127, 0); //-100 to avoid running out of range
  }

  if (digitalRead(PIN_BRAKE_SW) == 1 && nunck > 127 && stopOnBrake == 1) {
    nunck = 127; // interrupts acceleration when braking
  }

  nunck = constrain(nunck, 0, 255); // Avoid underflow and overflow
                                    // causing acceleration on throttle disconnect.

  // send nunchuck value to VESC
  if ((millis() - filterTime) > 1000 &&
      filterDelay == 1) { // this delay prevents the motors from stuttering at start when applying
                          // the kalman filter
    filterDelay = 0;
    filterTime = millis();
  } else {
    UART.nunchuck.valueY = nunck;
    UART.setNunchuckValues();
  }
  drawScreen();
}
