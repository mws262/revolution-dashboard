// /*
// Project: Revolution Dashboard 1.0
// Author: Roland Stollnberger (stollnberger@gmx.at)
// Created: 12.05.2024
// Description: Control VESC based ESC, set Profiles, Display Data, Lockscreen,
// Over The Air programming...

// Included Libraries:
// https://github.com/stolliroli/VescUart
// LipoCheck: https://github.com/Peemouse/SmartRing // modified
//
// Updates from Matt Sheen (matthewsheen@gmail.com)
// */
#define log_printf \
  custom_log_printf  // This lets me override the Arduino logging print function
                     // so I can also send all messages to the webserver.
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <CST816S.h>  //TouchLib
#include <ESPmDNS.h>
#include <HardwareSerial.h>
#include <Preferences.h>
#include <SimpleKalmanFilter.h>  //to smooth throttle value
#include <WiFi.h>
#include <WiFiUdp.h>

#include "LiPoCheck.h"
#include "TFT_eSPI.h"
#include "VescUart.h"
#include "Wire.h"
#include "config.h"
#include "driver/ledc.h"  //for PWM
#include "private_credentials.h"
// images & font
#include "DSEG7.h"
#include "Esc.h"
#include "Light.h"
#include "Mot.h"
#include "Rev1.h"
#include "debug_webserver.hpp"

const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;

enum class RevMode { LOCK, CALIBRATE, RIDE_NORMAL, RIDE_SPORT };

RevMode currentMode = RevMode::LOCK;

SimpleKalmanFilter thFilter(1, 2, 0.1);
Preferences pref;
HardwareSerial SerialVESC(2);
VescUart vescUart;
CST816S touch(PIN_TOUCH_SDA, PIN_TOUCH_SCL, PIN_TOUCH_RST, PIN_TOUCH_IRQ);

#ifdef WIFI_CONSOLE_ENABLED
DebugWebServer debugServer;
#endif

// setup PWM for rearlight
const uint8_t PWM_CHANNEL = 1;
const uint32_t PWM_FREQ = 500;
const uint8_t PWM_RESOLUTION = 8;

constexpr long TARGET_LOOP_RATE = 8;  // Hz
constexpr long TARGET_LOOP_PERIOD = 1000 / TARGET_LOOP_RATE;
const uint16_t WIFI_RECONNECTION_INTERVAL = 5000;

float erpm = 0;
float rpm = 0;
float speed_kmh = 0;
int batt_voltage = 0;
int battPerc;
float trip;
int escT = 0;
int motT = 0;
float maxNunck;

bool lightF = false;
bool throttleValuesSet =
    false;  // Prevent the Rev from going into one of the drive modes when the
            // throttle calibration values aren't set.

String pin_entry = "";

int nunck = 127;

uint16_t maxVal = analogRead(PIN_THROTTLE);
uint16_t minVal = analogRead(PIN_THROTTLE);
uint16_t thMax;
uint16_t thZero;
uint16_t thMin;
uint16_t throttleRAW;

uint32_t lastTouchTime = 0;  // to manage debounce
uint32_t lastModeChangeTimeMs = 0;
uint32_t lastLoopTime = 0;
uint32_t lastReconnectAttempt = 0;

bool wifiEverConnected = false;

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite cockpitSprite = TFT_eSprite(&tft);
TFT_eSprite lockSprite = TFT_eSprite(&tft);

uint8_t lastBrakeLightBrightness =
    0;  // Track the last value so we don't send the same command repeatedly.

void setVescProfileSport();
void setVescProfileNormal();
void changeMode(RevMode newMode);
bool waitForVescConnection(uint32_t timeout_ms);
int custom_log_printf(const char *format, ...) {
  char log_buf[256];

  // Process variadic arguments
  va_list args;
  va_start(args, format);
  int len = vsnprintf(log_buf, sizeof(log_buf), format, args);
  va_end(args);

  // Print to Serial
  Serial.print(log_buf);

// Send to WebSocket
#ifdef WIFI_CONSOLE_ENABLED
  debugServer.broadcast(String(log_buf));
#endif

  return len;
}

void setup() {
  Serial.begin(115200);
// Initiate wifi for OTA updates. This is non-blocking. The main loop will
// handle other initializations once the network is connected.
#ifdef WIFI_ENABLED
  log_d("Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
#endif

  // data storage
  log_d("Loading preferences...");
  pref.begin("thValues", false);  // "true" defines read-only access
  thMax = pref.getUInt("thMax", UINT32_MAX);
  thZero = pref.getUInt("thZero", UINT32_MAX);
  thMin = pref.getUInt("thMin", UINT32_MAX);
  pref.end();

  // If throttle values aren't set, keep the rev from going into a drive mode.
  if (thMax == UINT32_MAX || thZero == UINT32_MAX || thMin == UINT32_MAX) {
    log_w("Throttle values not found in preferences. Using defaults.");
  } else {
    throttleValuesSet = true;
  }

  // serial VESC
  log_d("Setting up serial VESC...");
  SerialVESC.begin(115200, SERIAL_8N1, 43,
                   44);  // Lilygo Pin43=RX to VescTX, Lilygo Pin44=TX to VescRX
  while (!SerialVESC) {
  }
  vescUart.setSerialPort(&SerialVESC);

  pinMode(PIN_BRAKE_SW, INPUT);
  pinMode(PIN_THROTTLE, INPUT);

  // display
  log_d("Setting up display...");
  tft.init();
  tft.setRotation(0);
  tft.setSwapBytes(true);
  tft.pushImage(0, 0, TFT_WIDTH, TFT_HEIGHT, Rev1);
  cockpitSprite.createSprite(TFT_WIDTH, TFT_HEIGHT);
  cockpitSprite.setSwapBytes(true);
  lockSprite.createSprite(TFT_WIDTH, TFT_HEIGHT);
  lockSprite.setSwapBytes(true);

  // touch
  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);
  touch.begin();

  if (!waitForVescConnection(3000)) {
    log_e("Unable to communicate with VESC. Continuing anyway...");
  }

#ifdef VESC_CONTROLS_LIGHTS
  vescUart.setBrakeLightBrightness(BACK_RUNNING_LIGHT_PERCENT);
  vescUart.setHeadlightState(headlightOnByDefault);
#else
  pinMode(PIN_HEADLIGHT, OUTPUT);
  ledcAttachPin(PIN_REARLIGHT, PWM_CHANNEL);
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(PWM_CHANNEL, BACK_RUNNING_LIGHT_PERCENT * 255 / 100);
  digitalWrite(PIN_HEADLIGHT, headlightOnByDefault);
#endif

  changeMode(RevMode::LOCK);
}

// Draws the entire lock screen including the keypad and current PIN display.
// Highlights the key corresponding to the most recent digit in 'pin' (if any).
void drawScreenLock(const String &pin) {
  lockSprite.fillSprite(BACKGROUND_COLOR);
  lockSprite.setTextColor(TFT_WHITE, TFT_BLACK);
  lockSprite.setTextDatum(4);

  int xpos[3] = {3, 58, 113};
  int ypos[4] = {73, 128, 183, 238};
  char keys[4][3] = {
      {'1', '2', '3'}, {'4', '5', '6'}, {'7', '8', '9'}, {' ', '0', 'x'}};

  // Highlight the last pressed key (if any)
  char lastChar = (pin.length() > 0) ? pin.charAt(pin.length() - 1) : '\0';

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      if (lastChar != '\0' && keys[i][j] == lastChar)
        lockSprite.drawRoundRect(xpos[j], ypos[i], 54, 54, 2, TFT_DARKGREY);
      else
        lockSprite.drawRoundRect(xpos[j], ypos[i], 54, 54, 2, TFT_WHITE);

      lockSprite.drawString(String(keys[i][j]), xpos[j] + 27, ypos[i] + 30, 4);
    }
  }

  // Draw the current PIN entry.
  lockSprite.drawString(pin, 85, 40, 6);

  // Push the sprite to the display.
  lockSprite.pushSprite(0, 0);
}

void drawScreenCockpit() {
  // Sprite
  cockpitSprite.fillSprite(BACKGROUND_COLOR);
  cockpitSprite.unloadFont();  // to draw all other txt before DSEG7 font

#ifdef DRY_RUN_MODE
  // Add a clear indicator for dry run mode
  cockpitSprite.setTextColor(TFT_YELLOW, TFT_BLACK);
  cockpitSprite.setTextDatum(4);
  cockpitSprite.drawString("DRY RUN MODE", 85, 164, 2);
#endif

  // WIFI connection
  cockpitSprite.setTextColor(TFT_BLUE, TFT_BLACK);
  cockpitSprite.setTextDatum(4);
  if (WiFi.status() == WL_CONNECTED) {
    cockpitSprite.drawString("WIFI connected", 110, 37, 2);
  }

  // batt bar
  cockpitSprite.setTextColor(TFT_WHITE, TFT_BLACK);
  if (battPerc > 15) {
    cockpitSprite.fillRoundRect(60, 10, battPerc, 15, 2, TFT_GREEN);
  } else {
    cockpitSprite.fillRoundRect(60, 10, battPerc, 15, 2, TFT_RED);
  }

  cockpitSprite.drawRoundRect(60, 10, 100, 15, 2, TFT_WHITE);
  // batt txt
  cockpitSprite.drawString(String(batt_voltage) + " V", 30, 8, 2);
  cockpitSprite.drawString(String(battPerc) + " %", 30, 26, 2);
  // trip txt
  cockpitSprite.setTextDatum(0);
  cockpitSprite.drawString(String("Trip"), 10, 182, 2);
  cockpitSprite.setTextDatum(2);

  if (useMiles) {
    cockpitSprite.drawString(String(trip * MI_PER_KM, 2) + " mi", 160, 175, 4);
  } else {
    cockpitSprite.drawString(String(trip, 2) + " km", 160, 175, 4);
  }

  // show throttle reading
  if (showThReading) {
    cockpitSprite.setTextDatum(0);
    cockpitSprite.drawString(String(throttleRAW), 10, 162, 2);
  }

  // line
  cockpitSprite.drawLine(0, 210, TFT_WIDTH, 210, TFT_DARKGREY);
  // ESCTemp txt
  cockpitSprite.setTextDatum(2);
  cockpitSprite.drawString(String(escT), 40, 290, 4);
  cockpitSprite.drawCircle(46, 293, 3, TFT_WHITE);
  cockpitSprite.pushImage(10, 230, 40, 40, Esc);
  // motTemp txt
  cockpitSprite.setTextDatum(2);
  cockpitSprite.drawString(String(motT), 152, 290, 4);
  cockpitSprite.drawCircle(158, 293, 3, TFT_WHITE);
  cockpitSprite.pushImage(120, 230, 40, 40, Mot);

  // mode
  if (currentMode == RevMode::RIDE_SPORT) {
    cockpitSprite.drawRoundRect(75, 286, 22, 27, 3, TFT_RED);
  } else {
    cockpitSprite.drawRoundRect(75, 286, 22, 27, 3, TFT_DARKGREY);
  }

  cockpitSprite.setTextDatum(4);
  cockpitSprite.drawString(String("S"), 86, 299, 2);

  // light
  cockpitSprite.pushImage(65, 230, 40, 40, Light);
  if (lightF) {
    cockpitSprite.drawRoundRect(62, 229, 46, 41, 3, TFT_BLUE);
  } else {
    cockpitSprite.drawRoundRect(62, 229, 46, 41, 3, TFT_DARKGREY);
  }

  // speed
  cockpitSprite.setTextDatum(4);
  cockpitSprite.loadFont(DSEG7);

  // Prevent trying to display 3-digit speeds or negative signs.
  float displaySpeed =
      constrain(useMiles ? speed_kmh * MI_PER_KM : speed_kmh, 0, 99);

  cockpitSprite.drawString(String(displaySpeed, 0), 79, 102, 8);

  // push Sprite to disp
  cockpitSprite.pushSprite(0, 0);
}

// Returns the key pressed based on the touch coordinates.
// Returns '\0' if no valid key was touched.
char handleLockScreenTouch(uint16_t x, uint16_t y) {
  uint16_t xpos[3] = {3, 58, 113};
  uint16_t ypos[4] = {73, 128, 183, 238};
  char keys[4][3] = {
      {'1', '2', '3'}, {'4', '5', '6'}, {'7', '8', '9'}, {' ', '0', 'x'}};

  int sx = -1, sy = -1;
  // Determine touched row
  for (int i = 0; i < 4; i++) {
    if (y >= ypos[i] && y <= ypos[i] + 54) {
      sy = i;
      break;
    }
  }
  // Determine touched column
  for (int j = 0; j < 3; j++) {
    if (x >= xpos[j] && x <= xpos[j] + 54) {
      sx = j;
      break;
    }
  }
  if (sx < 0 || sy < 0) {
    return '\0';  // No valid key touched.
  }
  return keys[sy][sx];
}

// These are the events that should occur when the Rev changes from
// one operating mode to another. This does not occur every loop.
void changeMode(RevMode newMode) {
  switch (newMode) {
    case RevMode::CALIBRATE:
      break;

    case RevMode::LOCK:
      pin_entry.clear();  // Just in case.
      drawScreenLock(pin_entry);
      break;

    case RevMode::RIDE_NORMAL:
      if (throttleValuesSet) {
        setVescProfileNormal();
        maxNunck = thPercentage * 0.01f * 127 + 127;
      } else {
        log_w("Throttle values not set. Cannot enter ride mode.");
        changeMode(RevMode::CALIBRATE);
      }
      break;

    case RevMode::RIDE_SPORT:
      if (throttleValuesSet) {
        setVescProfileSport();
        maxNunck = 255;
      } else {
        log_w("Throttle values not set. Cannot enter ride mode.");
        changeMode(RevMode::CALIBRATE);
      }
      break;

    default:
      log_e("Attempted to transition into unknown mode.");
  }
  currentMode = newMode;
  lastModeChangeTimeMs = millis();
}

// Handle WiFi connection and OTA updates.
void handleWifi() {
  // If WiFi is connected, check for OTA updates or, if this is the first
  // loop since we've connected to WiFi, start mDNS and OTA.
  if (WiFi.status() == WL_CONNECTED) {
    if (wifiEverConnected) {
      ArduinoOTA.handle();
    } else {  // First time we've connected to WiFi.
      wifiEverConnected = true;

      log_i("Connected to WiFi");
      // Start mDNS with a hostname -- makes it easier to find the ESP32 on your
      // network Try to ping it if the OTA upload isn't working.
      if (!MDNS.begin("rev")) {  // This will show up on the network as
                                 // rev.local or possibly
        // rev._arduino._tcp.local
        log_e("Error setting up mDNS. Use IP address instead: %s",
              WiFi.localIP().toString().c_str());
      }

      ArduinoOTA.setHostname("rev");
      ArduinoOTA.begin();

#ifdef WIFI_CONSOLE_ENABLED
      debugServer.begin();
#endif
    }
  } else {
    // Attempt a quick reconnect if we've ever connected to WiFi before. Only
    // check periodically.
    if (wifiEverConnected) {
      if (millis() - lastReconnectAttempt > WIFI_RECONNECTION_INTERVAL) {
        log_w("WiFi disconnected. Attempting to reconnect...");
        WiFi.reconnect();
        lastReconnectAttempt = millis();
      }
    }
    // TODO: periodically look for the wifi even if we've never successfully
    // connected to it before. For example, we turn the scooter on and then ride
    // home.
  }
}

void loop() {
#ifdef WIFI_ENABLED
  handleWifi();
#endif

  // Updated filtered throttle reading. TODO: maybe move this into only the
  // drive modes.
  throttleRAW = (uint16_t)thFilter.updateEstimate(analogRead(PIN_THROTTLE));

  // Read VESC data regardless of the mode. If we transition into a ride mode,
  // we'd like to already have values.
  if (vescUart.getVescValues()) {
    erpm = vescUart.data.rpm;
    batt_voltage = vescUart.data.inpVoltage;
    escT = vescUart.data.tempFET;
    motT = vescUart.data.tempMotor;
    trip = vescUart.data.tachometer;
  }
  rpm = erpm / MOT_POLE_PAIRS;
  speed_kmh = rpm * WHEEL_DIA_MM * M_PI * MIN_PER_HOUR * KM_PER_MM;
  trip = trip / WHEEL_DIA_MM / 1000.0f * tachComp;
  battPerc = CapCheckPerc(batt_voltage, BATT_SERIES_CELLS);

  // Brakelight will occur in all modes.
  uint8_t brakeLightBrightness =
      (digitalRead(PIN_BRAKE_SW) || throttleRAW < thZero - 250)
          ? BACK_BRAKING_LIGHT_PERCENT
          : BACK_RUNNING_LIGHT_PERCENT;

  // Only send the command when the brightness actually changes
  if (brakeLightBrightness != lastBrakeLightBrightness) {
#ifdef VESC_CONTROLS_LIGHTS
    vescUart.setBrakeLightBrightness(brakeLightBrightness);
    log_d("Brake light brightness changed from %d to %d",
          lastBrakeLightBrightness, brakeLightBrightness);
#else
    ledcWrite(PWM_CHANNEL, brakeLightBrightness * 255 / 100);
#endif

    // Update the last value
    lastBrakeLightBrightness = brakeLightBrightness;
  }

  switch (currentMode) {
    case RevMode::CALIBRATE: {
      cockpitSprite.fillSprite(BACKGROUND_COLOR);
      cockpitSprite.setTextColor(TFT_WHITE, TFT_BLACK);
      cockpitSprite.setTextDatum(4);
      cockpitSprite.drawString("move throttle", 85, 20, 2);
      cockpitSprite.drawString("a few times from", 85, 40, 2);
      cockpitSprite.drawString("full throttle to full brake", 85, 60, 2);
      cockpitSprite.drawString("and press OK", 85, 80, 2);
      cockpitSprite.drawRoundRect(10, 245, 70, 50, 2, TFT_RED);
      cockpitSprite.drawString("X", 45, 272, 4);
      cockpitSprite.drawRoundRect(90, 245, 70, 50, 2, TFT_GREEN);
      cockpitSprite.drawString("OK", 125, 272, 4);

      if (throttleRAW > maxVal) {
        maxVal = throttleRAW;
      }

      if (throttleRAW < minVal) {
        minVal = throttleRAW;
      }

      cockpitSprite.setTextDatum(0);
      cockpitSprite.drawString("old Value", 5, 110, 2);
      cockpitSprite.drawString(String(thMax), 5, 130, 4);
      cockpitSprite.drawString(String(thZero), 5, 160, 4);
      cockpitSprite.drawString(String(thMin), 5, 190, 4);

      cockpitSprite.setTextDatum(2);
      cockpitSprite.drawString("new Value", 165, 110, 2);
      cockpitSprite.drawString(String(maxVal), 165, 130, 4);
      cockpitSprite.drawString(String(throttleRAW), 165, 160, 4);
      cockpitSprite.drawString(String(minVal), 165, 190, 4);

      cockpitSprite.pushSprite(0, 0);

      if (touch.available()) {
        unsigned long currentTouchTime = millis();
        if (currentTouchTime - lastTouchTime > TOUCH_DEBOUNCE_TIME_MS) {
          lastTouchTime = currentTouchTime;

          if (touch.data.y > 245 && touch.data.y < 295 && touch.data.x > 85) {
            pref.begin("thValues", false);
            pref.putUInt("thMax",
                         maxVal);  // at 5V input, the Hall Sensor Value should
                                   // be 4095 on full throttle
            pref.putUInt("thZero",
                         throttleRAW);  // should be about 2880, depends on
                                        // input Voltage ~ 5V
            pref.putUInt(
                "thMin",
                minVal);  // should be about 2100, depends on input Voltage ~ 5V
            pref.end();
            cockpitSprite.fillSprite(BACKGROUND_COLOR);
            cockpitSprite.pushSprite(0, 0);
            changeMode(RevMode::LOCK);
          } else if (touch.data.y > 245 && touch.data.y < 295 &&
                     touch.data.x < 85) {
            cockpitSprite.fillSprite(BACKGROUND_COLOR);
            cockpitSprite.pushSprite(0, 0);
            changeMode(RevMode::LOCK);
          }
        }
      }
      break;
    }
    case RevMode::LOCK: {
      if (touch.available()) {
        uint32_t currentTouchTime = millis();
        if (currentTouchTime - lastTouchTime > TOUCH_DEBOUNCE_TIME_MS) {
          lastTouchTime = currentTouchTime;
          // Figure out of the touch was on one of the keys.
          char key = handleLockScreenTouch(touch.data.x, touch.data.y);

          if (key != '\0') {  // No key pressed.
            if (key == 'x' || key == 'X') {
              pin_entry.clear();
              log_i("Cleared PIN");
            } else {
              pin_entry += key;
              log_i("Keypad entry: %s", pin_entry.c_str());
            }
          } else {
            log_d(
                "Screen touch event on the lockscreen, but no digit selected.");
          }
          drawScreenLock(pin_entry);

          if (pin_entry.length() > 3) {
            int enteredPin = pin_entry.toInt();
            if (enteredPin == PIN_MODE_NORMAL) {
              log_i("Unlocked to normal mode.");
              changeMode(RevMode::RIDE_NORMAL);
            } else if (enteredPin == PIN_MODE_SPORT) {
              log_i("Unlocked to sport mode.");
              changeMode(RevMode::RIDE_SPORT);
            } else if (enteredPin == PIN_MODE_THROTTLE_CAL) {
              log_i("Unlocked to throttle calibration mode.");
              changeMode(RevMode::CALIBRATE);
            } else {
              log_w("Incorrect login attempt.");
              pin_entry.clear();
            }
          }
        }
      }
      break;
    }
    case RevMode::RIDE_SPORT:  // For right now anyway, the loop tasks for
                               // different ride modes are basically the same.
    case RevMode::RIDE_NORMAL: {
      // switch headlight
      if (touch.available()) {
        uint32_t currentTouchTime = millis();
        if (currentTouchTime - lastTouchTime > TOUCH_DEBOUNCE_TIME_MS) {
          lastTouchTime = currentTouchTime;
          //      if(touch.data.x == 85 && touch.data.y == 360) //only the touch
          //      button
          if (touch.data.y >= 212) lightF = !lightF;

#ifdef VESC_CONTROLS_LIGHTS
          vescUart.setHeadlightState(lightF);
#else
          digitalWrite(PIN_HEADLIGHT, lightF);
#endif
          log_i("Headlight %s", lightF ? "on" : "off");
        }
      }

      if (lightF && voltdropcomp) {  // compensation of the voltage drop when
                                     // headlight is turned on
        throttleRAW += thComp;
      }

      if (throttleRAW >= thZero) {
        nunck = map(throttleRAW, thZero, thMax + 100, 127,
                    maxNunck);  //+100 to avoid running out of range
      } else {
        nunck = map(throttleRAW, thZero, thMin - 100, 127,
                    0);  //-100 to avoid running out of range
      }

      if (digitalRead(PIN_BRAKE_SW) && nunck > 127 && stopOnBrake) {
        nunck = 127;  // interrupts acceleration when braking
      }

      nunck = constrain(nunck, 0,
                        255);  // Avoid underflow and overflow
                               // causing acceleration on throttle disconnect.

      // Send command to VESC if a mode change hasn't JUST occurred.
      uint32_t currentTimeMs = millis();
      if (currentTimeMs - lastModeChangeTimeMs > ANTI_STUTTER_DELAY_MS) {
        vescUart.nunchuck.valueY = nunck;

#ifndef DRY_RUN_MODE
        // Only send actual commands when not in dry run mode
        vescUart.setNunchuckValues();
#else
        // Log what we would have sent
        log_v("DRY RUN: Would send nunchuck value: %d", nunck);
#endif
      } else if (currentTimeMs < lastModeChangeTimeMs) {
        lastModeChangeTimeMs = currentTimeMs;
        log_e("Either millis() overflowed (unlikely), or we have problems.");
      } else {
        // We're still within the anti-stutter delay window
        // log_d("Waiting for anti-stutter delay to complete: %d ms remaining",
        //       ANTI_STUTTER_DELAY_MS - (currentTimeMs -
        //       lastModeChangeTimeMs));
      }
      drawScreenCockpit();
      break;
    }
    default:
      log_e("Invalid mode somehow.");
  }

  // This is a simple loop rate limiter.
  // It will delay the loop if it's running too fast.
  uint32_t thisLoopTime = millis();
  if (lastLoopTime > thisLoopTime) {
    log_e("millis() overflowed or other timing bug.");
  } else if (thisLoopTime - lastLoopTime < TARGET_LOOP_PERIOD) {
    delay(TARGET_LOOP_PERIOD - (thisLoopTime - lastLoopTime));
  } else {
    // TODO: Currently loop times are about 100-140ms. I'd like to speed that up
    // a tad if it's easy enough.
    log_i("Loop time exceeded target of %ld", TARGET_LOOP_PERIOD);
  }
  lastLoopTime = millis();
}

void setVescProfileSport() {
  bool store = false;       // save persistently the new profile in vesc memory
  bool forward_can = true;  // forward profile to slave Vesc through can bus
  bool ack = false;
  bool divide_by_controllers =
      false;  // divide the watt limit by the number of VESC
  float current_min_rel =
      1.0;  // this value is multiplied with the minimum current. It is a
            // convenient method to scale the current limits without forgetting
            // the actual minimal value.  DEFAULT = 1
  float current_max_rel =
      1.0;  // this value is multiplied with the maximum current. It is a
            // convenient method to scale the current limits without forgetting
            // the actual maximum value.  DEFAULT = 1
  float speed_max_reverse =
      -15;  // maximum reverse speed in m/s. In order, to use it make sure to
            // set the wheel diameter and gear ratio in VESC Tool   (m/s = (km/h
            // / 3.6) )
  float speed_max =
      15;  // maximum speed in m/s. In order, to use it make sure to set the
           // wheel diameter and gear ratio in VESC Tool
  float duty_min = 0.005;       // minimum duty cycle. DEFAULT = 0.005
  float duty_max = 1;           // maximum duty cycle. DEFAULT = 1
  float watt_min = -1500000.0;  // minimum watt value. DEFAULT = - 1500000.0
  float watt_max = 1500000.0;   // maximum watt value. DEFAULT =  1500000.0

#ifndef DRY_RUN_MODE
  vescUart.setLocalProfile(store, forward_can, divide_by_controllers,
                           current_min_rel, current_max_rel, speed_max_reverse,
                           speed_max, duty_min, duty_max, watt_min, watt_max);
#else
  log_i("DRY RUN: Would set SPORT profile");
#endif
}

void setVescProfileNormal() {
  // TODO
}

// Check if the VESC is connected and responding
bool waitForVescConnection(uint32_t timeout_ms = 3000) {
  uint32_t startTime = millis();
  log_i("Waiting for VESC to become ready...");

  while (millis() - startTime < timeout_ms) {
    // Try to get VESC firmware version as a simple connectivity check
    if (vescUart.getFWversion()) {
      log_i("VESC connected successfully! FW version: %d.%d",
            vescUart.fw_version.major, vescUart.fw_version.minor);
      return true;
    }

    // Brief delay between attempts
    delay(100);
  }

  log_w("VESC connection timed out after %d ms", timeout_ms);
  return false;
}
