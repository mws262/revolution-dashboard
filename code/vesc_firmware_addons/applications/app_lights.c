#include "app.h"
#include "ch.h"
#include "hal.h"

#include "comm_can.h"
#include "commands.h"
#include "hw.h"
#include "mc_interface.h"
#include "terminal.h"
#include "timeout.h"
#include "utils_math.h"

#include <stdio.h>
#include <string.h>

// Message IDs for setting light states via app data
#define MSG_SET_BRAKE_BRIGHTNESS 0x20
#define MSG_SET_HEADLIGHT_STATE  0x21

// Global brightness variable: 0 (off) to 100 (full brightness)
static volatile uint8_t brake_light_brightness = 0;

// Global flags for thread control
static volatile bool stop_now = true;
static volatile bool is_app_thread_running = false;
static volatile bool is_pwm_thread_running = false;

// Set brake light brightness (0-100)
void set_brake_light_brightness(uint8_t brightness) {
    brake_light_brightness = brightness;
}

// Set headlight state (true = on, false = off)
static void set_headlight_state(bool on) {
    if (on) {
        HEAD_LIGHT_ON();
    } else {
        HEAD_LIGHT_OFF();
    }
}

/* Brake light PWM thread:
   Uses a 10ms period (100Hz) and adjusts on-time based on brightness.
   Runs until stop_now is set. */
static THD_WORKING_AREA(waBrakeLightPWM, 128);
static THD_FUNCTION(brake_light_pwm_thread, arg) {
    (void)arg;
    is_pwm_thread_running = true;
    const uint32_t pwm_period_ms = 10;
    while (!stop_now) {
        uint8_t brightness = brake_light_brightness;
        uint32_t on_time = (brightness * pwm_period_ms) / 100;

        if (on_time == 0) {
            BRAKE_LIGHT_OFF();
            chThdSleepMilliseconds(pwm_period_ms);
        } else if (on_time == pwm_period_ms) {
            BRAKE_LIGHT_ON();
            chThdSleepMilliseconds(pwm_period_ms);
        } else {
            BRAKE_LIGHT_ON();
            chThdSleepMilliseconds(on_time);
            BRAKE_LIGHT_OFF();
            chThdSleepMilliseconds(pwm_period_ms - on_time);
        }
    }
    is_pwm_thread_running = false;
    chThdExit(0);
}

/* Main custom app thread for general logic */
static THD_WORKING_AREA(my_thread_wa, 1024);
static THD_FUNCTION(my_thread, arg) {
    (void)arg;
    chRegSetThreadName("App Custom");
    is_app_thread_running = true;
    while (!stop_now) {
        timeout_reset();
        // Additional custom app logic can be placed here.
        chThdSleepMilliseconds(10);
    }
    is_app_thread_running = false;
    chThdExit(0);
}

/* PWM callback called in interrupt context (optional) */
static void pwm_callback(void) {
    // Optionally add code that needs to run on each PWM control iteration.
}

/* Custom app data processing function.
   Expects the first byte to be the command ID.
   For MSG_SET_BRAKE_BRIGHTNESS, the next byte is brightness (0-100).
   For MSG_SET_HEADLIGHT_STATE, the next byte is the state (0=off, nonzero=on). */
static void process_lights_app_data(unsigned char *data, unsigned int len) {
    if (len < 2) {
        return;
    }
    uint8_t cmd = data[0];
    switch (cmd) {
        case MSG_SET_BRAKE_BRIGHTNESS: {
            uint8_t brightness = data[1];
            if (brightness > 100) {
                brightness = 100;
            }
            set_brake_light_brightness(brightness);
            // Acknowledge by echoing the command and brightness value back.
            uint8_t dataTx[2] = { cmd, brightness };
            commands_send_app_data(dataTx, 2);
            break;
        }
        case MSG_SET_HEADLIGHT_STATE: {
            uint8_t state = data[1];
            set_headlight_state(state != 0);
            // Acknowledge by echoing the command and state value back.
            uint8_t dataTx[2] = { cmd, state };
            commands_send_app_data(dataTx, 2);
            break;
        }
        default:
            break;
    }
}

/* Called when the custom application starts.
   Registers the PWM callback, starts threads, and sets the custom app data handler. */
void app_lights_start(void) {
    mc_interface_set_pwm_callback(pwm_callback);
    stop_now = false;
    chThdCreateStatic(my_thread_wa, sizeof(my_thread_wa), NORMALPRIO, my_thread, NULL);
    chThdCreateStatic(waBrakeLightPWM, sizeof(waBrakeLightPWM), NORMALPRIO, brake_light_pwm_thread, NULL);
    commands_set_app_data_handler(process_lights_app_data);
}

/* Called when the custom application stops.
   Clears the custom app data handler, unregisters callbacks, and waits for threads to exit. */
void app_lights_stop(void) {
    mc_interface_set_pwm_callback(0);
    commands_set_app_data_handler(0);
    stop_now = true;
    while (is_app_thread_running || is_pwm_thread_running) {
        chThdSleepMilliseconds(1);
    }
}

/* Unused custom configuration function */
void app_lights_configure(app_configuration *conf) { (void)conf; }
