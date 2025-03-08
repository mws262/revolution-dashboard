# Optional adjustments to VESC firmware
My motor controller is an FT85BD. I converted it from using Flipsky's proprietary hardware (see https://github.com/mws262/flipsky-ft85bd-vesc). It has its own MOSFETs for controlling accessories like brake lights and headlight. 

The app_lights VESC application can listen for these commands and switch accordingly. For now, the app is directly started in main.c. In the Rev code, there is a flag, VESC_CONTROLS_LIGHTS, that when set tells the VESC to turn the lights on instead of using ESP32 digital pins to switch the FETs directly.

Note that for the FT85BD, the outputs for the lights are 12V. On the Rev, I believe the lights are 5V. 12V will definitely fry the rear lights. My Rev is missing a bunch of parts, so I don't know about the stock front lights. 
