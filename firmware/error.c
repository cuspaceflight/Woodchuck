/**
* Woodchuck by CU Spaceflight
*
* This file is part of the Woodchuck project by Cambridge University Spaceflight.
*
* Gregory Brooks 2017
*/

#include "error.h"
#include "led.h"

void set_error(error_enum err){
    switch (err) {
    case ERROR_CONFIG:
        //do something e.g. blink red
        led_set(LED_RED, LED_BLINKING);
        break;
    case ERROR_RADIO:
        //do something e.g. blink green
        led_set(LED_GREEN, LED_BLINKING);
        break;
    case ERROR_GPS:
        //do something e.g. blink blue
        led_set(LED_BLUE, LED_BLINKING);
        break;
    case ERROR_BARO:
        //do something e.g. blink cyan
        led_set(LED_GB, LED_BLINKING);
        break;
    case ERROR_EEPROM:
        //do something e.g. blink magenta
        led_set(LED_RB, LED_BLINKING);
    default:
        //do something e.g. turn on red
        led_set(LED_RED, LED_ON);
        break;
    }
}
