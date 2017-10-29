/**
* Woodchuck by CU Spaceflight
*
* This file is part of the Woodchuck project by Cambridge University Spaceflight.
*
* It is an adapted version of the JOEY-M version by Jon Sowman and Badger 3 (avionics14)
*
* Jon Sowman 2012
* Gregory Brooks 2016
*/

#ifndef __LED_H__
#define __LED_H__

// #include <stdint.h>
// #include <stdbool.h>
#include "ch.h"
#include "hal.h"


// Enums backwards compatible with joey function calls
typedef enum {
	LED_OFF = 0,
	LED_ON,
	LED_BLINKING,
	MODES_SIZE
} led_modes;

typedef enum {
	LED_GREEN = 0,
	LED_RED,
	LED_BLUE,
    LED_GB,//cyan
	LED_RG,//yellow
	LED_RB,//magenta
	LED_RGB,//white
	COLOURS_SIZE
} led_colours;


void led_init(void);
void led_off(bool blocking);
void led_set(led_colours led, led_modes status);
void led_toggle(led_colours led);
void led_reset(void);  // Turn them all off incl. blinking
led_modes led_read(led_colours led);  // Check LED status


#endif /* __LED_H__ */
