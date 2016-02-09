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

#include <stdint.h>
#include <stdbool.h>

/*#define LED_DDR         DDRB
#define LED_PORT        PORTB
#define LED_RED         1
#define LED_GREEN       0
*/

typedef enum {
	ERROR_CONFIG = 1,
	ERROR_RADIO,
	ERROR_GPS,
	ERROR_BARO,
	ERROR_MAX
} errors_enum;

extern volatile bool error_states[ERROR_MAX];
extern voltatile led_modes statuses[3];

typedef enum {
	LED_OFF = 0,
	LED_ON,
	LED_BLINKING,
	LED_TOGGLE
} led_modes;

typedef enum {//backwards compatibility with joey function calls
	LED_RED = 0,
	LED_GREEN,
	LED_BLUE
} led_colours;

#define LED_RED       0
#define LED_GREEN     1
#define LED_BLUE      2

#define LED_OFF			0
#define LED_ON			1
#define LED_BLINKING    2
#define LED_TOGGLE      3
*/



void led_init();
void led_set(int led, int status);
void led_reset();//turn them all off incl. blink timers

/* Set a specific error.
* If `set` is true then the error is occurring.
*/
void led_set_error(error_enum err, bool set);

#endif /* __LED_H__ */
