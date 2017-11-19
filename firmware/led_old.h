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
} error_enum;

typedef enum {
	LED_OFF = 0,
	LED_ON,
	LED_BLINKING,
	LED_TOGGLE,
	MODES_SIZE
} led_modes;

typedef enum {//backwards compatibility with joey function calls
	LED_RED = 0,
	LED_GREEN,
	LED_BLUE,
	LED_RG,//yellow
	LED_RB,//magenta
	LED_GB,//cyan
	LED_RGB,//white
	COLOURS_SIZE
} led_colours;

extern volatile bool error_states[ERROR_MAX];
extern volatile led_modes statuses[COLOURS_SIZE];

/*
#define LED_RED       0
#define LED_GREEN     1
#define LED_BLUE      2

#define LED_OFF			0
#define LED_ON			1
#define LED_BLINKING    2
#define LED_TOGGLE      3
*/



void led_init(void);
void led_set(led_colours led, led_modes status);
void led_reset(void);//turn them all off incl. blink timers
bool led_read(led_colours led);//check whether red, green or blue (or any) is illuminated
//true if illuminated, false if not
void led_interrupt(void);
void error_response(void);


/* Set a specific error.
* If `set` is true then the error is occurring.
*/
void led_set_error(error_enum err, bool set);

#endif /* __LED_H__ */
