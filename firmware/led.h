/**
* Woodchuck by CU Spaceflight
*
* This file is part of the Woodchuck project by Cambridge University Spaceflight.
*
* It is an adapted version of the JOEY-M version by Jon Sowman
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
#define LED_RED			0
#define LED_GREEN		1
#define LED_BLUE		2

#define LED_OFF			0
#define LED_ON			1
#define LED_FLASHING	2


void led_init();
void led_set(int led, int status);

#endif /* __LED_H__ */
