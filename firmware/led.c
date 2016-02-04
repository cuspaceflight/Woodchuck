/**
* Woodchuck by CU Spaceflight
*
* This file is part of the Woodchuck project by Cambridge University Spaceflight.
*
* It is an adapted version of the JOEY-M version by Jon Sowman
*
* Jon Sowman 2012
* Gregory Brooks 2016
*
#include <avr/io.h>
*/
#include "led.h"


/**
 * Initialise the LEDs by setting them as outputs and turning
 * them off.
 */
void led_init()
{   
    //LED_DDR |= _BV(LED_RED) | _BV(LED_GREEN);
}

/**
 * Set the given LED to the given state.
 */
void led_set(int led, int status) {
	/*
    if( status & 0x01 )
        LED_PORT |= _BV(led & 0x01);
    else
        LED_PORT &= ~(_BV(led & 0x01));
	*/
}

