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
#include <libopencm3/stm32/gpio.h>
#include "led.h"

#define PORT_LED GPIOA
#define PIN_RED GPIO6
#define PIN_GREEN GPIO5
#define PIN_BLUE GPIO4

//variables to describe the current status
//!!! create class with led states to be read if necessary
//don't forget to set these state variables in the relevant led_set cases
//led_set  is not included in this class for backwards compatibility
void led_init()
{   
    //LED_DDR |= _BV(LED_RED) | _BV(LED_GREEN);

	//Set led pins as output, no internal pullup/down
	gpio_mode_setup(PORT_LED, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_RED | PIN_GREEN | PIN_BLUE);
	//set output options: open drain output,slow output (2MHz)
	gpio_output_options(PORT_LED, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, PIN_RED | PIN_GREEN | PIN_BLUE);

}

/**
 * Set the given LED to the given state.
 */
void led_set(int led, int status) {
	//status is 0 (off), 1(on), 2(blinking), 3(all off)

	/*TO DO:
	-enum arguments (led + status)
	-timer interrupt for blink, callback blink function
	-add toggle option to led_set


	-use libopencm3 for this
	-how to blink whilst freeing up processor?
	-led_init code
	-
	*/
	switch (status)
	{
		case 0:
			gpio_set(PORT_LED, led);//high on current sink output = led off
			break;
		case 1:
			
			gpio_clear(PORT_LED, led);//low on current sink output = led off
			break;
		case 2:
			//do blink thing
			break;
		case 3:
			gpio_clear(PORT_LED, PIN_RED | PIN_GREEN | PIN_BLUE);//turn everything off
		default:
			//do nothing
			
	}
}

