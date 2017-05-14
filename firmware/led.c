/**
* Woodchuck by CU Spaceflight
*
* This file is part of the Woodchuck project by Cambridge University Spaceflight.
*
* It is an adapted version of the JOEY-M version by Jon Sowman
*
*
* Jon Sowman 2012
*
* Gregory Brooks 2016
*
#include <avr/io.h>
*/
#include "ch.h"
#include "hal.h"
#include "led.h"
//#include "clock.h"

#define PORT_LED GPIOA
#define PIN_RED (1 << GPIOA_LED_RED)
#define PIN_GREEN (1 << GPIOA_LED_GREEN)
#define PIN_BLUE (1 << GPIOA_LED_BLUE)
#define PIN_RG (PIN_RED | PIN_GREEN)
#define PIN_RB (PIN_RED | PIN_BLUE)
#define PIN_GB (PIN_GREEN | PIN_BLUE)
#define PIN_RGB (PIN_RED | PIN_GREEN | PIN_BLUE)
uint16_t pins[COLOURS_SIZE] = {PIN_RED, PIN_GREEN, PIN_BLUE, PIN_RG, PIN_RB, PIN_GB, PIN_RGB};

volatile bool error_states[ERROR_MAX];
volatile led_modes statuses[COLOURS_SIZE];
int status_index = COLOURS_SIZE; //use this to record the current position in the led blinking queue
//must start as COLOURS_SIZE(index after index of last led) for blink queue to work

void led_init()
{
	//LED_DDR |= _BV(LED_RED) | _BV(LED_GREEN);

	//Set led pins as output, no internal pullup/down
	//gpio_mode_setup(PORT_LED, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_RGB);
    palSetPadMode(PORT_LED, PIN_RED, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(PORT_LED, PIN_GREEN, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(PORT_LED, PIN_BLUE, PAL_MODE_OUTPUT_PUSHPULL);
	//set output options: open drain output,slow output (2MHz)
	//gpio_set_output_options(PORT_LED, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, PIN_RGB);
}

/**
 * Set the given LED to the given state.
 */
void led_set(led_colours led, led_modes status) {
	//see status enum for possible values of status

	switch (status) {
	case LED_OFF:
		// high on current sink output = led off
		palSetPort(PORT_LED, pins[led]);
		statuses[led] = LED_OFF;
		break;
	case LED_ON:
		// low on current sink output = led on
		palClearPort(PORT_LED, pins[led]);
		statuses[led] = LED_ON;
		break;
	case LED_BLINKING:
		statuses[led] = LED_BLINKING;
		//LED has been added to blinking queue
		//setting it to any other state removes it from queue
		break;
	case LED_TOGGLE:
		//gpio_toggle(PORT_LED, pins[led]);
		palTogglePort(PORT_LED, pins[led]);
		if (statuses[led] == LED_OFF) {
			statuses[led] = LED_ON;
		} else if (statuses[led] == LED_ON) {
			statuses[led] = LED_OFF;
		}
		break;
	default:
		//do nothing
		break;
	}
}

/**
 * Turn all LEDs off, incl. blinking
 */
void led_reset()
{
	//gpio_set(PORT_LED, PIN_RGB);
	palClearPort(PORT_LED, PIN_RGB);
	for (int x = 0; x < COLOURS_SIZE; x++) {
		statuses[x] = LED_OFF;
	}
}

/**
 * Check whether red, green or blue is illuminated.
 * Returns true if illuminated, false if not.
 * Output pin must be low to illuminate led
 */
bool led_read(led_colours led)
{
	uint16_t outputreg = palReadLatch(PORT_LED);
	if (led == LED_RED || led == LED_GREEN || led == LED_BLUE) {
		return !(outputreg & led);
	} else {
		// no individual led specified or wrong enum used
		// just check whether any leds are illuminated
		return !((outputreg | PIN_RGB) == outputreg);
	}
}

void led_set_error(error_enum err, bool set)
{
	error_states[err] = set;
}

/**
 * timer interrupt for led blink
 * cycle through the blink queue, TODO: this function should run once per second
 */
void led_interrupt()
{
	if (status_index < COLOURS_SIZE) {
		// there is no pins[COLOURS_SIZE] variable
		// blinking leds will just be off for status_index == COLOURS_SIZE
		//gpio_toggle(PORT_LED, pins[status_index]);
		palTogglePort(PORT_LED, pins[status_index]);
	}

	do {
		status_index++;
		status_index = status_index % (COLOURS_SIZE + 1);  //range from 0 to COLOURS_SIZE
		if (status_index == COLOURS_SIZE) {
			break;   //there is no statuses[COLOURS_SIZE] variable
		}
	} while (statuses[status_index] != LED_BLINKING);  //only blink leds set to blink

	if (status_index < COLOURS_SIZE) {  //there is no pins[COLOURS_SIZE] variable
		//gpio_toggle(PORT_LED, pins[status_index]);
		palTogglePort(PORT_LED, pins[status_index]);
	}
}

void error_response()
{
	for (int x = 1; x < ERROR_MAX; x++) {
		if (error_states[x]) {
			//switch case for each errors_enum enumerator
			switch (x) {
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
			default:
				//do nothing
				break;
			}
		}
	}
}
