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
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#include "led.h"

#define PORT_LED GPIOA
#define PIN_RED GPIO6
#define PIN_GREEN GPIO5
#define PIN_BLUE GPIO4


volatile bool error_states[ERROR_MAX];
volatile led_modes statuses[3];//red, green, blue
int status_index = 0; //use this to record the current position in the led blinking queue


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

	//enable clock for timer interrupts
	//https://github.com/libopencm3/libopencm3-examples/blob/master/examples/stm32/l1/stm32l-discovery/button-irq-printf-lowpower/main.c
	//https://github.com/libopencm3/libopencm3-examples/blob/master/examples/stm32/f1/stm32-h103/timer/timer.c
	
	
	//https://github.com/libopencm3/libopencm3-examples/blob/393fe8e449d5334c715a3ca5f538328dbf387182/examples/stm32/f1/other/timer_interrupt/timer.c
	nvic_enable_irq(NVIC_TIM2_IRQ);
	nvic_set_priority(NVIC_TIM2_IRQ, 1);

	rcc_periph_clock_enable(RCC_TIM2);
	/* Set timer start value. */
	TIM_CNT(TIM2) = 1;

	/* Set timer prescaler. 8MHz/1000Hz*/
	TIM_PSC(TIM2) = 8000;

	/* End timer value. If this is reached an interrupt is generated. */
	TIM_ARR(TIM2) = 1000;//interrupt triggered every second

	/* Update interrupt enable. */
	TIM_DIER(TIM2) |= TIM_DIER_UIE;

	/* Start timer. */
	TIM_CR1(TIM2) |= TIM_CR1_CEN;
	
}

/**
 * Set the given LED to the given state.
 */
void led_set(int led, int status) {
	//status is 0 (off), 1(on), 2(blinking), 3(toggle)
	int pin;
	/*TO DO:
	-
	*/
	if (led == LED_RED || led == LED_GREEN || led == LED_BLUE) {
		
		switch (led)
		{
		case LED_RED:
			pin = PIN_RED;
			break;
		case LED_GREEN:
			pin = PIN_GREEN;
			break;
		case LED_BLUE:
			pin = PIN_BLUE
			break;
		}


		switch (status)
		{
		case LED_OFF:
			gpio_set(PORT_LED, pin);//high on current sink output = led off
			statuses[led] = LED_OFF;
			break;
		case LED_ON:
			gpio_clear(PORT_LED, pin);//low on current sink output = led off
			statuses[led] = LED_ON;
			break;
		case LED_BLINKING:
			statuses[led] = LED_BLINKING;
			//LED has been added to blinking queue
			//setting it to any other state removes it from queue
			break;
		case LED_TOGGLE:
			gpio_toggle(PORT_LED, pin);//toggle
			if (statuses[led] == LED_OFF) {
				statuses[led] == LED_ON;
			}
			else if (statuses[led] == LED_ON) {
				statuses[led] == LED_OFF;
			}
		default:
			//do nothing

		}
	}
}

void led_reset()//turn them all off (not blink timers)
{
	gpio_set(PORT_LED, PIN_RED | PIN_GREEN | PIN_BLUE);
	for (int x = 0, x < 3; x++) {
		statuses[x] = LED_OFF
	}
}

void led_set_error(error_enum err, bool set) {
	error_states[err] = set;
}


void tim2_isr(void)
{
	//timer interrupt for led blink
	
	/*cycle through the blink queue, this function runs once per second

	e.g.:
	//Remember: led and pin have different values (0,1,2 vs GPIO4,5,6)
	1. toggle the led set to blink mode
	2. increment position in queue
	3. toggle next one
	4.Call error response function?
	*/
	
	TIM_SR(TIM2) &= ~TIM_SR_UIF; /* Clear interrupt flag. */
}

int mod(int a, int b)
{
	if (b < 0) //you can check for b == 0 separately and do what you want
		return mod(-a, -b);
	int ret = a % b;
	if (ret < 0)
		ret += b;
	return ret;
}


void error_response()
{
	for (int x = 1; x < ERROR_MAX; x++)
	{
		if (error_states[x])
		{
			//switch case for each errors_enum enumerator
		}
	}
}