/**
 * Woodchuck by CU Spaceflight
 *
 * This file is part of the Woodchuck project by Cambridge University
 * Spaceflight.
 * 
 * Based on the radio drivers from the JOEY-M project by Cambridge 
 * University Spaceflight (written by Jon Sowman) and the Martlet2
 * project (written by Adam Greig)
 *
 * This is a driver written for the Radiometrix MTX2 module, designed
 * to be used run using libopencm3 as a HAL.
 *
 * Aniruddh Raghu 2016
 */

#include <stdio.h>
#include <string.h>
//#include <util/delay.h>
//#include <util/crc16.h>

// libopencm3 includes here
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/f0/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dac.h>

#include "led.h"
#include "radio.h"


#define RADIO_USART  			USART1
#define RADIO_EN_GPIO 	 		GPIOA
#define RADIO_EN_PIN    		GPIO9

#define RADIO_TX_GPIO 			GPIOB
#define RADIO_CHANNEL_LSB		GPIO13
#define RADIO_CHANNEL_MSB		GPIO14
#define RADIO_TX_DATA			GPIO15

#define RADIO_BAUD_RATE			9600
#define RADIO_NUM_DATA_BITS		8

//Defines for what each bit is meant to mean. 
//TODO check this adheres to the RTTY protocol properly!
#define START_BIT				0
#define STOP_BIT				1
#define LOGICAL_ONE				100
#define LOGICAL_ZERO			183

static void radio_transmit_string(char * string);
static uint16_t radio_calculate_checksum(char* data);


/**
* Take a channel and a value and send it to the radio.
* TODO Integration with DAC such that we can send any 
* signal- CHECK THIS
*/
static void radio_write(uint8_t channel, uint16_t val)
{
	//First, find out which channel we're sending data to
	channel &= 0x03;
	if ((channel & 0x01) == 1) 
		gpio_clear(RADIO_TX_GPIO,RADIO_CHANNEL_LSB);
	else 
		gpio_set(RADIO_TX_GPIO,RADIO_CHANNEL_LSB);

	if ((channel & 0x02) == 1) 
		gpio_clear(RADIO_TX_GPIO, RADIO_CHANNEL_MSB);
	else
		gpio_set(RADIO_TX_GPIO, RADIO_CHANNEL_MSB);

	//Send value to DAC. TODO CHECK THIS WORKS!!!
	dac_software_trigger(CHANNEL_1);
	dac_load_data_buffer_single(val, RIGHT8, CHANNEL_1);
}

/**
* Initialise the peripheral clocks.
*/
static void clock_setup()
{
	rcc_clock_setup_in_hsi_out_48mhz();
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Enable clocks for USART1. */
	rcc_periph_clock_enable(RCC_USART1);

	/* Enable clock for timer6, to be used for interrupts */
	rcc_periph_clock_enable(RCC_TIM2);

	//Enable APB1 (Advanced Peripheral Bus) clock for DAC
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_DACEN);
}

/**
* Set up GPIOA for USART transmit.
*/
static void gpio_setup()
{
	gpio_mode_setup(RADIO_EN_GPIO, GPIO_MODE_AF, GPIO_PUPD_NONE, RADIO_EN_PIN);

	/* Setup USART1 TX pin as alternate function. */
	/* GPIO_AF1 is USART1_TX */
	gpio_set_af(RADIO_EN_GPIO, GPIO_AF1, RADIO_EN_PIN);
}

/**
* Configure RADIO_USART (USART1) for Radiometrix MTX2.
*/
static void usart_setup()
{
	usart_set_baudrate(RADIO_USART, RADIO_BAUD_RATE);
	usart_set_databits(RADIO_USART, RADIO_NUM_DATA_BITS);
	usart_set_stopbits(RADIO_USART, USART_CR2_STOP_1_0BIT);
	usart_set_mode(RADIO_USART, USART_MODE_TX);
	usart_set_parity(RADIO_USART, USART_PARITY_NONE);
	usart_set_flow_control(RADIO_USART, USART_FLOWCONTROL_NONE);
	usart_enable(RADIO_USART);
}

/**
* Configure DAC- set RADIO_TX_DATA to be output of DAC
* Triggered by software.
*/
static void dac_setup()
{
	gpio_mode_setup(RADIO_TX_GPIO, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, RADIO_TX_DATA);
	dac_disable(CHANNEL_1);
	dac_set_waveform_characteristics(DAC_CR_MAMP1_8);
	dac_set_waveform_generation(DAC_CR_WAVE1_TRI);
	dac_enable(CHANNEL_1);
	dac_set_trigger_source(DAC_CR_TSEL1_SW);
	dac_load_data_buffer_single(0, RIGHT8, CHANNEL_1);
	dac_software_trigger(CHANNEL_1);
}

/**
* Set up TIM2 for radio interrupts
* TODO Check that we don't need to start the timer immediately
*/
static void setup_TIM2()
{
	timer_reset(TIM2);
	/* 48MHz / 12kHz -1. */
	timer_set_prescaler(TIM2, 3999); /* 48MHz/12kHz - 1 */
	/* 12kHz for 1 tick = 12 khz overflow */
	timer_set_period(TIM2, 1);
	timer_continuous_mode(TIM2);
}

/**
* Enable interrupts on TIM2 and start it.
* TODO Check that we don't need to reset the timer
*/
static void start_TIM2()
{
	timer_disable_counter(TIM2);
	timer_clear_flag(TIM2, TIM_SR_UIF);
	nvic_enable_irq(NVIC_TIM2_IRQ);
	timer_enable_update_event(TIM2); /* Unclear */
	timer_enable_irq(TIM2, TIM_DIER_UIE);
	timer_set_counter(TIM2, 0);
	timer_enable_counter(TIM2);
}

/**
* Disable interrupts on TIM2 and stop it
* TODO Check that we don't need to reset the timer
*/
static void stop_TIM2()
{
	timer_clear_flag(TIM2, TIM_SR_UIF);
	nvic_disable_irq(NVIC_TIM2_IRQ);
	timer_disable_counter(TIM2);
	timer_set_counter(TIM2, 0);
}

/**
 * Initialise the radio module and all the timers required.
 */
void radio_init()
{
	clock_setup();
	gpio_setup();
	usart_setup();
	setup_TIM2();
	dac_setup();
	//TODO add other initialisation stuff here, like setting
	//the operating frequency, channel number, and TX power.
	//See datasheet for more info on the command strings.

}

/**
 * Ported from JOEY-M driver.
 * Transmit a telemetry string.
 */
void radio_transmit_sentence(char * message)
{
	radio_transmit_string(message);
    
    // Calculate the checksum and send it
    uint16_t checksum = radio_calculate_checksum(message);
    char cs[7];
    sprintf(cs, "*%04X\n", checksum);
    radio_transmit_string(cs);
}

volatile uint8_t _txbyte = 0;
volatile uint8_t _txptr = 0;
volatile bool byte_complete = false;
volatile int systicks = 0;

static void radio_transmit_string(char* string)
{
	while(*string)
    {
        _txbyte = *string;
        _txptr = 0;
        byte_complete = false;
        // Enable interrupts
        start_TIM2();
        while(!byte_complete)
        	;
        iwdg_reset();
        string++;
    }
}

/**
* Some radio chatter so that Woodchuck can be located
*/
void radio_chatter()
{
	radio_write(0,100);
	delay(200);
	radio_write(0,200);
	delay(200);
	radio_write(0,100);
	delay(200);
	radio_write(0,200);
	delay(200);
}

/**
 * Calculate the checksum for the radio string excluding any $ signs
 * at the start.
 */
static uint16_t radio_calculate_checksum(char* data)
{
    uint16_t i;
    uint16_t crc = 0xFFFF;

    for (i = 0; i < strlen(data); i++) {
        if (data[i] != '$') crc = _crc_xmodem_update(crc,(uint8_t)data[i]);
    }
    return crc;
}

/**
 * Transmit a single bit at a pointer, also coping with start and 
 * stop bits.
 * Partially ported from JOEY-M.
 */
static void _radio_transmit_bit(uint8_t data, uint8_t ptr)
{
	//if we're at the start of the message...
    if(ptr == 0)
        radio_write(0,START_BIT);
    //for the data bits. TODO Check logic!
    else if(ptr >= 1 && ptr <= 8)
        if(data)
            radio_write(0,LOGICAL_ONE);
        else
            radio_write(0,LOGICAL_ZERO);
    else
    	//we're at the end of a message- send two stop bits
    {
    	radio_write(0, STOP_BIT);
    	radio_write(0, STOP_BIT);
    }
}


/**
 * Interrupt service routine for TIM2 (Radio)
 * Every time we get here, continue process of transmitting
 * a bit of data to the radio.
 */
void tim2_isr()
{
	if( systicks < 1 )
    {
        systicks++;
    }
    else
    {
        if( _txptr < 11 ) //Should this be 10?
        {
            _radio_transmit_bit(_txbyte, _txptr);
            _txptr++;
        } 
        else 
        {
            //Clear the timer
            stop_TIM2();
            byte_complete = true;
        }
        systicks = 0;
    }
}

int main()
{
	while(true)
		;
}
