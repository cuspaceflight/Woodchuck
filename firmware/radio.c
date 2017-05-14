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
 * to be used run using ChibiOS.
 *
 * Aniruddh Raghu 2016
 * Gregory Brooks 2017
 */

#include <stdio.h>
#include <string.h>
//#include <util/delay.h>
//#include <util/crc16.h>

/* libopencm3 includes here
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/f0/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dac.h>
*/

#include "led.h"
#include "radio.h"


#define RADIO_USART  			USART1
#define RADIO_EN_GPIO 	 		GPIOA
#define RADIO_EN_PIN    		GPIOA_RADIO_EN_PIN
#define RADIO_TX_DATA           GPIOA_RADIO_TX_DATA

#define RADIO_TX_GPIO 			GPIOB
#define RADIO_CHANNEL_LSB		GPIOB_RADIO_CHANNEL_LSB
#define RADIO_CHANNEL_MSB		GPIOB_RADIO_CHANNEL_MSB

#define RADIO_DAC               DACD1

#define SDRV                    SD1  // Serial port for radio
#define RADIO_BAUD_RATE			9600
#define RADIO_NUM_DATA_BITS		8
#define RADIO_DELAY				20  // ms delay between bits

//Defines for what each bit is meant to mean. 
//TODO check this adheres to the RTTY protocol properly!
/*
#define START_BIT				0
#define STOP_BIT				1
#define LOGICAL_ONE				100
#define LOGICAL_ZERO			        183
*/

//~0.21V swing (~425Hz)
#define LOGICAL_ONE                              120
#define LOGICAL_ZERO                             136
#define START_BIT                                LOGICAL_ZERO
#define STOP_BIT                                 LOGICAL_ONE

static void radio_transmit_string(char* string);
static void _radio_transmit_bit(uint8_t data, uint8_t ptr);
static uint16_t radio_calculate_checksum(char* data);

static SerialConfig usartRadio =
{
RADIO_BAUD_RATE, // bit rate
0, 
USART_CR2_STOP1_BITS,  // One stop bit
0
};

static const DACConfig daccfg = {
  .init         = LOGICAL_ONE,  // Start DAC output at rest condition
  .datamode     = DAC_DHRM_8BIT_RIGHT /* data holding register mode */
};

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
		                //gpio_clear(RADIO_TX_GPIO,RADIO_CHANNEL_LSB);
		palClearPad(RADIO_TX_GPIO, RADIO_CHANNEL_LSB);
	else 
		                //gpio_set(RADIO_TX_GPIO,RADIO_CHANNEL_LSB);
		palSetPad(RADIO_TX_GPIO, RADIO_CHANNEL_LSB);

	if ((channel & 0x02) == 1) 
		                //gpio_clear(RADIO_TX_GPIO, RADIO_CHANNEL_MSB);
		palClearPad(RADIO_TX_GPIO, RADIO_CHANNEL_MSB);
	else
		                //gpio_set(RADIO_TX_GPIO, RADIO_CHANNEL_MSB);
		palSetPad(RADIO_TX_GPIO, RADIO_CHANNEL_MSB);

	//Send value to DAC. TODO CHECK THIS WORKS!!!
	                //dac_software_trigger(CHANNEL_1);
	                //dac_load_data_buffer_single(val, RIGHT8, CHANNEL_1);
	dacPutChannelX(&RADIO_DAC, 0, val);
	
}

                /*
                * Initialise the peripheral clocks.
                */
                /*
                static void clock_setup(void)
                {
	                //rcc_clock_setup_in_hsi_out_48mhz();  // Necessary?
	                rcc_periph_clock_enable(RCC_GPIOA);

	                /* Enable clocks for USART1. 
	                rcc_periph_clock_enable(RCC_USART1);

	                /* Enable clock for timer6, to be used for interrupts 
	                rcc_periph_clock_enable(RCC_TIM2);

	                //Enable APB1 (Advanced Peripheral Bus) clock for DAC
	                rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_DACEN);
                }*/

                /*
                * Set up GPIOA for USART transmit.

                static void gpio_setup(void)
                {
	                gpio_mode_setup(RADIO_EN_GPIO, GPIO_MODE_AF, GPIO_PUPD_NONE, RADIO_EN_PIN);

	                /* Setup USART1 TX pin as alternate function. 
	                /* GPIO_AF1 is USART1_TX 
	                gpio_set_af(RADIO_EN_GPIO, GPIO_AF1, RADIO_EN_PIN);
                }
                */

/**
* Configure RADIO_USART (USART1) for Radiometrix MTX2.
*/
static void usart_setup(void)
{
                        /*
	                usart_set_baudrate(RADIO_USART, RADIO_BAUD_RATE);
	                usart_set_databits(RADIO_USART, RADIO_NUM_DATA_BITS);
	                usart_set_stopbits(RADIO_USART, USART_CR2_STOP_1_0BIT);
	                usart_set_mode(RADIO_USART, USART_MODE_TX);
	                usart_set_parity(RADIO_USART, USART_PARITY_NONE);
	                usart_set_flow_control(RADIO_USART, USART_FLOWCONTROL_NONE);
	                usart_enable(RADIO_USART);
	                */
	
	sdStart(&SDRV, &usartRadio);
}

/**
* Configure DAC- set RADIO_TX_DATA to be output of DAC
* Triggered by software.
*/
static void dac_setup(void)
{
                        /*
                        gpio_mode_setup(RADIO_TX_GPIO, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, RADIO_TX_DATA);
                        dac_disable(CHANNEL_1);
                        dac_set_waveform_characteristics(DAC_CR_MAMP1_8);
                        dac_set_waveform_generation(DAC_CR_WAVE1_TRI);
                        dac_enable(CHANNEL_1);
                        dac_set_trigger_source(DAC_CR_TSEL1_SW);
                        dac_load_data_buffer_single(0, RIGHT8, CHANNEL_1);
                        dac_software_trigger(CHANNEL_1);
                        */
        palSetPadMode(RADIO_EN_GPIO, RADIO_TX_DATA, PAL_MODE_INPUT_ANALOG);
	dacStart(&RADIO_DAC, &daccfg);
}

                /**
                * Set up TIM2 for radio interrupts
                * TODO Check that we don't need to start the timer immediately
                */
                /*static void setup_TIM2(void)
                {
                        timer_reset(TIM2);
                        /* 48MHz / 12kHz -1. 
                        timer_set_prescaler(TIM2, 3999); /* 48MHz/12kHz - 1 
                        /* 12kHz for 1 tick = 12 khz overflow 
                        timer_set_period(TIM2, 1);
                        timer_continuous_mode(TIM2);
                }*/

/**
* Enable interrupts on TIM2 and start it.
* TODO Check that we don't need to reset the timer
*/
/*static void start_TIM2(void)
{
	timer_disable_counter(TIM2);
	timer_clear_flag(TIM2, TIM_SR_UIF);
	nvic_enable_irq(NVIC_TIM2_IRQ);
	timer_enable_update_event(TIM2); /* Unclear 
	timer_enable_irq(TIM2, TIM_DIER_UIE);
	timer_set_counter(TIM2, 0);
	timer_enable_counter(TIM2);
}*/

/**
* Disable interrupts on TIM2 and stop it
* TODO Check that we don't need to reset the timer
*/
/*
static void stop_TIM2(void)
{
	timer_clear_flag(TIM2, TIM_SR_UIF);
	nvic_disable_irq(NVIC_TIM2_IRQ);
	timer_disable_counter(TIM2);
	timer_set_counter(TIM2, 0);
}*/

/**
 * Initialise the radio module and all the timers required.
 */
void radio_init(void)
{
	//clock_setup();
	//gpio_setup();
	usart_setup();
	//setup_TIM2();
	dac_setup();
	
	//!!!!!
	//TODO add other initialisation stuff here, like setting
	//the operating frequency, channel number, and TX power.
	//See datasheet for more info on the command strings.
    
    //radio_set_baud(RADIO_BAUD_50); RTTY NOT SERIAL

}

/*static THD_FUNCTION(radio_transmit_string, arg){
	char* string = (char* )arg;
	
	while(*string){
		//Transmit byte
		//Send 11 bits (1 start, 8 data, 2 stop)
		for(int _txptr = 0; _txptr < 10; _txptr++){
			_radio_transmit_bit(*string, _txptr);
			chThdSleepMilliseconds(20); // 50 baud
		}
		string++;
	}
	chthdExit(0);
}*/

static void radio_transmit_string(char* string){
	while(*string){
		//Transmit byte
		//Send 11 bits (1 start, 8 data, 2 stop)
		for(int _txptr = 0; _txptr < 10; _txptr++){
			systime_t startTime = chVTGetSystemTime();
			systime_t endTime = startTime + MS2ST(RADIO_DELAY);
			_radio_transmit_bit(*string, _txptr);
			
			while(chVTIsSystemTimeWithin(startTime, endTime)){
			}
		}
		string++;
	}
}

/**
 * Ported from JOEY-M driver.
 * Transmit a telemetry string.
 */
void radio_transmit_sentence(char * message)
{    
    // Calculate the checksum
    uint16_t checksum = radio_calculate_checksum(message);
    char cs[7];
    sprintf(cs, "*%04X\n", checksum);
    char fullmsg[strlen(message) + strlen(cs)];
    strcpy(fullmsg, message);
    strcat(fullmsg, cs);
    radio_transmit_string(fullmsg);
    //thread_t *tx = chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(128),NORMALPRIO, radio_transmit_string, (void *)fullmsg);
    //thread_t *tx = chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(128), NORMALPRIO, radio_transmit_string, (void *)fullmsg);
    /*if (tp == NULL)
		chSysHalt("out of memory");*/
	//msg_t msg = chThdWait(tx);  // Return thread memory to heap
}

/*static THD_FUNCTION(_radio_transmit_byte, arg){
	uint8_t _txbyte = (uint8_t)arg;
	
	//Send 11 bits (1 start, 8 data, 2 stop)
	for(int _txptr = 0; _txptr < 10; _txptr++){
		_radio_transmit_bit(_txbyte, _txptr);
		chThdSleepMilliseconds(20); // 50 baud
	}
	
	chthdExit(0);
}*/


/*static void radio_transmit_string(char* string)
{
	/*while(*string)
    {
        _txbyte = *string;
        _txptr = 0;
        byte_complete = false;
        // Enable interrupts
        start_TIM2();
        while(!byte_complete)
        	;
        //iwdg_reset();
        string++;
    }*//*
    while(*string){
		thread_t *tx = chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(128),
		                                   NORMALPRIO, _radio_transmit_byte, *string);
		/*if (tp == NULL)
			chSysHalt("out of memory");*//*
		msg_t msg = chThdWait(tx);  // Return thread memory to heap
		string++;
	}
}*/

/**
* Some radio chatter so that Woodchuck can be located
*/
void radio_chatter(void)
{
	radio_write(0,100);
	chThdSleepMilliseconds(200);
	radio_write(0,200);
	chThdSleepMilliseconds(200);
	radio_write(0,100);
	chThdSleepMilliseconds(200);
	radio_write(0,200);
	chThdSleepMilliseconds(200);
}

/**
 * Calculate the checksum for the radio string excluding any $ signs
 * at the start.
 */
static uint16_t radio_calculate_checksum(char* data)
{
	//http://www.nongnu.org/avr-libc/user-manual/group__util__crc.html#gaca726c22a1900f9bad52594c8846115f
    uint16_t i;
    uint16_t crc = 0xFFFF;

    for (i = 0; i < strlen(data); i++) {
        //if (data[i] != '$') crc = _crc_xmodem_update(crc,(uint8_t)data[i]);
        if (data[i] != '$'){
			int j;
			crc = crc ^ ((uint16_t)data[i] << 8);
			for (j = 0; j < 8; j++){
				if(crc & 0x8000)
					crc = (crc << 1) ^ 0x1021;
				else
					crc << 1;
			}
		}
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
    //for the data bits.
    else if(ptr >= 1 && ptr <= 8)
        if(data)
            radio_write(0,LOGICAL_ONE);
        else
            radio_write(0,LOGICAL_ZERO);
    else if(ptr == 9)
    	//we're at the end of a message- send two stop bits
    {
    	radio_write(0, STOP_BIT);
    	radio_write(0, STOP_BIT);
    }
}
