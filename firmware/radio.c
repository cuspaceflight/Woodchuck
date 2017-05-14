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
#define RADIO_FREQ_KHZ			434650

//~0.21V swing (~425Hz)
#define LOGICAL_ONE                              120
#define LOGICAL_ZERO                             136
#define START_BIT                                LOGICAL_ZERO
#define STOP_BIT                                 LOGICAL_ONE

static void radio_transmit_string(char* string);
static void _radio_transmit_bit(uint8_t data, uint8_t ptr);
static uint16_t radio_calculate_checksum(char* data);
static int mtx2SetFreqTemp(uint32_t freqKHz);

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

	dacPutChannelX(&RADIO_DAC, 0, val);
	
}

/**
* Configure RADIO_USART (USART1) for Radiometrix MTX2.
*/
static void usart_setup(void)
{	
	sdStart(&SDRV, &usartRadio);
}

/**
* Configure DAC- set RADIO_TX_DATA to be output of DAC
* Triggered by software.
*/
static void dac_setup(void)
{
    palSetPadMode(RADIO_EN_GPIO, RADIO_TX_DATA, PAL_MODE_INPUT_ANALOG);
	dacStart(&RADIO_DAC, &daccfg);
}

/**
 * Initialise the radio module and all the timers required.
 */
void radio_init(void)
{
	usart_setup();
	dac_setup();
	
	
	mtx2SetFreqTemp(RADIO_FREQ_KHZ);
	
	//TODO add other initialisation stuff here, like setting
	//the operating frequency, channel number, and TX power.
	//See datasheet for more info on the command strings.

}

static void radio_transmit_string(char* string){
	while(*string){
		//Transmit byte
		//Send 11 bits (1 start, 8 data, 2 stop)
		for(int _txptr = 0; _txptr < 10; _txptr++){
			systime_t startTime = chVTGetSystemTime();
			systime_t endTime = startTime + MS2ST(RADIO_DELAY);
			_radio_transmit_bit(*string, _txptr);
			
			while(chVTIsSystemTimeWithin(startTime, endTime)){
				//iwdg_reset();
			}
		}
		//iwdg_reset();
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
}

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
	//iwdg_reset();  // Necessary?
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
					crc <<= 1;
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

static int mtx2SetFreqTemp(uint32_t freqKHz){
	uint8_t integer;
	uint64_t fraction;
	integer = 0xFF & freqKHz/6500;  // Round down
	integer--;
	fraction = (uint64_t)freqKHz * 0x80000 / 6500;
	fraction -= (uint64_t)integer * 0x80000;
	if(fraction < 0x80000 || fraction > 0xFFFFF){
		return 1;  // Fraction should not lie outside these values
	}
	char command[14];
	snprintf(command, sizeof(command), "@PRG_%02X%06llX\r", integer, fraction);
	sdWrite(&SDRV, (uint8_t *) command, strlen(command));
	return 0;  // Success
}
