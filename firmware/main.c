/**
 * Woodchuck by CU Spaceflight
 * 
 * This file is part of the Woodchuck project by Cambridge University Spaceflight. 
 *
 * It is an adapted version of the JOEY-M version by Jon Sowman
 *
 * Jon Sowman 2012
 * Jamie Wood 2016
 */

//#include <avr/io.h>
#include <stdio.h>
//#include <util/delay.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

#include "ch.h"
#include "hal.h"

#include "eeprom.h"
#include "led.h"
#include "radio.h"
#include "gps.h"

char s[100];
uint8_t ticks_addr = 0x00;

int main(void)
{
	halInit();
	chSysInit();
    // Disable, configure, and start the watchdog timer
    /*
    iwdg_reset();
    iwdg_set_period_ms(8000);
    iwdg_start();
    */

    // Start and configure all hardware peripherals
    led_init();
    radio_init();
    gps_init();
    eeprom_init();

    // Radio chatter
    for(uint8_t i = 0; i < 5; i++)
    {
        radio_chatter();
        //iwdg_reset();
    }
    
    int32_t lat = 0, lon = 0, alt = 0;
    uint8_t hour = 0, minute = 0, second = 0, lock = 0, sats = 0;

    while(true)
    {
        led_set(LED_GREEN, 1);

        // Get the current system tick and increment
        uint32_t tick;
        eeprom_read_dword(ticks_addr, &tick);
        tick += 1;

        // Check that we're in airborne <1g mode
        if( gps_check_nav() != 0x06 ) led_set(LED_RED, 1);

        // Get information from the GPS
        gps_check_lock(&lock, &sats);
        if( lock == 0x02 || lock == 0x03 || lock == 0x04 )
        {
            gps_get_position(&lat, &lon, &alt);
            gps_get_time(&hour, &minute, &second);
        }

        led_set(LED_GREEN, 0);

        // Format the telemetry string & transmit
        double lat_fmt = (double)lat / 10000000.0;
        double lon_fmt = (double)lon / 10000000.0;
        alt /= 1000;

        sprintf(s, "$$" CALLSIGN ",%lu,%02u:%02u:%02u,%02.7f,%03.7f,%ld,%u,%x",
            tick, hour, minute, second, lat_fmt, lon_fmt, alt,
            sats, lock);
        radio_chatter();
        radio_transmit_sentence(s);
        radio_chatter();

        led_set(LED_RED, 0);
        eeprom_write_dword(ticks_addr, tick);
        //iwdg_reset();
        chThdSleepMilliseconds(500);
    }

    return 0;
}
