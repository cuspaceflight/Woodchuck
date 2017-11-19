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
#include "error.h"

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
        // Get the current system tick and increment
        uint32_t tick;
        eeprom_read_dword(ticks_addr, &tick);
        tick += 1;


		ublox_pvt_t pvt_packet;
	 	bool res = gps_poll_pvt(&pvt_packet);
		if ( res && (pvt_packet.fix_type == 0x02 || pvt_packet.fix_type == 0x03 || pvt_packet.fix_type == 0x04) ) {
			lat = pvt_packet.lat;
			lon = pvt_packet.lon;
			alt = pvt_packet.height;
			hour = pvt_packet.hour;
			minute = pvt_packet.minute;
			second = pvt_packet.second;
			lock = pvt_packet.fix_type;
			sats = pvt_packet.num_sv;
		}
		else{
			set_error(ERROR_GPS);
		}

        // Format the telemetry string & transmit
        double lat_fmt = (double)lat / 10000000.0;
        double lon_fmt = (double)lon / 10000000.0;
        alt /= 1000;

        sprintf(s, "$$" CALLSIGN ",%lu,%02u:%02u:%02u,%02.7f,%03.7f,%ld,%u,%x",
            tick, hour, minute, second, lat_fmt, lon_fmt, alt,
            sats, lock);

		led_set(LED_GREEN, 1);

        radio_chatter();
        radio_transmit_sentence(s);
        radio_chatter();



        eeprom_write_dword(ticks_addr, tick);
        //iwdg_reset();
		led_set(LED_GREEN, 0);
        chThdSleepMilliseconds(20000);
    }

    return 0;
}
