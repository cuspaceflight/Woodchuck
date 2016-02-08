/*
 * Timer Driver
 * Woodchuck
 * 
 * Adapted from
 * OKGO 2 - 2014 David Turner, Cambridge University Spaceflight
 * 
 * Woodchuck - 2016 Eivind Roson Eide, Cambridge University Spaceflight
 */

#ifndef CLOCK_H
#define CLOCK_H

#include <stdint.h>
#include <stdbool.h>


/**
 * Delay for a number of milliseconds, based off the systick timer
 * Delays around a small number of milliseconds may be inaccurate
 * @param delay Number of milliseconds to delay.
 */
void delay_ms(const uint32_t delay);


/**
 * Delay for approximately a microsecond. Very roughly calibrated by eye to
 * within about 20% precision.
 * @param delay Number of microseconds to delay.
 */
void delay_us(const uint32_t delay);

/* Setup systick */
void systick_init(void);

/* Get current millisecond timer value */
uint32_t get_millis(void);

#endif
