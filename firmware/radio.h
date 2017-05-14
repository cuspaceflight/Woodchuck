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

#ifndef __RADIO_H__
#define __RADIO_H__

#include <stdint.h>
#include <stdbool.h>

#include "ch.h"
#include "hal.h"

 void radio_transmit_sentence(char * message);

 void radio_chatter(void);

 void radio_init(void);

#endif /* __RADIO_H__ */
