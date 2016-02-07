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

#ifndef __RADIO_H__
#define __RADIO_H__

 void radio_transmit_sentence(char * sentence);

 void radio_chatter();

 void radio_init();

 #endif 