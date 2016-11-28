/**
 * Woodchuck by CU Spaceflight
 *
 * This file is part of the Woodchuck project by Cambridge University
 * Spaceflight.
 * 
 * Jamie Wood 2016
 */

#ifndef __EEPROM_H__
#define __EEPROM_H__

#include "ch.h"
#include "hal.h"

// Initial setup function
void eeprom_init(void);

// Read and write a single byte
void eeprom_read(uint8_t addr, uint8_t *data);
void eeprom_write(uint8_t addr, uint8_t data);
void eeprom_read_dword(uint8_t addr, uint32_t *data);
void eeprom_write_dword(uint8_t addr, uint32_t data);

#endif /* __EEPROM_H__ */
