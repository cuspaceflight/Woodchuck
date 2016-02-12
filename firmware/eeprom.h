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

#include <stdint.h>

// Initial setup function
void eeprom_init(void);

// Read and write a single byte
void eeprom_read(uint8_t addr, uint8_t *data);
void eeprom_write(uint8_t addr, uint8_t data);
void eeprom_read_dword(uint8_t addr, uint32_t *data);
void eeprom_write_dword(uint8_t addr, uint32_t data);

/* INTERNAL METHODS */

// Send the address and read/n-write bit and wait for it to be sent.
void _eeprom_send_7bit_address_blocking(uint8_t addr, uint8_t readwrite);

// Send a byte and wait for it to be sent
void _eeprom_send_blocking(uint8_t data);

// Wait for a byte to arrive and return it
uint8_t _eeprom_read_blocking(void);

// Helper function to set speed to 400 kHz
void _i2c_set_speed(uint32_t i2c, uint8_t fast);


#endif /* __EEPROM_H__ */
