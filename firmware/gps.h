/**
 * Woodchuck by CU Spaceflight
 *
 * This file is part of the Woodchuck project by Cambridge University
 * Spaceflight.
 *
 * Based on the gps driver from the JOEY-M project by Cambridge University
 * Spaceflight.
 * 
 * Jon Sowman 2012
 * Jamie Wood 2016
 */

#ifndef __GPS_H__
#define __GPS_H__

#include <stdint.h>
#include <stdbool.h>

// Initial setup function
void gps_init(void);

// Poll for position and UTC time
void gps_get_position(int32_t* lat, int32_t* lon, int32_t* alt);
void gps_get_time(uint8_t* hour, uint8_t* min, uint8_t* second);

// Poll for fix status, and check OK
void gps_check_lock(uint8_t* lock, uint8_t* sats);

// Set the GPS to <1g airborne mode
void gps_set_mode(void);

// Check the GPS is set to <1g airborne mode
uint8_t gps_check_nav(void);


/* INTERNAL METHODS */

// Try to set the baudrate. Returns success
void _gps_set_baud(uint32_t baudrate);

// Calculates the UBX checksum for a given data and length
void _gps_ubx_checksum(uint8_t* data, uint8_t len, uint8_t* cka, uint8_t* ckb);

// Verifies the checksum of the given data is correct
bool _gps_verify_checksum(uint8_t* data, uint8_t len);

// Transmits a byte-array of data to the GPS
void _gps_send_msg(uint8_t* data, uint8_t len);

// Reads a single byte from the GPS
uint8_t _gps_get_byte(void);

// Flushes the receive buffer
void _gps_flush_buffer(void);

#endif /*__GPS_H__ */
