/**
* Woodchuck by CU Spaceflight
*
* This file is part of the Woodchuck project by Cambridge University Spaceflight.
*
* Gregory Brooks 2017
*/

#ifndef __ERROR_H__
#define __ERROR_H__

// #include <stdint.h>
// #include <stdbool.h>
#include "ch.h"
#include "hal.h"

typedef enum {
	ERROR_CONFIG = 0,
	ERROR_RADIO,
	ERROR_GPS,
	ERROR_BARO,
    ERROR_EEPROM,
	ERROR_MAX
} error_enum;

void set_error(error_enum err);


#endif /* __ERROR_H__ */
