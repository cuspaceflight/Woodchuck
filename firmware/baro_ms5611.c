/*
 * MS5611-01BA03 Driver
 * Woodchuck
 * 
 * Adapted from
 * M2FC - 2014 Adam Greig, Cambridge University Spaceflight
 * 
 * Woodchuck - Eivind Roson Eide 2016
 */


#include <libopencm3/stm32/spi.h>

#include "gps.h"
#include "led.h"
#include "radio.h"

/*
 * The microcontroller talks to the barometer using SPI,
 * The driver uses libopemcm3/spi.h 
 * (documentation: http://libopencm3.github.io/docs/latest/stm32f0/html/group__spi__file.html)
 * 
 */

//#define MS5611_SPID        SPID2
//#define MS5611_SPI_CS_PORT GPIOB
//#define MS5611_SPI_CS_PIN  GPIOB_BARO_CS



static void ms5611_reset(void);
static void ms5611_read_u16(uint8_t adr, uint16_t* c);
static void ms5611_read_s24(uint8_t adr, int32_t* d);
static void ms5611_init(MS5611CalData* cal_data);
static void ms5611_read_cal(MS5611CalData* cal_data);
static void ms5611_read(MS5611CalData* cal_data,
                        int32_t* temperature, int32_t* pressure);

int32_t global_temperature;
int32_t global_pressure;

/*
 * Resets the MS5611. Sends 0x1E, waits 5ms.
 * 
 */
static void ms5611_reset()
{
    uint8_t adr = 0x1E;
//    spiSelect(&MS5611_SPID);
//    spiSend(&MS5611_SPID, 1, (void*)&adr);
//    chThdSleepMilliseconds(5);
//    spiUnselect(&MS5611_SPID);
}


