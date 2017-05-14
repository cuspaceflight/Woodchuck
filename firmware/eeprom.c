/**
 * Woodchuck by CU Spaceflight
 *
 * This file is part of the Woodchuck project by Cambridge University
 * Spaceflight.
 * 
 * Jamie Wood 2016
 */

#include "eeprom.h"

// Choose which I2C peripheral to use
#define I2C     I2CD1

// 24AA01 device address
#define EEPROM_ADDR     0b1010000

//static const I2CConfig i2ccfg = {OPMODE_I2C, 400000, STD_DUTY_CYCLE};

// Values taken from example in STM32F0 reference manual RM0091
static const I2CConfig i2ccfg = {
   STM32_TIMINGR_PRESC(0U) |
   STM32_TIMINGR_SCLDEL(0x3U)  |  STM32_TIMINGR_SDADEL(0x1U) |
   STM32_TIMINGR_SCLH(0x3U)    |  STM32_TIMINGR_SCLL(0x9U),
   0,
   0,
};

/**
 * Set up I2C communication at 400kHz.
 */
void eeprom_init(void)
{
    i2cStart(&I2C, &i2ccfg);
}

/**
 * Perform an I2C transaction, and retry up to n times if it fails.
 */
msg_t i2c_transmit_retry_n(uint8_t *txdat, uint8_t txdatlen, uint8_t *rxdat, uint8_t rxdatlen, systime_t timeout, uint8_t retries){
  static uint8_t i;
  for(i=0; i<(retries+1); i++){
    msg_t status = i2cMasterTransmitTimeout(&I2C, EEPROM_ADDR, txdat, txdatlen, rxdat, rxdatlen, timeout);

    if(status == MSG_TIMEOUT){
      i2cStop(&I2C);
      i2cStart(&I2C, &i2ccfg);
    }else{
      return status;
    }
  }
  return MSG_TIMEOUT;
}

/**
 * Read a single byte from the EEPROM at address addr.
 */
void eeprom_read(uint8_t addr, uint8_t *data)
{
    static uint8_t txdat[1] __attribute__((section("DATA_RAM")));
    static uint8_t rxdat[1] __attribute__((section("DATA_RAM")));

    txdat[0] = addr;

    i2cAcquireBus(&I2C);
    //i2c_transmit_retry_n(EEPROM_ADDR, txdat, 1, rxdat, 1, MS2ST(20), 5);
    msg_t status = i2c_transmit_retry_n(txdat, 1, rxdat, 1, MS2ST(20), 5);
    i2cReleaseBus(&I2C);

    if(status == MSG_OK){
        *data = rxdat[0];
    }
}


/**
 * Write a single byte to the EEPROM at address addr.
 */
void eeprom_write(uint8_t addr, uint8_t data)
{
    static uint8_t txdat[2] __attribute__((section("DATA_RAM"))); // Can't DMA from Core-Coupled Memory (where the stack resides)

    txdat[0] = addr;
    txdat[1] = data;

    i2cAcquireBus(&I2C);
    i2c_transmit_retry_n(txdat, 2, NULL, 0, MS2ST(20), 5);
    i2cReleaseBus(&I2C);
}

/**
 * Read a 32-bit little-endian dword from the EEPROM, starting at address addr.
 * 
 * It should be possible to use a sequential-read to speed this up, but
 * libopencm3 doesn't seem to support sending ACKs from master midway through
 * an I2C communication, which is needed to signal the next byte read.
 */
void eeprom_read_dword(uint8_t addr, uint32_t *data)
{
    static uint8_t rxdat[4] __attribute__((section("DATA_RAM")));
    static uint8_t txdat[1] __attribute__((section("DATA_RAM")));

    txdat[0] = addr;

    i2cAcquireBus(&I2C);
    msg_t status = i2c_transmit_retry_n(txdat, 1, rxdat, 4, MS2ST(20), 5);
    i2cReleaseBus(&I2C);

    if(status == MSG_OK){
        *data = ((rxdat[3] << 24) | (rxdat[2] << 16) |
                   (rxdat[1] << 8) | rxdat[0]);
    }
}

/**
 * Write a 32-bit dword to the EEPROM, starting at address addr.
 * The value is stored in little-endian format.
 *
 * Note that the address must be a multiple of 4, to align to page boundaries.
 */
void eeprom_write_dword(uint8_t addr, uint32_t value)
{
    static uint8_t txdat[5] __attribute__((section("DATA_RAM")));
    txdat[0] = addr;
    txdat[1] = value & 0xff;
    txdat[2] = (value >> 8) & 0xff;
    txdat[3] = (value >> 16) & 0xff;
    txdat[4] = (value >> 24) & 0xff;
    
    i2cAcquireBus(&I2C);
    i2c_transmit_retry_n(txdat, 5, NULL, 0, MS2ST(20), 5);
    i2cReleaseBus(&I2C);
}
