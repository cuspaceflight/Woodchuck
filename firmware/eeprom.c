/**
 * Woodchuck by CU Spaceflight
 *
 * This file is part of the Woodchuck project by Cambridge University
 * Spaceflight.
 * 
 * Jamie Wood 2016
 */

#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include "eeprom.h"

// Choose which I2C peripheral to use
#define I2C     I2C1

// 24AA01 device address and read/n-write bit
#define EEPROM_ADDR     0b1010000
#define EEPROM_READ     1
#define EEPROM_WRITE    0

// Needed by _i2c_set_speed (added by same pull request)
#define I2C_CR2_FREQ_MASK   0x3ff
#define I2C_CCR_CCRMASK     0xfff
#define I2C_TRISE_MASK      0x3f
// Should be included by i2c.h (found in i2c_common_all.h), but aren't :(
#define I2C_CCR(i2c_base)       MMIO32((i2c_base) + 0x1c)
#define I2C_TRISE(i2c_base)     MMIO32((i2c_base) + 0x20)
#define I2C_CCR_FS              (1<<15)

/**
 * Set up I2C communication at 400kHz.
 */
void eeprom_init(void)
{
    // Reset the peripheral
    i2c_reset(I2C);

    // Set the clock registers to set the peripheral to 400kHz
    _i2c_set_speed(I2C, 1);

    // Enable the peripheral
    i2c_peripheral_enable(I2C);
}

/**
 * Read a single byte from the EEPROM at address addr.
 */
void eeprom_read(uint8_t addr, uint8_t *data)
{
    i2c_send_start(I2C); // Send START condition

    _eeprom_send_7bit_address_blocking(EEPROM_ADDR, EEPROM_WRITE); // send the address
    _eeprom_send_blocking(addr); // send the read address

    i2c_send_start(I2C); // Send RESTART

    _eeprom_send_7bit_address_blocking(EEPROM_ADDR, EEPROM_READ); // send the address
    *data = _eeprom_read_blocking(); // read the data

    i2c_send_stop(I2C); // Send STOP condition
}


/**
 * Write a single byte to the EEPROM at address addr.
 */
void eeprom_write(uint8_t addr, uint8_t data)
{
    i2c_send_start(I2C); // Send START condition

    // send the address
    _eeprom_send_7bit_address_blocking(EEPROM_ADDR, EEPROM_WRITE);

    _eeprom_send_blocking(addr); // send the write address
    _eeprom_send_blocking(data); // send the data

    i2c_send_stop(I2C); // Send STOP condition
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
    uint32_t tmp;
    uint8_t tmp_byte;
    eeprom_read(addr, &tmp_byte);
    tmp = tmp_byte;
    eeprom_read(addr+1, &tmp_byte);
    tmp = (tmp << 8) | tmp_byte;
    eeprom_read(addr+2, &tmp_byte);
    tmp = (tmp << 8) | tmp_byte;
    eeprom_read(addr+3, &tmp_byte);
    tmp = (tmp << 8) | tmp_byte;
    *data = tmp;
}

/**
 * Write a 32-bit dword to the EEPROM, starting at address addr.
 * The value is stored in little-endian format.
 *
 * Note that the address must be a multiple of 4, to align to page boundaries.
 */
void eeprom_write_dword(uint8_t addr, uint32_t data)
{
    i2c_send_start(I2C); // Send START condition

    // send the address
    _eeprom_send_7bit_address_blocking(EEPROM_ADDR, EEPROM_WRITE);

    _eeprom_send_blocking(addr); // send the write address
    _eeprom_send_blocking(data & 0xff); // send the data
    _eeprom_send_blocking((data>>8) & 0xff);
    _eeprom_send_blocking((data>>16) & 0xff);
    _eeprom_send_blocking((data>>24) & 0xff);

    i2c_send_stop(I2C); // Send STOP condition
}

/**
 * Send the address and read/n-write bit and wait for it to be sent.
 */
void _eeprom_send_7bit_address_blocking(uint8_t addr, uint8_t readwrite)
{
    i2c_send_7bit_address(I2C, addr, readwrite);
    while( (I2C_ISR(I2C) & I2C_ISR_TXIS) == 0 );
}

/**
 * Send a byte and wait for it to be sent.
 */
void _eeprom_send_blocking(uint8_t data)
{
    i2c_send_data(I2C, data);
    while( (I2C_ISR(I2C) & I2C_ISR_TXIS) == 0 );
}

/**
 * Wait for a byte to arrive and return it.
 */
uint8_t _eeprom_read_blocking(void)
{
    while( (I2C_ISR(I2C) & I2C_ISR_RXNE) != 0 );
    return i2c_get_data(I2C);
}

/**
 * Set the speed of the I2C peripheral to 400kHz
 * 
 * Taken from a pending (as of Feb 2016) pull request to libopencm3 (#470).
 * This may get merged in future, so this method might be obsolete.
 */
void _i2c_set_speed(uint32_t i2c, uint8_t fast)
{
    int freq;
    uint32_t reg;

    /* force disable, to set clocks */
    I2C_CR1(i2c) &= ~(I2C_CR1_PE);

    /* frequency in megahertz */
    freq = rcc_apb1_frequency / 1000000;
    reg = (I2C_CR2(i2c) & ~(I2C_CR2_FREQ_MASK)) | (freq & I2C_CR2_FREQ_MASK);
    I2C_CR2(i2c) = reg;

    if (fast) {
        I2C_CCR(i2c) = I2C_CCR_FS | (((freq * 5) / 6) & I2C_CCR_CCRMASK);
    } else {
        I2C_CCR(i2c) = (freq * 5) & I2C_CCR_CCRMASK;
    }

    /* set rise time to 1000ns */
    reg = (I2C_TRISE(i2c) & ~(I2C_TRISE_MASK)) | ((freq + 1) & I2C_TRISE_MASK);
    I2C_TRISE(i2c) = reg;
    /* enable i2c device */
    I2C_CR1(i2c) |= I2C_CR1_PE;
}
