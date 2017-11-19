/**
 * Woodchuck by CU Spaceflight
 *
 * This file is part of the Woodchuck project by Cambridge University
 * Spaceflight.
 *
 * Based on the gps driver from the JOEY-M project by Cambridge University
 * Spaceflight.
 *
 * Jon Sowman  2012
 * Jamie Wood  2016
 * Greg Brooks 2017
 */

#include "gps.h"
#include "led.h"
#include "radio.h"
#include "error.h"

// Choose which USART module to use
#define SDRV       SD2  // Also see mcuconf.h

// Header sync bytes - no touching!
#define UBX_SYNC_1  (0xB5)
#define UBX_SYNC_2  (0x62)

// Class and ID bytes - no touching!
#define UBX_NAV_CLASS       (0x01)
#define UBX_ACK_CLASS       (0x05)
#define UBX_CFG_CLASS       (0x06)

#define UBX_NAV_POSLLH_ID   (0x02)
#define UBX_NAV_SOL_ID      (0x06)
#define UBX_NAV_TIMEUTC_ID  (0x21)
#define UBX_ACK_ACK_ID      (0x01)
#define UBX_ACK_NAK_ID      (0x00)
#define UBX_CFG_PRT_ID      (0x00)
#define UBX_CFG_MSG_ID      (0x01)
#define UBX_CFG_NAV5_ID     (0x24)

/**
 * Global serial config structure for GPS interface
 * Best not to touch this directly, use getter/setter functions instead
 * _serial_begin and _serial_get_baud
 * 8N1, no flow control
 */
static SerialConfig uartGPS =
{
    .speed = 9600,
    .cr1 = 0,
    .cr2 = 0,
    .cr3 = 0,
};

// Setter function for serial port
void _serial_begin(uint32_t baud){
    uartGPS.speed = baud;
    sdStop(&SDRV);
    sdStart(&SDRV, &uartGPS);
}

// Getter function to get current baud rate
uint32_t _serial_get_baud(void){
    return uartGPS.speed;
}

/**
 * Set up USART for communication with the uBlox GPS at 38400 baud.
 *
 * We have to start at 9600, then tell the uBlox to change up to 38400 baud.
 */
void gps_init(void)
{
    // 9600 baud rate
    _serial_begin(9600);

    _gps_set_baud(9600);


    // Set the GPS into the correct mode (<1g airborne)
    gps_set_mode();
}

/**
 * Poll the GPS for a position message then extract the useful
 * information from it, using a NAV-POSLLH message.
 */
void gps_get_position(int32_t* lat, int32_t* lon, int32_t* alt)
{
    // Request a NAV-POSLLH message from the GPS
    uint8_t request[8] = {UBX_SYNC_1, UBX_SYNC_2, UBX_NAV_CLASS,
        UBX_NAV_POSLLH_ID, 0x00, 0x00, 0x03, 0x0A};
    _gps_send_msg(request);

    uint8_t buf[36];
    //uint8_t i = 0;
    //for(i = 0; i < 36; i++)
    //    buf[i] = _gps_get_byte();
    _gps_recv_msg(buf, 36);

    // Verify the sync and header bits
    if( buf[0] != UBX_SYNC_1 || buf[1] != UBX_SYNC_2 )
        set_error(ERROR_GPS);
    if( buf[2] != UBX_NAV_CLASS || buf[3] != UBX_NAV_POSLLH_ID )
        set_error(ERROR_GPS);

    // 4 bytes of longitude (1e-7)
    *lon = (int32_t)buf[10] | (int32_t)buf[11] << 8 |
        (int32_t)buf[12] << 16 | (int32_t)buf[13] << 24;

    // 4 bytes of latitude (1e-7)
    *lat = (int32_t)buf[14] | (int32_t)buf[15] << 8 |
        (int32_t)buf[16] << 16 | (int32_t)buf[17] << 24;

    // 4 bytes of altitude above MSL (mm)
    *alt = (int32_t)buf[22] | (int32_t)buf[23] << 8 |
        (int32_t)buf[24] << 16 | (int32_t)buf[25] << 24;

    if( !_gps_verify_checksum(&buf[2], 32) ) set_error(ERROR_GPS);
}

/**
 * Get the hour, minute and second from the GPS using the NAV-TIMEUTC
 * message.
 */
 void gps_get_time(uint8_t* hour, uint8_t* minute, uint8_t* second)
{
    // Send a NAV-TIMEUTC message to the receiver
    uint8_t request[8] = {UBX_SYNC_1, UBX_SYNC_2, UBX_NAV_CLASS,
        UBX_NAV_TIMEUTC_ID, 0x00, 0x00, 0x22, 0x67};
    _gps_send_msg(request);

    // Get the message back from the GPS
    uint8_t buf[28];
    //uint8_t i = 0;
    //for(i = 0; i < 28; i++)
    //    buf[i] = _gps_get_byte();
    _gps_recv_msg(buf, 28);

    // Verify the sync and header bits
    if( buf[0] != UBX_SYNC_1 || buf[1] != UBX_SYNC_2 )
        set_error(ERROR_GPS);
    if( buf[2] != UBX_NAV_CLASS || buf[3] != UBX_NAV_TIMEUTC_ID )
        set_error(ERROR_GPS);

    *hour = buf[22];
    *minute = buf[23];
    *second = buf[24];

    if( !_gps_verify_checksum(&buf[2], 24) ) set_error(ERROR_GPS);
}

/**
 * Check the navigation status to determine the quality of the
 * fix currently held by the receiver with a NAV-SOL message.
 */
void gps_check_lock(uint8_t* lock, uint8_t* numsats)
{
    // Construct the request to the GPS
    uint8_t request[8] = {UBX_SYNC_1, UBX_SYNC_2, UBX_NAV_CLASS, UBX_NAV_SOL_ID,
        0x00, 0x00, 0x07, 0x16};
    _gps_send_msg(request);

    // Get the message back from the GPS
    uint8_t buf[60];
    //uint8_t i;
    //for(i = 0; i < 60; i++)
    //   buf[i] = _gps_get_byte();
    _gps_recv_msg(buf, 60);

    // Verify the sync and header bits
    if( buf[0] != UBX_SYNC_1 || buf[1] != UBX_SYNC_2 )
        set_error(ERROR_GPS);
    if( buf[2] != UBX_NAV_CLASS || buf[3] != UBX_NAV_SOL_ID )
        set_error(ERROR_GPS);

    // Check 60 bytes minus SYNC and CHECKSUM (4 bytes)
    if( !_gps_verify_checksum(&buf[2], 56) ) set_error(ERROR_GPS);

    // Return the value if GPSfixOK is set in 'flags'
    if( buf[17] & 0x01 )
        *lock = buf[16];
    else
        *lock = 0;

    *numsats = buf[53];
}

/**
 * Set the GPS to <1g airborne navigation mode, using a CFG-NAV5 message.
 */
void gps_set_mode(void)
{
    /* Parameters:
     * mask:        0x0001 (only apply dynModel setting)
     * dynModel:    0x06 (airborne <1g)
     * blank x33:   0x00 (not needed due to mask)
     *
     * Checksum bytes are pre-computed
     */
    uint8_t request[44] = {UBX_SYNC_1, UBX_SYNC_2, UBX_CFG_CLASS,
        UBX_CFG_NAV5_ID, 36, 0x00, 0x00, 0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x55, 0x8f};
    _gps_send_msg(request);
    chThdSleepMilliseconds(300);

    // Read the response from the GPS
    uint8_t buf[10];
    //uint8_t i = 0;
    //for(i = 0; i < 10; i++)
    //    buf[i] = _gps_get_byte();
    uint8_t result = _gps_recv_ack(buf);

    if (result == 2) led_set(LED_RGB, LED_BLINKING);
    else if (result == UBX_ACK_ACK_ID) led_set(LED_RB, LED_ON);
    chThdSleepSeconds(9999);

    // // Verify sync and header bytes
    // if( buf[0] != UBX_SYNC_1 || buf[1] != UBX_SYNC_2 )
    //     set_error(ERROR_GPS);
    // if( buf[2] != UBX_ACK_CLASS)
    //     set_error(ERROR_GPS);
    //
    // // Check message checksum
    // if( !_gps_verify_checksum(&buf[2], 6) ) set_error(ERROR_GPS);
    //
    // // Check if we received an ACK or NACK
    // if(buf[3] != UBX_ACK_ACK_ID){ // If not an ACK
    //     set_error(ERROR_GPS);
    // }
}

/**
 * Verify that the uBlox 6 GPS receiver is set to the <1g airborne
 * navigaion mode, using a CFG-NAV5 message.
 */
uint8_t gps_check_nav(void)
{
    uint8_t request[8] = {UBX_SYNC_1, UBX_SYNC_2, UBX_CFG_CLASS, UBX_CFG_NAV5_ID, 0x00, 0x00,
        0x2A, 0x84};
    _gps_send_msg(request);

    // Get the message back from the GPS
    uint8_t buf[44];
    //uint8_t i = 0;
    //for(i = 0; i < 44; i++)
    //    buf[i] = _gps_get_byte();
    _gps_recv_msg(buf, 44);

    // Verify sync and header bytes
    if( buf[0] != UBX_SYNC_1 || buf[1] != UBX_SYNC_2 )
        set_error(ERROR_GPS);
    if( buf[2] != UBX_CFG_CLASS || buf[3] != UBX_CFG_NAV5_ID )
        set_error(ERROR_GPS);

    // Check 40 bytes of message checksum
    if( !_gps_verify_checksum(&buf[2], 40) ) set_error(ERROR_GPS);

    // Clock in and verify the ACK/NACK
    uint8_t ack[10];
    //for(i = 0; i < 10; i++)
    //    ack[i] = _gps_get_byte();
    _gps_recv_ack(ack);

    // If we got a NACK, then return 0xFF
    if( ack[3] == 0x00 ) return 0xFF;

    // Return the navigation mode and let the caller analyse it
    return buf[8];
}

/**
 * Set the baudrate of the GPS using a CFG-PRT message.
 * We also update our own baudrate to match.
 */
void _gps_set_baud(uint32_t baudrate)
{
    // Backup the old baudrate incase something goes wrong
    //uint32_t oldBRR = _serial_get_baud();

    /* First stop NMEA from clogging the buffer
     * Parameters:
     * portID:      0x01 (UART)
     * reserved:    0x00
     * txReady:     0x0000 (disable txReady pin)
     * mode:        0b00001000 11000000 = 0x08 0xC0 (1 stop bit, no parity, 8 data bits)
     * baudrate:    <baudrate> (little endian)
     * inProtoMask: 0x0001 (UBX input only)
     * outProtoMask:0x0000 (stop transmitting for now)
     * flags:       0x0000 (disable extended timeout)
     * reserved x2: 0x0000
     */
    uint8_t cka = 0;
    uint8_t ckb = 0;
    uint8_t pre_request[28] = {UBX_SYNC_1, UBX_SYNC_2, UBX_CFG_CLASS,
        UBX_CFG_PRT_ID, 20, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0xC0,
        baudrate & 0xff, (baudrate >> 8) & 0xff, (baudrate >> 16) & 0xff,
        (baudrate >> 24) & 0xff, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, cka, ckb};

    // Compute the checksum and fill it out
    _gps_ubx_checksum(&pre_request[2], 24, &cka, &ckb);
    pre_request[26] = cka;
    pre_request[27] = ckb;

    // Transmit the request
    _gps_send_msg(pre_request);

    chThdSleepMilliseconds(500);
    _gps_flush_buffer();

    // // TESTING
    // while(1){
    //     _gps_send_msg(pre_request);
    // }

    // /* Now set new baudrate
    //  * Parameters:
    //  * portID:      0x01 (UART)
    //  * reserved:    0x00
    //  * txReady:     0x0000 (disable txReady pin)
    //  * mode:        0b00001000 11000000 = 0x08 0xC0 (1 stop bit, no parity, 8 data bits)
    //  * baudrate:    <baudrate> (little endian)
    //  * inProtoMask: 0x0001 (UBX input only)
    //  * outProtoMask:0x0001 (UBX output only)
    //  * flags:       0x0000 (disable extended timeout)
    //  * reserved x2: 0x0000
    //  */
    // cka = 0;
    // ckb = 0;
    // uint8_t request[28] = {UBX_SYNC_1, UBX_SYNC_2, UBX_CFG_CLASS,
    //     UBX_CFG_PRT_ID, 0x00, 20, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0xC0,
    //     baudrate & 0xff, (baudrate >> 8) & 0xff, (baudrate >> 16) & 0xff,
    //     (baudrate >> 24) & 0xff, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
    //     0x00, cka, ckb};
    //
    // // Compute the checksum and fill it out
    // _gps_ubx_checksum(&request[2], 24, &cka, &ckb);
    // request[26] = cka;
    // request[27] = ckb;
    //
    // // Transmit the request
    // _gps_send_msg(request);
    //
    // // Change our baud rate to match the new speed
    // _serial_begin(baudrate);
    //
    // chThdSleepMilliseconds(300);
    // // Read the response from the GPS
    // uint8_t buf[10];
    // //uint8_t i = 0;
    // //for(i = 0; i < 9; i++)
    // //    buf[i] = _gps_get_byte();
    // uint8_t result = _gps_recv_ack(buf);
    // if (result != UBX_ACK_ACK_ID){
    //     if (result == 2) led_set(LED_RB, LED_BLINKING);
    //     else set_error(ERROR_GPS);
    //     _serial_begin(oldBRR);
    // }
}

/**
 * Verify the checksum for the given data and length.
 */
bool _gps_verify_checksum(uint8_t* data, uint8_t len)
{
    uint8_t a, b;
    _gps_ubx_checksum(data, len, &a, &b);
    if( a != *(data + len) || b != *(data + len + 1))
        return false;
    else
        return true;
}

/**
 * Calculate a UBX checksum using 8-bit Fletcher (RFC1145)
 */
void _gps_ubx_checksum(uint8_t* data, uint8_t len, uint8_t* cka, uint8_t* ckb)
{
    *cka = 0;
    *ckb = 0;
    uint8_t i = 0;
    for( i = 0; i < len; i++ )
    {
        *cka += *data;
        *ckb += *cka;
        data++;
    }
}

/**
 * Send a binary message to the GPS.
 */
void _gps_send_msg(uint8_t* data)
{
    _gps_flush_buffer();
    // Length determined from message length field
    size_t len = 8 + ((uint16_t*)data)[2];
    size_t nwritten;
    nwritten = sdWrite(&SDRV, data, len);
    if (nwritten==0) led_set(LED_RED,LED_ON);

}

/**
 * Receive bytes from GPS (up to specified number of bytes).
 */
void _gps_recv_msg(uint8_t* buf, size_t len){
    sdReadTimeout(&SDRV, buf, len, MS2ST(50));  /** Time for GPS to finish transmitting
                                                 *  Change to TIME_IMMEDIATE if necessary
                                                 */
}

uint8_t _gps_recv_ack(uint8_t* buf){
    // buf is 10 bytes long
    // Returns ACK id, NAK id or other uint8_t (neither received)
    msg_t dummy;

    do {
        dummy = sdGetTimeout(&SDRV, MS2ST(50));
        if(dummy == Q_TIMEOUT) return 2;
    } while ( dummy != 0xB5);

    buf[0] = 0xB5;
    _gps_recv_msg(&buf[1], 9);

    // Verify header byte
    if( buf[1] != UBX_SYNC_2 )
        return 3;
    if( buf[2] != UBX_ACK_CLASS)
        return 4;

    // Check message checksum
    if( !_gps_verify_checksum(&buf[2], 5) ) return 5;

    // Check if we received an ACK or NACK
    if(buf[3] == UBX_ACK_ACK_ID || buf[3] == UBX_ACK_NAK_ID ){
        return buf[3];
    }
    return 6;
}


/**
 * Receive a single byte from the GPS and return it.
 */
uint8_t _gps_get_byte(void)
{
    return sdGet(&SDRV);
}

/**
 * Flush the USART recieve buffer.
 */
void _gps_flush_buffer(void)
{
    msg_t dummy;

    do {
        dummy = sdGetTimeout(&SDRV, TIME_IMMEDIATE);
    } while (dummy != Q_TIMEOUT);

    (void)dummy;
}
