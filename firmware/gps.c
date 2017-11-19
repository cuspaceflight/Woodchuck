/**
//  * Woodchuck by CU Spaceflight
//  *
//  * This file is part of the Woodchuck project by Cambridge University
//  * Spaceflight.
//  *
//  * Based on the Martlet 3 gps driver by Adam Greig.
//  *
//  * Adam Greig  2017
//  * Greg Brooks 2017
//  */


#include <string.h>
#include "gps.h"
#include "led.h"
#include "radio.h"
#include "error.h"


// Choose which USART module to use
#define SDRV       SD2  // Also see mcuconf.h


/* Config Flag */
static bool gps_configured = false;

/* Function Prototypes */
static uint16_t gps_fletcher_8(uint16_t chk, uint8_t *buf, uint8_t n);
static void gps_checksum(uint8_t *buf);
static bool gps_transmit(uint8_t *buf);
// static enum ublox_result ublox_state_machine(uint8_t b);
static state_return_t ublox_state_machine(uint8_t b);
static bool gps_configure(void);
static bool gps_tx_ack(uint8_t *buf);
void _serial_begin(uint32_t baud);
uint32_t _serial_get_baud(void);

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

/* UBX Decoding State Machine States */
typedef enum {
    STATE_IDLE = 0, STATE_SYNC1, STATE_SYNC2,
    STATE_CLASS, STATE_ID, STATE_L1, STATE_L2,
    STATE_PAYLOAD, STATE_CK_A, NUM_STATES
} ubx_state;


/* Run the Fletcher-8 checksum, initialised to chk, over n bytes of buf */
static uint16_t gps_fletcher_8(uint16_t chk, uint8_t *buf, uint8_t n)
{
    int i;
    uint8_t ck_a = chk & 0xff, ck_b = chk>>8;

    /* Run Fletcher-8 algorithm */
    for(i=0; i<n; i++) {
        ck_a += buf[i];
        ck_b += ck_a;
    }

    return (ck_b<<8) | (ck_a);
}


/* Computes the Fletcher-8 checksum over buf, using its length fields
 * to determine how much to read, returning the new checksum.
 */
static void gps_checksum(uint8_t *buf)
{
    uint16_t plen;

    /* Check SYNC bytes are correct */
    if(buf[0] != UBX_SYNC1 && buf[1] != UBX_SYNC2)
        return;

    /* Extract payload length */
    plen = ((uint16_t*)buf)[2];

    uint16_t ck = gps_fletcher_8(0, &buf[2], plen+4);

    /* Write new checksum to the buffer */
    buf[plen+6] = ck;
    buf[plen+7] = ck >> 8;
}


/* Transmit a UBX message over the Serial.
 * Message length is determined from the UBX length field.
 * Checksum is added automatically.
 */
static bool gps_transmit(uint8_t *buf)
{
    size_t n, nwritten;
    systime_t timeout;

    /* Add checksum to outgoing message */
    gps_checksum(buf);

    /* Determine length and thus suitable timeout in systicks (ms) */
    n = 8 + ((uint16_t*)buf)[2];
    timeout = MS2ST(n*2);

    /* Transmit message */
    nwritten = sdWriteTimeout(&SDRV, buf, n, timeout);
    return nwritten == n;
}


/* Transmits a UBX message and blocks
 * until a ACK/NAK is recieved in response
 */
static bool gps_tx_ack(uint8_t *buf)
{
    /* Clear the read buffer */
    while(sdGetTimeout(&SDRV, TIME_IMMEDIATE) != Q_TIMEOUT);

    if(!gps_transmit(buf)) {
        return false;
    }

    chThdSleepMilliseconds(300);

    state_return_t r;
    do {
        r = ublox_state_machine(sdGet(&SDRV));
    } while( (r.result != UBLOX_ACK) && (r.result != UBLOX_NAK) );

    if(r.result == UBLOX_NAK){
        set_error(ERROR_GPS);
        return false;
    }
    return true;
}


/* Run new byte b through the UBX decoding state machine. Note that this
 * function preserves static state and processes new messages as appropriate
 * once received.
 */
// static enum ublox_result ublox_state_machine(uint8_t b, )
static state_return_t ublox_state_machine(uint8_t b)
{
    static ubx_state state = STATE_IDLE;

    static uint8_t class, id;
    static uint16_t length;
    static uint16_t length_remaining;
    static uint8_t payload[128];
    static uint8_t ck_a, ck_b;
    static uint16_t ck;

    state_return_t return_state;
    return_state.no_pckt = 1;  // Default behaviour is to not return a packet

    switch(state) {
        case STATE_IDLE:
            if(b == UBX_SYNC1)
                state = STATE_SYNC1;
            break;

        case STATE_SYNC1:
            if(b == UBX_SYNC2)
                state = STATE_SYNC2;
            else
                state = STATE_IDLE;
            break;

        case STATE_SYNC2:
            class = b;
            state = STATE_CLASS;
            break;

        case STATE_CLASS:
            id = b;
            state = STATE_ID;
            break;

        case STATE_ID:
            length = (uint16_t)b;
            state = STATE_L1;
            break;

        case STATE_L1:
            length |= (uint16_t)b << 8;
            if(length >= 128) {
                set_error(ERROR_GPS);
                state = STATE_IDLE;
                // return UBLOX_RXLEN_TOO_LONG;
                return_state.result = UBLOX_RXLEN_TOO_LONG;
                return return_state;

            }
            length_remaining = length;
            state = STATE_PAYLOAD;
            break;

        case STATE_PAYLOAD:
            if(length_remaining) {
                payload[length - length_remaining--] = b;
            } else {
                ck_a = b;
                state = STATE_CK_A;
            }
            break;

        case STATE_CK_A:
            ck_b = b;
            state = STATE_IDLE;

            /* Verify checksum */
            ck = gps_fletcher_8(0, &class, 1);
            ck = gps_fletcher_8(ck, &id, 1);
            ck = gps_fletcher_8(ck, (uint8_t*)&length, 2);
            ck = gps_fletcher_8(ck, payload, length);
            if(ck_a != (ck&0xFF) || ck_b != (ck>>8)) {
                set_error(ERROR_GPS);
                state=STATE_IDLE;
                // return UBLOX_BAD_CHECKSUM;
                return_state.result = UBLOX_BAD_CHECKSUM;
                return return_state;
            }

            /* Handle Payload */
            switch(class) {

                /* Acknowledge */
                case UBX_ACK:
                    if(id == UBX_ACK_NAK) {
                        /* NAK */
                        set_error(ERROR_GPS);
                        // return UBLOX_NAK;
                        return_state.result = UBLOX_NAK;
                        return return_state;
                    } else if(id == UBX_ACK_ACK) {
                        /* ACK - Do Nothing */
                        // return UBLOX_ACK;
                        return_state.result = UBLOX_ACK;
                        return return_state;
                    } else {
                        set_error(ERROR_GPS);
                        // return UBLOX_UNHANDLED;
                        return_state.result = UBLOX_UNHANDLED;
                        return return_state;
                    }
                    break;

                /* Nav Payload */
                case UBX_NAV:
                    if(id == UBX_NAV_PVT) {

                        /* Extract NAV-PVT Payload */
                        memcpy(&return_state.pvt, payload, length);
                        //log_pvt(&pvt_latest);

                        // /* Check for FIX */
	                    // if(pvt_latest.fix_type != 3) {
                        //     set_error(ERROR_GPS);
	                    // }

                        // return UBLOX_NAV_PVT;
                        return_state.result = UBLOX_NAV_PVT;
                        return return_state;

                    } else if(id == UBX_NAV_POSECEF){

                        /* Extract NAV-POSECEF Payload */
                        memcpy(&return_state.posecef, payload, length);

                        // return UBLOX_NAV_POSECEF;
                        return_state.result = UBLOX_NAV_POSECEF;
                        return return_state;

                    } else {
                        set_error(ERROR_GPS);
                        // return UBLOX_UNHANDLED;
                        return_state.result = UBLOX_UNHANDLED;
                        return return_state;
                    }
                    break;

                /* Config Payload */
                case UBX_CFG:
                    if(id == UBX_CFG_NAV5) {

                        /* NAV5 */
                        memcpy(return_state.cfg_nav5.payload, payload, length);
                        if(return_state.cfg_nav5.dyn_model != 6) {
                            set_error(ERROR_GPS);
                        }
                        // return UBLOX_CFG_NAV5;
                        return_state.result = UBLOX_CFG_NAV5;
                        return return_state;
                    } else {
                        set_error(ERROR_GPS);
                        // return UBLOX_UNHANDLED;
                        return_state.result = UBLOX_UNHANDLED;
                        return return_state;
                    }
                    break;

                /* Unhandled */
                default:
                    // return UBLOX_UNHANDLED;
                    return_state.result = UBLOX_UNHANDLED;
                    return return_state;
            }
            break;

        default:
            state = STATE_IDLE;

            set_error(ERROR_GPS);
            // return UBLOX_ERROR;
            return_state.result = UBLOX_ERROR;
            return return_state;
    }
    // return UBLOX_WAIT;
    return_state.result = UBLOX_WAIT;
    return return_state;
}


/* Configure uBlox GPS */
static bool gps_configure() {

    gps_configured = true;

    ubx_cfg_prt_t prt;
    ubx_cfg_nav5_t nav5;
    ubx_cfg_msg_t msg;
    ubx_cfg_msg_t msg2;
    ubx_cfg_rate_t rate;
    ubx_cfg_sbas_t sbas;
    ubx_cfg_gnss_t gnss;
    ubx_cfg_tp5_t tp5_1;
    ubx_cfg_tp5_t tp5_2;

    /* Disable NMEA on UART */
    prt.sync1 = UBX_SYNC1;
    prt.sync2 = UBX_SYNC2;
    prt.class = UBX_CFG;
    prt.id = UBX_CFG_PRT;
    prt.length = sizeof(prt.payload);
    /* Program UART1 */
    prt.port_id = 1;
    /* Don't use TXReady GPIO */
    prt.tx_ready = 0;
    /* 8 bits, no polarity, 1 stop bit */
    prt.mode = (1<<4) | (3<<6) | (4<<9);
    /* 9600 baud */
    prt.baud_rate = 9600;
    /* only receive UBX protocol */
    prt.in_proto_mask = (1<<0);
    /* only send UBX protocol */
    prt.out_proto_mask = (1<<0);
    /* no weird timeout */
    prt.flags = 0;
    /* must be 0 */
    prt.reserved5 = 0;
    gps_configured &= gps_transmit((uint8_t*)&prt);
    if(!gps_configured) return false;

    /* Wait for it to stop barfing NMEA */
    chThdSleepMilliseconds(300);

    /* Clear the read buffer */
    while(sdGetTimeout(&SDRV, TIME_IMMEDIATE) != Q_TIMEOUT);


    /* Set to airborne <1g mode */
    nav5.sync1 = UBX_SYNC1;
    nav5.sync2 = UBX_SYNC2;
    nav5.class = UBX_CFG;
    nav5.id = UBX_CFG_NAV5;
    nav5.length = sizeof(nav5.payload);

    nav5.mask = 1 ;
    nav5.dyn_model = 6;

    gps_configured &= gps_tx_ack((uint8_t*)&nav5);
    if(!gps_configured) return false;


    /* Enable NAV PVT messages */
    msg.sync1 = UBX_SYNC1;
    msg.sync2 = UBX_SYNC2;
    msg.class = UBX_CFG;
    msg.id = UBX_CFG_MSG;
    msg.length = sizeof(msg.payload);

    msg.msg_class = UBX_NAV;
    msg.msg_id    = UBX_NAV_PVT;
    msg.rate      = 0;

    gps_configured &= gps_tx_ack((uint8_t*)&msg);
    if(!gps_configured) return false;


    // /* Enable NAV POSECEF messages */
    // if (nav_posecef){
    //     msg2.sync1 = UBX_SYNC1;
    //     msg2.sync2 = UBX_SYNC2;
    //     msg2.class = UBX_CFG;
    //     msg2.id = UBX_CFG_MSG;
    //     msg2.length = sizeof(msg2.payload);
    //
    //     msg2.msg_class = UBX_NAV;
    //     msg2.msg_id    = UBX_NAV_POSECEF;
    //     msg2.rate      = 1;
    //
    //     gps_configured &= gps_tx_ack((uint8_t*)&msg2);
    //     if(!gps_configured) return false;
    // }

    return gps_configured;
}

/* Configure uBlox GPS - public function */
void gps_init(){

    _serial_begin(9600);

    while(!gps_configure()) {
        set_error(ERROR_GPS);
        chThdSleepMilliseconds(500);
    }

    return;
}

bool gps_get_position(int32_t* lat, int32_t* lon, int32_t* alt){
    ublox_pvt_t pvt_message;
    bool worked = gps_poll_pvt(&pvt_message);
    if (worked){
        // Success
        *lat = pvt_message.lat;
        *lon = pvt_message.lon;
        *alt = pvt_message.height;
        return true;
    }
    return false;
}

bool gps_get_time(uint8_t* hour, uint8_t* minute, uint8_t* second){
    ublox_pvt_t pvt_message;
    bool worked = gps_poll_pvt(&pvt_message);
    if (worked){
        // Success
        *hour = pvt_message.hour;
        *minute = pvt_message.minute;
        *second = pvt_message.second;
        return true;
    }
    return false;
}

bool gps_get_pos_time(int32_t* lat, int32_t* lon, int32_t* alt, uint8_t* hour, uint8_t* minute, uint8_t* second){
    ublox_pvt_t pvt_message;
    bool worked = gps_poll_pvt(&pvt_message);
    if (worked){
        // Success
        *lat = pvt_message.lat;
        *lon = pvt_message.lon;
        *alt = pvt_message.height;
        *hour = pvt_message.hour;
        *minute = pvt_message.minute;
        *second = pvt_message.second;
        return true;
    }
    return false;
}

bool gps_poll_pvt(ublox_pvt_t *pvt_message){
    if(gps_configured) {
        // Send pvt request
        ubx_poll_t poll;
        poll.sync1 = UBX_SYNC1;
        poll.sync2 = UBX_SYNC2;
        poll.class = UBX_NAV;
        poll.id = UBX_NAV_PVT;
        poll.length = 0;

        /* Clear the read buffer */
        while(sdGetTimeout(&SDRV, TIME_IMMEDIATE) != Q_TIMEOUT);

        gps_transmit((uint8_t*)&poll);
        chThdSleepMilliseconds(300);

        state_return_t res;
        do{
            do{
                msg_t byte_in = sdGetTimeout(&SDRV, MS2ST(50));
                if(byte_in == Q_TIMEOUT){
                    set_error(ERROR_GPS);
                    return false;
                }
                res = ublox_state_machine((uint8_t)byte_in);
            } while (res.result == UBLOX_WAIT);
        } while (res.result != UBLOX_NAV_PVT);
        *pvt_message = res.pvt;
        //memcpy(*pvt_message, &res.pvt, res.pvt.length + 8);
        return true;
    }
    return false;
}
