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

TODO:
- Implement polling
- Implement other functions from the old gps driver, or remove them if unnecessary

#include <string.h>
#include "gps.h"
#include "led.h"
#include "radio.h"
#include "error.h"
#include "ubx.h"

// Choose which USART module to use
#define SDRV       SD2  // Also see mcuconf.h

// MUTEX_DECL(pvt_stamp_mutex);
// MUTEX_DECL(pos_pkt_mutex);

/* Serial Setup */
//static SerialDriver* gps_seriald;
// 8N1, no flow control
// static SerialConfig serial_cfg = {
//     .speed = 9600,
//     .cr1 = 0,
//     .cr2 = 0,
//     .cr3 = 0,
// };

/* Config Flag */
static bool gps_configured = false;

/* Function Prototypes */
static uint16_t gps_fletcher_8(uint16_t chk, uint8_t *buf, uint8_t n);
static void gps_checksum(uint8_t *buf);
static bool gps_transmit(uint8_t *buf);
static enum ublox_result ublox_state_machine(uint8_t b);
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

// /* Global Timestamped iTOW */
// pvt_capture stamped_pvt;
//
// /* PVT Stamp Mutex */
// mutex_t pvt_stamp_mutex;
//
// /* Global Position Packet */
// position_packet pos_pkt;
//
// /* Position Packet Mutex */
// mutex_t pos_pkt_mutex;

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
    if(!gps_transmit(buf)) {
        return false;
    }

    enum ublox_result r;
    do {
        r = ublox_state_machine(sdGet(&SDRV));
    } while( (r != UBLOX_ACK) && (r != UBLOX_NAK) );

    if(r == UBLOX_NAK){
        set_error(ERROR_GPS);
        return false;
    }
    return true;
}


/* Run new byte b through the UBX decoding state machine. Note that this
 * function preserves static state and processes new messages as appropriate
 * once received.
 */
static enum ublox_result ublox_state_machine(uint8_t b)
{
    static ubx_state state = STATE_IDLE;

    static uint8_t class, id;
    static uint16_t length;
    static uint16_t length_remaining;
    static uint8_t payload[128];
    static uint8_t ck_a, ck_b;
    static uint16_t ck;

    static ubx_cfg_nav5_t cfg_nav5;
    static ublox_posecef_t posecef;
    static ublox_pvt_t pvt_latest;


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
                return UBLOX_RXLEN_TOO_LONG;
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
                return UBLOX_BAD_CHECKSUM;
            }

            /* Handle Payload */
            switch(class) {

                /* Acknowledge */
                case UBX_ACK:
                    if(id == UBX_ACK_NAK) {
                        /* NAK */
                        set_error(ERROR_GPS);
                        return UBLOX_NAK;
                    } else if(id == UBX_ACK_ACK) {
                        /* ACK - Do Nothing */
                        return UBLOX_ACK;
                    } else {
                        set_error(ERROR_GPS);
                        return UBLOX_UNHANDLED;
                    }
                    break;

                /* Nav Payload */
                case UBX_NAV:
                    if(id == UBX_NAV_PVT) {

                        /* Extract NAV-PVT Payload */
                        memcpy(&pvt_latest, payload, length);
                        //log_pvt(&pvt_latest);

                        // /* Check for FIX */
	                    // if(pvt_latest.fix_type != 3) {
                        //     set_error(ERROR_GPS);
	                    // }

                        return UBLOX_NAV_PVT;

                    } else if(id == UBX_NAV_POSECEF){

                        /* Extract NAV-POSECEF Payload */
                        memcpy(&posecef, payload, length);

                        return UBLOX_NAV_POSECEF;

                    } else {
                        set_error(ERROR_GPS);
                        return UBLOX_UNHANDLED;
                    }
                    break;

                /* Config Payload */
                case UBX_CFG:
                    if(id == UBX_CFG_NAV5) {

                        /* NAV5 */
                        memcpy(cfg_nav5.payload, payload, length);
                        if(cfg_nav5.dyn_model != 6) {
                            set_error(ERROR_GPS);
                        }
                        return UBLOX_CFG_NAV5;
                    } else {
                        set_error(ERROR_GPS);
                        return UBLOX_UNHANDLED;
                    }
                    break;

                /* Unhandled */
                default:
                    return UBLOX_UNHANDLED;
            }
            break;

        default:
            state = STATE_IDLE;

            set_error(ERROR_GPS);
            return UBLOX_ERROR;
    }
    return UBLOX_WAIT;
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

    // /* Disable non GPS systems */
    // gnss.sync1 = UBX_SYNC1;
    // gnss.sync2 = UBX_SYNC2;
    // gnss.class = UBX_CFG;
    // gnss.id = UBX_CFG_GNSS;
    // gnss.length = sizeof(gnss.payload);
    //
    // gnss.msg_ver = 0;
    // gnss.num_trk_ch_hw = 32;
    // gnss.num_trk_ch_use = 32;
    // gnss.num_config_blocks = 5;
    //
    // /* Enable GPS, use all-1 channels */
    // gnss.gps_gnss_id = 0;
    // gnss.gps_res_trk_ch = 31;
    // gnss.gps_max_trk_ch = 31;
    // gnss.gps_flags = 1+(1<<16);
    //
    // /* Enable QZSS as per protocol spec */
    // gnss.qzss_gnss_id = 5;
    // gnss.qzss_res_trk_ch = 1;
    // gnss.qzss_max_trk_ch = 1;
    // gnss.qzss_flags = 1+(1<<16);
    //
    // /* Leave all other GNSS systems disabled */
    // gnss.sbas_gnss_id = 1;
    // gnss.beidou_gnss_id = 3;
    // gnss.glonass_gnss_id = 6;
    // gps_configured &= gps_tx_ack((uint8_t*)&gnss);
    // if(!gps_configured) return false;
    //
    // /* Wait for reset */
    // chThdSleepMilliseconds(500);
    //
    //
    // /* Re-disable NMEA Output */
    // gps_configured &= gps_transmit((uint8_t*)&prt);
    // if(!gps_configured) return false;
    //
    // /* Wait for it to stop barfing NMEA */
    // chThdSleepMilliseconds(300);
    //
    // /* Clear the read buffer */
    // while(sdGetTimeout(gps_seriald, TIME_IMMEDIATE) != Q_TIMEOUT);


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

    // /* Set solution rate to 1Hz */
    // rate.sync1 = UBX_SYNC1;
    // rate.sync2 = UBX_SYNC2;
    // rate.class = UBX_CFG;
    // rate.id = UBX_CFG_RATE;
    // rate.length = sizeof(rate.payload);
    //
    // rate.meas_rate = 1000;
    // rate.nav_rate = 1;
    // rate.time_ref = 0;  // UTC
    //
    // gps_configured &= gps_tx_ack((uint8_t*)&rate);
    // if(!gps_configured) return false;


    // /* Disable sbas */
    // sbas.sync1 = UBX_SYNC1;
    // sbas.sync2 = UBX_SYNC2;
    // sbas.class = UBX_CFG;
    // sbas.id = UBX_CFG_SBAS;
    // sbas.length = sizeof(sbas.payload);
    // sbas.mode = 0;
    //
    // gps_configured &= gps_tx_ack((uint8_t*)&sbas);
    // if(!gps_configured) return false;
    //
    //
    // /* Set up 1MHz timepulse on TIMEPULSE pin*/
    // tp5_1.sync1 = UBX_SYNC1;
    // tp5_1.sync2 = UBX_SYNC2;
    // tp5_1.class = UBX_CFG;
    // tp5_1.id = UBX_CFG_TP5;
    // tp5_1.length = sizeof(tp5_1.payload);
    //
    // tp5_1.tp_idx =           0;                 // TIMEPULSE
    // tp5_1.version =          0;
    // tp5_1.ant_cable_delay =  0;
    // tp5_1.freq_period =      1000000;           // 1MHz
    // tp5_1.pulse_len_ratio =  0xffffffff >> 1;   // (2^32/2)/2^32 = 50% duty cycle
    // tp5_1.freq_period_lock = 1000000;
    // tp5_1.pulse_len_ratio_lock = 0xffffffff >> 1;
    // tp5_1.user_config_delay = 0;
    // tp5_1.flags = (
    //     UBX_CFG_TP5_FLAGS_ACTIVE                    |
    //     UBX_CFG_TP5_FLAGS_LOCK_GNSS_FREQ            |
    //     UBX_CFG_TP5_FLAGS_IS_FREQ                   |
    //     UBX_CFG_TP5_FLAGS_ALIGN_TO_TOW              |
    //     UBX_CFG_TP5_FLAGS_POLARITY                  |
    //     UBX_CFG_TP5_FLAGS_GRID_UTC_GNSS_UTC);
    //
    // gps_configured &= gps_tx_ack((uint8_t*)&tp5_1);
    // if(!gps_configured) return false;
    //
    //
    // /* Set up 1Hz pulse on SAFEBOOT pin */
    // tp5_2.sync1 = UBX_SYNC1;
    // tp5_2.sync2 = UBX_SYNC2;
    // tp5_2.class = UBX_CFG;
    // tp5_2.id = UBX_CFG_TP5;
    // tp5_2.length = sizeof(tp5_2.payload);
    //
    // tp5_2.tp_idx               = 1;     // Safeboot pin
    // tp5_2.version              = 0;
    // tp5_2.ant_cable_delay      = 0;
    // tp5_2.freq_period          = 1;
    // tp5_2.pulse_len_ratio      = 10000; // us
    // tp5_2.freq_period_lock     = 1;
    // tp5_2.pulse_len_ratio_lock = 10000;
    //
    // if(rising_edge) {
    //
    //     /* Rising edge on top of second */
    //     tp5_2.flags = (
    //         UBX_CFG_TP5_FLAGS_ACTIVE                    |
    //         UBX_CFG_TP5_FLAGS_LOCK_GNSS_FREQ            |
    //         UBX_CFG_TP5_FLAGS_LOCKED_OTHER_SET          |
    //         UBX_CFG_TP5_FLAGS_IS_FREQ                   |
    //         UBX_CFG_TP5_FLAGS_IS_LENGTH                 |
    //         UBX_CFG_TP5_FLAGS_ALIGN_TO_TOW              |
    //         UBX_CFG_TP5_FLAGS_POLARITY                  |
    //         UBX_CFG_TP5_FLAGS_GRID_UTC_GNSS_UTC);
    // } else {
    //
    //     /* Falling edge on top of second */
    //     tp5_2.flags = (
    //         UBX_CFG_TP5_FLAGS_ACTIVE                    |
    //         UBX_CFG_TP5_FLAGS_LOCK_GNSS_FREQ            |
    //         UBX_CFG_TP5_FLAGS_LOCKED_OTHER_SET          |
    //         UBX_CFG_TP5_FLAGS_IS_FREQ                   |
    //         UBX_CFG_TP5_FLAGS_IS_LENGTH                 |
    //         UBX_CFG_TP5_FLAGS_ALIGN_TO_TOW              |
    //         UBX_CFG_TP5_FLAGS_GRID_UTC_GNSS_UTC);
    // }
    //
    // gps_configured &= gps_tx_ack((uint8_t*)&tp5_2);
    // if(!gps_configured) return false;


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

    // /* Reset uBlox */
    // palClearLine(LINE_GPS_RST);
    // chThdSleepMilliseconds(300);
    // palSetLine(LINE_GPS_RST);

    // /* Wait for GPS to restart */
    // chThdSleepMilliseconds(500);

    _serial_begin(9600);

    while(!gps_configure()) {
        set_error(ERROR_GPS);
        chThdSleepMilliseconds(1000);
    }

    return;
}

void gps_get_position(int32_t* lat, int32_t* lon, int32_t* alt){
    if(gps_configured) {
        enum ublox_result res;
        do{
            res = ublox_state_machine(sdGet(&SDRV));
        } while (res == UBLOX_WAIT)
    }
}
void gps_get_time(uint8_t* hour, uint8_t* minute, uint8_t* second);


// /* Thread to Run State Machine */
// static THD_WORKING_AREA(gps_thd_wa, 512);
// static THD_FUNCTION(gps_thd, arg) {
//
//     (void)arg;
//     chRegSetThreadName("GPS");
//
//     while(true) {
//
//         if(gps_configured) {
//
//             ublox_state_machine(sdGet(&SDRV));
//         }
//         else {
//
//             set_error(ERROR_GPS);
//
//             break;
//         }
//     }
// }
//
//
// /* Init GPS Thread */
// void gps_thd_init(void) {
//
//     chThdCreateStatic(gps_thd_wa, sizeof(gps_thd_wa), NORMALPRIO, gps_thd, NULL);
// }



// /**
//  * Woodchuck by CU Spaceflight
//  *
//  * This file is part of the Woodchuck project by Cambridge University
//  * Spaceflight.
//  *
//  * Based on the gps driver from the JOEY-M project by Cambridge University
//  * Spaceflight.
//  *
//  * Jon Sowman  2012
//  * Jamie Wood  2016
//  * Greg Brooks 2017
//  */
//
// #include "gps.h"
// #include "led.h"
// #include "radio.h"
// #include "error.h"
//
// // Choose which USART module to use
// #define SDRV       SD2  // Also see mcuconf.h
//
// // Header sync bytes - no touching!
// #define UBX_SYNC_1  (0xB5)
// #define UBX_SYNC_2  (0x62)
//
// // Class and ID bytes - no touching!
// #define UBX_NAV_CLASS       (0x01)
// #define UBX_ACK_CLASS       (0x05)
// #define UBX_CFG_CLASS       (0x06)
//
// #define UBX_NAV_POSLLH_ID   (0x02)
// #define UBX_NAV_SOL_ID      (0x06)
// #define UBX_NAV_TIMEUTC_ID  (0x21)
// #define UBX_ACK_ACK_ID      (0x01)
// #define UBX_ACK_NAK_ID      (0x00)
// #define UBX_CFG_PRT_ID      (0x00)
// #define UBX_CFG_MSG_ID      (0x01)
// #define UBX_CFG_NAV5_ID     (0x24)
//
// /**
//  * Global serial config structure for GPS interface
//  * Best not to touch this directly, use getter/setter functions instead
//  * _serial_begin and _serial_get_baud
//  * 8N1, no flow control
//  */
// static SerialConfig uartGPS =
// {
//     .speed = 9600,
//     .cr1 = 0,
//     .cr2 = 0,
//     .cr3 = 0,
// };
//
// // Setter function for serial port
// void _serial_begin(uint32_t baud){
//     uartGPS.speed = baud;
//     sdStop(&SDRV);
//     sdStart(&SDRV, &uartGPS);
// }
//
// // Getter function to get current baud rate
// uint32_t _serial_get_baud(void){
//     return uartGPS.speed;
// }
//
// /**
//  * Set up USART for communication with the uBlox GPS at 38400 baud.
//  *
//  * We have to start at 9600, then tell the uBlox to change up to 38400 baud.
//  */
// void gps_init(void)
// {
//     // 9600 baud rate
//     _serial_begin(9600);
//
//     _gps_set_baud(9600);
//
//
//     // Set the GPS into the correct mode (<1g airborne)
//     gps_set_mode();
// }
//
// /**
//  * Poll the GPS for a position message then extract the useful
//  * information from it, using a NAV-POSLLH message.
//  */
// void gps_get_position(int32_t* lat, int32_t* lon, int32_t* alt)
// {
//     // Request a NAV-POSLLH message from the GPS
//     uint8_t request[8] = {UBX_SYNC_1, UBX_SYNC_2, UBX_NAV_CLASS,
//         UBX_NAV_POSLLH_ID, 0x00, 0x00, 0x03, 0x0A};
//     _gps_send_msg(request);
//
//     uint8_t buf[36];
//     //uint8_t i = 0;
//     //for(i = 0; i < 36; i++)
//     //    buf[i] = _gps_get_byte();
//     _gps_recv_msg(buf, 36);
//
//     // Verify the sync and header bits
//     if( buf[0] != UBX_SYNC_1 || buf[1] != UBX_SYNC_2 )
//         set_error(ERROR_GPS);
//     if( buf[2] != UBX_NAV_CLASS || buf[3] != UBX_NAV_POSLLH_ID )
//         set_error(ERROR_GPS);
//
//     // 4 bytes of longitude (1e-7)
//     *lon = (int32_t)buf[10] | (int32_t)buf[11] << 8 |
//         (int32_t)buf[12] << 16 | (int32_t)buf[13] << 24;
//
//     // 4 bytes of latitude (1e-7)
//     *lat = (int32_t)buf[14] | (int32_t)buf[15] << 8 |
//         (int32_t)buf[16] << 16 | (int32_t)buf[17] << 24;
//
//     // 4 bytes of altitude above MSL (mm)
//     *alt = (int32_t)buf[22] | (int32_t)buf[23] << 8 |
//         (int32_t)buf[24] << 16 | (int32_t)buf[25] << 24;
//
//     if( !_gps_verify_checksum(&buf[2], 32) ) set_error(ERROR_GPS);
// }
//
// /**
//  * Get the hour, minute and second from the GPS using the NAV-TIMEUTC
//  * message.
//  */
//  void gps_get_time(uint8_t* hour, uint8_t* minute, uint8_t* second)
// {
//     // Send a NAV-TIMEUTC message to the receiver
//     uint8_t request[8] = {UBX_SYNC_1, UBX_SYNC_2, UBX_NAV_CLASS,
//         UBX_NAV_TIMEUTC_ID, 0x00, 0x00, 0x22, 0x67};
//     _gps_send_msg(request);
//
//     // Get the message back from the GPS
//     uint8_t buf[28];
//     //uint8_t i = 0;
//     //for(i = 0; i < 28; i++)
//     //    buf[i] = _gps_get_byte();
//     _gps_recv_msg(buf, 28);
//
//     // Verify the sync and header bits
//     if( buf[0] != UBX_SYNC_1 || buf[1] != UBX_SYNC_2 )
//         set_error(ERROR_GPS);
//     if( buf[2] != UBX_NAV_CLASS || buf[3] != UBX_NAV_TIMEUTC_ID )
//         set_error(ERROR_GPS);
//
//     *hour = buf[22];
//     *minute = buf[23];
//     *second = buf[24];
//
//     if( !_gps_verify_checksum(&buf[2], 24) ) set_error(ERROR_GPS);
// }
//
// /**
//  * Check the navigation status to determine the quality of the
//  * fix currently held by the receiver with a NAV-SOL message.
//  */
// void gps_check_lock(uint8_t* lock, uint8_t* numsats)
// {
//     // Construct the request to the GPS
//     uint8_t request[8] = {UBX_SYNC_1, UBX_SYNC_2, UBX_NAV_CLASS, UBX_NAV_SOL_ID,
//         0x00, 0x00, 0x07, 0x16};
//     _gps_send_msg(request);
//
//     // Get the message back from the GPS
//     uint8_t buf[60];
//     //uint8_t i;
//     //for(i = 0; i < 60; i++)
//     //   buf[i] = _gps_get_byte();
//     _gps_recv_msg(buf, 60);
//
//     // Verify the sync and header bits
//     if( buf[0] != UBX_SYNC_1 || buf[1] != UBX_SYNC_2 )
//         set_error(ERROR_GPS);
//     if( buf[2] != UBX_NAV_CLASS || buf[3] != UBX_NAV_SOL_ID )
//         set_error(ERROR_GPS);
//
//     // Check 60 bytes minus SYNC and CHECKSUM (4 bytes)
//     if( !_gps_verify_checksum(&buf[2], 56) ) set_error(ERROR_GPS);
//
//     // Return the value if GPSfixOK is set in 'flags'
//     if( buf[17] & 0x01 )
//         *lock = buf[16];
//     else
//         *lock = 0;
//
//     *numsats = buf[53];
// }
//
// /**
//  * Set the GPS to <1g airborne navigation mode, using a CFG-NAV5 message.
//  */
// void gps_set_mode(void)
// {
//     /* Parameters:
//      * mask:        0x0001 (only apply dynModel setting)
//      * dynModel:    0x06 (airborne <1g)
//      * blank x33:   0x00 (not needed due to mask)
//      *
//      * Checksum bytes are pre-computed
//      */
//     uint8_t request[44] = {UBX_SYNC_1, UBX_SYNC_2, UBX_CFG_CLASS,
//         UBX_CFG_NAV5_ID, 36, 0x00, 0x00, 0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00,
//         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//         0x00, 0x00, 0x00, 0x00, 0x55, 0x8f};
//     _gps_send_msg(request);
//     chThdSleepMilliseconds(300);
//
//     // Read the response from the GPS
//     uint8_t buf[10];
//     //uint8_t i = 0;
//     //for(i = 0; i < 10; i++)
//     //    buf[i] = _gps_get_byte();
//     uint8_t result = _gps_recv_ack(buf);
//
//     if (result == 2) led_set(LED_RGB, LED_BLINKING);
//     else if (result == UBX_ACK_ACK_ID) led_set(LED_RB, LED_ON);
//     chThdSleepSeconds(9999);
//
//     // // Verify sync and header bytes
//     // if( buf[0] != UBX_SYNC_1 || buf[1] != UBX_SYNC_2 )
//     //     set_error(ERROR_GPS);
//     // if( buf[2] != UBX_ACK_CLASS)
//     //     set_error(ERROR_GPS);
//     //
//     // // Check message checksum
//     // if( !_gps_verify_checksum(&buf[2], 6) ) set_error(ERROR_GPS);
//     //
//     // // Check if we received an ACK or NACK
//     // if(buf[3] != UBX_ACK_ACK_ID){ // If not an ACK
//     //     set_error(ERROR_GPS);
//     // }
// }
//
// /**
//  * Verify that the uBlox 6 GPS receiver is set to the <1g airborne
//  * navigaion mode, using a CFG-NAV5 message.
//  */
// uint8_t gps_check_nav(void)
// {
//     uint8_t request[8] = {UBX_SYNC_1, UBX_SYNC_2, UBX_CFG_CLASS, UBX_CFG_NAV5_ID, 0x00, 0x00,
//         0x2A, 0x84};
//     _gps_send_msg(request);
//
//     // Get the message back from the GPS
//     uint8_t buf[44];
//     //uint8_t i = 0;
//     //for(i = 0; i < 44; i++)
//     //    buf[i] = _gps_get_byte();
//     _gps_recv_msg(buf, 44);
//
//     // Verify sync and header bytes
//     if( buf[0] != UBX_SYNC_1 || buf[1] != UBX_SYNC_2 )
//         set_error(ERROR_GPS);
//     if( buf[2] != UBX_CFG_CLASS || buf[3] != UBX_CFG_NAV5_ID )
//         set_error(ERROR_GPS);
//
//     // Check 40 bytes of message checksum
//     if( !_gps_verify_checksum(&buf[2], 40) ) set_error(ERROR_GPS);
//
//     // Clock in and verify the ACK/NACK
//     uint8_t ack[10];
//     //for(i = 0; i < 10; i++)
//     //    ack[i] = _gps_get_byte();
//     _gps_recv_ack(ack);
//
//     // If we got a NACK, then return 0xFF
//     if( ack[3] == 0x00 ) return 0xFF;
//
//     // Return the navigation mode and let the caller analyse it
//     return buf[8];
// }
//
// /**
//  * Set the baudrate of the GPS using a CFG-PRT message.
//  * We also update our own baudrate to match.
//  */
// void _gps_set_baud(uint32_t baudrate)
// {
//     // Backup the old baudrate incase something goes wrong
//     //uint32_t oldBRR = _serial_get_baud();
//
//     /* First stop NMEA from clogging the buffer
//      * Parameters:
//      * portID:      0x01 (UART)
//      * reserved:    0x00
//      * txReady:     0x0000 (disable txReady pin)
//      * mode:        0b00001000 11000000 = 0x08 0xC0 (1 stop bit, no parity, 8 data bits)
//      * baudrate:    <baudrate> (little endian)
//      * inProtoMask: 0x0001 (UBX input only)
//      * outProtoMask:0x0000 (stop transmitting for now)
//      * flags:       0x0000 (disable extended timeout)
//      * reserved x2: 0x0000
//      */
//     uint8_t cka = 0;
//     uint8_t ckb = 0;
//     uint8_t pre_request[28] = {UBX_SYNC_1, UBX_SYNC_2, UBX_CFG_CLASS,
//         UBX_CFG_PRT_ID, 20, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0xC0,
//         baudrate & 0xff, (baudrate >> 8) & 0xff, (baudrate >> 16) & 0xff,
//         (baudrate >> 24) & 0xff, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
//         0x00, cka, ckb};
//
//     // Compute the checksum and fill it out
//     _gps_ubx_checksum(&pre_request[2], 24, &cka, &ckb);
//     pre_request[26] = cka;
//     pre_request[27] = ckb;
//
//     // Transmit the request
//     _gps_send_msg(pre_request);
//
//     chThdSleepMilliseconds(500);
//     _gps_flush_buffer();
//
//     // // TESTING
//     // while(1){
//     //     _gps_send_msg(pre_request);
//     // }
//
//     // /* Now set new baudrate
//     //  * Parameters:
//     //  * portID:      0x01 (UART)
//     //  * reserved:    0x00
//     //  * txReady:     0x0000 (disable txReady pin)
//     //  * mode:        0b00001000 11000000 = 0x08 0xC0 (1 stop bit, no parity, 8 data bits)
//     //  * baudrate:    <baudrate> (little endian)
//     //  * inProtoMask: 0x0001 (UBX input only)
//     //  * outProtoMask:0x0001 (UBX output only)
//     //  * flags:       0x0000 (disable extended timeout)
//     //  * reserved x2: 0x0000
//     //  */
//     // cka = 0;
//     // ckb = 0;
//     // uint8_t request[28] = {UBX_SYNC_1, UBX_SYNC_2, UBX_CFG_CLASS,
//     //     UBX_CFG_PRT_ID, 0x00, 20, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0xC0,
//     //     baudrate & 0xff, (baudrate >> 8) & 0xff, (baudrate >> 16) & 0xff,
//     //     (baudrate >> 24) & 0xff, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
//     //     0x00, cka, ckb};
//     //
//     // // Compute the checksum and fill it out
//     // _gps_ubx_checksum(&request[2], 24, &cka, &ckb);
//     // request[26] = cka;
//     // request[27] = ckb;
//     //
//     // // Transmit the request
//     // _gps_send_msg(request);
//     //
//     // // Change our baud rate to match the new speed
//     // _serial_begin(baudrate);
//     //
//     // chThdSleepMilliseconds(300);
//     // // Read the response from the GPS
//     // uint8_t buf[10];
//     // //uint8_t i = 0;
//     // //for(i = 0; i < 9; i++)
//     // //    buf[i] = _gps_get_byte();
//     // uint8_t result = _gps_recv_ack(buf);
//     // if (result != UBX_ACK_ACK_ID){
//     //     if (result == 2) led_set(LED_RB, LED_BLINKING);
//     //     else set_error(ERROR_GPS);
//     //     _serial_begin(oldBRR);
//     // }
// }
//
// /**
//  * Verify the checksum for the given data and length.
//  */
// bool _gps_verify_checksum(uint8_t* data, uint8_t len)
// {
//     uint8_t a, b;
//     _gps_ubx_checksum(data, len, &a, &b);
//     if( a != *(data + len) || b != *(data + len + 1))
//         return false;
//     else
//         return true;
// }
//
// /**
//  * Calculate a UBX checksum using 8-bit Fletcher (RFC1145)
//  */
// void _gps_ubx_checksum(uint8_t* data, uint8_t len, uint8_t* cka, uint8_t* ckb)
// {
//     *cka = 0;
//     *ckb = 0;
//     uint8_t i = 0;
//     for( i = 0; i < len; i++ )
//     {
//         *cka += *data;
//         *ckb += *cka;
//         data++;
//     }
// }
//
// /**
//  * Send a binary message to the GPS.
//  */
// void _gps_send_msg(uint8_t* data)
// {
//     _gps_flush_buffer();
//     // Length determined from message length field
//     size_t len = 8 + ((uint16_t*)data)[2];
//     size_t nwritten;
//     nwritten = sdWrite(&SDRV, data, len);
//     if (nwritten==0) led_set(LED_RED,LED_ON);
//
// }
//
// /**
//  * Receive bytes from GPS (up to specified number of bytes).
//  */
// void _gps_recv_msg(uint8_t* buf, size_t len){
//     sdReadTimeout(&SDRV, buf, len, MS2ST(50));  /** Time for GPS to finish transmitting
//                                                  *  Change to TIME_IMMEDIATE if necessary
//                                                  */
// }
//
// uint8_t _gps_recv_ack(uint8_t* buf){
//     // buf is 10 bytes long
//     // Returns ACK id, NAK id or other uint8_t (neither received)
//     msg_t dummy;
//
//     do {
//         dummy = sdGetTimeout(&SDRV, MS2ST(50));
//         if(dummy == Q_TIMEOUT) return 2;
//     } while ( dummy != 0xB5);
//
//     buf[0] = 0xB5;
//     _gps_recv_msg(&buf[1], 9);
//
//     // Verify header byte
//     if( buf[1] != UBX_SYNC_2 )
//         return 3;
//     if( buf[2] != UBX_ACK_CLASS)
//         return 4;
//
//     // Check message checksum
//     if( !_gps_verify_checksum(&buf[2], 5) ) return 5;
//
//     // Check if we received an ACK or NACK
//     if(buf[3] == UBX_ACK_ACK_ID || buf[3] == UBX_ACK_NAK_ID ){
//         return buf[3];
//     }
//     return 6;
// }
//
//
// /**
//  * Receive a single byte from the GPS and return it.
//  */
// uint8_t _gps_get_byte(void)
// {
//     return sdGet(&SDRV);
// }
//
// /**
//  * Flush the USART recieve buffer.
//  */
// void _gps_flush_buffer(void)
// {
//     msg_t dummy;
//
//     do {
//         dummy = sdGetTimeout(&SDRV, TIME_IMMEDIATE);
//     } while (dummy != Q_TIMEOUT);
//
//     (void)dummy;
// }
