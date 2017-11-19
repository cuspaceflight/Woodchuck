#ifndef __GPS_H__
#define __GPS_H__

#include <stdint.h>
#include <stdbool.h>

#include "ch.h"
#include "hal.h"
#include "ubx.h"

/* NAV-POSECEF Payload Data */
typedef struct __attribute__((packed)) {
    uint32_t i_tow;
    int32_t ecef_x, ecef_y, ecef_z;
    uint32_t p_acc;
} ublox_posecef_t;


/* NAV-PVT Payload Data */
typedef struct __attribute__((packed)) {
    uint32_t i_tow;
    uint16_t year;
    uint8_t month, day, hour, minute, second;
    uint8_t valid;
    uint32_t t_acc;
    int32_t nano;
    uint8_t fix_type;
    uint8_t flags;
    uint8_t reserved1;
    uint8_t num_sv;
    int32_t lon, lat;
    int32_t height, h_msl;
    uint32_t h_acc, v_acc;
    int32_t velN, velE, velD, gspeed;
    int32_t head_mot;
    uint32_t s_acc;
    uint32_t head_acc;
    uint16_t p_dop;
    uint16_t reserved2;
    uint32_t reserved3;
    int32_t head_veh;
    uint32_t reserved4;
} ublox_pvt_t;

typedef struct __attribute__((packed)){
    enum ublox_result result;
    union{
        uint8_t no_pckt;  // No packet to return
        ubx_cfg_nav5_t cfg_nav5;
        ublox_posecef_t posecef;
        ublox_pvt_t pvt;
    };
} state_return_t;

/* Configure uBlox GPS */
void gps_init(void);

// Poll for pvt messages
bool gps_poll_pvt(ublox_pvt_t *pvt_message);
bool gps_get_position(int32_t* lat, int32_t* lon, int32_t* alt);
bool gps_get_time(uint8_t* hour, uint8_t* minute, uint8_t* second);
bool gps_get_pos_time(int32_t* lat, int32_t* lon, int32_t* alt, uint8_t* hour, uint8_t* minute, uint8_t* second);

#endif /*__GPS_H__*/
