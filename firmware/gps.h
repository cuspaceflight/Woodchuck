#ifndef __GPS_H__
#define __GPS_H__

#include <stdint.h>
#include <stdbool.h>

#include "ch.h"
#include "hal.h"

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


// /* Timestamped NAV-PVT Data */
// typedef struct __attribute__((packed)) {
//     uint32_t pps_timestamp;
//     uint32_t time_of_week;
// } pvt_capture;


// /* Global Timestamped iTOW */
// extern pvt_capture stamped_pvt;
//
// /* PVT Stamp Mutex */
// extern mutex_t pvt_stamp_mutex;
//
// /* Global Position Packet */
// extern position_packet pos_pkt;
//
// /* Position Packet Mutex */
// extern mutex_t pos_pkt_mutex;

/* Configure uBlox GPS */
void gps_init(void);

// Poll for position and UTC time
void gps_get_position(int32_t* lat, int32_t* lon, int32_t* alt);
void gps_get_time(uint8_t* hour, uint8_t* minute, uint8_t* second);

#endif /*__GPS_H__*/










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
//  * Greg Brooks 2016
//  */
//
// #ifndef __GPS_H__
// #define __GPS_H__
//
// #include <stdint.h>
// #include <stdbool.h>
//
// #include "ch.h"
// #include "hal.h"
//
// // Initial setup function
// void gps_init(void);
//
// // Poll for position and UTC time
// void gps_get_position(int32_t* lat, int32_t* lon, int32_t* alt);
// void gps_get_time(uint8_t* hour, uint8_t* minute, uint8_t* second);
//
// // Poll for fix status, and check OK
// void gps_check_lock(uint8_t* lock, uint8_t* sats);
//
// // Set the GPS to <1g airborne mode
// void gps_set_mode(void);
//
// // Check the GPS is set to <1g airborne mode
// uint8_t gps_check_nav(void);
//
//
// /* INTERNAL METHODS */
//
// //Start serial port at desired baud rate, 8N1 format
// void _serial_begin(uint32_t baud);
//
// // Get serial port's current baud rate
// uint32_t _serial_get_baud(void);
//
// // Try to set the baudrate. Returns success
// void _gps_set_baud(uint32_t baudrate);
//
// // Calculates the UBX checksum for a given data and length
// void _gps_ubx_checksum(uint8_t* data, uint8_t len, uint8_t* cka, uint8_t* ckb);
//
// // Verifies the checksum of the given data is correct
// bool _gps_verify_checksum(uint8_t* data, uint8_t len);
//
// // Transmits a byte-array of data to the GPS
// void _gps_send_msg(uint8_t* data);
//
// // Receive bytes from GPS (up to specified number of bytes)
// void _gps_recv_msg(uint8_t* buf, size_t len);
//
// // Receive ack message from GPS
// uint8_t _gps_recv_ack(uint8_t* buf);
//
// // Reads a single byte from the GPS
// uint8_t _gps_get_byte(void);
//
// // Flushes the receive buffer
// void _gps_flush_buffer(void);
//
// #endif /*__GPS_H__ */
