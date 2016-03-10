
/*
 * MS5611-01BA03 Driver
 * Woodchuck
 *
 * Adapted from
 * M2FC - 2014 Adam Greig, Cambridge University Spaceflight
 *
 * Woodchuck - Eivind Roson Eide 2016
 */

#ifndef MS5611_H
#define MS5611_H

typedef struct {
    uint16_t c1, c2, c3, c4, c5, c6;
} MS5611CalData;

/* global variables of temperature and pressure for easy access */
extern int32_t global_temperature;
extern int32_t global_pressure;
 

/* The main script. Run this. */
void ms5611_run();
void ms5611_init(MS5611CalData* cal_data);

#endif /* MS5611_H */
