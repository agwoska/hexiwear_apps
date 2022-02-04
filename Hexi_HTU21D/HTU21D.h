/**
 * @file HTU21D.h
 * @author Andrew Woska (agwoska@buffalo.edu)
 * @date Jan 02, 2022
 * @brief header file of the HTU21D for
 *      Hexiwear development in Mbed OS 5.12
 * 
 * datasheet: https://cdn-shop.adafruit.com/datasheets/1899_HTU21D.pdf
 * 
 * last updated: Feb 3, 2022
 */

#pragma once

#ifndef HTU21D_H
#define HTU21D_H

#include "mbed.h"


/** constants **/

/* from datasheet */

// 0.4 MHz (pg. 9)
#define HTU21D_FREQ         0x61A80

// command table (pg. 11)
#define HTU21D_TRIG_TEMP    0xE3
#define HTU21D_TRIG_HUMD    0xE5
#define HTU21D_TRIG_TEMP_NH 0xF3
#define HTU21D_TRIG_HUMD_NH 0xF5
#define HTU21D_WRITE_REG    0xE6
#define HTU21D_READ_REG     0xE7
#define HTU21D_SOFT_RESET   0xFE

// sending commands (pg. 10)
#define HTU21D_I2C_ADDR     0x40
#define HTU21D_WRITE_ADDR   0x80
#define HTU21D_READ_ADDR    0x81


/* class prototype */

class HTU21D {

public:

    /**
     * Construct a new HTU21D object
     * @param sda pin for SDA of I2C
     * @param scl pin for SCL of I2C
     */
    HTU21D(PinName sda, PinName scl);

    /**
     * Construct a new HTU21D object
     * with default Hexiwear pins
     */
    HTU21D();

    /**
     * Destroy the HTU21D object
     */
    ~HTU21D();

    /**
     * Get the relative humidity as a percent
     * @return relative humidity 
     */
    int getHumidity();

    /**
     * Get measured temperature in Celcius
     * @return temperature
     */
    int getTemp();

    /** macros for backwards compatability */
    /* Get temperature in Celcius */
    int sample_ctemp() { return getTemp(); }
    /* Get humidity as a percent */
    int sample_humid() { return getHumidity(); }

private:

    /**
     * I2C object used for measurements
     */
    I2C i2c;

};

#endif // HTU21D_H
