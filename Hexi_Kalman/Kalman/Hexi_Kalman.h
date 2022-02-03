/**
 * @file Hexi_Kalman.h
 * @author Andrew Woska (agwoska@buffalo.edu)
 * @date created 11/30/21
 * @brief defines a Kalman Filter for
 *      the a Hexiwear IMU
 *
 * 
 * last updated 12/03/21
 */

#pragma once

#ifndef Hexi_Kalman_h_
#define Hexi_Kalman_h_

#include "mbed.h"
#include "FXOS8700.h"
#include "FXAS21002.h"
#include "Kalman.h"

/** Constants/Macros */

#define HEXI_FX_SDA PTC11
#define HEXI_FX_SCL PTC10

#ifndef DEBUG
#define DEBUG
#endif // DEBUG

// #define RESTRICT_PITCH

#define PI          (3.14159265)
#define RAD_TO_DEG  (180.0 / PI)    // approx. 57.29578

/** structures */

/**
 * structure to hold 3 dimentional vector
 * @deprecated use variables in the class under public
 */
typedef struct Vector {
    float x;
    float y;
    float z;
} axis_t;

/**
 * holds important values for Kalman Filter processing
 */
typedef struct {
    float roll;
    float pitch;
    float yaw;
    float kalAngle[3];
    float gyroAngle[3];
} kalman_data_t;



class KalmanIMU {

public:
    /** public functions */

    /**
     * constructor for KalmanIMU
     */
    KalmanIMU();
    
    /**
     * sets up everything for the Kalman Filter to run correctly
     */
    void setup();

    /**
     * get the next Kalman Filtered value
     */
    void setKalman();

    /**
     * calculate roll using Kalman Filter
     */
    float calcRoll();

    /**
     * calculate pitch using Kalman Filter
     */
    float calcPitch();

    /**
     * calculate yaw using Kalman Filter TODO testing
     */
    float calcYaw();

    /**
     * @return copy of current Kalman Filtered values
     */
    kalman_data_t *getKalmanData();

    /** public variables */

    /**
     * array positions and values:
     * [0] - x-axis
     * [1] - y-axis
     * [2] - z-axis
     */

    float accel_data[3];
    float mag_data[3]; 
    float gyro_data[3];

    /* holds the data for the current Kalman Filtered values */
    kalman_data_t *kalData;

private:
    /** all sensor object declarations */
    FXOS8700 accel;
    FXOS8700 mag;
    FXAS21002 gyro;

    Kalman kalman_pitch;
    Kalman kalman_roll;
    Kalman kalman_yaw;

    /* timer for Kalman calculations */
    Timer t;
    /* holds last time value found */
    uint32_t lTime;

};


#endif // Hexi_Kalman_h