/**
 * @file Hexi_Kalman.h
 * @author Andrew W
 * @date created 11/30/21
 * @brief defines a Kalman Filter for
 *      the a Hexiwear IMU
 * 
 * last updated 12/01/21
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

typedef struct {
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
    KalmanIMU();
    
    void setup();

    void setKalman();

    float calcRoll();
    float calcPitch();
    float calcYaw();

    float accel_data[3];
    float mag_data[3]; 
    float gyro_data[3];

    kalman_data_t *kalData;
    kalman_data_t *getKalmanData();

private:

    FXOS8700 accel;
    FXOS8700 mag;
    FXAS21002 gyro;

    Kalman kalman_pitch;
    Kalman kalman_roll;
    Kalman kalman_yaw;

    Timer t;

    uint32_t lTime;

};


#endif // Hexi_Kalman_h