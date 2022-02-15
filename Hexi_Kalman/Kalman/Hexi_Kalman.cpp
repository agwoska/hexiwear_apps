/**
 * @file Hexi_Kalman.cpp
 * @author Andrew Woska (agwoska@buffalo.edu)
 * @date created 11/30/21
 * @brief implements a Kalman Filter for
 *      the a Hexiwear IMU
 * 
 * last updated 2/15/22
 */


#include "mbed.h"
#include "Hexi_Kalman.h"


KalmanIMU::KalmanIMU() :
    accel(HEXI_FX_SDA, HEXI_FX_SCL),
    mag(HEXI_FX_SDA, HEXI_FX_SCL),
    gyro(HEXI_FX_SDA, HEXI_FX_SCL) {}


void KalmanIMU::setup() {
    // set up sensors
    accel.accel_config();
    mag.mag_config();
    gyro.gyro_config();

    kalData = (kalman_data_t *)calloc(1, sizeof(kalman_data_t));

    // set up Kalman Filter by setting starting angle
    accel.acquire_accel_data_g(accel_data);
    mag.acquire_mag_data_uT(mag_data);
    float roll  = calcRoll();
    kalman_roll.setAngle(roll);
    kalData->kalAngle[0] = roll;
    kalData->gyroAngle[0] = roll;
    float pitch = calcPitch();
    kalman_pitch.setAngle(pitch);
    kalData->kalAngle[1]  = pitch;
    kalData->gyroAngle[1] = pitch;
    float yaw = calcYaw();
    kalman_yaw.setAngle(yaw);
    kalData->kalAngle[2]  = yaw;
    kalData->gyroAngle[2] = yaw;

    // start time
    t.start();
    lTime = t.read_ms();

    wait_us(5);
}


void KalmanIMU::setKalman() {
    // get current data from sensors
    accel.acquire_accel_data_g(accel_data);
    mag.acquire_mag_data_uT(mag_data);
    gyro.acquire_gyro_data_dps(gyro_data);
    wait_us(12);

    // get time
    uint32_t cTime = t.read_ms();
    double dt = (cTime - lTime) / 1000000.0;
    lTime = cTime;

    // calculate new Kalman Angles
    float roll  = calcRoll();
    float pitch = calcPitch();
    float yaw   = calcYaw();
    float gyroRateX = gyro_data[0];
    float gyroRateY = gyro_data[1];
    float gyroRateZ = gyro_data[2];
    // use pointers for closer access
    float *kalAngle = kalData->kalAngle;
    float *gyroAngle = kalData->gyroAngle;

    #ifdef RESTRICT_PITCH

    if ( (roll < -90.0 && kalAngle[0] > 90.0) ||
         (roll > 90.0 && kalAngle[0] < -90.0) ) {
            kalman_roll.setAngle(roll);
            kalAngle[0]  = roll;
            gyroAngle[0] = roll;
    }
    else {
        // calculate the angle using a Kalman filter
        kalAngle[0] = kalman_roll.getAngle(roll, gyro_data[0], dt);
    }
    if ( kalAngle[0] > 90.0 || kalAngle[0] < -90.0 ) {
        // Invert rate, so it fits the restriced accelerometer reading
        gyroRateY = -gyroRateY;
    }
    kalAngle[1] = kalman_pitch.getAngle(pitch, gyroRateY, dt);

    #else

    if ( (pitch < -90.0 && kalAngle[1] > 90.0) ||
         (pitch > 90.0 && kalAngle[1] < -90.0) ) {
            kalman_pitch.setAngle(pitch);
            kalAngle[1]  = pitch;
            gyroAngle[1] = pitch;
    }
    else {
        kalAngle[1] = kalman_pitch.getAngle(pitch, gyroAngle[1], dt);
    }
    if ( kalAngle[1] > 90.0 || kalAngle[1] < -90.0 ) {
        gyroRateX = -gyroRateX;
    }
    kalAngle[0] = kalman_roll.getAngle(roll, gyroRateX, dt);
    
    #endif // RESTRICT_PITCH

    gyroAngle[0] += gyroRateX * dt;
    gyroAngle[1] += gyroRateY * dt;

    // reset if drift exceeds limit
    if (gyroAngle[0] > 180.0 || gyroAngle[0] < -180.0) {
        gyroAngle[0] = kalAngle[0];
    }
    if (gyroAngle[1] > 180.0 || gyroAngle[1] < -180.0) {
        gyroAngle[1] = kalAngle[1];
    }

    // set Yaw
    kalAngle[2] = yaw;
    // complementary filter for yaw
    // TODO - Discord is down
    


    #ifdef DEBUG
    // printf("Accelerometer \tX-Axis %4.2f \tY-Axis %4.2f \tZ-Axis %4.2f\n\r",
    //     accel_data[0],
    //     accel_data[1],
    //     accel_data[2]
    // );
    // printf("Magnetometer \tX-Axis %4.2f \tY-Axis %4.2f \tZ-Axis %4.2f\n\r",
    //     mag_data[0],
    //     mag_data[1],
    //     mag_data[2]
    // );
    // printf("Gyroscope \tRoll %4.2f \tPitch %4.2f \tYaw %4.2f\n\r",
    //     gyro_data[0],
    //     gyro_data[1],
    //     gyro_data[2]
    // );
    // printf("Kalman X \tRoll %4.2f \tGyro %4.2f \tAngle %4.2f\n\r",
    //     roll,
    //     gyroAngle[0],
    //     kalAngle[0]
    // );
    // printf("Kalman Y \tPitch %4.2f \tGyro %4.2f \tAngle %4.2f\n\r",
    //     pitch,
    //     gyroAngle[1],
    //     kalAngle[1]
    // );
    // printf("Kalman Z \tYaw %4.2f \tGyro %4.2f \tAngle %4.2f\n\r",
    //     yaw,
    //     gyroAngle[2],
    //     kalAngle[2]
    // );
    // printf("\n\r");
    printf("%4.2f\t%4.2f\t%4.2f\n\r",
        roll,
        pitch,
        yaw
    );
    // printf("%4.2f\n\r", yaw);
    #endif // DEBUG
}


float KalmanIMU::calcRoll() {
    // #ifdef RESTRICT_PITCH
    float theta = atan2(
        accel_data[1], accel_data[2]
    ) * RAD_TO_DEG;
    angles[0] = theta;
    return theta;
    // #else
    // return atan(
    //     accel_data[1] / sqrt (
    //         accel_data[0] * accel_data[0] +
    //         accel_data[2] * accel_data[2]
    //     ) * RAD_TO_DEG
    // );
    // #endif // RESTRICT_PATH
}

float KalmanIMU::calcPitch() {
    // #ifdef RESTRICT_PITCH
    // return atan(
    //     -accel_data[0] / sqrt(
    //         accel_data[1] * accel_data[1] +
    //         accel_data[2] * accel_data[2]
    //     ) * RAD_TO_DEG
    // );
    // #else
    float phi = atan2(
        -accel_data[0], accel_data[2]
    ) * RAD_TO_DEG;
    angles[1] = phi;
    return phi;
    // #endif // RESTRICT_PATH
}

float KalmanIMU::calcYaw() {
    // return atan2(
    //     -accel_data[1], accel_data[0]
    // );
    // float x = 
    //     mag_data[0] * cos(gyro_data[0]) -
    //     mag_data[1] * sin(gyro_data[1]) +
    //     mag_data[2] * cos(gyro_data[1]) *
    //     sin(gyro_data[0]);
    // float y =
    //     mag_data[1] * cos(gyro_data[1]) +
    //     mag_data[2] * sin(gyro_data[2]);

    // return atan2(y, x) / 23.14 * 360.0;

    // TODO
    // https://digitalcommons.calpoly.edu/cgi/viewcontent.cgi?referer=&httpsredir=1&article=1422&context=eesp
    float Mx = 
        mag_data[0] * cos(angles[1]) + 
        mag_data[2] * sin(angles[1]);
    float My =
        mag_data[0] * sin(angles[0]) * sin(angles[1]) +
        mag_data[1] * cos(angles[0]) -
        mag_data[2] * sin(angles[0]) * cos(angles[1]);
    float psi = atan2(-My,Mx) * RAD_TO_DEG;
    // complementary filter
    psi = 0.05f * angles[2] + 0.95f * psi;
    angles[2] = psi;
    return psi;
}


/**
 * returns a copy of the structures current values
 * only use for debugging purposes
 */
kalman_data_t *KalmanIMU::getKalmanData() {
    kalman_data_t *foo;
    memcpy((void *)foo, (void *)kalData, sizeof(kalman_data_t));
    return foo;
}

// from the Quake III Q_rsqrt function
// https://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float num) {
    int i;
    float x, y;
    const float threehalfs = 1.5f;

    x = num * 0.5f;
    y = num;
    i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (threehalfs - (x * y * y));
    
    return y;
}