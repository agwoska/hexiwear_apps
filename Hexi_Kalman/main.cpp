/**
 * @file main.cpp
 * @author Andrew W.
 * @date created 11/17/21
 * @brief
 *      Implement an IMU with a Kalman Filter using
 *      the accelerometer and gyroscope
 * 
 * @note code for Kalman elemented supplied by Shardul S.
 * @note add yaw and magnetometer
 *
 * @note possibly replace sqrt with fast inverse sqrt function; check licensing
 * @warning extremely expensive math functions used
 * 
 * last updated 12/01/21
 */


#include "mbed.h"
/* Libraries */
#include "Hexi_OLED_SSD1351.h"
#include "Hexi_Kalman.h"

#include "main.h"

/** Instantiations */

SSD1351 oled(PTB22,PTB21,PTC13,PTB20,PTE6, PTD15);
KalmanIMU imu;

// help control screen scheduling
Mutex mut;


/** Implementation */

int main() {
    imu.setup();
    setup();
    
    while (true) {
        mut.lock();
        imu.setKalman();
        mut.unlock();

        // mut.lock();
        // // imu.showKalman();
        // mut.unlock();

        wait(1.0f); // decrease drastically after testing
    }
}


/**
 * set up sensors and starting display
 */
void setup() {
    // set up OLED
    oled.FillScreen(COLOR_BLACK);
    wait_us(5);
}


/**
 * displays data on OLED based on which screen is active
 */
void showKalman() {
    oled_text_properties_t txtProps = { 0 };
    txtProps.fontColor = COLOR_BLUE;
    txtProps.alignParam = OLED_TEXT_ALIGN_RIGHT;
    oled.SetTextProperties( &txtProps );
    
}


/**
 * writes results to OLED
 * @param kal_data contains data from setKalman
 * TODO!!! for Hexi display.
 */
void showRoll() {
    uint8_t txt[20];
    strcpy((char *)txt, "Roll");
    oled.Label(txt, 5, 25);
    strcpy((char *)txt, "Gyro");
    oled.Label(txt, 5, 39);
    strcpy((char *)txt, "Angle");
    oled.Label(txt, 5, 53);

    sprintf((char *)txt, "%4.2f", imu.kalData->roll);
    oled.TextBox(txt,40,25,25,5);
    sprintf((char *)txt, "%4.2f", imu.kalData->gyroAngle[0]);
    oled.TextBox(txt,5,39,25,5);
    sprintf((char *)txt, "%4.2f", imu.kalData->kalAngle[0]);
    oled.TextBox(txt,5,53,25,5);
}


/**
 * writes results to OLED
 */
void showPitch() {
    uint8_t txt[20];
    strcpy((char *)txt, "Pitch");
    oled.Label(txt, 5, 25);
    strcpy((char *)txt, "Gyro");
    oled.Label(txt, 5, 39);
    strcpy((char *)txt, "Angle");
    oled.Label(txt, 5, 53);
    sprintf((char *)txt, "%4.2f", imu.kalData->pitch);

    sprintf((char *)txt, "%4.2f", imu.kalData->gyroAngle[1]);

    sprintf((char *)txt, "%4.2f", imu.kalData->kalAngle[1]);

}


void showYaw() {
    
}
