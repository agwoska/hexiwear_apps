/**
 * @file main.cpp
 * @author Andrew Woska (agwoska@buffalo.edu)
 * @date created 11/17/21
 * @brief
 *      Implement an IMU with a Kalman Filter using
 *      the accelerometer and gyroscope
 * 
 * @note code for Kalman elemented supplied by Shardul S.
 *
 * @note possibly replace sqrt with fast inverse sqrt function; check licensing
 * @warning extremely expensive math functions used
 * 
 * last updated 12/05/21
 */


#include "mbed.h"
/** Libraries */
#include "Hexi_OLED_SSD1351.h"
#include "Hexi_KW40Z.h"
#include "Hexi_Kalman.h"

#include "main.h"

/** Instantiations */

SSD1351 oled(PTB22,PTB21,PTC13,PTB20,PTE6, PTD15);
KW40Z kw40z(PTE24, PTE25);
KalmanIMU imu;

// help control screen scheduling
// TODO: lower size
EventQueue evq(32 * EVENTS_EVENT_SIZE);
Mutex mut;
Thread the;

/** Global Variables */

int cScreen; // current screen
int lScreen; // last screen


/** Implementation */

int main() {
    imu.setup();
    setup();

    the.start( callback(&evq, &EventQueue::dispatch_forever) );
    
    while (true) {
        // while (mut.trylock()) {}
        evq.call( &setKalman );
        // imu.setKalman();
        // mut.unlock();

        // #ifdef OLED_ON
        // while (mut.trylock()) {}
        evq.call( &showKalman );
        // showKalman();
        // mut.unlock();
        // #endif // OLED_ON

        // wait(0.002f); // at 200Hz
        wait_ms( 2 );
    }
}


/**
 * set up sensors and starting display
 */
void setup() {
    cScreen = SCREEN_X;
    lScreen = SCREEN_X;
    // set up KW40Z button interrupts
    kw40z.attach_buttonLeft(&ltBtnPress);
    kw40z.attach_buttonRight(&rtBtnPress);
    // kw40z.attach_buttonUp(&upBtnPress);
    // kw40z.attach_buttonDown(&dnBtnPress);
    // set up OLED
    oled.FillScreen(COLOR_BLACK);
    oled_text_properties_t txtProps = { 0 };
    txtProps.fontColor = COLOR_BLUE;
    txtProps.alignParam = OLED_TEXT_ALIGN_RIGHT;
    oled.SetTextProperties( &txtProps );
    wait_us(5);
}


/**
 *
 */
void setKalman() {
    mut.lock();
    imu.setKalman();
    mut.unlock();
}

/**
 * displays data on OLED based on which screen is active
 */
void showKalman() {
    if ( lScreen != cScreen ) {
        oled.FillScreen(COLOR_BLACK);
        lScreen = cScreen;
        wait_us(5);
    }
    mut.lock();
    switch ( cScreen ) {
        case SCREEN_X: {
            showRoll();
            break;
        }
        case SCREEN_Y: {
            showPitch();
            break;
        }
        case SCREEN_Z: {
            showYaw();
            break;
        }
        default: {}
    }
    mut.unlock();
}


/**
 * writes results to OLED
 */
void showRoll() {
    oled_text_properties_t txtProps = { 0 };
    txtProps.fontColor = COLOR_BLUE;
    txtProps.alignParam = OLED_TEXT_ALIGN_RIGHT;
    oled.SetTextProperties( &txtProps );
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
    oled.TextBox(txt,40,25,25,5);
    sprintf((char *)txt, "%4.2f", imu.kalData->gyroAngle[1]);
    oled.TextBox(txt,5,39,25,5);
    sprintf((char *)txt, "%4.2f", imu.kalData->kalAngle[1]);
    oled.TextBox(txt,5,53,25,5);
}


void showYaw() {
    uint8_t txt[20];
    strcpy((char *)txt, "Pitch");
    oled.Label(txt, 5, 25);
    strcpy((char *)txt, "Gyro");
    oled.Label(txt, 5, 39);
    strcpy((char *)txt, "Angle");
    oled.Label(txt, 5, 53);
    sprintf((char *)txt, "%4.2f", imu.kalData->yaw);
    oled.TextBox(txt,40,25,25,5);
    sprintf((char *)txt, "%4.2f", imu.kalData->gyroAngle[2]);
    oled.TextBox(txt,5,39,25,5);
    sprintf((char *)txt, "%4.2f", imu.kalData->kalAngle[2]);
    oled.TextBox(txt,5,53,25,5);
}



void ltBtnPress() {
    --cScreen;
    if ( cScreen < 0 ) cScreen = SCREEN_MAX-1;
}


void rtBtnPress() {
    ++cScreen;
    cScreen %= SCREEN_MAX;
}
