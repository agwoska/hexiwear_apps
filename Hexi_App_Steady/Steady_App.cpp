/**
 * @file Steady_App.cpp
 * @author Andrew Woska (agwoska@buffalo.edu)
 * @date 2022-03-15
 * @brief An application implementation as a 
 *      POC for the Hexiwear to work as a 
 *      therapeutic tool/technology
 * @version 0.0.1
 * 
 * last modified 2022-03-15
 */

#include "Steady_App.h"
#include "mbed.h"
#include <string>

#include "OLED_types.h"
#include "Hexi_OLED_SSD1351.h"
#include "Hexi_KW40Z.h"
#include "Hexi_Kalman.h"


/** initialize objects and variables **/

SSD1351 oled(PTB22, PTB21, PTC13, PTB20, PTE6, PTD15);
KW40Z kw40z(PTE24, PTE25);
KalmanIMU imu();
DigitalOut redLED(LED2);

Ticker hapticsTimer();
Ticker activityTimer();
Thread idler();

int time;
uint8_t state;
oled_text_properties_t txtProps;

/** implementation **/

int main() {
    setup();
    // splash screen
    oled.FillScreen(COLOR_BLACK);

    // splash screen


    while (true) {
        // set idle until btn interrupt
        idler.start(idleState); // pause until butotn pressed
        idler.join();           // resume when not idle

        
    }
}


void setup() {
    redLED = 0;
    state = SCREEN_OPEN;
    oled.PowerON();
    oled.DimScreenON();
    // set up text properties
    txtProps = { 0 };
}


void hapticsStop() {}


void hapticsOnce() {}


void hapticsLow() {}


void hapticsMed() {}


void hapticsMax() {}


void secTimeUp() {
    ++time;
}


void openState() {}


void idleState() {
    while (1) {
        // do nothing - testings
    }
}


void beginState() {}


void activeState() {}


void completeState() {}
