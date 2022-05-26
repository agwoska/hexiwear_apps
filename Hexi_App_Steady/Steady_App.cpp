/**
 * @file Steady_App.cpp
 * @author Andrew Woska (agwoska@buffalo.edu)
 * @date 2022-04-05
 * @brief An application implementation as a 
 *      POC for the Hexiwear to work as a 
 *      therapeutic tool/technology
 * @version 1.1.1
 * last update 2022-05-25
 */


#include "Steady_App.h"
#include "mbed.h"
#include <string>

#include "OLED_types.h"
#include "Hexi_OLED_SSD1351.h"
#include "Hexi_KW40Z.h"
#include "Hexi_Kalman.h"
#include "Hexi_Battery.h"

/** initialize objects and variables **/

SSD1351 oled(PTB22, PTB21, PTC13, PTB20, PTE6, PTD15);
KW40Z kw40z(PTE24, PTE25);
KalmanIMU imu;
HexiwearBattery bat;

DigitalOut haptics(PTB9);
DigitalOut redLED(LED1);
DigitalOut greenLED(LED2);
DigitalOut blueLED(LED3);

Ticker hapticsTimer;
Ticker activityTimer;

oled_text_properties_t txtProps;
uint8_t startFlag;
char txt[20];


/** implementation **/

int main() {
    setup();
    while (true) {
        openState();
        beginState();
        if ( FLAG_ON == startFlag ) {   // case 1
            startFlag = FLAG_OFF;
            activeState();
        }
        else {                          // case 2
            startFlag = FLAG_OFF;
            activeStateAlt();
        }
        completeState();
        wait(1);                        // wait an extra second until repeat
    }
}



void setLED(int led) {
    uint8_t red = led & 0x1;
    uint8_t green = (led & 0x2) >> 1;
    uint8_t blue = led >> 2;
    redLED = !red;
    greenLED = !green;
    blueLED = !blue;
}


void setup() {
    setLED(0x0);
    // oled setup
    oled.PowerON();
    oled.DimScreenON();
    oled.FillScreen(COLOR_BLACK);
    oled.GetTextProperties(&txtProps);
    txtProps.fontColor = COLOR_WHITE;
    txtProps.alignParam = OLED_TEXT_ALIGN_LEFT;
    oled.SetTextProperties(&txtProps);
    // sensor setup
    bat.sensorOn();
    imu.setup();
    imu.setKalman();
    // button interrupt setup
    kw40z.attach_buttonLeft(btnLeftHdlr);
    kw40z.attach_buttonRight(btnRightHdlr);
    startFlag = FLAG_OFF;
}


void btnLeftHdlr() {
    startFlag = FLAG_ON;
}

void btnRightHdlr() {
    startFlag = FLAG_ALT;
}

void hapticsStop() {
    hapticsTimer.detach();
    haptics = 0;
}
void hapticsMin() {
    hapticsTimer.attach(hapticsStop, 0.05f);
    haptics = 1;
    setLED(0x7); // white
}
void hapticsLow() {
    hapticsTimer.attach(hapticsStop, 0.1f);
    haptics = 1;
    setLED(0x6); // cyan
}
void hapticsLowMed() {
    hapticsTimer.attach(hapticsStop, 0.2f);
    haptics = 1;
    setLED(0x4); // blue
}
void hapticsMed() {
    hapticsTimer.attach(hapticsStop, 0.3f);
    haptics = 1;
    setLED(0x5); // purple
}
void hapticsMax() {
    hapticsTimer.attach(hapticsStop, 0.4f);
    haptics = 1;
    setLED(0x1); // red
}


void openState() {
    setLED(0x0);
    oled.FillScreen(COLOR_BLACK);
    strcpy(txt, "Press left");
    oled.Label((uint8_t *)txt, 5, 20);
    strcpy(txt, "for case 1 &");
    oled.Label((uint8_t *)txt, 5, 35);
    strcpy(txt, "Press right");
    oled.Label((uint8_t *)txt, 5, 50);
    strcpy(txt, "for case 2");
    oled.Label((uint8_t *)txt, 5, 65);
    strcpy(txt, "to start       ");
    oled.Label((uint8_t *)txt, 5, 80);
    printf("Waiting...\r\n");
    // have access to battery to make sure there isadequate time to demo
    uint8_t btry = bat.readLevelPercent();
    sprintf(txt, "Battry: %i%%", btry);
    oled.Label((uint8_t *)txt, 5, 5);
    while (!startFlag) { // wait until a button is pressed
        wait_ms(200);
        btry = bat.readLevelPercent();
        sprintf(txt, "Battery: %i%%   ", btry);
        oled.Label((uint8_t *)txt, 5, 5);
    }
}


void beginState() {
    oled.FillScreen(COLOR_BLACK);
    setLED(0x2);
    wait_us(50);
    // count down
    strcpy(txt, "3");
    oled.Label((uint8_t *)txt, 5, 25);
    printf("3\r\n");
    wait(1);
    strcpy(txt, "2");
    oled.Label((uint8_t *)txt, 5, 25);
    printf("2\r\n");
    wait(1);
    strcpy(txt, "1");
    oled.Label((uint8_t *)txt, 5, 25);
    printf("1\r\n");
    wait(1);
    strcpy(txt, "Go!");
    oled.Label((uint8_t *)txt, 5, 25);
    printf("Go!\r\n");
}


void activeState() {
    uint8_t i = 0;
    float pitch = 0, roll = 0;
    activityTimer.attach(secTimeUp, 1);
    while ( i < 60 ) {
        if (startFlag) {
            ++i;
            startFlag = 0;
            if ( i < 60 ) { // prevent extra ticker event
                activityTimer.attach(secTimeUp, 1);
            }
            imu.setKalman();
            pitch = abs( imu.calcPitch() );
            roll = abs( imu.calcRoll() );
            if ( pitch < 95 || roll < 95 ) {
                hapticsMax();
            }
            else if ( pitch < 120 || roll < 120 ) {
                hapticsMed();
            }
            else if ( pitch < 140 || roll < 140 ) {
                hapticsLowMed();
            }
            else if ( pitch < 160 || roll < 160 ) {
                hapticsLow();
            }
            else if ( pitch < 170 || roll < 170 ) {
                hapticsMin();
            }
            else {
                setLED(0x2);
            }
            // update display
            if ( i < 10 ) {
                sprintf(txt, "Time left:   %i  ", 60-i);
            }
            else {
                sprintf(txt, "Time left: %i  ", 60-i);
            }
            oled.Label((uint8_t *)txt, 5, 40);
            printf("Time: %i \tPitch: %3.2f\tRoll: %3.2f\r\n", i, pitch, roll);
        }
    }
}

void activeStateAlt() {
    uint8_t i = 0;
    float pitch = 0, roll = 0;
    activityTimer.attach(secTimeUp, 1);
    while ( i < 60 ) {
        if (startFlag) {
            ++i;
            startFlag = 0;
            if ( i < 60 ) { // prevent extra ticker event
                activityTimer.attach(secTimeUp, 1);
            }
            imu.setKalman();
            pitch = abs( imu.calcPitch() );
            roll = abs( imu.calcRoll() );
            if ( pitch < 10 || pitch > 170) {
                hapticsMax();
            }
            else if ( pitch < 30 || pitch > 150 ) {
                hapticsMed();
            }
            else if ( pitch < 50 || pitch > 130 ) {
                hapticsLowMed();
            }
            else if ( pitch < 70 || pitch > 110 ) {
                hapticsLow();
            }
            else if ( pitch < 80 || pitch > 100 ) {
                hapticsMin();
            }
            else {
                setLED(0x2);
            }
            // update display
            if ( i < 10 ) {
                sprintf(txt, "Time left:   %i  ", 60-i);
            }
            else {
                sprintf(txt, "Time left: %i  ", 60-i);
            }
            oled.Label((uint8_t *)txt, 5, 40);
            printf("Time: %i \tPitch: %3.2f\tRoll: %3.2f\r\n", i, pitch, roll);
        }
    }
}


void secTimeUp() {
    activityTimer.detach();
    startFlag = FLAG_ON;
}


void completeState() {
    startFlag = FLAG_OFF;
    strcpy(txt, "Activity");
    oled.Label((uint8_t *)txt, 5, 25);
    strcpy(txt, "completed  ");
    oled.Label((uint8_t *)txt, 5, 40);
    printf("Done\r\n");
    wait(1);    // return to start screen after a second
}
