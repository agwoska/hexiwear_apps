/**
 * @file Steady_App.cpp
 * @author Andrew Woska (agwoska@buffalo.edu)
 * @date 2022-04-05
 * last update 2022-04-05
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
KalmanIMU imu;

DigitalOut haptics(PTB9);
DigitalOut greenLED(LED2);

Ticker hapticsTimer;
Ticker activityTimer;

Thread the;
Mutex mut;

// int time;
// uint8_t state;
oled_text_properties_t txtProps;
uint8_t startFlag;
char txt[20];


/** implementation **/

int main() {
    setup();
    while (true) {
        openState();
        beginState();
        activeState();
        completeState();
        wait(1);
    }
}



void setup() {
    greenLED = 1;
    // oled
    oled.PowerON();
    // oled.DimScreenON();
    oled.FillScreen(COLOR_BLACK);
    oled.GetTextProperties(&txtProps);
    txtProps.fontColor = COLOR_WHITE;
    txtProps.alignParam = OLED_TEXT_ALIGN_LEFT;
    oled.SetTextProperties(&txtProps);
    // sensors
    imu.setup();
    imu.setKalman();
    kw40z.attach_buttonLeft(btnInterHdlr);
    startFlag = 0;
}


void btnInterHdlr() {
    startFlag = 1;
}


void hapticsStop() {
    hapticsTimer.detach();
    haptics = 0;
}
void hapticsMin() {
    hapticsTimer.attach(hapticsStop, 0.05f);
    haptics = 1;
}
void hapticsLow() {
    hapticsTimer.attach(hapticsStop, 0.1f);
    haptics = 1;
}
void hapticsLowMed() {
    hapticsTimer.attach(hapticsStop, 0.2f);
    haptics = 1;
}
void hapticsMed() {
    hapticsTimer.attach(hapticsStop, 0.3f);
    haptics = 1;
}
void hapticsMax() {
    hapticsTimer.attach(hapticsStop, 0.4f);
    haptics = 1;
}


void openState() {
    strcpy(txt, "Press left");
    oled.Label((uint8_t *)txt, 5, 25);
    strcpy(txt, "to start       ");
    oled.Label((uint8_t *)txt, 5, 40);
    printf("Waiting...\r\n");
    while (!startFlag) {
        wait_ms(100);
    }
    startFlag = 0;
}


void beginState() {
    oled.FillScreen(COLOR_BLACK);
    // txtProps.alignParam = OLED_TEXT_ALIGN_CENTER;
    // oled.SetTextProperties(&txtProps);
    wait_us(50);
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
    // txtProps.alignParam = OLED_TEXT_ALIGN_LEFT;
    // oled.SetTextProperties(&txtProps);
    uint8_t i = 0;
    float pitch = 0;
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
            if ( pitch < 95 ) {
                hapticsMax();
            }
            else if ( pitch < 120 ) {
                hapticsMed();
            }
            else if ( pitch < 140 ) {
                hapticsLowMed();
            }
            else if ( pitch < 160 ) {
                hapticsLow();
            }
            else if ( pitch < 170 ) {
                hapticsMin();
            }
            // update display
            if ( i < 10 ) {
                sprintf(txt, "Time left:   %i  ", 60-i);
            }
            else {
                sprintf(txt, "Time left: %i  ", 60-i);
            }
            oled.Label((uint8_t *)txt, 5, 40);
            printf("Time: %i \tPitch: %3.2f\r\n", i, pitch);
        }
    }
}

void secTimeUp() {
    activityTimer.detach();
    startFlag = 1;
}


void completeState() {
    startFlag = 0;
    strcpy(txt, "Activity");
    oled.Label((uint8_t *)txt, 5, 25);
    strcpy(txt, "completed  ");
    oled.Label((uint8_t *)txt, 5, 40);
    printf("Done\r\n");
    wait(1);
}
