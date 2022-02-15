/**
 * @file KW40Z.cpp
 * @author Andrew Woska (agwoska@buffalo.edu)
 * @date Feb 10, 2022
 * @brief implementation of the KW40Z for
 *      Hexiwear development in Mbed OS 5.12
 *      with customized outputs
 * 
 * last updated: Feb 15, 2022
 */

#include "KW40Z.h"

KW40Z::KW40Z(PinName txPin, PinName rxPin) :
    serial(txPin, rxPin) {
        device.format(8, Serial::None, 2);

        // initialize all necessary variables
        rxBuffer = (uint8_t *)&hostInterface_txMask;

        btnUp    = NULL;
        btnDown  = NULL;
        btnLeft  = NULL;
        btnRight = NULL;
        btnSlide = NULL;
        alert    = NULL;
        passkey  = NULL
        notif    = NULL;
        
        // version

        // states

        // start threads
        rxThread.start(this, &KW40Z::rxTask);
        mainThread.start(this, &KW40Z::mainTask);
}

KW40Z::~KW40Z() {
    // terminate threads
    rxThread.join();
    mainThread.join();
}

void KW40Z::attach_btnUp(button_t btnFunct) { 
    btnUp= btnFunct;
}
void KW40Z::attach_btnDown(button_t btnFunct) { 
    btnDown = btnFunct;
}
void KW40Z::attach_btnLeft(button_t btnFunct) { 
    btnLeft = btnFunct;
}
void KW40Z::attach_btnRight(button_t btnFunct) { 
    btnRight = btnFunct;
}
