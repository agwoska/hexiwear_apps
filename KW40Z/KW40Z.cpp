/**
 * @file KW40Z.cpp
 * @author Andrew Woska (agwoska@buffalo.edu)
 * @date Feb 10, 2022
 * @brief implementation of the KW40Z for
 *      Hexiwear development in Mbed OS 5.12
 *      with customized outputs
 * 
 * last updated: Feb 16, 2022
 */

#include "KW40Z.h"

KW40Z::KW40Z(PinName txPin, PinName rxPin) :
    serial(txPin, rxPin) {
        serial.format(8, Serial::None, 2);

        // initialize all necessary variables
        rxBuffer = (uint8_t *)&hostInterface_rx;

        btnUp    = NULL;
        btnDown  = NULL;
        btnLeft  = NULL;
        btnRight = NULL;
        btnSlide = NULL;
        alert    = NULL;
        passkey  = NULL;
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










void KW40Z::rxTask() {
    while(1) {
        if( serial.readable() ) {
            *rxBuffer++ = serial.getc();
            // TODO

        }
    }
}

void KW40Z::mainTask() {
 // TODO
 while(1) {
     osEvent e = queue.get();
     if ( osEventMessage == e.status ) {
        kwHostInterface_packet_t *packet =
            (kwHostInterface_packet_t *)e.value.p;
        // TODO finish
        mpool.free(packet);
     }
 }
}








hexi_version_t KW40Z::GetVersion() {
    hexi_version_t ver = { 0 };
    ver.verMajor = hexi_version_major;
    ver.verMinor = hexi_version_minor;
    ver.verPatch = hexi_version_patch;
}
