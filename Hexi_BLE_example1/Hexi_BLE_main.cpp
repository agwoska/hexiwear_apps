/**
 * @file Hexi_BLE_main.h
 * @author Andrew Woska (you@domain.com)
 * @brief Example program for the KW40Z processor
 *      using BLE and comunicating with a smartphone
 * @version 0.1
 * @date 2022-03-03
 * 
 * @note based on Mbed program
 * 
 * last updated 2022-03-03
 * 
 */

/* get libraries */
#include "Hexi_BLE_main.h"
#include "mbed.h"
#include "Hexi_KW40Z.h"
#include "HTU21D.h"
#include "FXOS8700.h"
#include "FXAS21002.h"
#include "MAX30101.h"
#include "TSL2561.h"
#include "hexi_battery.h"
#include "Hexi_OLED_SSD1351.h"

/* define objects */

RtosTimer hapticTimer(stopHaptics, osTimerOnce);

KW40Z kw40z(PTE24, PTE25);

HTU21D htu;

// TODO add pins
FXOS8700 accel();
FXOS8700 mag();
FXAS21002 gyro();
MAX30101 hr();

HexiwearBattery bat();

SSD1351 oled();

Thread txThread;

// main() runs in its own thread in the OS
int main()
{
    while (true) {

    }
}

