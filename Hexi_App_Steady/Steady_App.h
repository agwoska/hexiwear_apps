/**
 * @file Steady_App.h
 * @author Andrew Woska (agwoska@buffalo.edu)
 * @date 2022-04-05
 * @brief An application header as a 
 *      POC for the Hexiwear to work as a 
 *      therapeutic tool/technology
 * @version 1.1.0
 * last modified 2022-04-20
 */

#pragma once


#ifndef STEADY_APP_H
#define STEADY_APP_H

/** constants **/

typedef enum {
    FLAG_OFF    = 0,    // for no flags
    FLAG_ON     = 1,    // for left button and ticker
    FLAG_ALT    = 2,    // for right button
} flags_t;

/** prototype functions **/

/**
 * @brief stop haptics on normal timer
 */
void hapticsStop();
/**
 * @brief starts haptics on shortest interval 
 */
void hapticsMin();
/**
 * @brief have haptics on for short interval
 */
void hapticsLow();
/**
 * @brief have haptics on for short-ish
 */
void hapticsLowMed();
/**
 * @brief have haptics on for moderate interval
 */
void hapticsMed();
/**
 * @brief have haptics on for maximum interval
 * @warning do not overuse in case it causes damage
 */
void hapticsMax();

/**
 * @brief interrupt handler for button left
 */
void btnLeftHdlr();
/**
 * @brief interrupt handler for button right
 */
 void btnRightHdlr();

/**
 * @brief counts up timer every second until
 *      activityTimer detached
 */
void secTimeUp();

/**
 * @brief set LED
 * @param led set LED color RGB
 * Red -> 0x1
 * Blue -> 0x2
 * Green -> 0x4
 * Purple -> 0x3
 * Yellow -> 0x5
 * Cyan -> 0x6
 * White -> 0x7
 */
void setLED(int led);


void offLED();

/**
 * @brief sets up the OLED and KW40Z drivers for general use 
 */
void setup();

/**
 * @brief starts the system with a splash screen 
 */
void openState();
/**
 * @brief waits for button to be pressed 
 */
void idleState();
/**
 * @brief starts the activity 
 */
void beginState();
/**
 * @brief continues until 1 minute is over
 * then go to completeState; for case 1
 */
void activeState();
/**
 * @brief continues until 1 minute is over
 * then go to completeState; for case 2
 */
void activeStateAlt();
/**
 * @brief display message; then go back to idleState 
 */
void completeState();

#endif // STEADY_APP_H
