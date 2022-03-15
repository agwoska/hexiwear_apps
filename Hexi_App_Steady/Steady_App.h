/**
 * @file Steady_App.h
 * @author Andrew Woska (agwoska@buffalo.edu)
 * @date 2022-03-15
 * @brief An application header as a 
 *      POC for the Hexiwear to work as a 
 *      therapeutic tool/technology
 * @version 0.0.1
 * last modified 2022-03-15
 */

#pragma once

#include "mbed.h"

#ifndef STEADY_APP_H
#define STEADY_APP_H

/** structures **/
typedef enum {
    SCREEN_OPEN,
    SCREEN_IDLE,
    SCREEN_ACTIVE,
    SCREEN_COMPLETE,
    SCREEN_MAX
} screens_t;

/** prototype functions **/

/**
 * @brief stop haptics on normal timer
 */
void hapticsStop();
/**
 * @brief starts haptics on normal timer 
 */
void hapticsOnce();
/**
 * @brief have haptics continue at slow intervals 
 */
void hapticsLow();
/**
 * @brief have haptics continue at moderate intervals 
 */
void hapticsMed();
/**
 * @brief have haptics continue at fast intervals
 * @warning do not overuse in case it causes damage
 */
void hapticsMax();

/**
 * @brief counts up timer every second until
 *      activityTimer detached
 */
void secTimeUp();

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
 * then go to completeState
 */
void activeState();
/**
 * @brief display message; then go back to idleState 
 */
void completeState();

#endif // STEADY_APP_H
