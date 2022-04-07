/**
 * @file Steady_App.h
 * @author Andrew Woska (agwoska@buffalo.edu)
 * @date 2022-04-05
 * @brief An application header as a 
 *      POC for the Hexiwear to work as a 
 *      therapeutic tool/technology
 * last modified 2022-04-05
 */

#pragma once


#ifndef STEADY_APP_H
#define STEADY_APP_H

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
 * @brief interrupt handler for button 
 */
void btnInterHdlr();

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
