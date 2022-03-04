/**
 * @file Hexi_BLE_main.h
 * @author Andrew Woska (you@domain.com)
 * @brief Example program header for the KW40Z processor
 *      using BLE and comunicating with a smartphone
 * @version 0.1
 * @date 2022-03-03
 * 
 * @note based on Mbed program 
 *      https://os.mbed.com/teams/Hexiwear/code/Hexi_BLE_Example/
 * 
 * last updated 2022-03-04
 * 
 */

#pragma once

#ifndef HEXI_BLE_MAIN_H
#define HEXI_BLE_MAIN_H

#define LED_ON  0
#define LED_OFF 1

void setup();

void txTask();
void updateSensors();

void btnLeft();
void btnRight();
void passKey();

void startHaptics();
void stopHaptics();

#endif // HEXI_BLE_MAIN_H
