/**
 * @file main.h
 * @author Andrew W
 * @date created 11/12/21
 *
 * last updated: 11/16/21
 */

#pragma once

#include "stdint.h"

/** 
 * debug option 
 * comment out if not debugging
 */
// #define DEBUG

#define FIFO_DATA_MAX 288   // for hr


/** declare structures */

/**
 * used to keep track of which screen the device is on
 */
typedef enum {
    HTU_screen,
    ACCEL_screen,
    MAG_screen,
    GYRO_screen,
    HR_screen,
} screens_t;

typedef struct {
    int temp;
    int humid;
    float accel[3];
    float mag[3];
    float gyro[3];
} sensor_data_t;

/* function prototypes */
void oled_printLabel(uint8_t *txt, uint8_t xCrd, uint8_t yCrd);
void oled_printTextBox(uint8_t *txt, ...); // TODO: complete from functs
void oled_clear();

void setup();
void splashScreen();

void systemOFF();
void flipLED();

void btnLeft();
void btnRight();
void hapticON();
void hapticOFF();

void updateScreen();

void getSensors();
void showSensor();

void showHTU();
void showACCEL();
void showMAG();
void showGYRO();
void showHR();

void readHTU();
void readACCEL();
void readMAG();
void readGYRO();

void interHdlr();
void readHR();

void startHaptic();
void stopHaptic();
