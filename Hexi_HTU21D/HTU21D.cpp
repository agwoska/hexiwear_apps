/**
 * @file HTU21D.cpp
 * @author Andrew Woska (agwoska@buffalo.edu)
 * @date Jan 02, 2022
 * @brief implementation of the HTU21D for
 *      Hexiwear development in Mbed OS 5.12
 * 
 * documentation: https://cdn-shop.adafruit.com/datasheets/1899_HTU21D.pdf
 * 
 * last updated: Jan 03, 2022
 */

#include "HTU21D.h"

HTU21D::HTU21D(PinName sda, PinName scl):
    i2c(sda, scl) {
        i2c.frequency(HTU21D_FREQ);
    }

HTU21D::HTU21D() :
    i2c(PTB1, PTB0) {
        i2c.frequency(HTU21D_FREQ);
    };

HTU21D::~HTU21D() {
    delete &i2c;
}

int HTU21D::getTemp() {
    char tx[1];
    char rx[2];

    // send temperature trigger
    tx[0] = HTU21D_TRIG_TEMP;
    i2c.write((HTU21D_I2C_ADDR << 1) & HTU21D_SOFT_RESET, tx, 1);

    ThisThread::sleep_for(50); // 50ms (max time req pg. 5)

    // read measurement
    i2c.read((HTU21D_I2C_ADDR << 1) | 1, rx, 2);
    ThisThread::sleep_for(1);

    // compute temp (pg. 15)
    unsigned int rawTemp = ((unsigned int)rx[0] << 8) | (unsigned int)rx[1];
    rawTemp &= 0xFFFC; // remove status bits
    float actTemp = -46.85 + 175.72 * rawTemp / (float)(1 << 16);
    return (int)actTemp;

}

int HTU21D::getHumidity() {
    char tx[1];
    char rx[2];

    // send temperature trigger
    tx[0] = HTU21D_TRIG_HUMD;
    i2c.write((HTU21D_I2C_ADDR << 1) & HTU21D_SOFT_RESET, tx, 1);
    ThisThread::sleep_for(16); // (max time req pg. 3)

    // read measurement
    i2c.read((HTU21D_I2C_ADDR << 1) | 1, rx, 2);
    ThisThread::sleep_for(1);

    // compute temp (pg. 15)
    unsigned int rawHumd = ((unsigned int)rx[0] << 8) | (unsigned int)rx[1];
    rawHumd &= 0xFFFC; // remove status bits
    return -6 + 125 * rawHumd / (1 << 16);

}
