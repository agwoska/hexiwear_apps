# Hexiwear Kalman Filter Implementation

## About

Implements a Kalman Filter utilizing the on-board
Hexiwear IMU.

A class, KalmanIMU, has been created that uses the
necessary Hexiwear sensors to compute all necessary
updated angle values excluding yaw.

Last updated: December 1, 2021

## Contributions

Kalman Filter implementation by Shardul.

Hexiwear integration and all other implementaions by Andrew.

## Sensors

The following sensors were used:
- Accelerometer and Magnetometer (FSAX21002)
- Gyroscope (FXOS8700)
- KW40Z

## Instructions

**Use MBED OS 5.12!**

If you want print statements on the console, define DEBUG
in the main.h **and** Hexi_Kalman.h files.
Comment those lines out to disable it.

## Licenses

The following licenses were used by libraries:
- GPLv2
