# Full Sensor Implementation 2

## About

Implements an updated version of "Full Sensor Implementation"
under "Hexi_fullsensor" in this repository.
Major improvements have been made to prevent bad
access practices and improve scheduling.
The units and the magnetometer have been implemented or fixed.

## Sensors

The following sensors were used:
- Humidity and Temperature (HTU)
- Accelerometer and Magnetometer (FSAX21002)
- Gyroscope (FXOS8700)
- KW40Z
- Heart Rate Monitor (MAX31010)

## Instructions

**Use MBED OS 5.12!**

Flash compiled binary from the Mbed IDE onto the Hexiwear device and
press the 'reset MK64' buttonon the development board to load/reset.

The left and right buttons on the bottom will switch between sensors.
Going right only will give you the following order which will repeat:
- Temperature and Humidity
- Accelerometer
- Magnetometer
- Gyroscope
- Heart Rate

The up button on the right side disables the OLED display. 
You will need to reset the device to turn it back on.

The down button on the right side will turn the red LED on and off.

Note that the buttons on the left side have no functionality.

## Licensing

All libraries used in this implementation use the licenses:
- Apache v2
- MIT
- GPLv3
- Beerware
