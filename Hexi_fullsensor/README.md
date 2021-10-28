# Full Sensor Implementation

## About

Implements Hexiwear utilizing all of its sensors (except light).

## BOM

- Hexiwear device and development board
- Mbed account and IDE

## Sensors

The sensors present on Hexiwear are:
- Humidity and Temperature (HTU)
- Accelerometer and Magnometer (FXOS)
- Gyroscope (FX8700)
- KW40
- Heart Rate Monitor (MAX31010)
- Light Sensor (not used)

## Instructions

From Mbed, either copy the bin file to the device or click the run button
present on the Mbed IDE. Ensure you are using Mbed OS 5.12.
All other versions have deprecation issues or will not load
at all.

Click the 'reset MK64' on the development board to reload all
settings and take the Hexiwear off while it is powered.

The left and right buttons on the bottom will switch between the sensors
present on the device. Note that no two sensors will run at the same time
for purposes of power and personal control (by me).

(There is currently an exception to this rule as the heart rate monitor
currently does not turn off as I do not know how to.)

The up button on the right side turns off the entire system. To turn
it back on, you need to press the reset on the development board
with the device plugged into its stop.

The down button on the right side turns on and off the red LED
present on the device.

The up and down buttons present on the left side of the device
do nothing as the device can only control either the left _or_
right side buttons. The default is right so I kept it that way.

## Licensing

All libraries used in this implementation use the licenses:
- Apache v2
- GPLv3
- Beerware
