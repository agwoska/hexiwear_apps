# Wristesense: Steady Arm Activity

Note that this document is not official or complete,
it is simply a general reference for developers
in the Embedded Sensing and Computing Lab under
Dr. Wenyao Xu.

Created by Andrew and Shardul

## Overview

This activity is used as a Proof-of-Concept for the
Wristsense project. 
This application implements an activity that 
will track the user's arm orientation to make sure 
they keep their arm steady.

There are two cases defined in the [poster](https://woska.net/papers/SURC22_poster.pdf)
about this project.

Your hand level to ground is considered the ideal state for case 1.
Your hand 90 degrees offset in the position of holding a bottle
is considered the ideal state for case 2.
Anything exceeding 10 degrees will be considered
non-ideal and will count for negative feedback to the user.

The feedback present in the system will include 
haptic (or vibrational) and visual feedback.
This feedback will increase until the maximum negative feedback
is reached.
A 90 degree offset from the base of the case is where the maximum feedback is reached.
Audio feedback will be implemented in a future version
to simulate rythimic stimulation to the user.

This project uses the Hexiwear smartwatch with 
the Mbed OS 5 platform.
This application was used for SURC
(SUNY Undergraduate Research Conference) in April 2022.

## Summary

This application supplies the user with activities
to test the stability of a user's arm movement.
The task tests if the user's arm is steady.
If the user's arm is not steady, the haptic feedback
will trigger a lower response. 
If steady, the haptic feedback will trigger a fast response.

## Libraries

All libraries with an asterisk next to them are libraries that
I, Andrew, have personally created. This being the case, there are no licenses currently attached to them. Links to the library can be found in [hexi_libs.txt](../hexi_libs.txt).

- FXAS21002
- FXOS8700
- Kalman (Hexi_Kalman) *
- KalmanFilter
- KW40Z (Hexi_KW40Z)
- SSD1351 (Hexi_OLED_SSD1351)

## Licensing

The Kalman Filter library utilizes the GPLv2 license.
All other libraries utilize a modified MIT license with
extra copyright protections.
Respect the licenses and their copyright.
