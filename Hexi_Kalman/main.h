/**
 * @file main.h
 * @author Andrew W
 * @date created 11/17/21
 * 
 * last updated 11/18/21
 */

/** Constants, Definitions, and Debugging */
/* Comment out below if wanted */
#define DEBUG
// #define RESTRICT_PITCH

#define PI          (3.14159265)
#define RAD_TO_DEG  (180.0 / PI)    // approx. 57.29578

/** Function Prototypes */

void setup();

void setKalman();

double calcRoll();
double calcPitch();
double calcYaw();

void showKalman();
void showRoll();
void showPitch();
void showYaw();
