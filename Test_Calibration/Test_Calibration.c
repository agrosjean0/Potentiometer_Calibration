//**************************************************************************
// Test_Calibration.c
// Author: Alecea Grosjean 
// Date: 3/11/2024
// Description: Code that can be uploaded to an Arduino Uno that utilizes
// the position of a linear potentiometer to provide precision control 
// of other devices such as LED brightness, servo position, and piezo volume.
// Copyright (c) 2024
//*************************************************************************

#define __DELAY_BACKWARD_COMPATIBLE__
#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#include "Calibration/potentiometer_calibration.h"

int main (void) {
    calibrate(0, 100);   // use a library function
    return 0; 
}
