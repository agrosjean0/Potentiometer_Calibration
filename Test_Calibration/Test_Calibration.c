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
    // int8_t pwm_pin;
    // printf("Enter the PWM pin port connected to the system being controlled by the potentiometer: ");
    // scanf("%d", &pwm_pin);
    // while (pwm_pin != 3 | pwm_pin != 5 | pwm_pin != 6 | pwm_pin != 9 | pwm_pin != 10 | pwm_pin != 11) {
    //     printf("The value entered is not a pwm pin on an arduino uno (please enter a value of 3,5,6,9,10, or 11)");
    //     printf("Enter the PWM pin port connected to the system being controlled by the potentiometer: ");
    //     scanf("%d", &pwm_pin);
    // }
    calibrate(0, 255);   // use a library function
    return 0; 
}
