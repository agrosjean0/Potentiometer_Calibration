/**************************************************************************
* Pin_Assignments.h
* Author: Alecea Grosjean 
* Date: 3/26/2024
* Description: Header file for the pin assignments of the potentiometer 
* signal input and PWM output signal. If different pins are used, update 
* the following values, just make sure the PWM pin variable is one of the 
* PWM pins on the Arduino board being used.
* Copyright (c) 2024
***************************************************************************/
#ifndef Pin_Assignments_H_INCLUDED
#define Pin_Assignments_H_INCLUDED

/*************************** Configuration ********************************/
// Potentiometer Input
/* Note, if SENSOR_PIN is changed from PC0, MUX3-MUX0 of the ADMUX register
must be changed (0 or 1) to reflect the change in analog pin used for the 
potentiometer. Use ATmega328P_Datasheet page 217 for help */
#define SENSOR_PIN PC0          // Pin Analog 0
#define SENSOR_DDR DDRC         // Analog is DDRC
#define SENSOR_PORT PORTC       // Analog pins are portc
#define MUX_3   0
#define MUX_2   0
#define MUX_1   0
#define MUX_0   0

// PWM output controlled by Potentiometer
#define PWM_PIN DDB2            // Pin 10 
#define PWM_DDR DDRB            // Pin 10 is DDRB
/**************************************************************************/
#endif