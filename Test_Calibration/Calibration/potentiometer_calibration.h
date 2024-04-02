/**************************************************************************
* potentiometer_calibration.h
* Author: Alecea Grosjean 
* Date: 3/26/2024
* Description: Header file for the Calibration library that assists in 
* calibrating linear potentiometers.
* Copyright (c) 2024
***************************************************************************/
#define __DELAY_BACKWARD_COMPATIBLE__
#ifndef _POTENTIOMETER_CALIBRATION_H_
#define _POTENTIOMETER_CALIBRATION_H_

/***************************** Headers ************************************/ 
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include <math.h>
#include "Pin_Assignments.h"
/**************************************************************************/

/*********************** Variable Declarations ****************************/
// On board LED ouput to indicate calibration period 
#define CALIB_PERIOD_PIN PB5    // Pin 13
#define CALIB_PERIOD_DDR DDRB
#define CALIB_PERIOD_PORT PORTB
/**************************************************************************/

/*********************** Function Declarations ****************************/
/* 
* The io_setup function sets up the pins declared in the Pin_Assignments.h
* file as inputs or outputs. This includes the PWM pin as output potentiometer
* pin as input and the onboard LED as an output to track the calibration period. 
*/
void io_setup();

/* 
* mux3-mux0: Each of these represent the logic to set or clear a bit of the 
* ADMUX register for reading ADC values. 0 means the bit is cleared, 1 means
* the bit will be set. These can be changed in the Pin_Assignments.h file.
*
* The ADC_setup function assigns values to the internal registers of the 
* Atmega328p for reading analog values using the onboard Analog to digital
* converter of the Arduino uno.  
*/
void ADC_setup(bool mux3, bool mux2, bool mux1, bool mux0);

/*
* The pwm_setup function sets up Timer 1, the 16 bit timer for pwm 
* signal output controlled by the potentiometer value.
*/
void pwm_setup();

/*
* sensorValue: the current 10 bit analog value read from the potentiometer
* sensorMin: the maximum analog value read from the potentiometer over the calibration period
* sensorMax: the minimum analog value read from the potentiometer over the calibration period
* limitMin: the minimum value of the new range of values for the sensorValue to be mapped to 
* limitMax: the maximum value of the new range of values for the sensorValue to be mapped to
* Returns: the value of sensorValue mapped within the new range
* The map function takes the current value read from the potentiometer and maps it to its 
* equivalent value within the new limits of the output pwm set by the user.
*/
uint16_t map(uint16_t sensorValue, uint16_t sensorMin, uint16_t sensorMax, uint16_t limitMin, uint16_t limitMax);

/*
* sensorValue: the current 10 bit analog value read from the potentiometer
* sensorMin: the minimum value of the new range of values for the sensorValue to be constrained to
* sensorMax: the maximum value of the new range of values for the sensorValue to be constrained to
* Returns: the value of sensorValue constrained within the sensorMax and sensorMin values
* The constrain function takes the current value read from the potentiometer and keeps it within 
* constraints of the output pwm set by the user.
*/
uint16_t constrain(uint16_t sensorValue, uint16_t sensorMin, uint16_t sensorMax);

/*
* PWM_MIN: the minimum value of the new range of values for the sensorValue set by the user
* PWM_MAX: the maximum value of the new range of values for the sensorValue set by the user
* Returns: 0 once run is complete.
*/
void calibrate(uint8_t PWM_MIN, uint8_t PWM_MAX);
/**************************************************************************/
#endif