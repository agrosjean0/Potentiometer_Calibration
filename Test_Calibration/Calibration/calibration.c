/**************************************************************************
* calibration.c
* Author: Alecea Grosjean 
* Date: 3/26/2024
* Description: C file for the Calibration library that assists in 
* calibrating linear potentiometers.
* Copyright (c) 2024
***************************************************************************/
#define __DELAY_BACKWARD_COMPATIBLE__
#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#include "potentiometer_calibration.h"

/************************* Variable Declarations ****************************/
// Sensor Values for Mapping
uint16_t SENSOR_VALUE = 0;      // the sensor value
uint16_t SENSOR_MIN = 1023;     // minimum sensor value (10 bit ADC on Atmega 328p)
uint16_t SENSOR_MAX = 0;        // maximum sensor value

// Output values from User
uint16_t PWM_MIN;
uint16_t PWM_MAX;

uint16_t pwm_lower_limit; 
uint16_t pwm_upper_limit;

// Counter Varaibles for Calibration Period
uint16_t sec_cntr;              // Counter for 5 seconds
bool calibration_complete = 0;  // Calibration period flag, 1 when  complete
/**************************************************************************/

/************************* Function Prototypes ****************************/
void io_setup();
void ADC_setup(bool mux3, bool mux2, bool mux1, bool mux0);
void pwm_setup();
uint16_t map(uint16_t sensorValue, uint16_t sensorMin, uint16_t sensorMax, uint16_t limitMin, uint16_t limitMax);
uint16_t constrain(uint16_t sensorValue, uint16_t sensorMin, uint16_t sensorMax);
void calibrate(uint8_t PWM_MIN, uint8_t PWM_MAX);
/**************************************************************************/


/****************************** Functions *********************************/
/* The calibration function can be called from other files utilizing the 
* calibration library to map/calibrate the values of a linear potentiometer 
* to useful values of a PWM signal to control other peripherals */
void calibrate(uint8_t PWM_MIN, uint8_t PWM_MAX) 
{
    // Setup
    io_setup();
    ADC_setup(MUX_3, MUX_2, MUX_1, MUX_0);
    pwm_setup();

    // Convert % values of PWM_MIN and PWM_MAX to 16 bit value
    pwm_lower_limit = floor(65535 * (PWM_MIN / 10.0));
    pwm_upper_limit = floor(65535 * (PWM_MAX / 10.0));
     
    // Turn on on-board LED to signal the start of the calibration period
    CALIB_PERIOD_PORT |= (1 << CALIB_PERIOD_PIN);

    /* Timer Setup for Interrupt after 10 millisecond - do this 1000 times for calibration period */
    // This uses Timer 0 so 8 bit timer
    TCNT0 = 0;
    TCCR0B = (1 << CS00) | (1 << CS02); // set the pre-scalar as 1024
	OCR0A = 156; 	   // 10 ms delay

    // This is the calibration period to identify the min and max values of the potentiometer
    while(!calibration_complete) 
    {
        //If flag is set toggle the led	
        while((TIFR0 & (1<<OCF0A)) == 0); // wait till the timer overflow flag is SET
        sec_cntr++;
        // This means 10,000 ms aka 10 seconds has past
        if (sec_cntr >= 1000) 
        {
            // Turn off on board led after 5 seconds
            CALIB_PERIOD_PORT ^= (1 << CALIB_PERIOD_PIN);
            calibration_complete = 1; // break loop
        } else 
        {
            // Wait for ADC conversion to complete
            while (!(ADCSRA & (1 << ADIF)));
            // Read ADC result from ADC register (10-bit resolution)
            SENSOR_VALUE = ADC;

            // record the maximum sensor value
            if (SENSOR_VALUE > SENSOR_MAX) 
            {
                SENSOR_MAX = SENSOR_VALUE;
            }

            // record the minimum sensor value
            if (SENSOR_VALUE < SENSOR_MIN) 
            {
                SENSOR_MIN = SENSOR_VALUE;
            } 

            TCNT0 = 0; 
            // Reset timer overflow flag 
            TIFR0 |= (1 << OCF0A); 

            // Reset ADC interrupt flag
            ADCSRA |= (1 << ADIF);
        }
    }

    /* Loop for mapping and constraining new calibration to user values */
    while (1) 
    {
        // Wait for ADC conversion to complete
        while (!(ADCSRA & (1 << ADIF)));
        // Read ADC result from ADC register (10-bit resolution)
        SENSOR_VALUE = ADC;

        // Map the sensor reading to user values
        SENSOR_VALUE = map(SENSOR_VALUE, SENSOR_MIN, SENSOR_MAX, pwm_lower_limit, pwm_upper_limit);
        // Constrain the value in case the sensor value is outside the range seen during calibration
        SENSOR_VALUE = constrain(SENSOR_VALUE, pwm_lower_limit, pwm_upper_limit);
        
        // fade the LED using the calibrated value:
        // PIN 10 - OCR1B, output pin
	    OCR1B = SENSOR_VALUE;
        // Reset ADC interrupt flag
        ADCSRA |= (1 << ADIF);
    }
}

/* Setup ports as I/O */
void io_setup() { 
    // Set the PWM PWM_PIN as output
    PWM_DDR |= (1 << PWM_PIN);
    // Set the SENSOR_PIN as input
    SENSOR_DDR &= ~(1 << SENSOR_PIN);
    // Set the CALIB_PERIOD_PIN as output
    CALIB_PERIOD_DDR |= (1 << CALIB_PERIOD_PIN);
}

/* Setup the ADC so that the analog potentiometer values can be read */
void ADC_setup(bool mux3, bool mux2, bool mux1, bool mux0) {
    /*
    The ADMUX register is the ADC Multiplexer selection register.  
    - The reference selection bits selects the reference voltage used (bit7:6 - REFS1 and REFS0)
        For this I used the onboard 5V AVcc which is code 01, aka REFS0 is 1. 
    - ADLAR is 0, aka right justified the default of location of data bits in the ADC Data Register
    */
    ADMUX |= (1 << REFS0);

    /*
    The Analog channel selection bits are set in the Pin_Assignments.h
    file and the logic to set/clear these bits is here. (bit 3:0 - MUX3-0)
    */
    if (mux3) 
    {
        ADMUX |= (1 << MUX3);  // Set the bit
    } else {
        ADMUX &= ~(1 << MUX3); // Clear the bit
    }
    if (mux2) 
    {
        ADMUX |= (1 << MUX2);  // Set the bit
    } else {
        ADMUX &= ~(1 << MUX2); // Clear the bit
    }
    if (mux1) 
    {
        ADMUX |= (1 << MUX1);  // Set the bit
    } else {
        ADMUX &= ~(1 << MUX1); // Clear the bit
    }
    if (mux0) 
    {
        ADMUX |= (1 << MUX0);  // Set the bit
    } else {
        ADMUX &= ~(1 << MUX0); // Clear the bit
    }

    /*
    The ADCSRA register is the control and status register for the ADC. 
    - The ADC must be enabled (bit 7 - ADEN)
    - The ADC prescaler makes the ADC run based on the clock (bit 2:0 - ADPS) 
        For this, I used the max 128 division which is 111
    - The ADATE is the auto trigger enable bit, setting this to 1 means that the 
      the trigger source in the ADCSRB register. The default setup here is 
      Free running mode which means continuous analog input conversion without trigger (ADTS2:0 = 0)
    - The ADSC bit is set to start the ADC Conversion
    */
    ADCSRA = (1 << ADEN) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2) | (1 << ADATE) | (1 << ADSC);
}

/* This is the setup for Timer 1 - 16 bit timer */
void pwm_setup()
{
	// TIMER 1 - 16 bit
	// OC1A and OC1B synced
	
    // set TOP to 16bit
	ICR1 = 0xFFFF;

	OCR1A = 0x0000;
	OCR1B = 0x0000;

	// set none-inverting mode
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1);
	// set Fast PWM mode using ICR1 as TOP - MODE 14
	TCCR1A |= (1 << WGM11);
	TCCR1B |= (1 << WGM13) | (1 << WGM12);
    
	// START the timer with no prescaler
	TCCR1B |= (1 << CS10);
}

/* Maps the sensor value from the potentiometer to the limits set by the user */
uint16_t map(uint16_t sensorValue, uint16_t sensorMin, uint16_t sensorMax, uint16_t limitMin, uint16_t limitMax)
{
    uint32_t numerator = (uint32_t)(sensorValue - sensorMin) * (uint32_t)(limitMax - limitMin);
    return (uint16_t)(((numerator) / (sensorMax - sensorMin)) + limitMin);
}

/* constrains the sensor value from the potentiometer to the limits set by the user */
uint16_t constrain(uint16_t sensorValue, uint16_t sensorMin, uint16_t sensorMax)
{
    if(sensorValue < sensorMin) 
    {
        return sensorMin;
    } else if (sensorValue > sensorMax) 
    {
        return sensorMax;
    } else 
    {
        return sensorValue;
    }
}
