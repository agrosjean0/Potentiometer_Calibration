# Potentiometer_Calibration
The Potentiometer_Calibration library was created to simplify the code for precisely utilizing potentiometers to manually control a pulse width modulated signal from an Arduino REV3. This library gives users control over the range of the output PWM signal from the calibrate function which takes in the lower and upper limits of the PWM signal (given as percentage values 0-100%). The library increases precision by calibrating the potentiometers and gives users control over their PWM outputs through one simple function call.

After compilation, the calibration period is started as indicated by the on-board LED. During this time, the user has 10 seconds to adjust the potentiometer to its greatest position and least position until the on-board LED is no longer lit. This means the potentiometer has been calibrated and the user can move and adjust the PWM output for the LED/Servo using the potentiometer.

Low-level embedded systems programming for Arduino functionality such as timer, ADC, and PWM control is contained in the code of the calibration folder.
