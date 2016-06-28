#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance  
* @Platform: Raspberry PI 2 B+                       
* @Description: Motion header file
                DC Motor with gearbox
                Motor driver L298N
* @Owner: Guilherme Chinellato 
* @Email: guilhermechinellato@gmail.com                                                
*************************************************
"""

#Define output for motors A & B
MA_PWM_GPIO = 17 #Motor A PWM
MB_PWM_GPIO = 27 #Motor B PWM

MA_CLOCKWISE_GPIO = 5
MA_ANTICLOCKWISE_GPIO = 6 

MB_CLOCKWISE_GPIO = 20
MB_ANTICLOCKWISE_GPIO = 21

#PWM frequency (20ms = 50)
PWM_FREQ = 50

#PWM Constrains
PWM_MAX = 100.0
PWM_MIN = 0.0
COMPENSATION = 12.0 #min value to pwm move the motors

#PID
WINDUP_GUARD = 100.0

#Analog Constrains
ANALOG_MAX = 1.0
ANALOG_MIN = -1.0



