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

#PWM Constrains
PWM_MAX = 100.0
PWM_MIN = 0.0
COMPENSATION = 12.0 #min value to pwm move the motors

#PID
WINDUP_GUARD = 100.0

#Analog Constrains
ANALOG_MAX = 1.0
ANALOG_MIN = -1.0



