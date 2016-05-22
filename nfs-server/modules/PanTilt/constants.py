#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance  
* @Platform: Raspberry PI 2 B+                       
* @Description: PanTilt header file
* @Owner: Guilherme Chinellato 
* @Email: guilhermechinellato@gmail.com                                                
*************************************************
"""

#Define output from servo V and H
SERVO_V_GPIO = 23 #servo vertical move
SERVO_H_GPIO = 24 #servo horizontal move

#PWM frequency (20ms = 50Hz)
FREQ = 50

#Position
POS_MAX = 12.5 # Max position 180 degree
POS_MIN = 2.5 # Min position 0 degree
POS_NEUTRAL = 7.5 # Default position 90 degreee (neutral)

#Position limits
HORIZONTAL_MAX = 8.5
HORIZONTAL_MIN = 4.5

VERTICAL_MAX = 12.5
VERTICAL_MIN = 9.5

ANGLE_MAX = 180.0
ANGLE_MIN = 0.0

#Analog Constrains
ANALOG_MAX = 1.0
ANALOG_MIN = -1.0


