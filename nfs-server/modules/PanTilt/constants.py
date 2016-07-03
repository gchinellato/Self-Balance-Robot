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

#Position
POS_MAX = 100 # Max position 180 degree
POS_MIN = 0 # Min position 0 degree
POS_NEUTRAL = 50 # Default position 90 degreee (neutral)

#Position limits (in percentage)
HORIZONTAL_MAX = 60
HORIZONTAL_MIN = 25

VERTICAL_MAX = 95
VERTICAL_MIN = 60

#Angle limits
ANGLE_MAX = 180.0
ANGLE_MIN = 0.0

#Analog Constrains
ANALOG_MAX = 1.0
ANALOG_MIN = -1.0


