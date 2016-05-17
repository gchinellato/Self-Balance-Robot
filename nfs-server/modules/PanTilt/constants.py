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

#Position Constrains
POS_MAX = 12.5 # Max position 180 degree
POS_MIN = 2.5 # Min position 0 degree
POS_NEUTRAL = 7.5 # Default position 90 degreee (neutral)

