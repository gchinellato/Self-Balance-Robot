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
SERVO_V_GPIO = 18 #servo vertical move
SERVO_H_GPIO = 23 #servo horizontal move

'''Servo mapping:
     0 on P1-7           GPIO-4
     1 on P1-11          GPIO-17
     2 on P1-12          GPIO-18
     3 on P1-13          GPIO-27
     4 on P1-15          GPIO-22
     5 on P1-16          GPIO-23
     6 on P1-18          GPIO-24
     7 on P1-22          GPIO-25'''

#Servo pins
SERVO_H = '2' #pin 12 BCM 18 - servo horizontal
SERVO_V = '5' #pin 16 BCM 23 - servo vertical

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


