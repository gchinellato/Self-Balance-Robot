#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance                       
* @Description: Serial API header file
* @Owner: Guilherme Chinellato 
* @Email: guilhermechinellato@gmail.com                                                
*************************************************
"""

TRACE_BEGIN = "#BEGIN#"
TRACE_END = "#END#"

STARTED = 0
DIRECTION = 1
STEERING = 2
SPEED_PID = 3
ANGLE_PID_AGGR = 4
ANGLE_PID_CONS = 5
CALIBRATED_ZERO_ANGLE = 6
ANGLE_LIMIT = 7

#Analog Constrains
ANALOG_MAX = 1.0
ANALOG_MIN = -1.0

#Position limits (in percentage)
PWM_MAX = 100
PWM_MIN = -100
