#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance  
* @Platform: Raspberry PI 2 B+                       
* @Description: Motion header file
*               DC Motor with gearbox/encoder
*               Motor driver VNH2SP30
* @Owner: Guilherme Chinellato 
* @Email: guilhermechinellato@gmail.com                                                
*************************************************
"""

#PWM frequency 
PWM_FREQ = 100

#PWM Constrains
PWM_MAX = 100.0
PWM_MIN = 0.0
COMPENSATION_A = 0.0
COMPENSATION_B = 0.0

#Analog Constrains
ANALOG_MAX = 1.0
ANALOG_MIN = -1.0

#PID Parameters
WINDUP_GUARD = 100.0

SPEED_SETPOINT = 0.0
SPEED_KP = 0.02#0.85
SPEED_KI = 0.0#0.1
SPEED_KD = 0.0#0.0
ANGLE_SETPOINT = -1.0
ANGLE_LIMIT = 5.0
ANGLE_KP_AGGR = 10.5#15.0
ANGLE_KI_AGGR = 0.8#0.2
ANGLE_KD_AGGR = 0.3#0.7
ANGLE_KP_CONS = 6.0#10.0
ANGLE_KI_CONS = 0.7#0.0
ANGLE_KD_CONS = 0.3#0.2
ANGLE_IRRECOVERABLE = 30.0

PID_CONSERVATIVE = 0
PID_AGGRESSIVE = 1

#Wheel radius (cm)
WHEEL_RADIUS = 6.0

#64 of motor shaft * 29 (29:1) gearbox = 1856 counter per revolution with gearbox when both A and B channels and rising and falling edges
#16 of motor shaft * 29 (29:1) gearbox = 464 counter per revolution with gearbox when only one channel either rising or falling edges
TICKS_PER_TURN = 464 





