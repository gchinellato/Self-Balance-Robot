#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance
* @Description: GPIO Mapping
* @Owner: Guilherme Chinellato
* @Email: guilhermechinellato@gmail.com
*************************************************
"""

"""
#
#Arduino GPIO
#

4x encoder (INT0-D2, INT1-D3, D4, D7)
4x motor enable (D5, D6, D11, D12)
2x PWM (D9, D10)
2x I2C (SCL-A5, SDA-A4)
"""

'''
Deprecated (replaced to Arduino)
#
#Motors GPIOs
#

#Motor A & B PWM outputs (BCM pinout)
MA_PWM_GPIO = 19
MB_PWM_GPIO = 26

#Motor A & B enable outputs (BCM pinout)
MA_CLOCKWISE_GPIO = 5
MA_ANTICLOCKWISE_GPIO = 6
MB_CLOCKWISE_GPIO = 20
MB_ANTICLOCKWISE_GPIO = 21

#
#Encoders GPIOs
#

#Enconders 1 & 2 for each motor (BCM pinout)
MA_ENCODER_1 = 12
MA_ENCODER_2 = 13
MB_ENCODER_1 = 7
MB_ENCODER_2 = 8
'''

#
#PanTilt GPIOs
#

#MicroServo Vertical and Horizontal outputs (BCM pinout)
#SERVO_V_GPIO = 4
#SERVO_H_GPIO = 17

#Servo pins
SERVO_H = '0'
SERVO_V = '1'

'''Servo mapping for servoblaster:
     0 on P1-7      GPIO-4
     1 on P1-11     GPIO-17
     2 on P1-12     GPIO-18
     3 on P1-13     GPIO-27
     4 on P1-15     GPIO-22
     5 on P1-16     GPIO-23
     6 on P1-18     GPIO-24
     7 on P1-22     GPIO-25'''
