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
Arduino
4x encoder (INT0-D2:32, INT1-D3:1, D4:2. D7:11) 
4x motor enable (D9:13, D10:14, D11:15, D12:16)
2x PWM (D5: 9, D6: 10)
2x I2C (SCL-A5:28, SDA-A4:27)
2x UART (TX:31, RX:30)
"""

#
#Motors GPIOs
#

#Motor A & B PWM outputs (BCM pinout)
#MA_PWM_GPIO = 19
#MB_PWM_GPIO = 26

#Motor A & B enable outputs (BCM pinout)
#MA_CLOCKWISE_GPIO = 5
#MA_ANTICLOCKWISE_GPIO = 6 
#MB_CLOCKWISE_GPIO = 20
#MB_ANTICLOCKWISE_GPIO = 21

#
#Encoders GPIOs
#

#Enconders 1 & 2 for each motor (BCM pinout)
#MA_ENCODER_1 = 12
#MA_ENCODER_2 = 13
#MB_ENCODER_1 = 7
#MB_ENCODER_2 = 8

#
#PanTilt GPIOs
#

#MicroServo Vertical and Horizontal outputs (BCM pinout)
SERVO_V_GPIO = 18
SERVO_H_GPIO = 23

'''Servo mapping for servoblaster:
     0 on P1-7      GPIO-4
     1 on P1-11     GPIO-17
    *2 on P1-12     GPIO-18*
     3 on P1-13     GPIO-27
     4 on P1-15     GPIO-22
    *5 on P1-16     GPIO-23*
     6 on P1-18     GPIO-24
     7 on P1-22     GPIO-25'''

#Servo pins
SERVO_H = '2' #pin 12 BCM 18
SERVO_V = '5' #pin 16 BCM 23


