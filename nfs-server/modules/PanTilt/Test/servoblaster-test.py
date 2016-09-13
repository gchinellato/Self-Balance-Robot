#!/usr/bin/python

import os
import time

'''Servo mapping:
     0 on P1-7           GPIO-4
     1 on P1-11          GPIO-17
     2 on P1-12          GPIO-18
     3 on P1-13          GPIO-27
     4 on P1-15          GPIO-22
     5 on P1-16          GPIO-23
     6 on P1-18          GPIO-24
     7 on P1-22          GPIO-25'''

#SERVO PINS
SERVO_H = '2' #pin 12 BCM 18
SERVO_V = '5' #pin 16 BCM 23

''' 100% = 180 degree
    0% = 0 degree'''

#Limits
HORIZONTAL_MAX = 60
HORIZONTAL_MIN = 25

VERTICAL_MAX = 95
VERTICAL_MIN = 60

try: 
    os.system('sudo servod')

    time.sleep(1)

    # ServoBlaster is what we use to control the servo motors
    ServoBlaster = open('/dev/servoblaster', 'w')	

    ServoBlaster.write(SERVO_H + '=' + str(20) + '%' + '\n')
    #ServoBlaster.write(SERVO_H + '=' + str((HORIZONTAL_MAX-HORIZONTAL_MIN)/2) + '%' + '\n')
    ServoBlaster.flush()
    #ServoBlaster.write(SERVO_V + '=' + str(VERTICAL_MAX) + '%' + '\n')
    #ServoBlaster.flush()
    time.sleep(0.1) 

    for i in range(HORIZONTAL_MIN, HORIZONTAL_MAX):
        print "testeee"
        print "Horizontal: " + SERVO_H + '=' + str(i) + '%' + '\n'
        ServoBlaster.write(SERVO_H + '=' + str(i) + '%' + '\n')
        ServoBlaster.flush()
        time.sleep(0.1) 

    time.sleep(1) 

    for i in reversed(range(HORIZONTAL_MIN, HORIZONTAL_MAX)):
        print "Horizontal: " + SERVO_H + '=' + str(i) + '%' + '\n'
        ServoBlaster.write(SERVO_H + '=' + str(i) + '%' + '\n')
        ServoBlaster.flush()
        time.sleep(0.1) 

    time.sleep(1) 

    for i in range(VERTICAL_MIN, VERTICAL_MAX):
        print "Vertical: " + SERVO_V + '=' + str(i) + '%' + '\n'
        ServoBlaster.write(SERVO_V + '=' + str(i) + '%' + '\n')
        ServoBlaster.flush()
        time.sleep(0.1) 

    time.sleep(1)

    for i in reversed(range(VERTICAL_MIN, VERTICAL_MAX)):
        print "Vertical: " + SERVO_V + '=' + str(i) + '%' + '\n'
        ServoBlaster.write(SERVO_V + '=' + str(i) + '%' + '\n')
        ServoBlaster.flush()
        time.sleep(0.1)

    time.sleep(1) 

    #ServoBlaster.write(SERVO_H + '=' + str((HORIZONTAL_MAX-HORIZONTAL_MIN)/2) + '%' + '\n')
    #ServoBlaster.flush()
    #ServoBlaster.write(SERVO_V + '=' + str(VERTICAL_MAX) + '%' + '\n')
    #ServoBlaster.flush()
    time.sleep(0.1) 

    ServoBlaster.close()

    time.sleep(1)

finally:
    os.system('sudo killall servod')
