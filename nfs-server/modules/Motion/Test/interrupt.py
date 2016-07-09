#!/usr/bin/python

import RPi.GPIO as GPIO
import time

#Enconders 1 & 2 for each motor
MA_ENCODER_1 = 12
MA_ENCODER_2 = 13
MB_ENCODER_1 = 7
MB_ENCODER_2 = 8

#Initialize global encoders count
countEncoderA = 0
countEncoderB = 0
lastInterruptTime = 0

GPIO.setwarnings(False) # disable warnings
GPIO.setmode(GPIO.BCM) # set up BCM GPIO numbering 

#Set GIPO as input
GPIO.setup(MA_ENCODER_1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) 
GPIO.setup(MA_ENCODER_2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(MB_ENCODER_1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) 
GPIO.setup(MB_ENCODER_2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def _encoderA(MA_ENCODER_1):  
    print "Interrupt Motor A"
    global countEncoderA

    #when the callback function is called due interrup on MA_ENCODER_1 and MA_ENCODER_2 is true, then is clockwise, if not counter clockwise
    if (GPIO.input(MA_ENCODER_2) == True):
        countEncoderA += 1
    else:
        countEncoderA -= 1

def _encoderB(MB_ENCODER_1):  
    print "Interrupt Motor B"
    global countEncoderB

    #when the callback function is called due interrup on MB_ENCODER_1 and MB_ENCODER_2 is true, then is clockwise, if not counter clockwise
    if (GPIO.input(MB_ENCODER_2) == True):
        countEncoderB += 1
    else:
        countEncoderB -= 1

#Set GPIO as interrupt inputs with callback functions
GPIO.add_event_detect(MA_ENCODER_1, GPIO.FALLING, callback=_encoderA, bouncetime=100)
GPIO.add_event_detect(MB_ENCODER_2, GPIO.FALLING, callback=_encoderB, bouncetime=100)

def readEncoderA():
    global countEncoderA
    return countEncoderA

def readEncoderB():
    global countEncoderB
    return countEncoderB

def getWheelsPosition():
    global countEncoderA
    global countEncoderB
    return (countEncoderA + countEncoderB) / 2

def test(): 
    time.sleep(1000)   

def main():
    try:
        while True:
            print readEncoderB()
            
            '''print "read inputs"
            print GPIO.input(MA_ENCODER_1)
            print GPIO.input(MA_ENCODER_2)
            print GPIO.input(MB_ENCODER_1)
            print GPIO.input(MB_ENCODER_2)'''

            time.sleep(0.5)

    except KeyboardInterrupt:
        print "Quit main"

if __name__ == '__main__':
    try:
        main()
        #test()

    except KeyboardInterrupt:
        print "Quit"
        GPIO.cleanup()

