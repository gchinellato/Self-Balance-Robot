#!/usr/bin/python

import RPi.GPIO as GPIO
import time

MA_PWM_GPIO = 19 #Motor A PWM
MB_PWM_GPIO = 26 #Motor B PWM

MA_CLOCKWISE_GPIO = 5
MA_ANTICLOCKWISE_GPIO = 6 

MB_CLOCKWISE_GPIO = 20
MB_ANTICLOCKWISE_GPIO = 21

#Enconders 1 & 2 for each motor
MA_ENCODER_1 = 23
MA_ENCODER_2 = 13
MB_ENCODER_1 = 17
MB_ENCODER_2 = 8

GPIO.setwarnings(False) # disable warnings
GPIO.setmode(GPIO.BCM) # set up BCM GPIO numbering 

#set GPIO as output
GPIO.setup(MA_PWM_GPIO, GPIO.OUT) 
GPIO.setup(MB_PWM_GPIO, GPIO.OUT)
GPIO.setup(MA_CLOCKWISE_GPIO, GPIO.OUT)
GPIO.setup(MA_ANTICLOCKWISE_GPIO, GPIO.OUT)
GPIO.setup(MB_CLOCKWISE_GPIO, GPIO.OUT)
GPIO.setup(MB_ANTICLOCKWISE_GPIO, GPIO.OUT)

GPIO.output(MA_CLOCKWISE_GPIO, False)
GPIO.output(MA_ANTICLOCKWISE_GPIO, False)
GPIO.output(MB_CLOCKWISE_GPIO, False)
GPIO.output(MB_ANTICLOCKWISE_GPIO, False)

FREQ = 50 # PWM frequency (20ms = 50Hz)

#set gpio as PWM output
maPWM = GPIO.PWM(MA_PWM_GPIO, FREQ) 
mbPWM = GPIO.PWM(MB_PWM_GPIO, FREQ)

#start PWM (stopped)
maPWM.start(0)
mbPWM.start(0)

#Set GIPO as input
GPIO.setup(MA_ENCODER_1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) 
GPIO.setup(MA_ENCODER_2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(MB_ENCODER_1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) 
GPIO.setup(MB_ENCODER_2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def _encoderA(MA_ENCODER_1):  
    #when the callback is called due interrup on MA_ENCODER_1 and MA_ENCODER_2 is true, then is clockwise, if not counter clockwise
    print "interrupt"
    if (GPIO.input(MA_ENCODER_2 == True)):
        self.countEncoderA += 1
    else:
        self.countEncoderA -= 1

def _encoderB(MB_ENCODER_1): 
    print "interrupt" 
    if (GPIO.input(MB_ENCODER_2 == True)):
        countEncoderB += 1
    else:
        countEncoderB -= 1

#Set GPIO as interrupt inputs with callback functions
GPIO.add_event_detect(MA_ENCODER_1, GPIO.FALLING, callback=_encoderA)
GPIO.add_event_detect(MB_ENCODER_1, GPIO.FALLING, callback=_encoderB)

#Initialize encoders count
countEncoderA = 0
countEncoderB = 0


DT = 20
SLEEP = 0.1

def motorMove(pwmA, pwmB):
    #Clockwise
    if pwmA > 0:
        _motorA(direction="CW", pwm=abs(pwmA))
    #Anti-Clockwise
    elif pwmA < 0: 
        _motorA(direction="CCW", pwm=abs(pwmA))
    else:
        _motorA()

    #Clockwise
    if pwmB > 0:
        _motorB(direction="CW", pwm=abs(pwmB))
    #Anti-Clockwise
    elif pwmB < 0: 
        _motorB(direction="CCW", pwm=abs(pwmB))
    else:
        _motorB()   

def motorStop():
    motorMove(0, 0)

def motorShutDown():
    motorStop()
    maPWM.stop()
    mbPWM.stop()
    GPIO.cleanup()

def _motorA(direction="", pwm=0):
    #print "Motor A: " + str(direction) + " ,pwm: " + str(pwm)
    if direction == "CW":
        GPIO.output(MA_CLOCKWISE_GPIO, True)
        GPIO.output(MA_ANTICLOCKWISE_GPIO, False)
    elif direction == "CCW":
        GPIO.output(MA_CLOCKWISE_GPIO, False)
        GPIO.output(MA_ANTICLOCKWISE_GPIO, True)
    else:
        GPIO.output(MA_CLOCKWISE_GPIO, False)
        GPIO.output(MA_ANTICLOCKWISE_GPIO, False)        
    maPWM.ChangeDutyCycle(pwm)

def _motorB(direction="", pwm=0):
    #print "Motor A: " + str(direction) + " ,pwm: " + str(pwm)
    if direction == "CW":
        GPIO.output(MB_CLOCKWISE_GPIO, True)
        GPIO.output(MB_ANTICLOCKWISE_GPIO, False)
    elif direction == "CCW":
        GPIO.output(MB_CLOCKWISE_GPIO, False)
        GPIO.output(MB_ANTICLOCKWISE_GPIO, True)
    else:
        GPIO.output(MB_CLOCKWISE_GPIO, False)
        GPIO.output(MB_ANTICLOCKWISE_GPIO, False)        
    mbPWM.ChangeDutyCycle(pwm)

def readEncoderA():
    return countEncoderA

def readEncoderB():
    return countEncoderB

def getWheelsPosition():
    return (countEncoderA + countEncoderB) / 2

def test():    
    GPIO.output(MA_CLOCKWISE_GPIO, True)
    GPIO.output(MA_ANTICLOCKWISE_GPIO, False)
    GPIO.output(MB_CLOCKWISE_GPIO, True)
    GPIO.output(MB_ANTICLOCKWISE_GPIO, False)
    maPWM.ChangeDutyCycle(100)  
    mbPWM.ChangeDutyCycle(100) 
    time.sleep(1000)   

def main():
    try:
        while True: 
            print "wd: gira direita frente"
            print "sd: gira direita tras"
            print "aw: gira esquerda frente"
            print "sa: gira esquerda tras"
            print "w: frente"
            print "s: tras"
            print "f: stop"

            key = raw_input("Insert command: ")
            if key == 'wd' or key == 'dw': #gira direita frente                
                for i in range(DT):
                    motorMove(i, 0)
                    time.sleep(SLEEP) 
                
            if key == 'sd' or key == 'ds': #gira direita tras
                for i in range(DT):
                    motorMove(-i, 0)
                    time.sleep(SLEEP)

            if key == 'aw' or key == 'wa': #gira esquerda frente
                for i in range(DT):
                    motorMove(0, i)
                    time.sleep(SLEEP)        
                
            if key == 'sa' or key == 'as': #gira esquerda tras
                for i in range(DT):
                    motorMove(0, -i)
                    time.sleep(SLEEP)

            if key == 'w': # frente
                for i in range(DT):
                    motorMove(i, i)
                    print "Value: " + str(i)
                    time.sleep(SLEEP)

            if key == 's': # tras
                for i in range(DT):
                    motorMove(-i, -i)
                    time.sleep(SLEEP)

            if key == 'f':
                motorStop()           
        
            if key == 'x':  
                motorShutDown()

    except KeyboardInterrupt:
        print "Quit"
        motorShutDown()

if __name__ == '__main__':
    try:
        main()
        #test()

    except KeyboardInterrupt:
        print "Quit"
        motorShutDown()

