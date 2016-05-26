import RPi.GPIO as GPIO
import time

MA_PWM_GPIO = 17 #Motor A PWM
MB_PWM_GPIO = 27 #Motor B PWM

MA_CLOCKWISE_GPIO = 5
MA_ANTICLOCKWISE_GPIO = 6 

MB_CLOCKWISE_GPIO = 20
MB_ANTICLOCKWISE_GPIO = 21

GPIO.setwarnings(False) # disable warnings
GPIO.setmode(GPIO.BCM) # set up BCM GPIO numbering 

# set GPIO as output
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

#set gpio as PWM output - 50Hz
maPWM = GPIO.PWM(MA_PWM_GPIO, FREQ) 
mbPWM = GPIO.PWM(MB_PWM_GPIO, FREQ)

#start PWM (stopped)
maPWM.start(0)
mbPWM.start(0)

DT = 100
SLEEP = 0.02

def motorMove(pwmA, pwmB):
    #Clockwise
    if pwmA > 0:
        _motorA(direction="CW", pwm=abs(pwmA))
    #Anti-Clockwise
    elif pwmA < 0: 
        _motorA(direction="CCW", pwm=abs(pwmA))
    else:
        _motorStopA()

    #Clockwise
    if pwmB > 0:
        _motorB(direction="CW", pwm=abs(pwmB))
    #Anti-Clockwise
    elif pwmB < 0: 
        _motorB(direction="CCW", pwm=abs(pwmB))
    else:
        _motorStopB()   

def motorStop():
    _motorStopA()
    _motorStopB()

def motorShutDown():
    motorStop()
    maPWM.stop()
    mbPWM.stop()
    GPIO.cleanup()

def _motorA(direction, pwm=0):
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

def _motorStopA():
    GPIO.output(MA_CLOCKWISE_GPIO, False)
    GPIO.output(MA_ANTICLOCKWISE_GPIO, False)
    maPWM.ChangeDutyCycle(0)

def _motorB(direction, pwm=0):
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

def _motorStopB():
    GPIO.output(MB_CLOCKWISE_GPIO, False)
    GPIO.output(MB_ANTICLOCKWISE_GPIO, False)
    mbPWM.ChangeDutyCycle(0)

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
        #main()
        test()

    except KeyboardInterrupt:
        print "Quit"
        motorShutDown()

