import RPi.GPIO as GPIO
import time

MA_PWM1_GPIO = 18 #Motor A forward - RODA DIREITA
MA_PWM2_GPIO = 17 #Motor A backward - RODA DIREITA
MB_PWM1_GPIO = 22 #Motor B forward - RODA ESQUERDA
MB_PWM2_GPIO = 27 #Motor B backward - RODA ESQUERDA

GPIO.setwarnings(False) # disable warnings
GPIO.setmode(GPIO.BCM) # set up BCM GPIO numbering 

GPIO.setup(MA_PWM1_GPIO, GPIO.OUT) # set GPIO as output
GPIO.setup(MA_PWM2_GPIO, GPIO.OUT) # set GPIO as output
GPIO.setup(MB_PWM1_GPIO, GPIO.OUT) # set GPIO as output
GPIO.setup(MB_PWM2_GPIO, GPIO.OUT) # set GPIO as output

FREQ = 50 # PWM frequency (20ms = 50Hz)

#set gpio as PWM output - 50Hz
ma_pwm1 = GPIO.PWM(MA_PWM1_GPIO, FREQ) 
ma_pwm2 = GPIO.PWM(MA_PWM2_GPIO, FREQ)
mb_pwm1 = GPIO.PWM(MB_PWM1_GPIO, FREQ) 
mb_pwm2 = GPIO.PWM(MB_PWM2_GPIO, FREQ)

#start PWM (stopped)
ma_pwm1.start(0)
ma_pwm2.start(0)
mb_pwm1.start(0)
mb_pwm2.start(0)

DT = 100
SLEEP = 0.02

try:
    while True:
        key = raw_input("Insert command: ")
        if key == 'wd' or key == 'dw': #gira direita frente
            for i in range(DT):
                mb_pwm1.ChangeDutyCycle(i)
                time.sleep(SLEEP) 
            
        if key == 'sd' or key == 'ds': #gira direita tras
            for i in range(DT):
                mb_pwm2.ChangeDutyCycle(i)
                time.sleep(SLEEP)

        if key == 'aw' or key == 'wa': #gira esquerda frente
            for i in range(DT):
                ma_pwm1.ChangeDutyCycle(i)
                time.sleep(SLEEP)        
            
        if key == 'sa' or key == 'as': #gira esquerda tras
            for i in range(DT):
                ma_pwm2.ChangeDutyCycle(i)
                time.sleep(SLEEP)

        if key == 'w': # frente
            for i in range(DT):
                ma_pwm1.ChangeDutyCycle(i)
                mb_pwm1.ChangeDutyCycle(i)
                time.sleep(SLEEP)

        if key == 's': # tras
            for i in range(DT):
                ma_pwm2.ChangeDutyCycle(i)
                mb_pwm2.ChangeDutyCycle(i)
                time.sleep(SLEEP)

        if key == 'f':
           ma_pwm1.ChangeDutyCycle(0)
           ma_pwm2.ChangeDutyCycle(0)
           mb_pwm1.ChangeDutyCycle(0)
           mb_pwm2.ChangeDutyCycle(0)            
    
        if key == 'x':  
            ma_pwm1.stop()
            ma_pwm2.stop()
            mb_pwm1.stop()
            mb_pwm2.stop()


except KeyboardInterrupt:
    print "Quit"
    ma_pwm1.stop()
    ma_pwm2.stop()
    mb_pwm1.stop()
    mb_pwm2.stop()
    GPIO.cleanup()
