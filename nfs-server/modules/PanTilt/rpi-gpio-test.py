import RPi.GPIO as GPIO
import time

SERVO_V_GPIO = 23 #servo vertical move
SERVO_H_GPIO = 24 #servo horizontal move

POS_MAX = 12.5 # Max position 180 degree
POS_MIN = 2.5 # Min position 0 degree
POS_DEFAULT = 7.5 # Default position 90 degreee (neutral)

FREQ = 50 # PWM frequency

GPIO.setwarnings(False) # disable warnings
GPIO.setmode(GPIO.BCM) # set up BCM GPIO numbering 
GPIO.setup(SERVO_V_GPIO, GPIO.OUT) # set GPIO as output
GPIO.setup(SERVO_H_GPIO, GPIO.OUT) # set GPIO as output

# SERVO
# PERIOD = 20ms (50Hz)
# 
# DT(%)    Time(ms)     Degree
# 2,5       0,5         0
# 5.0       1.0         45   
# 7.5       1.5         90
# 10.0      2.0         135
# 12.5      2.5         180
#

pwm_v = GPIO.PWM(SERVO_V_GPIO, FREQ) #PWM output for f=50Hz / t=20ms 
pwm_v.start(POS_MAX)
pwm_h = GPIO.PWM(SERVO_H_GPIO, FREQ) #PWM output for f=50Hz / t=20ms
pwm_h.start(POS_MAX)

dutyCycleHorizontal = POS_MAX

try:
    while True:
        #scan around myself
        while dutyCycleHorizontal >= POS_MIN:
            print dutyCycleHorizontal
            pwm_h.ChangeDutyCycle(dutyCycleHorizontal)
            time.sleep(0.5) 
            dutyCycleVertical = POS_MAX
            while dutyCycleVertical >= POS_MIN:
                pwm_v.ChangeDutyCycle(dutyCycleVertical)
                time.sleep(0.5)
                dutyCycleVertical -= 2.5  
            dutyCycleHorizontal -= 2.5 

        pwm_v.ChangeDutyCycle(POS_MAX)
        pwm_h.ChangeDutyCycle(POS_MAX)

except KeyboardInterrupt:
    print "Quit"
    pwm_v.stop()
    pwm_h.stop()
    GPIO.cleanup()
