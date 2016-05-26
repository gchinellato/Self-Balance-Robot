#!/usr/bin/python

import wiringpi
import time

#GPIO_WIRING_PIN = 1 # gpio pin 12 = wiringpi no. 1 (BCM 18)
GPIO_BCM = 18

# Initialize PWM output for LED
'''wiringpi.wiringPiSetup()
wiringpi.pinMode(GPIO_WIRING_PIN, 2)     # PWM mode
wiringpi.pwmWrite(GPIO_WIRING_PIN, 0)    # OFF'''

wiringpi.wiringPiSetupGpio()
wiringpi.pinMode(GPIO_BCM, 2)     # PWM mode
'''wiringpi.pwmSetMode()
wiringpi.pwmSetRange()
wiringpi.pwmSetClock()
wiringpi.pwmWrite(GPIO_BCM, 0)    # OFF'''

wiringpi.softPwmCreate(GPIO_BCM,0,200) # Setup PWM using Pin, Initial Value and Range parameters

# Set PWM
def pwm(value):
    #wiringpi.pwmWrite(GPIO_BCM, value)
    wiringpi.softPwmWrite(GPIO_BCM, value)

print "PWM 0"
for i in range(5,18):
    pwm(i)
    print i
    time.sleep(1)
pwm(0)
