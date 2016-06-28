from Motion.motion import Motion
from Utils.traces.trace import *
import time
import RPi.GPIO as GPIO

try:
    motion = Motion(debug=True)
    print "Moving...."
    #motion.motorMove(10, 10)
    motion._motorB(direction="CW", pwm=abs(20))
    time.sleep(20)
    motion.motorShutDown()
except KeyboardInterrupt:
    print "Quit"
    motion.motorShutDown()
