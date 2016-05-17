import RPi.GPIO as GPIO
from time import sleep
import datetime

PIR_GPIO = 17

GPIO.setmode(GPIO.BCM) # set up BCM GPIO numbering  
GPIO.setup(PIR_GPIO, GPIO.IN) # set GPIO17 as input

# Callback function
def Motion(channel):
    if GPIO.input(PIR_GPIO):        
        timeStart = datetime.datetime.now().time()
        print "TON: " + str(timeStart) 
        print "Motion Detected - Rising!"
    else:        
        timeEnd = datetime.datetime.now().time()
        print "TOFF: " + str(timeEnd)  
        print "Motion Detected - Falling!"

try:
    GPIO.add_event_detect(PIR_GPIO, GPIO.BOTH, callback=Motion) 
    while True:
        sleep(100)

except KeyboardInterrupt:
    print "Quit"
    GPIO.cleanup()
