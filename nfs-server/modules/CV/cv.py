#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance                         
* @Description: Computer Vision - OpenCV
                PiCam
* @Owner: Guilherme Chinellato                   
* @Email: guilhermechinellato@gmail.com                              
*************************************************
"""

import imutils
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import threading
import multiprocessing
import Queue
from Utils.traces.trace import *

class ComputerVisionThread(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, queue=Queue.Queue(), debug=False):
        threading.Thread.__init__(self, group=group, target=target, name=name)
        self.args = args
        self.kwargs = kwargs
        self.name = name
        self.debug = debug 

        #Queue to communicate between threads
        self._workQueue = queue
        self._lock = threading.Lock()        
        
        #Event to signalize between threads
        self._stopEvent = threading.Event()
        self.stop = threading.Event()
        self._sleepPeriod = 0.01 

        self.width=640
        self.height=480      

        logging.debug("Computer Vision Thread initialized")

    #Override method
    def run(self):
        # define the lower and upper boundaries of the "green"
        # ball in the HSV color space, then initialize the
        # list of tracked points
        greenLower = (29, 86, 6)
        greenUpper = (64, 255, 255)
        blueLower = (110, 50, 50)
        blueUpper = (130, 255, 255)

        camera = PiCamera()
        camera.resolution = (self.width, self.height)
        camera.framerate = 32
        rawCapture = PiRGBArray(camera, size=(self.width, self.height)) 

        time.sleep(1)

        lastTime = 0.0

        while not self._stopEvent.wait(self._sleepPeriod):
            try:
                for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                    if self.stop.isSet() != True:
                        currentTime = time.time()

                        #Calculate time since the last time it was called
                        #if (self.debug):
                        #    logging.debug("Duration: " + str(currentTime - lastTime))
                        
                        frame = frame.array
                        #logging.debug(frame)

                        # resize the frame, blur it, and convert it to the HSV color space
                        frame = imutils.resize(frame, width=self.width, height=self.height)
                        # blurred = cv2.GaussianBlur(frame, (11, 11), 0)
                        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                        # construct a mask for the color "green", then perform a series of dilations and erosions to remove any small blobs left in the mask
                        #mask = cv2.inRange(hsv, greenLower, greenUpper)
                        mask = cv2.inRange(hsv, blueLower, blueUpper)
                        mask = cv2.erode(mask, None, iterations=2)
                        mask = cv2.dilate(mask, None, iterations=2)

                        # find contours in the mask and initialize the current (x, y) center of the ball
                        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
                        center = None

                        #Center of the window
                        cv2.circle(frame, (int(self.width/2), int(self.height/2)), 5, (0, 0, 255), -1)

                        # only proceed if at least one contour was found
                        if len(cnts) > 0:
                            # find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
                            c = max(cnts, key=cv2.contourArea)
                            ((x, y), radius) = cv2.minEnclosingCircle(c)

                            M = cv2.moments(c)
                            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                            #Delta measure from object up to center of the vision
                            dWidth = center[0]-(self.width/2)
                            dHeight = center[1]-(self.height/2)

                            if (self.debug):
                                logging.debug(("Position obj X: " + str(center[0]) + ", Y: " + str(center[1])))
                                logging.debug(("Distance center X: " + str(dWidth) + ", Y: " + str(dHeight)))

                            #+1.0 ~ -1.0
                            #dWidth = (dWidth/float((self.width/2)))
                            #dHeight = (dHeight/float((self.height/2)))
                            self.putEvent(self.name, (dWidth, dHeight))

                            # only proceed if the radius meets a minimum size
                            if radius > 5:
                                # draw the circle and centroid on the frame,then update the list of tracked points
                                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                                cv2.circle(frame, center, 5, (0, 0, 255), -1)

                        # show the frame
                        #cv2.imshow("Frame", frame)
                    #key = cv2.waitKey(10) & 0xFF
                    logging.debug("reading frames")
                     
                    # clear the stream in preparation for the next frame
                    rawCapture.truncate(0) 
                    lastTime = currentTime 
            except KeyboardInterrupt:
                break               
        
    #Override method  
    def join(self, timeout=None):
        #Stop the thread and wait for it to end
        self._stopEvent.set()
        threading.Thread.join(self, timeout) 

    def getEvent(self, timeout=2):
        return self._workQueue.get(timeout=timeout)  

    def putEvent(self, name, event):
        #Bypass if full, to not block the current thread
        if not self._workQueue.full():     
            self._workQueue.put((name, event))

