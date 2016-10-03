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
import numpy as np
from picamera.array import PiRGBArray
import picamera
import time
import multiprocessing
from Utils.traces.trace import *

class ComputerVisionThread(multiprocessing.Process):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, queue=multiprocessing.Queue(), debug=0):
        multiprocessing.Process.__init__(self, group=group, target=target, name=name)
        self.args = args
        self.kwargs = kwargs
        self.name = name
        self.debug = debug 

        #Queue to communicate between threads
        self._workQueue = queue
        self._lock = multiprocessing.Lock()        
        
        #Event to signalize between threads
        self._stopEvent = multiprocessing.Event()
        self._sleepPeriod = 0.02 

        self.width=640
        self.height=480      

        logging.info("Tracking Module initialized")

    #Override method
    def run(self):
        logging.info("Tracking Thread Started")

        #define the lower and upper boundaries in the HSV color space
        lower, upper = self.findHSVfromRGB(0,0,255)

        camera = picamera.PiCamera()
        camera.resolution = (self.width, self.height)
        camera.framerate = 32
        rawCapture = PiRGBArray(camera, size=(self.width, self.height)) 

        time.sleep(1)

        lastTime = 0.0

        try:
            for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                if self._stopEvent.is_set() != True:
                    currentTime = time.time()

                    #Calculate time since the last time it was called
                    #if (self.debug & MODULE_CV):
                    #    logging.debug("Duration: " + str(currentTime - lastTime))

                    frame = frame.array

                    #resize the frame, blur it, and convert it to the HSV color space
                    frame = imutils.resize(frame, width=self.width, height=self.height)
                    #blurred = cv2.GaussianBlur(frame, (11, 11), 0)
                    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                    # construct a mask for the color, then perform a series of dilations and erosions to remove any small blobs left in the mask
                    mask = cv2.inRange(hsv, lower, upper)
                    mask = cv2.erode(mask, None, iterations=2)
                    mask = cv2.dilate(mask, None, iterations=2)

                    # find contours in the mask and initialize the current (x, y) center of the ball
                    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
                    center = None

                    #Center of the window
                    cv2.circle(frame, (int(self.width/2), int(self.height/2)), 5, (0, 0, 255), -1)

                    #only proceed if at least one contour was found
                    if len(cnts) > 0:
                        # find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
                        c = max(cnts, key=cv2.contourArea)
                        ((x, y), radius) = cv2.minEnclosingCircle(c)

                        M = cv2.moments(c)
                        if M["m00"] != 0:
                            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                        else:
                            center = (0, 0)

                        #Delta measure from object up to center of the vision
                        dWidth = center[0]-(self.width/2)
                        dHeight = center[1]-(self.height/2)

                        #only proceed if the radius meets a minimum size
                        if radius > 10:
                            #draw the circle and centroid on the frame,then update the list of tracked points
                            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                            cv2.circle(frame, center, 5, (0, 0, 255), -1)

                            if (self.debug & MODULE_CV):
                                cv2.putText(frame,"Radius: " + str(radius),(int(x),int(y)),cv2.FONT_HERSHEY_SIMPLEX,.5, (255,255,255),2)
                                cv2.putText(frame,"Position X: " + str(center[0]) + ", Y: " + str(center[1]),(int(x),int(y+20)),cv2.FONT_HERSHEY_SIMPLEX,.5, (255,255,255),2)
                                logging.debug(("Position X: " + str(center[0]) + ", Y: " + str(center[1])))
                                logging.debug(("Distance to center X: " + str(dWidth) + ", Y: " + str(dHeight)))
                                logging.debug(("Radius: " + str(radius)))

                            self.putEvent(self.name, (dWidth, dHeight, round(radius,2)))
                        
                        logging.debug("reading frames...") 

                        #show the frame
                        #cv2.imshow("Frame", frame)
                        #cv2.waitKey(10) & 0xFF
                        #if (self.debug & MODULE_CV):
                            #logging.debug("reading frames...")                 
                    #clear the stream in preparation for the next frame
                    rawCapture.truncate(0)
                    lastTime = currentTime 
                    self._stopEvent.wait(self._sleepPeriod) 
                else:
                    break
        except picamera.PiCameraValueError:
            logging.error("PiCameraValueError")
            pass         
        
    #Override method  
    def join(self, timeout=None):
        #Stop the thread and wait for it to end
        logging.info("Killing Tracking Thread...") 
        self._stopEvent.set()
        multiprocessing.Process.join(self, timeout) 

    def getEvent(self, timeout=2):
        return self._workQueue.get(timeout=timeout)  

    def putEvent(self, name, event):
        #Bypass if full, to not block the current thread
        if not self._workQueue.full():     
            self._workQueue.put((name, event))

    def findHSVfromRGB(self, R, G, B):
        bgrColor = np.uint8([[[B,G,R]]])
        hsvColor = cv2.cvtColor(bgrColor,cv2.COLOR_BGR2HSV)
        upper = (hsvColor[0][0][0]+10, 255, 255)
        lower = (hsvColor[0][0][0]-10, 50, 50)
        return lower, upper

