#!/usr/bin/python

import imutils
import cv2
import numpy as np
from picamera.array import PiRGBArray
import picamera
import time
import threading
import multiprocessing
import Queue
import logging

logging.basicConfig(level=logging.DEBUG, format='[%(levelname)s] (%(processName)s) (%(threadName)s) (%(module)s) %(message)s')

class VideoStreamThread(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, queue=Queue.Queue(), debug=0):
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
        self._sleepPeriod = 0.0

        self.width=640
        self.height=480

        self.camera = picamera.PiCamera()
        self.camera.resolution = (self.width, self.height)
        self.camera.framerate = 24
        self.rawCapture = PiRGBArray(self.camera, size=(self.width, self.height)) 
        time.sleep(1)

        self.frame = None

        logging.info("Cam Module initialized")

    def run(self):
        logging.info("Cam Thread Started")        

        lastTime = 0.0        

        try:
            for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
                if self._stopEvent.isSet() != True:
                    currentTime = time.time()

                    #Calculate time since the last time it was called
                    #logging.debug("Duration: " + str(currentTime - lastTime))
                    
                    self.frame = frame.array

                    #clear the stream in preparation for the next frame
                    self.rawCapture.truncate(0)
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
        threading.Thread.join(self, timeout) 

    def read(self):
        return self.frame

    def getEvent(self, timeout=2):
        return self._workQueue.get(timeout=timeout)  

    def putEvent(self, event):
        #Bypass if full, to not block the current thread
        if not self._workQueue.full():     
            self._workQueue.put(event)

class VideoStreamProcess(multiprocessing.Process):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, queue=multiprocessing.Queue(), debug=0):
        threading.Thread.__init__(self, group=group, target=target, name=name)

    def run(self):
        pass
     
    #Override method  
    def join(self, timeout=None):
        #Stop the thread and wait for it to end
        logging.info("Killing Tracking Thread...") 
        threading.Thread.join(self, timeout) 

def findHSVfromRGB(R, G, B):
    bgrColor = np.uint8([[[B,G,R]]])
    hsvColor = cv2.cvtColor(bgrColor,cv2.COLOR_BGR2HSV)
    upper = (hsvColor[0][0][0]+10, 255, 255)
    lower = (hsvColor[0][0][0]-10, 50, 50)
    return lower, upper

def main():
    threads = []  
    videoStream = VideoStreamThread()
    threads.append(videoStream)
    videoStream.start()
    time.sleep(5)

    (lower, upper) = findHSVfromRGB(0,0,255)

    lastTime = 0.0 
    LP = 0.02
    try:
        while True:
            currentTime = time.time()
            logging.debug("Duration: " + str(currentTime - lastTime))
            
            frame = videoStream.read()

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # construct a mask for the color, then perform a series of dilations and erosions to remove any small blobs left in the mask
            mask = cv2.inRange(hsv, lower, upper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            # find contours in the mask and initialize the current (x, y) center of the ball
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            center = None

            #Center of the window
            cv2.circle(frame, (int(videoStream.width/2), int(videoStream.height/2)), 5, (0, 0, 255), -1)

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
                dWidth = center[0]-(videoStream.width/2)
                dHeight = center[1]-(videoStream.height/2)

                #only proceed if the radius meets a minimum size
                if radius > 10:
                    #draw the circle and centroid on the frame,then update the list of tracked points
                    cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)

                    cv2.putText(frame,"Radius: " + str(radius),(int(x),int(y)),cv2.FONT_HERSHEY_SIMPLEX,.5, (255,255,255),2)
                    cv2.putText(frame,"Position X: " + str(center[0]) + ", Y: " + str(center[1]),(int(x),int(y+20)),cv2.FONT_HERSHEY_SIMPLEX,.5, (255,255,255),2)
                    logging.debug(("Position X: " + str(center[0]) + ", Y: " + str(center[1])))
                    logging.debug(("Distance to center X: " + str(dWidth) + ", Y: " + str(dHeight)))
                    logging.debug(("Radius: " + str(radius)))
            
            #cv2.imshow("Frame", frame)
            #key = cv2.waitKey(1) & 0xFF
            time.sleep(LP)
            lastTime = currentTime
    
    finally:  
        for t in threads:
            logging.info("Killing "+ str(t.name) + "...")
            t.join()

if __name__ == '__main__':
    main()
