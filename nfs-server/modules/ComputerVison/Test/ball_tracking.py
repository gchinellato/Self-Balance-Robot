#!/usr/bin/python

# USAGE
# python ball_tracking.py --video ball_tracking_example.mp4
# python ball_tracking.py

# import the necessary packages
import imutils
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)
blueLower = (110, 50, 50)
blueUpper = (130, 255, 255)

width=640
height=480

camera = PiCamera()
camera.resolution = (width, height)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(width, height))

# allow the camera to warmup
time.sleep(0.1)

# keep looping
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    frame = frame.array

    # resize the frame, blur it, and convert it to the HSV
    # color space
    frame = imutils.resize(frame, width=width, height=height)
    # blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    #mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.inRange(hsv, blueLower, blueUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None

    #Center of the window
    cv2.circle(frame, (int(width/2), int(height/2)), 5, (0, 0, 255), -1)

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)

        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        print "Position X: " + str(center[0]) + ", Y: " + str(center[1])
        print "Distance center X: " + str(center[0]-(width/2)) + ", Y: " + str(center[1]-(height/2))

        # only proceed if the radius meets a minimum size
        if radius > 5:
	        # draw the circle and centroid on the frame,
	        # then update the list of tracked points

	        cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
	        cv2.circle(frame, center, 5, (0, 0, 255), -1)

	# show the frame
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
 
	# clear the stream in preparation for the next frame
    rawCapture.truncate(0)
 
    # if the `q` key was pressed, break from the loop
    #if key == ord("q"):
	#    break


