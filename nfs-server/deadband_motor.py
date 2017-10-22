#!/usr/bin/python

import datetime
import time
import serial
import serial.tools.list_ports as prtlst
import os

TRACE_BEGIN = "#BEGIN#"
TRACE_END = "#END#"

STARTED = 0
DIRECTION = 1
STEERING = 2
SPEED_PID = 3
ANGLE_PID_AGGR = 4
ANGLE_PID_CONS = 5
CALIBRATED_ZERO_ANGLE = 6
ANGLE_LIMIT = 7

#Analog Constrains
ANALOG_MAX = 1.0
ANALOG_MIN = -1.0

#Position limits (in percentage)
PWM_MAX = 100
PWM_MIN = -100

ser = serial.Serial()

def convertTo(value, fromMax, fromMin, toMax, toMin):
    if not value >= fromMin and value <= fromMax:
        if value > fromMax:
            value = fromMax
        elif value < fromMin:
            value = fromMin

    factor = (value-fromMin)/(fromMax-fromMin)
    return factor*(toMax-toMin)+toMin

def checkData(command, value):
    '''(TRACE_BEGIN)(COMMAND),(NUM_PARAM),(PARAM_1),(PARAM_2),(...)(TRACE_END)'''
    if command == STARTED:
        msg = str(command) + "," + "1" + "," + str(value)
    elif command == DIRECTION:
        value = convertTo(value, ANALOG_MAX, ANALOG_MIN, PWM_MAX, PWM_MIN)
        print(value)
        msg = str(command) + "," + "1" + "," + str(round(value,2))
    elif command == STEERING:
        value = convertTo(value, ANALOG_MAX, ANALOG_MIN, PWM_MAX, PWM_MIN)
        msg = str(command) + "," + "1" + "," + str(round(value,2))
    elif command == ANGLE_PID_CONS:
        msg = str(command) + "," + "3" + "," + str(round(value[0],2)) + "," +  str(round(value[1],2)) + "," + str(round(value[2],2))
    elif command == CALIBRATED_ZERO_ANGLE:
        msg = str(command) + "," + "1" + "," + str(round(value,2))
    elif command == ANGLE_LIMIT:
        msg = str(command) + "," + "1" + "," + str(round(value,2))
    else:
        msg = "unknown"

    return TRACE_BEGIN + msg + TRACE_END + "\r\n"

while True:
    try:
        if not ser.isOpen():
            print("porta fechada")
            time.sleep(2)
            ser.port = prtlst.comports()[0][0]
            ser.baudrate = 19200
            ser.timeout = 5	
            print("Opening serial port " + str(ser.port) + " " + str(ser.baudrate))					
            ser.open()

        #val=input("Analoga value (-100 / 100): ")
        #msg = checkData(DIRECTION, val/100.0)
        val = input("Start: ")
        msg = checkData(STARTED, val)
        print("Sending to Arduino: " + str(msg))
        size = ser.write(''.join(msg))
        
        V = (0,0,0)
        v = list(V)
        v[0] = input("Kp: ")
        v[1] = input("Ki: ")
        v[2] = input("Kd: ")
        msg = checkData(ANGLE_PID_CONS, v)
        print("Sending to Arduino: " + str(msg))
        size = ser.write(''.join(msg))

        recv = ser.readline()
        print("Reading from Arduino: " + str(recv))
        time.sleep(0.001)
    except serial.SerialException:
        print("SerialException")
        ser.close()
        pass

'''
Results:
PWM		Status
1%		Stopped
2%		Stopped
3%		Stopped
4%		Stopped
5%		Stopped
6%		Stopped
7%		Stopped
8%		Stopped
9%		Running
10%		Running

-1%		Stopped
-2%		Stopped
-3%		Stopped
-4%		Stopped
-5%		Stopped
-6%		Stopped
-7%		Stopped
-8%		Stopped
-9%		Running
-10%	            Running
'''

