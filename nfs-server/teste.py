import threading
import datetime
import time
import serial
import Queue as queue
from Utils.traces.trace import *

#SIZE, CMD, 1

port = serial.Serial("/dev/ttyUSB0", timeout=2, baudrate=38400)
print(port.name)

while True:
    try:
        opt = input("Choose option: ")

        if opt == 0:
            msg = str(input())
            print("Written >>> %d: %s" % (len(msg), msg))
            port.write(''.join(msg))

        recv = port.readline()
        print("Received <<< : %s" % (recv))

    except KeyboardInterrupt:
        port.close()
        break

