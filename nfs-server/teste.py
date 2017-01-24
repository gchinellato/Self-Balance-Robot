import threading
import datetime
import time
import serial
import Queue as queue
from Utils.traces.trace import *

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

class SerialThread(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, queue=queue.Queue(), COM="/dev/ttyUSB0"):
        threading.Thread.__init__(self, group=group, target=target, name=name)
        self.args = args
        self.kwargs = kwargs
        self.name = name

        self._workQueue = queue

        self._stopEvent = threading.Event()
        self._sleepPeriod = 0.00

        self.COM = COM
        self.port = None

        logging.info("Serial Module initialized")

    def run(self):
            logging.info("Serial Thread Started")

            try:
                self.port = serial.Serial(self.COM, baudrate=19200)
                logging.info(self.port.name)

                while not self._stopEvent.wait(self._sleepPeriod):
                    msg = self.getMessage()
                    if msg != None:
                        logging.info(("Writing to Arduino >>>: " + str(msg)))
                        size = self.port.write(''.join(msg))

                    recv = self.port.readline()
                    logging.info(("Reading from Arduino <<<: " + str(recv)))

            except Exception as e:
                logging.info("Exception: " + str(e))
                pass

    def join(self, timeout=2):
        logging.info("Killing Serial Thread...")
        self._stopEvent.set()
        self.port.close()
        threading.Thread.join(self, timeout=timeout)

    def getMessage(self, timeout=2):
        #Bypass if empty, to not block the current thread
        if not self._workQueue.empty():
            return self._workQueue.get(timeout=timeout)
        else:
            return None

    def putMessage(self, command, msg):
        #Bypass if full, to not block the current thread
        if not self._workQueue.full():
            msg = self.checkData(command, msg)
            self._workQueue.put(msg)

    def checkData(self, command, msg):
        '''(TRACE_BEGIN)(COMMAND),(NUM_PARAM),(PARAM_1),(PARAM_2),(...)(TRACE_END)'''
        if command == STARTED:
            msg = str(command) + "," + "1" + "," + str(msg)
        elif command == DIRECTION:
            msg = self.convertTo(msg, ANALOG_MAX, ANALOG_MIN, PWM_MAX, PWM_MIN)
            msg = str(command) + "," + "1" + "," + str(round(msg,2))
        elif command == STEERING:
            msg = self.convertTo(msg, ANALOG_MAX, ANALOG_MIN, PWM_MAX, PWM_MIN)
            msg = str(command) + "," + "1" + "," + str(round(msg,2))
        elif command == SPEED_PID:
            msg = str(command) + "," + "3" + "," + str(round(msg[0],2)) + "," + str(round(msg[1],2)) + "," + str(round(msg[2],2))
        elif command == ANGLE_PID_AGGR:
            msg = str(command) + "," + "3" + "," + str(round(msg[0],2)) + "," + str(round(msg[1],2)) + "," + str(round(msg[2],2))
        elif command == ANGLE_PID_CONS:
            msg = str(command) + "," + "3" + "," + str(round(msg[0],2)) + "," + str(round(msg[1],2)) + "," + str(round(msg[2],2))
        elif command == CALIBRATED_ZERO_ANGLE:
            msg = str(command) + "," + "1" + "," + str(round(msg,2))
        elif command == ANGLE_LIMIT:
            msg = str(command) + "," + "1" + "," + str(round(msg,2))
        else:
            msg = "unknown"

        return TRACE_BEGIN + msg + TRACE_END + "\r\n"


def main():
    setVerbosity("debug")

    serialToWrite = queue.Queue()

    serial = SerialThread(name="Thread-Serial", queue=serialToWrite)
    serial.daemon = True
    serial.start()

    try:
        while True:
            serial.putMessage(STARTED, 0)
            time.sleep(1)

    except KeyboardInterrupt:
        logging.info("Exiting...")
        serial.join()

if __name__ == '__main__':
    main()



