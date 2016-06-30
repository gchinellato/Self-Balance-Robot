#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance                         
* @Description: Trace module
* @Owner: Guilherme Chinellato                   
* @Email: guilhermechinellato@gmail.com                              
*************************************************
"""

import logging

'''
NOTSET = 0
DEBUG: Detailed information, typically of interest only when diagnosing problems.
INFO: Confirmation that things are working as expected.
WARNING: An indication that something unexpected happened, or indicative of some problem in the near future (e.g. 'disk space low'). The software is still working as expected.
ERROR: Due to a more serious problem, the software has not been able to perform some function.
CRITICAL: A serious error, indicating that the program itself may be unable to continue running.
'''

NOTSET = 0
DEBUG = 10
INFO = 20
WARNING = 30
ERROR = 40
CRITICAL = 50

#logging.basicConfig(level=logging.INFO, format='[%(levelname)s] (%(processName)s) (%(threadName)s) (%(module)s) %(message)s')
logging.basicConfig(level=logging.DEBUG, format='[%(levelname)s] (%(processName)s) (%(threadName)s) (%(module)s) %(message)s')
#logging.basicConfig(level=logging.DEBUG, format='[%(levelname)s] (%(processName)s) (%(threadName)s) (%(module)s) (%(filename)s) (Fct: %(funcName)s) (Line:%(lineno)d) %(message)s')

