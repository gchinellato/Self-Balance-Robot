#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance
* @Platform: Raspberry PI 3
* @Description: Constants header file
* @Owner: Guilherme Chinellato
* @Email: guilhermechinellato@gmail.com
*************************************************
"""

#Service names
CLIENT_UDP_NAME = "Client-UDP-Thread"
SERVER_UDP_NAME = "Server-UDP-Thread"
BALANCE_NAME = "Balance-Thread"
PAN_TILT_NAME = "PanTilt-Thread"
PS3_CTRL_NAME = "PS3-Controller-Thread"
TRACKING_NAME = "Tracking-Thread"
SERIAL_NAME = "Serial-Thread"
LOG_FILE_NAME = "LogFile-Thread"

#Commands
CMD_BALANCE = "BALANCE"
CMD_PAN_TILT = "PAN_TILT"
CMD_PID_ANGLE = "PID_ANGLE"
CMD_PID_SPEED = "PID_SPEED"
CMD_UDP_CLIENT = "UDP_CLIENT"
CMD_UDP_SERVER = "UDP_SERVER"
CMD_SERIAL = "SERIAL"
CMD_MANAGER = "MANAGER"
