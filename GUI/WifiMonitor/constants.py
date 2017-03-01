#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance
*  @Platform: Raspberry PI 3 / Ubuntu / Qt
* @Description: Manager header file
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

#Commands
CMD_BALANCE = "BALANCE"
CMD_PAN_TILT = "PAN_TILT"
CMD_PID_ANGLE = "PID_ANGLE"
CMD_PID_SPEED = "PID_SPEED"
CMD_UDP_CLIENT = "UDP_CLIENT"
CMD_UDP_SERVER = "UDP_SERVER"
CMD_SERIAL = "SERIAL"
CMD_MANAGER = "MANAGER"

#Initial values
SPEED_SETPOINT	=	0.0
SPEED_KP 		=	0.0
SPEED_KI 		=	0.0
SPEED_KD 		=	0.0
ANGLE_SETPOINT 	=	0.0
ANGLE_LIMIT 	=	2.0
ANGLE_KP_AGGR 	=	5.0
ANGLE_KI_AGGR 	=	1.0
ANGLE_KD_AGGR 	=	0.5
ANGLE_KP_CONS 	=	5.0
ANGLE_KI_CONS 	=	0.1
ANGLE_KD_CONS 	=	0.3
ANGLE_IRRECOVERABLE = 30.0
CALIBRATED_ZERO_ANGLE = 1.5
WINDUP_GUARD 		= 100
