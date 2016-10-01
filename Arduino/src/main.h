/**
*************************************************
* @Project: Self Balance
* @Platform: Arduino Nano ATmega328
* @Description: Main thread
* @Owner: Guilherme Chinellato
* @Email: guilhermechinellato@gmail.com
*************************************************
*/

#ifndef MAIN_H
#define MAIN_H

#define SERIAL_BAUDRATE 	115200
#define CALIBRATION_MAGNETO	0
#define DATA_INTERVAL 	    100 // ms
float G_Dt=0; // gyroscope integration interval
unsigned int timestamp;
unsigned int timestamp_old;
float *ori;

#define BRAKEVCC 0
#define CW   1
#define CCW  2
#define BRAKEGND 3
#define CS_THRESHOLD 100

#endif
