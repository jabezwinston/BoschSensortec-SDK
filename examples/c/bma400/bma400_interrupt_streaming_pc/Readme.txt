Overview
========
This example streams accel data along with tap detection on event basis from bma400 sensor 

Steps to follow to read sensor data
===================================
1.	Set the sensor interface SPI/I2C using bma400 sensor api  and map coines read/write interface calls to sensor api calls .
/* macro definitions */
/*enables i2c interface communication*/
#define BMA400_INTERFACE_I2C             1
/*enables spi interface communication*/
#define BMA400_INTERFACE_SPI             0

2.	Initialize the application/board communication interface (USB) using coines API
3.	Read the board information and check the valid shuttle id 
4.	Wait for 200ms 
5.	Initialize the sensor interface 
6.	Initialize the sensors (sensor init function takes a configuration structure where several configuration can be specified ).
7. 	Enable bma400 interrupts for Data ready and tap feature
8.  Stream accel data
9.	User can give a single or double tap on the sensor shuttle to see the hardware interrupt belongs to tap feature
10.	Close the communication interface

Sensor data print format
========================
 * First value 't[s]':  Sensortime in LSB
 * Values 'ax', 'ay', 'az': acceleration data, the sensor is configured in 2g mode.
