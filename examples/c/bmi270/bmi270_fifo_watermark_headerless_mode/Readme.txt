Overview
========
This example demonstrates fifo watermark interrupt in headerless mode.
Accel and Gyro are enabled and the fifo configurations are set. The FIFO status is read and the interrupt occurs when the watermark is achieved.

Steps to follow:
================
1.	Set the sensor interface SPI/I2C using bmi270 sensor api and map coines read/write interface calls to sensor api calls .
/* Macro definitions */
/* Enables i2c interface communication */
#define BMI270_INTERFACE_I2C             0

/* Enables spi interface communication */
#define BMI270_INTERFACE_SPI             1

2.	Initialize the application/board communication interface (USB) using coines API
3.	Read the board information and check the valid shuttle id 
4.	Initialize the sensor interface 
5.	Initialize the sensors (sensor init function takes a configuration structure where several configuration can be specified ).
6.  Watermark feature is enabled and the configurations are set. Since the mode is headerless, both accel and gyro must be set to the same ODR.
7. 	The interrupt is mapped and the status is continously read.
8.	Interrupt occurs when the watermark level is achieved.
9.	Close the communication interface
#####################################################################################################
