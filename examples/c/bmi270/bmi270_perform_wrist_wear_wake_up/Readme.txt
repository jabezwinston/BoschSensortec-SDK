Overview
========
This example demonstrates wrist wear wakeup detection.
Accel is enabled and the map interrupt for wrist wear wakeup is set. The wrist wakeup status is read and the interrupt occurs according to the user's actions.

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
6.  Wrist wear wakeup feature is chosen.
7. 	The interrupt is mapped and the status is continously read.
8.	Interrupt occurs when the board is lifted vertically and oriented in a particular direction.
9.	Close the communication interface
#####################################################################################################

