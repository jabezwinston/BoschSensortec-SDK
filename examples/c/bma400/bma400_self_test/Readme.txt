Overview
========
This example performs self test for BMA400

Steps to follow to perform self test
====================================
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
7.  Perform self test by calling self test function
8.	Close the communication interface
================================================================================================================================
