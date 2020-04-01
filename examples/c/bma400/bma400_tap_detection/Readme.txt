Overview
========
This example demonstrates step counting with ultra low power consumption for extensive battery lifetime.

Steps to follow:
================
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
7.  Call step counter in order to count the number of steps taken by the user and recogize the activity
8.	Close the communication interface

Note:
- This example is configured to run for 60 seconds. User need to re-run this example after timeout
- The user is advised to fine 'tap' parameters like 'sensitivity','tics_th', etc., as per the application needs

#####################################################################################################
