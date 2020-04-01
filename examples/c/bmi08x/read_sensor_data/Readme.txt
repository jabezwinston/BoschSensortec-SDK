#####################################################################################################
This software example read the bmi08x accel. and gyro. sensor data

Uses bmi08a_get_data() and bmi08g_get_data()

Sensor (accel/gyro) values can be read from 'bmi08x_sensor_data' structure members x,y,z

This example supports both BMI085 and BMI088. Works with both I2C and SPI mode.

Corresponding sensor flag need to be enabled in the sensor API  in bmi08x_defs.h.

/** \name enable bmi085 sensor */
#ifndef BMI08X_ENABLE_BMI085
#define BMI08X_ENABLE_BMI085       1
#endif
/** \name enable bmi088 sensor */
#ifndef BMI08X_ENABLE_BMI088
#define BMI08X_ENABLE_BMI088       0
#endif

                                   (or)
								 
Specify SHUTTLE_BOARD=<BMI08x> as commandline parameter to make utility

Eg : mingw32-make SHUTTLE_BOARD=BMI088

#####################################################################################################
Read sensor data
1.	Set the sensor interface SPI/I2C using bmi08x sensor API  and map coines read/write interface calls to sensor API calls .
/* macro definitions */
/*enables i2c interface communication*/
#define BMI08x_INTERFACE_I2C             1
/*enables spi interface communication*/
#define BMI08x_INTERFACE_SPI             0

2.	Initialize the application/board communication interface (USB) using coines API
3.	Read the board information and check the valid shuttle ID 
4.	Wait for 200ms 
5.	Initialize the sensor interface 
6.	Initialize the sensors (sensor init function takes a configuration structure where several configuration can be specified ).
7.	Read sensor using sensor api and print it on console for some iteration.
8.	Close the communication interface

#####################################################################################################


