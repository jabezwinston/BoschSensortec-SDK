#####################################################################################################
This software example reads data from the FIFOs of accel. and gyro. chip in bmi08x sensor

Uses bmi08a_init() and bmi08g_init() sensorAPI 
Chip IDs can be read from 'bmi08x_dev' structure members 'accel_chip_id' and 'gyro_chip_id'  

This example supports both BMI085 and BMI088. Works with both I2C and SPI mode.

Corresponding sensor flag need to be enabled in the sensor API in bmi08x_defs.h.

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
1.	Set the sensor interface SPI/I2C using bmi08x sensor api  and map coines read/write interface calls to sensor api calls .
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
7.	Configure gyro FIFO to stop-at-full, INT3 as FIFO interrrupt and set watermark interrupt at 10 frames
8.	Read and unpack gyro sensor FIFO data
9.	Configure accel FIFO to stop-at-full, read accel data from FIFO and set watermark interrupt at 6 frames
10.	Read and unpack accel sensor FIFO data
11.	Close the communication interface
#####################################################################################################

