#####################################################################################################
This example reads the chip ID of bmi160 sensor

Uses bmi160_init() sensorAPI 
Chip ID can be read from 'bmi160_dev' structure member 'chip_id' 

#####################################################################################################
Chip ID read 
1. Set the sensor interface SPI/I2C using bmi160 sensor api  and map coines read/write interface calls to sensor api calls.
/* macro definitions */
/*enables i2c interface communication*/
#define BMI160_INTERFACE_I2C        	1
/*enables spi interface communication*/
#define BMI160_INTERFACE_SPI       		0

2. Initialize the application/board communication interface (USB) using coines API
3. Read the board information and check the valid shuttle id 
4. Initialize the sensor interface 
5. Wait for 200ms 
6. Initialize the sensors (sensor init function takes a configuration structure where several configuration can be specified).
7. Close the communication interface
#####################################################################################################


