#####################################################################################################
This example reads the chip-id of bme280 sensor 

Uses bme280_init() sensorAPI 
Chip ID can be read from 'bme280_dev' structure member 'chip_id' 

#####################################################################################################
Chip ID read 
1.	Set the sensor interface SPI/I2C using BME280 sensor API and map coines read/write interface calls to sensor API calls .
/* macro definitions */
/*enables I2C interface communication*/
#define BME280_INTERFACE_I2C             1
/*enables SPI interface communication*/
#define BME280_INTERFACE_SPI             0

2.	Initialize the application/board communication interface (USB) using coines API
3.	Read the board information and check the valid shuttle ID 
4.	Wait for 200ms 
5.	Initialize the sensor interface 
6.	Initialize the sensors (sensor init function takes a configuration structure where several configuration can be specified ).
7.	Close the communication interface
#####################################################################################################
