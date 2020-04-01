#####################################################################################################
This example reads the temperature (°C),humidity (%)and pressure (Pascal) from bme280 sensor using 
bme280_get_sensor_data() sensor API

#####################################################################################################
Chip ID read 
1.	Set the sensor interface SPI/I2C using BME280 sensor API  and map coines read/write interface calls to sensor API calls .
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
7.  Read sensor data using bme280_get_sensor_data()
8.	Close the communication interface
#####################################################################################################
