####################################################################################################################
This software example stream the bme280 sensor data based on the polling using coinesAPI 

The application execution starts from main function .

bme280_parse_sensor_data() to convert raw register data to uncompensated (°C),humidity (%)and pressure (Pascal) values

bme280_compensate_data() converts the uncompensated uncompensated,humidity and pressure values  to compensated values

######################################################################################################################
To perform the polling streaming

1.	Set the sensor interface SPI/I2C using bme280 sensor API and map coines read/write interface calls to sensor API calls .
/* macro definitions */
/*enables i2c interface communication*/
#define BME280_INTERFACE_I2C             1
/*enables spi interface communication*/
#define BME280_INTERFACE_SPI             0

2.	Initialize the application/board communication interface (USB) using coines API
3.	Read the board information and check the valid shuttle id 
4.	Wait for 200ms 
5.	Initialize the sensor interface 
6.	Initialize the sensors (sensor init function takes a configuration structure where several configuration can be specified ).
7.	Send the streaming command to the application board, which requires several configuration parameters
		coines_config_streaming(1, &stream_config, &stream_block)
8.	Start the interrupt streaming – and specify the no of samples to read .
 coines_start_stop_streaming(COINES_STREAMING_MODE_INTERRUPT, COINES_STREAMING_START);
9.	Read, manipulate the print the stream response data on console for some iteration.
10.	Stop the polling streaming 
 coines_start_stop_streaming(COINES_STREAMING_MODE_INTERRUPT, COINES_STREAMING_STOP);
11.	Close the communication interface

#########################################################################################################################

NOTE:

- Polling streaming can reliably operate in I2C standard mode (100kHz) for sampling time > 1250us
- For sampling time < 1250us , use I2C Fast mode (400 kHz)
