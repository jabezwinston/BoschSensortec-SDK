#####################################################################################################
This software example stream the bmi08x sensor data based on the polling using coines API

Uses bmi08x sensorAPI
  --> bmi08a_set_power_mode() / bmi08g_set_power_mode() to switch accel/gyro between power mode like sleep ,normal ,etc.,
  --> Use sub-members in 'accel_cfg'/'gyro_cfg' member of  bmi80x_dev to set Output Data Rate (ODR), Bandwidth (BW) , Range
  --> Use bmi08a_set_meas_conf() / bmi08g_set_meas_conf() to reflect the change in accel/gyro sensor registers

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
To perform the polling streaming
@Note - Maximum 2 sensors can be streamed at the same time 

1.	Set the sensor interface SPI/I2C using bmi08x sensor API and map coines read/write interface calls to sensor API calls .
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
7.	Send the streaming command to the application board, which requires several configuration parameters
					
						#if BMI08x_INTF_I2C==1
						stream_config.intf = COINES_SENSOR_INTF_I2C;
						#endif
						#if BMI08x_INTF_SPI==1
						stream_config.intf = COINES_SENSOR_INTF_SPI;
						#endif
						stream_config.i2c_bus = COINES_I2C_BUS_0; //if intf is I2C
						stream_config.spi_bus = COINES_SPI_BUS_0; // if intf is SPI
						/*for I2C */
						stream_config.dev_addr = BMI08x_ACCEL_DEV_ADDR;
						stream_config.cs_pin = COINES_SHUTTLE_PIN_8;
						stream_config.sampling_time = 625; // 1.6 khz
						/*1 micro second / 2 milli second*/
						stream_config.sampling_units = COINES_SAMPLING_TIME_IN_MICRO_SEC; //micro second
						stream_block.no_of_blocks = 1;
						stream_block.reg_start_addr[0] = 0x12; //accel data start address
						#if BMI08x_INTERFACE_I2C==1
						stream_block.no_of_data_bytes[0] = 6;
						#endif
						#if BMI08x_INTF_SPI==1
						stream_block.no_of_data_bytes[0] = 7; // actual data length is 6 bytes 1 byte needed to initiate the spi communication
						#endif
						/*send the polling streaming configration*/
						coines_config_streaming(1, &stream_config, &stream_block);

8.	Start the interrupt streaming – and specify the no of samples to read .
 coines_start_stop_streaming(COINES_STREAMING_MODE_POLLING, COINES_STREAMING_START);

9.	Read, manipulate the print the stream response data on console for some iteration.
10.	Stop the polling streaming 
11.	Close the communication interface

#####################################################################################################

NOTE:

It is recommended to operate BMI08x in fast mode when using I2C interface

Accel or Gyro only
- Polling streaming can reliably operate in I2C standard mode (100kHz) for sampling time > 1250us
- For sampling time < 1250us , use I2C Fast mode (400 kHz)
 
Accel and Gyro
- Use I2C Fast mode only
