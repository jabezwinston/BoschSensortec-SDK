#####################################################################################################
This example stream the bmi160 sensor data based on the polling using coines API

Uses bmi160 senor API
  --> bmi160_set_sens_conf() is used to configures the Output Data Rate (ODR), Bandwidth (BW), Range and Power mode.
  --> Use sub-members in 'accel_cfg'/'gyro_cfg' member of  bmi160_dev to set Output Data Rate (ODR), Bandwidth (BW), Range and Power mode.

#####################################################################################################
To perform the polling streaming
1. Set the sensor interface SPI/I2C using bmi160 sensor api and map coines read/write interface calls to sensor api calls .
/* macro definitions */
/*enables i2c interface communication*/
#define BMI160_INTERFACE_I2C             1
/*enables spi interface communication*/
#define BMI160_INTERFACE_SPI             0

2. Initialize the application/board communication interface (USB) using coines API
3. Read the board information and check the valid shuttle ID 
4. Initialize the sensor interface
5. Wait for 200ms  
6. Initialize the sensors (sensor init function takes a configuration structure where several configuration can be specified).
7. Send the streaming command to the application board, which requires several configuration parameters
					
						#if BMI160_INTERFACE_I2C==1
						stream_config.intf = COINES_SENSOR_INTF_I2C;
						stream_config.i2c_bus = COINES_I2C_BUS_0;
						#endif
						#if BMI160_INTERFACE_SPI==1
						stream_config.intf = COINES_SENSOR_INTF_SPI;
						stream_config.spi_bus = COINES_SPI_BUS_0;
						#endif

						/*for I2C */
						stream_config.dev_addr = BMI160_DEV_ADDR;
						stream_config.cs_pin = COINES_SHUTTLE_PIN_7;
						stream_config.sampling_time = 1250;      /* For 800 Hz */
						/*1 micro second / 2 milli second*/
						stream_config.sampling_units = COINES_SAMPLING_TIME_IN_MICRO_SEC;   /* Micro second */
						stream_block.no_of_blocks = 1;
						stream_block.reg_start_addr[0] = BMI160_ACCEL_DATA_ADDR; /* Accel data start address */

						stream_block.no_of_data_bytes[0] = 6;

						coines_config_streaming(1, &stream_config, &stream_block);

						stream_config.sampling_time = 625;      /* 1.6 Khz */
						stream_block.reg_start_addr[0] = BMI160_GYRO_DATA_ADDR; /* Gyro data start address */

						coines_config_streaming(2, &stream_config, &stream_block);

8. Start the Polling streaming – and specify the no of samples to read.
 coines_start_stop_streaming(COINES_STREAMING_MODE_POLLING, COINES_STREAM_INFINITE_SAMPLES, 1);
 
9. Read, manipulate, print the stream response data on console for some iteration
10. Stop the polling streaming
11. Close the communication interface

#####################################################################################################

NOTE:
It is recommended to operate BMI160 in fast mode while using I2C interface

Accel or Gyro only
- Polling streaming can reliably operate in I2C standard mode (100kHz) for sampling time > 1250us
- For sampling time < 1250us , use I2C Fast mode (400 kHz)
 
Accel and Gyro
- Use I2C Fast mode only
