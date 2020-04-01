#####################################################################################################
This example is to integration the BSEC with the bme680 sensor in low power mode. 

Uses BSEC related API's and examples.

#####################################################################################################
BSEC integration
1. Set the interface either SPI or I2C using the following macros in "bsec_integration.h" file 
/* macro definitions */
/*! i2c interface communication, 1 - Enable; 0- Disable */
#define BME680_INTERFACE_I2C             0
/*! spi interface communication, 1 - Enable; 0- Disable */
#define BME680_INTERFACE_SPI             1

2. Initialize device structure with corresponding interface (SPI/I2C), coines read/write interface and coines delay functions.
3. Initialize the application/board communication interface (USB) using coines API.
4. Read the board information and check the valid shuttle ID.
5. Initialize the sensor interface.
6. Initialize the sensor.
7. Initialize the BSEC by calling "bsec_iot_init()".
8. Create a file to log the data.
9. Run the loop by calling "bsec_iot_loop()"


Note: 
1. Reading Sensor data every 3 seconds once in this example.
2. Please refer BSEC related documentation for more information.
#####################################################################################################
