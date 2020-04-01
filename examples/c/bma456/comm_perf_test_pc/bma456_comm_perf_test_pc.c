/**
 * Copyright (C) 2017 - 2018 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bma456_comm_perf_test.c
 * @brief 	Sample file how to test BMA456 communication, functionality 
 *			and performance using LIB COINES
 *
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
/*********************************************************************/
/* own header files */
/*********************************************************************/
#include "coines.h"
#include "bma456.h"

/*********************************************************************/
/* macro definitions */
/*********************************************************************/
/*! i2c interface communication, 1 - Enable; 0- Disable */
#define BMA456_INTERFACE_I2C             0
/*! spi interface communication, 1 - Enable; 0- Disable */
#define BMA456_INTERFACE_SPI             1

#if (!((BMA456_INTERFACE_I2C==1) && (BMA456_INTERFACE_SPI==0)) && \
	(!((BMA456_INTERFACE_I2C==0) && (BMA456_INTERFACE_SPI==1))))
#error "Invalid value given for the macros BMA456_INTERFACE_I2C / BMA456_INTERFACE_SPI"
#endif
/*! BMA4xy shuttle board ID */
#define BMA4XY_SHUTTLE_ID		0x141

/*! This variable holds the device address of BMA456 */
#define BMA456_DEV_ADDR 		BMA4_I2C_ADDR_PRIMARY

/*! defines the PASS threshold for the offset test */
/*! typical value for this parameter is 20mg for BMA456 */
#define OFFSET_THRESHOLD_MG		60.0

/*! defines the PASS threshold for the noise density test */
/*! typical value for this parameter is 120ug/sqrt(Hz) for BMA456 */
#define NOISE_THRESHOLD_MG		180.0

/*! Defines rest position of the device. Adjust according to test conditions */
#define X_REST_POSITION_MG		0.0
#define Y_REST_POSITION_MG		0.0
#define Z_REST_POSITION_MG		1000.0

/*! Average Command response latency for APP2.0 Firmware in milli-seconds */
#define COMMAND_RESPONSE_LATENCY  (5)

/*********************************************************************/
/* global variables */
/*********************************************************************/
/*! @brief This structure containing relevant bma456 info */
struct bma4_dev bma456dev;

/*********************************************************************/
/* function declarations */
/*********************************************************************/

/*!
 * @brief	This internal API quits the program and outputs an error message
 */
static void coines_exit_error(const char*);

/*!
 * @brief	This internal API is used to initialize the sensor interface
 */
static void init_sensor_interface(void);

/*!
 * @brief	This internal API is used to initialize the bma456 and test communication
 */
static void init_comm_test_bma456(void);

/*!
 * @brief	 This internal API is used to set the sensor driver interface to read/write the data
 */
static void init_bma456_sensor_driver_interface(void);

/*!
 * @brief	This internal API is used to test if the sensor is working
 */
static void function_test_bma456(void);

/*!
 * @brief	 This internal API is used to test the sensor performance
 */
static void perf_test_bma456(void);


/*!
 * @brief	 This internal API quits the program and outputs an error message
 */
static void coines_exit_error(const char* err_msg)
{

    printf("%s\n", err_msg);
    fflush(stdout);
    exit(COINES_E_FAILURE);
}
/*********************************************************************/
/* functions */
/*********************************************************************/
/*!
 *  @brief This internal API is used to initializes the sensor interface depending
 *   on selection either SPI or I2C.
 *
 *   @param[in] void
 *
 *  @return void
 *
 */
static void init_sensor_interface(void)
{

    /* Switch VDD for sensor off */
    coines_set_shuttleboard_vdd_vddio_config(0, 0);
    /* wait until the sensor goes off */
    coines_delay_msec(10);

#if BMA456_INTERFACE_I2C==1
    /* set the sensor interface as I2C */
    coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_STANDARD_MODE);

#elif BMA456_INTERFACE_SPI==1
    /* set the sensor interface as SPI */
    coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_5_MHZ,
                          COINES_SPI_MODE3);

#endif
    coines_delay_msec(10);
    /* Switch VDD for sensor on */
    coines_set_shuttleboard_vdd_vddio_config(3300, 3300);
}

/*!
 * @brief 	This internal API is used to initializes the bma456 and verify the 
 *			communication by reading the chip id.
 *
 * @param[in] void
 *
 * @return void
 *
 */
static void init_comm_test_bma456(void)
{
    int8_t rslt;

#if BMA456_INTERFACE_SPI==1
    /* BMA456 requires a dummy SPI transaction to switch to SPI mode at power-on */
    uint8_t dummybyte;
    bma4_write_regs(COINES_SHUTTLE_PIN_7, &dummybyte, 1, &bma456dev);
#endif

    rslt = bma456_init(&bma456dev);
    if (rslt == BMA4_OK)
    {
        printf("BMA456 Initialization Success!\n");
        printf("Test #1: Communication. PASSED. Chip ID 0x%x\n",
               bma456dev.chip_id);
    }

    else
    {
        char err_string[255];
        printf("BMA456 Initialization Failure!\n");

        if (BMA4_E_INVALID_SENSOR == rslt)
        {
            sprintf(err_string,
                    "Test #1: Communication. FAILED. Expected Chip ID: 0x%x. Received 0x%x\n",
                    BMA456_CHIP_ID, bma456dev.chip_id);
        }
        else
        {
            sprintf(err_string,
                    "Test #1: Communication. FAILED. No response from the sensor.");
        }

        coines_exit_error(err_string);
    }
    fflush(stdout);
    coines_delay_msec(100);
}
/*!
 * @brief 	This internal API is used to test if the sensor is working 
 *			by triggering the self-test.
 *
 * @param[in] void
 *
 * @return void
 *
 */
static void function_test_bma456(void)
{

    uint16_t commrslt;
    uint8_t testrslt;

    commrslt = bma4_perform_accel_selftest(&testrslt, &bma456dev);

    if (0 != commrslt)
        coines_exit_error(
                          "Test #2: Functionality. FAILED. Unknown communication error\n");

    if (BMA4_SELFTEST_PASS == testrslt)
    {
        printf("Test #2: Functionality. PASSED. Sensor self-test successful\n");
    }
    else
    {
        printf("Test #2: Functionality. FAILED. Sensor self-test failed\n");
    }
    fflush(stdout);
}

/*!
 * @brief 	This internal API is used to test if the sensor performance 
 *
 * @param[in] void
 *
 * @return void
 *
 */
static void perf_test_bma456(void)
{
    uint16_t commrslt;
    /* Declare an accelerometer configuration structure */
    struct bma4_accel_config accel_conf;
    /* Declare a data buffer for the sensor data structure */
    struct bma4_accel sens_data[1000];

    /* Enable the accelerometer */
    commrslt = bma4_set_accel_enable(BMA4_ENABLE, &bma456dev);

    if (0 != commrslt)
        coines_exit_error(
                          "Test #3: Performance. FAILED. Unknown communication error\n");

    /* Assign the desired settings */
    accel_conf.odr = BMA4_OUTPUT_DATA_RATE_100HZ; /* 100Hz data rate */
    accel_conf.range = BMA4_ACCEL_RANGE_4G; /* 4G range */
    accel_conf.bandwidth = BMA4_ACCEL_NORMAL_AVG4; /* normal mode */
    accel_conf.perf_mode = BMA4_CONTINUOUS_MODE; /* normal mode */

    /* Set the configuration */
    commrslt = bma4_set_accel_config(&accel_conf, &bma456dev);

    if (0 != commrslt)
        coines_exit_error(
                          "Test #3: Performance. FAILED. Unknown communication error\n");

    coines_delay_msec(100);
    /* getting some data points */
    for (int i = 0; i < 1000; ++i)
    {

        commrslt = bma4_read_accel_xyz(&sens_data[i], &bma456dev);

        if (0 != commrslt)
            coines_exit_error(
                              "Test #3: Performance. FAILED. Unknown communication error\n");

        if (0 == i % 10)
            printf("Getting data %d/1000\r", i);
        fflush(stdout);
        coines_delay_msec(10-COMMAND_RESPONSE_LATENCY);
    }

    /*! Average value calculation */
    double x_avg_mg = 0;
    double y_avg_mg = 0;
    double z_avg_mg = 0;
    for (int i = 0; i < 1000; ++i)
    {
        double xval, yval, zval;
        /* ( 'datapoint' * '4G' * '2' * '1000mg') / ('scaling factor for 16 bits') */
        xval = (((double)sens_data[i].x) * 4. * 2. * 1000) / pow(2, 16);
        yval = (((double)sens_data[i].y) * 4. * 2. * 1000) / pow(2, 16);
        zval = (((double)sens_data[i].z) * 4. * 2. * 1000) / pow(2, 16);

        x_avg_mg += xval / 1000.;
        y_avg_mg += yval / 1000.;
        z_avg_mg += zval / 1000.;
    }

    /*! Offset Calculation */
    double x_off_mg = x_avg_mg - X_REST_POSITION_MG;
    double y_off_mg = y_avg_mg - Y_REST_POSITION_MG;
    double z_off_mg = z_avg_mg - Z_REST_POSITION_MG;

    if ( OFFSET_THRESHOLD_MG > x_off_mg &&
    OFFSET_THRESHOLD_MG > y_off_mg
         &&
         OFFSET_THRESHOLD_MG > z_off_mg)
    {
        printf(
               "Test #3: Performance. Offset. PASSED: X=%4.1lfmg Y=%4.1lfmg Z=%4.1lfmg Threshold < %4.1lf\n",
               x_off_mg, y_off_mg, z_off_mg, OFFSET_THRESHOLD_MG);
    }
    else
    {
        printf(
               "Test #3: Performance. Offset. FAILED: X=%4.1lfmg Y=%4.1lfmg Z=%4.1lfmg Threshold < %4.1lf\n",
               x_off_mg, y_off_mg, z_off_mg, OFFSET_THRESHOLD_MG);
    }

    /*! RMS Noise Calculation */
    double x_rms_noise_mg = 0;
    double y_rms_noise_mg = 0;
    double z_rms_noise_mg = 0;
    for (int i = 0; i < 1000; ++i)
    {
        double xval, yval, zval;
        /* ( 'datapoint' * '4G' * '2' * '1000mg') / ('scaling factor for 16 bits') */
        xval = (((double)sens_data[i].x) * 4. * 2. * 1000.) / pow(2, 16);
        yval = (((double)sens_data[i].y) * 4. * 2. * 1000.) / pow(2, 16);
        zval = (((double)sens_data[i].z) * 4. * 2. * 1000.) / pow(2, 16);

        x_rms_noise_mg += pow(xval - x_avg_mg, 2) / 1000.;
        y_rms_noise_mg += pow(yval - y_avg_mg, 2) / 1000.;
        z_rms_noise_mg += pow(zval - z_avg_mg, 2) / 1000.;
    }

    /* rms noise is the square root of the arithmetic mean of the squares of the noise values */
    x_rms_noise_mg = sqrt(x_rms_noise_mg);
    y_rms_noise_mg = sqrt(y_rms_noise_mg);
    z_rms_noise_mg = sqrt(z_rms_noise_mg);

    /*! RMS Noise to RMS noise density convertion */
    /* noise density = RMS noise  / sqrt ( 1.22 * bandwidth) */
    /* BMA456 has 40.5Hz bandwidth at 100Hz normal mode */

    double x_noise_dens_ug = 1000 * x_rms_noise_mg / sqrt(1.22 * 40.5);
    double y_noise_dens_ug = 1000 * y_rms_noise_mg / sqrt(1.22 * 40.5);
    double z_noise_dens_ug = 1000 * z_rms_noise_mg / sqrt(1.22 * 40.5);

    if ( NOISE_THRESHOLD_MG > x_noise_dens_ug &&
    NOISE_THRESHOLD_MG > y_noise_dens_ug
         &&
         NOISE_THRESHOLD_MG > z_noise_dens_ug)
    {
        printf(
               "Test #3: Performance. Noise. PASSED: X=%4.1lfug/sqrt(Hz) Y=%4.1lfug/sqrt(Hz) Z=%4.1lfug/sqrt(Hz) Threshold < %4.1lf\n",
               x_noise_dens_ug, y_noise_dens_ug, z_noise_dens_ug,
               NOISE_THRESHOLD_MG);
    }
    else
    {
        printf(
               "Test #3: Performance. Noise. FAILED: X=%4.1lfug/sqrt(Hz) Y=%4.1lfug/sqrt(Hz) Z=%4.1lfug/sqrt(Hz) Threshold < %4.1lf\n",
               x_noise_dens_ug, y_noise_dens_ug, z_noise_dens_ug,
               NOISE_THRESHOLD_MG);
    }
    fflush(stdout);
}

/*!
 *  @brief This internal API is used to set the sensor driver interface to
 *  read/write the data.
 *
 *  @param[in] void
 *
 *  @return void
 */
static void init_bma456_sensor_driver_interface(void)
{
#if BMA456_INTERFACE_I2C==1
    /* I2C setup */
    /* link read/write/delay function of host system to appropriate
     * bma456 function call prototypes */
    bma456dev.bus_write = (bma4_com_fptr_t)coines_write_i2c;
    bma456dev.bus_read = (bma4_com_fptr_t)coines_read_i2c;
    bma456dev.delay = coines_delay_msec;
    bma456dev.interface = BMA4_I2C_INTERFACE;;
    /* set correct i2c address */
    bma456dev.dev_addr = BMA456_DEV_ADDR;
    bma456dev.read_write_len = 32;
#elif BMA456_INTERFACE_SPI==1
    /* SPI setup */
    /* link read/write/delay function of host system to appropriate
     *  bma456 function call prototypes */
    bma456dev.bus_write = (bma4_com_fptr_t)coines_write_spi;
    bma456dev.bus_read = (bma4_com_fptr_t)coines_read_spi;
    bma456dev.delay = coines_delay_msec;
    bma456dev.interface = BMA4_SPI_INTERFACE;
    ;
    /* shuttle board pin for CS */
    bma456dev.dev_addr = COINES_SHUTTLE_PIN_7;
    bma456dev.read_write_len = 32;
#endif
}

/*!
 *  @brief Main Function where the execution getting started to test the code.
 *
 *  @param[in] argc
 *  @param[in] argv
 *
 *  @return status
 *
 */
int main(int argc, char *argv[])
{
    int16_t rslt;
    struct coines_board_info board_info;

    init_bma456_sensor_driver_interface();

    rslt = coines_open_comm_intf(COINES_COMM_INTF_USB);

    if (rslt < 0)
    {
        printf(
               "\n Unable to connect with Application Board ! \n"
               " 1. Check if the board is connected and powered on. \n"
               " 2. Check if Application Board USB driver is installed. \n"
               " 3. Check if board is in use by another application. (Insufficient permissions to access USB) \n");
        exit(rslt);
    }

    rslt = coines_get_board_info(&board_info);

    if (rslt == COINES_SUCCESS)
    {
        if (board_info.shuttle_id != BMA4XY_SHUTTLE_ID)
        {

            printf("! Warning invalid sensor shuttle \n ,"
                   "This application will not support this sensor \n");
            exit(COINES_E_FAILURE);
        }
    }

    init_sensor_interface();

    /* after sensor init introduce 200 msec sleep */
    coines_delay_msec(200);

    /* initialize sensor and perform communication test */
    init_comm_test_bma456();

    /* performs sensor function test */
    function_test_bma456();

    /* performs sensor performance test */
    perf_test_bma456();

    /* end of the test */
    coines_set_shuttleboard_vdd_vddio_config(0, 0);
    coines_close_comm_intf(COINES_COMM_INTF_USB);
    fflush(stdout);
    return EXIT_SUCCESS;
}

