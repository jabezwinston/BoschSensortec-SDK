/**
 * Copyright (C) 2017 - 2018 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bma423_calibration.c
 * @brief 	Sample file to calibrate BMA423 offset using LIB COINES
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
#include "bma423.h"

/*********************************************************************/
/* macro definitions */
/*********************************************************************/
/*! i2c interface communication, 1 - Enable; 0- Disable */
#define BMA423_INTERFACE_I2C             0
/*! spi interface communication, 1 - Enable; 0- Disable */
#define BMA423_INTERFACE_SPI             1

#if (!((BMA423_INTERFACE_I2C==1) && (BMA423_INTERFACE_SPI==0)) && \
	(!((BMA423_INTERFACE_I2C==0) && (BMA423_INTERFACE_SPI==1))))
#error "Invalid value given for the macros BMA423_INTERFACE_I2C / BMA423_INTERFACE_SPI"
#endif
/*! BMA4xy shuttle board ID */
#define BMA4XY_SHUTTLE_ID		0x141

/*! This variable holds the device address of BMA423 */
#define BMA423_DEV_ADDR 		BMA4_I2C_ADDR_PRIMARY

/*! Defines rest position of the device. Adjust according to test conditions */
#define X_REST_POSITION_MG		0.0
#define Y_REST_POSITION_MG		0.0
#define Z_REST_POSITION_MG		1000.0

/*********************************************************************/
/* global variables */
/*********************************************************************/
/*! @brief This structure containing relevant bma423 info */
struct bma4_dev bma423dev;

/*********************************************************************/
/* function declarations */
/*********************************************************************/

/*!
 * @brief	 This internal API quits the program and outputs an error message
 */
static void coines_exit_error(const char*);

/*!
 * @brief	 This internal API is used to initialize the sensor interface
 */
static void init_sensor_interface(void);

/*!
 * @brief	This internal API is used to initialize the bma423 and calibrate offset
 */
static void bma423_calibration(void);

/*!
 * @brief	 This internal API is used to set the sensor driver interface to read/write the data
 */
static void init_bma423_sensor_driver_interface(void);

/*!
 * @brief	 This internal API is used to measure the sensor offset
 */
static void bma423_get_offset(double *, double *, double *);

/*!
 * @brief	 This internal API is used to update the nvm content
 */
static void bma423_update_nvm(void);


/*!
 * @brief	This internal API quits the program and outputs an error message
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

#if BMA423_INTERFACE_I2C==1
    /* set the sensor interface as I2C */
    coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_STANDARD_MODE);

#elif BMA423_INTERFACE_SPI==1
    /* set the sensor interface as SPI */
    coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_5_MHZ,
                          COINES_SPI_MODE3);

#endif
    coines_delay_msec(10);
    /* Switch VDD for sensor on */
    coines_set_shuttleboard_vdd_vddio_config(3300, 3300);
}

/*!
 * @brief 	This internal API is used to measure the sensor offset
 *
 * @param[out] x_off_mg, y_off_mg, z_off_mg
 *
 * @return void
 *
 */
static void bma423_get_offset(double *x_off_mg, double *y_off_mg,
                              double *z_off_mg)
{
    uint16_t commrslt;
    /* Declare an accelerometer configuration structure */
    struct bma4_accel_config accel_conf;
    /* Declare a data buffer for the sensor data structure */
    struct bma4_accel sens_data[20];

    /* Enable the accelerometer */
    commrslt = bma4_set_accel_enable(BMA4_ENABLE, &bma423dev);

    if (0 != commrslt)
        coines_exit_error("Unknown communication error. Exiting...\n");

    /* Assign the desired settings */
    accel_conf.odr = BMA4_OUTPUT_DATA_RATE_100HZ; /* 100Hz data rate */
    accel_conf.range = BMA4_ACCEL_RANGE_2G; /* 2G range */
    accel_conf.bandwidth = BMA4_ACCEL_NORMAL_AVG4; /* normal mode */
    accel_conf.perf_mode = BMA4_CONTINUOUS_MODE; /* normal mode */

    /* Set the configuration */
    commrslt = bma4_set_accel_config(&accel_conf, &bma423dev);

    if (0 != commrslt)
        coines_exit_error("Unknown communication error. Exiting...\n");

    coines_delay_msec(100);
    /* getting some data points */
    for (int i = 0; i < 20; ++i)
    {

        commrslt = bma4_read_accel_xyz(&sens_data[i], &bma423dev);

        if (0 != commrslt)
            coines_exit_error("Unknown communication error. Exiting...\n");

        coines_delay_msec(10);
    }

    /*! Average value calculation */
    double x_avg_mg = 0;
    double y_avg_mg = 0;
    double z_avg_mg = 0;
    for (int i = 0; i < 20; ++i)
    {
        double xval, yval, zval;
        /* ( 'datapoint' * '2G' * '2' * '1000mg') / ('scaling factor for 12 bits') */
        xval = (((double)sens_data[i].x) * 2. * 2. * 1000) / pow(2, 12);
        yval = (((double)sens_data[i].y) * 2. * 2. * 1000) / pow(2, 12);
        zval = (((double)sens_data[i].z) * 2. * 2. * 1000) / pow(2, 12);

        x_avg_mg += xval / 20.;
        y_avg_mg += yval / 20.;
        z_avg_mg += zval / 20.;
    }

    /*! Offset Calculation */
    *x_off_mg = x_avg_mg - X_REST_POSITION_MG;
    *y_off_mg = y_avg_mg - Y_REST_POSITION_MG;
    *z_off_mg = z_avg_mg - Z_REST_POSITION_MG;
}

/*!
 * @brief 	This internal API is used to update the nvm content
 *
 * @param[in] void
 *
 * @return void
 *
 */
static void bma423_update_nvm(void)
{
    uint16_t commrslt;
    uint8_t data;

    /* unlocks the NVM for writing */
    data = 0x02;
    bma4_write_regs(0x6A, &data, 1, &bma423dev);

    /* makes sure the BMA423 is not executing another command */
    do
    {
        commrslt = bma4_read_regs(BMA4_STATUS_ADDR, &data, 1, &bma423dev);
        if (0 != commrslt)
            coines_exit_error("Unknown communication error. Exiting...\n");
    }
    while (0 == (data & 0x10));

    /* performs the writing of the NVM */
    commrslt = bma4_set_command_register(0xA0, &bma423dev);
    if (0 != commrslt)
        coines_exit_error("Unknown communication error. Exiting...\n");

    /* wait for the command to be completed */
    do
    {
        commrslt = bma4_read_regs(BMA4_STATUS_ADDR, &data, 1, &bma423dev);
        if (0 != commrslt)
            coines_exit_error("Unknown communication error. Exiting...\n");
    }
    while (0 == (data & 0x10));

    /* locks the NVM writing */
    data = 0x00;
    bma4_write_regs(0x6A, &data, 1, &bma423dev);
}

/*!
 * @brief 	This internal API is used to initializes the bma423 and verify the 
 *			communication by reading the chip id.
 *
 * @param[in] void
 *
 * @return void
 *
 */
static void bma423_calibration(void)
{
    uint16_t commrslt;
    int8_t rslt;
    double x_off_mg, y_off_mg, z_off_mg;

#if BMA423_INTERFACE_SPI==1
    /* BMA423 requires a dummy SPI transaction to switch to SPI mode at power-on */
    uint8_t dummybyte;
    bma4_write_regs(COINES_SHUTTLE_PIN_7, &dummybyte, 1, &bma423dev);
#endif

    rslt = bma423_init(&bma423dev);

    if (rslt == BMA4_OK)
    {
        printf("BMA423 Initialization Success!\n");
    }
    else
    {
        coines_exit_error("BMA423 Initialization Failure!\n");
    }
    coines_delay_msec(100);

    /*! calculates the offset before compensation */
    bma423_get_offset(&x_off_mg, &y_off_mg, &z_off_mg);

    printf("\nPre-calibration offset : X=%4.1lfmg Y=%4.1lfmg Z=%4.1lfmg\n",
           x_off_mg,
           y_off_mg, z_off_mg);

    fflush(stdout);
    /*! performs fast offset compensation */
    const int32_t target_values[3] = { X_REST_POSITION_MG * 1000,
    Y_REST_POSITION_MG * 1000,
                                       X_REST_POSITION_MG * 1000 };
    commrslt = bma4_perform_accel_foc(target_values, &bma423dev);

    if (0 != commrslt)
        coines_exit_error("Unknown communication error. Exiting...\n");

    /*! calculates the offset after compensation */
    bma423_get_offset(&x_off_mg, &y_off_mg, &z_off_mg);
    
    printf("\nPost-calibration offset : X=%4.1lfmg Y=%4.1lfmg Z=%4.1lfmg", x_off_mg, y_off_mg, z_off_mg);
    
    
    /*! prompts the user before saving to NVM */
    char c;
    do {
        printf("\nDo you want to save this offset to to NVM? (Y/N): ");
        c = getc(stdin);
    } while ('y' != c && 'Y' != c && 'n' != c && 'N' != c);
    
    switch (c) {
        case 'y':
        case 'Y':
            bma423_update_nvm();
            printf("Offsets saved to NVM!\n");
            break;
            
        case 'n':
        case 'N':
        default:
            printf("Offsets not saved.\n");
            break;
    }   
}

/*!
 *  @brief This internal API is used to set the sensor driver interface to
 *  read/write the data.
 *
 *  @param[in] void
 *
 *  @return void
 */
static void init_bma423_sensor_driver_interface(void)
{
#if BMA423_INTERFACE_I2C==1
    /* I2C setup */
    /* link read/write/delay function of host system to appropriate
     * bma423 function call prototypes */
    bma423dev.bus_write = (bma4_com_fptr_t)coines_write_i2c;
    bma423dev.bus_read = (bma4_com_fptr_t)coines_read_i2c;
    bma423dev.delay = coines_delay_msec;
    bma423dev.interface = BMA4_I2C_INTERFACE;;
    /* set correct i2c address */
    bma423dev.dev_addr = BMA423_DEV_ADDR;
    bma423dev.read_write_len = 32;
#elif BMA423_INTERFACE_SPI==1
    /* SPI setup */
    /* link read/write/delay function of host system to appropriate
     *  bma423 function call prototypes */
    bma423dev.bus_write = (bma4_com_fptr_t)coines_write_spi;
    bma423dev.bus_read = (bma4_com_fptr_t)coines_read_spi;
    bma423dev.delay = coines_delay_msec;
    bma423dev.interface = BMA4_SPI_INTERFACE;
    ;
    /* shuttle board pin for CS */
    bma423dev.dev_addr = COINES_SHUTTLE_PIN_7;
    bma423dev.read_write_len = 32;
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

    init_bma423_sensor_driver_interface();

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

    /* initialize sensor and calibrate offset */
    bma423_calibration();
    fflush(stdout);
    /* end of the test */
    coines_set_shuttleboard_vdd_vddio_config(0, 0);
    coines_close_comm_intf(COINES_COMM_INTF_USB);

    return EXIT_SUCCESS;
}

