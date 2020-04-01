/**
 * Copyright (C) 2017 - 2018 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bme680_read_chip_id.c
 * @brief   Sample file how to read bme680 sensor data using LIB COINES
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
#include "bme680.h"

/*********************************************************************/
/* macro definitions */
/*********************************************************************/
/*! i2c interface communication, 1 - Enable; 0- Disable */
#define BME680_INTERFACE_I2C             1
/*! spi interface communication, 1 - Enable; 0- Disable */
#define BME680_INTERFACE_SPI             0

#if (!((BME680_INTERFACE_I2C==1) && (BME680_INTERFACE_SPI==0)) && \
        (!((BME680_INTERFACE_I2C==0) && (BME680_INTERFACE_SPI==1))))
#error "Invalid value given for the macros BME680_INTERFACE_I2C / BME680_INTERFACE_SPI"
#endif
/*! BME680 shuttle board ID */
#define BME680_SHUTTLE_ID          0x93
/*! This variable holds the device address of BME680 */
#define BME680_DEV_ADDR BME680_I2C_ADDR_PRIMARY

/*********************************************************************/
/* global variables */
/*********************************************************************/
/*! @brief This structure containing relevant bme680 info */
struct bme680_dev bme680Dev;

/*********************************************************************/
/* function declarations */
/*********************************************************************/

/*!
 * @brief	This internal API is used to initialize the sensor interface
 */
static void init_sensor_interface(void);

/*!
 * @brief	This internal API is used to initialize the bme680 sensor with default settings
 */
static void init_bme680(void);

/*!
 * @brief	 This internal API is used to set the sensor driver interface to read/write the data
 */
static void init_bme680_sensor_driver_interface(void);

/*********************************************************************/
/* functions */
/*********************************************************************/
/*!
 *  @brief This internal API is used to initializes the sensor interface depending
 *   on selection either SPI or I2C.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void init_sensor_interface(void)
{
    /* Switch VDD for sensor off */
    coines_set_shuttleboard_vdd_vddio_config(0, 0);
    coines_delay_msec(10);

#if BME680_INTERFACE_I2C==1
    /* set the sensor interface as I2C */
    coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_STANDARD_MODE);

#elif BME680_INTERFACE_SPI==1
    /* set the sensor interface as SPI */
    coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_5_MHZ, COINES_SPI_MODE3);

#endif
    coines_delay_msec(10);
    /* Switch VDD for sensor on */
    coines_set_shuttleboard_vdd_vddio_config(3300, 3300);
}

/*!
 * @brief This internal API is used to initializes the bme680 sensor with default
 * settings like power mode and OSRS settings
 *
 * @param[in] void
 *
 * @return void
 *
 */
static void init_bme680(void)
{
    int8_t rslt;
    rslt = bme680_init(&bme680Dev);
    if (rslt == BME680_OK)
    {
        printf("BME680 Initialization Success!\n");
        printf("Chip ID 0x%x\n", bme680Dev.chip_id);
        fflush(stdout);
    }
    else
    {
        printf("Chip Initialization failure !\n");
        fflush(stdout);
        exit(COINES_E_FAILURE);
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
static void init_bme680_sensor_driver_interface(void)
{
#if BME680_INTERFACE_I2C==1
    /* I2C setup */
    /* set correct i2c address */
    bme680Dev.dev_id = BME680_DEV_ADDR;
    bme680Dev.intf = BME680_I2C_INTF;
    /* link read/write/delay function of host system to appropriate
     * bme680 function call prototypes */
    bme680Dev.read = coines_read_i2c;
    bme680Dev.write = coines_write_i2c;
    bme680Dev.delay_ms = coines_delay_msec;

#elif BME680_INTERFACE_SPI==1
    /* SPI setup */
    bme680Dev.dev_id = 9;
    bme680Dev.intf = BME680_SPI_INTF;
    /* link read/write/delay function of host system to appropriate
     *  bme680 function call prototypes */
    bme680Dev.read = coines_read_spi;
    bme680Dev.write = coines_write_spi;
    bme680Dev.delay_ms = coines_delay_msec;

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

    init_bme680_sensor_driver_interface();

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
        if (board_info.shuttle_id != BME680_SHUTTLE_ID)
        {

            printf("! Warning invalid sensor shuttle \n ,"
                   "This application will not support this sensor \n");
            fflush(stdout);
            exit(COINES_E_FAILURE);
        }
    }

    init_sensor_interface();

    /* after sensor interface init introduce 200 msec sleep */
    coines_delay_msec(200);

    init_bme680();

    coines_close_comm_intf(COINES_COMM_INTF_USB);
    return EXIT_SUCCESS;
}

