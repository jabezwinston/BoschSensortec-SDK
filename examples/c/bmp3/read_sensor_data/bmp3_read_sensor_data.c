/**
 * Copyright (C) 2017 - 2018 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bmp3_read_sensor_data.c
 * @brief 	Sample file to read bmp3 sensor data using LIB COINES
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
#include "bmp3.h"

/*********************************************************************/
/* macro definitions */
/*! i2c interface communication, 1 - Enable; 0- Disable */
#define BMP3_INTERFACE_I2C             1
/*! spi interface communication, 1 - Enable; 0- Disable */
#define BMP3_INTERFACE_SPI             0

#if (!((BMP3_INTERFACE_I2C==1) && (BMP3_INTERFACE_SPI==0)) && \
	(!((BMP3_INTERFACE_I2C==0) && (BMP3_INTERFACE_SPI==1))))
#error "Invalid value given for the macros BMP3_INTERFACE_I2C / BMP3_INTERFACE_SPI"
#endif

/*! BMP3 shuttle board ID */
#define BMP3_SHUTTLE_ID          0xD3
/*! This variable holds the device address of BMP3 */
#define BMP3_DEV_ADDR BMP3_I2C_ADDR_PRIM

/*********************************************************************/
/* global variables */
/*********************************************************************/
/*! @brief This structure containing relevant bmp3 info */
struct bmp3_dev bmp3Dev;
/*! @brief variable to hold the compensated sensor data*/
struct bmp3_data bmp3_comp_data;

/*********************************************************************/
/* function declarations */
/*********************************************************************/
/*!
 * @brief	internal API is used to initialize the sensor interface
 */
static void init_sensor_interface(void);

/*!
 * @brief	This internal API is used to initialize the bmp3 sensor with default settings
 */
static void init_bmp3(void);

/*!
 * @brief	This internal API is used to set the sensor driver interface to read/write the data
 */
static void init_bmp3_sensor_driver_interface(void);

/*!
 * @brief	This internal API is used to read the streaming data in a while loop and print in console
 */
static void read_sensor_data(void);

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

#if BMP3_INTERFACE_I2C==1
    /* set the sensor interface as I2C */
    coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_STANDARD_MODE);

#elif BMP3_INTERFACE_SPI==1
    /* set the sensor interface as SPI */
    coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_5_MHZ, COINES_SPI_MODE3);
#endif
    coines_delay_msec(10);
    /* Switch VDD for sensor on */
    coines_set_shuttleboard_vdd_vddio_config(3300, 3300);
}

/*!
 * @brief This internal API is used to initializes the bmp3 sensor with default
 * settings like power mode and OSRS settings
 *
 * @param[in] void
 *
 * @return void
 *
 */
static void init_bmp3(void)
{
    int8_t rslt;
    uint16_t settings_sel;

    rslt = bmp3_init(&bmp3Dev);
    if (rslt == BMP3_OK)
    {
        printf("BMP3 Initialization Success!\n");
        printf("Chip ID 0x%X\n", bmp3Dev.chip_id);
    }
    else
    {
        printf("BMP3 Initialization failure !\n");
        exit(COINES_E_FAILURE);
    }
    coines_delay_msec(100);

    /* Select the pressure and temperature sensor to be enabled */
    bmp3Dev.settings.press_en = BMP3_ENABLE;
    bmp3Dev.settings.temp_en = BMP3_ENABLE;
    /* Select the output data rate and oversampling settings for pressure and temperature */
    bmp3Dev.settings.odr_filter.press_os = BMP3_OVERSAMPLING_2X;
    bmp3Dev.settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
    bmp3Dev.settings.odr_filter.odr = BMP3_ODR_100_HZ;

    /* Assign the settings which needs to be set in the sensor */
    settings_sel = BMP3_PRESS_EN_SEL | BMP3_TEMP_EN_SEL | BMP3_PRESS_OS_SEL
                   | BMP3_TEMP_OS_SEL | BMP3_ODR_SEL;
    rslt = bmp3_set_sensor_settings(settings_sel, &bmp3Dev);

    /* Set the power mode to normal mode */
    bmp3Dev.settings.op_mode = BMP3_NORMAL_MODE;
    rslt = bmp3_set_op_mode(&bmp3Dev);
}

/*!
 *  @brief This internal API is used to set the sensor driver interface to
 *  read/write the data.
 *
 *  @param[in] void
 *
 *  @return void
 */
static void init_bmp3_sensor_driver_interface(void)
{
#if BMP3_INTERFACE_I2C==1
    /* I2C setup */
    /* link read/write/delay function of host system to appropriate
     * bmp3 function call prototypes */
    bmp3Dev.write = coines_write_i2c;
    bmp3Dev.read = coines_read_i2c;
    bmp3Dev.delay_ms = coines_delay_msec;
    bmp3Dev.intf = BMP3_I2C_INTF;
    /* set correct i2c address */
    bmp3Dev.dev_id = BMP3_DEV_ADDR;
#elif BMP3_INTERFACE_SPI==1
    /* SPI setup */
    /* link read/write/delay function of host system to appropriate
     *  bmp3 function call prototypes */
    bmp3Dev.write = coines_write_spi;
    bmp3Dev.read = coines_read_spi;
    bmp3Dev.delay_ms = coines_delay_msec;
    bmp3Dev.intf = BMP3_SPI_INTF;
    bmp3Dev.dev_id = COINES_SHUTTLE_PIN_7;
#endif
}

/*!
 * @brief This internal API is used to read the streaming data in a while loop and
 * print in console.
 *
 *  @param[in] void
 *
 *  @return void
 */
static void read_sensor_data(void)
{
    int times_to_read = 0;

    while (times_to_read < 200)
    {

        bmp3_get_sensor_data(BMP3_ALL, &bmp3_comp_data, &bmp3Dev);

        printf("T: %.2f, P: %.2f \n", (bmp3_comp_data.temperature / 100.),
               (bmp3_comp_data.pressure / 100.));
        fflush(stdout);

        coines_delay_msec(10);

        times_to_read = times_to_read + 1;
    }
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

    init_bmp3_sensor_driver_interface();

    rslt = coines_open_comm_intf(COINES_COMM_INTF_USB);

    if (rslt < 0)
    {
        printf("\n Unable to connect with Application Board ! \n"
               " 1. Check if the board is connected and powered on. \n"
               " 2. Check if Application Board USB driver is installed. \n"
               " 3. Check if board is in use by another application. (Insufficient permissions to access USB) \n");
        exit(rslt);
    }

    rslt = coines_get_board_info(&board_info);

    if (rslt == COINES_SUCCESS)
    {
        if (board_info.shuttle_id != BMP3_SHUTTLE_ID)
        {

            printf("! Warning invalid sensor shuttle. This application will not support this sensor \r\n"
                   "1.Check the sensor shuttle \r\n"
                   "2.Reset the board  \r\n");
            exit(COINES_E_FAILURE);
        }
    }

    init_sensor_interface();

    /* after sensor init introduce 200 msec sleep */
    coines_delay_msec(200);

    init_bmp3();

    read_sensor_data();

    coines_close_comm_intf(COINES_COMM_INTF_USB);
    return EXIT_SUCCESS;
}

