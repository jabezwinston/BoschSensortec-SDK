/**
 * Copyright (C) 2017 - 2018 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bme280_stream_sensor_data.c
 * @brief   Sample file how to read bme280 sensor data using LIB COINES
 *
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>

/*********************************************************************/
/* own header files */
/*********************************************************************/
#include "coines.h"
#include "bme280.h"

/*********************************************************************/
/* macro definitions */
/*********************************************************************/
/*! i2c interface communication, 1 - Enable; 0- Disable */
#define BME280_INTERFACE_I2C             1
/*! spi interface communication, 1 - Enable; 0- Disable */
#define BME280_INTERFACE_SPI             0

#if (!((BME280_INTERFACE_I2C==1) && (BME280_INTERFACE_SPI==0)) && \
	(!((BME280_INTERFACE_I2C==0) && (BME280_INTERFACE_SPI==1))))
#error "Invalid value given for the macros BME280_INTERFACE_I2C / BME280_INTERFACE_SPI"
#endif
/*! BME280 data pin */
#define BME280_SDO_PIN             8
/*! BME280 chip select pin */
#define BME280_CS_PIN       	   9
/*! BME280 shuttle board ID */
#define BME280_SHUTTLE_ID          0x33
/*! This variable holds the device address of BME280 */
#define BME280_DEV_ADDR BME280_I2C_ADDR_PRIM

/*********************************************************************/
/* global variables */
/*********************************************************************/
/*! @brief This structure containing relevant bme280 info */
struct bme280_dev bme280Dev;
/*! bme280 streaming response buffer */
uint8_t bme280_stream_buffer[COINES_STREAM_RSP_BUF_SIZE];

/*********************************************************************/
/* function declarations */
/*********************************************************************/

/*!
 * @brief	This internal API is used to initialize the sensor interface
 */
static void init_sensor_interface(void);
/*!
 * @brief	This internal API is used to initialize the bme280 sensor with default settings
 */
static void init_bme280(void);
/*!
 * @brief	This internal API is used to set the sensor driver interface to read/write the data
 */
static void init_bme280_sensor_driver_interface(void);
/*!
 * @brief	This internal API is used to read the streaming data in a while loop and print in console */
static void read_sensor_data(void);

/*!
 * @brief	This internal API is used to send the streaming settings to the board
 */
static void send_streaming_settings(void);

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

#if BME280_INTERFACE_I2C==1
    /* set the sensor interface as I2C */
    coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_STANDARD_MODE);

#elif BME280_INTERFACE_SPI==1
    /* set the sensor interface as SPI */
    coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_5_MHZ, COINES_SPI_MODE3);

#endif
    coines_delay_msec(10);
    /* Switch VDD for sensor on */
    coines_set_shuttleboard_vdd_vddio_config(3300, 3300);
}

/*!
 * @brief This internal API is used to initializes the bme280 sensor with default
 * settings like power mode and OSRS settings
 *
 * @param[in] void
 *
 * @return void
 *
 */
static void init_bme280(void)
{
    int8_t rslt;
    uint8_t settings_sel;

    rslt = bme280_init(&bme280Dev);
    if (rslt == BME280_OK)
    {
        printf("BME280 Initialization Success!\n");
        printf("Chip ID 0x%x\n", bme280Dev.chip_id);
    }
    else
    {
        printf("BME280 Initialization failure !\n");
        exit(COINES_E_FAILURE);
    }

    /* Recommended mode of operation: Indoor navigation */
    bme280Dev.settings.osr_h = BME280_OVERSAMPLING_1X;
    bme280Dev.settings.osr_p = BME280_OVERSAMPLING_16X;
    bme280Dev.settings.osr_t = BME280_OVERSAMPLING_2X;
    bme280Dev.settings.filter = BME280_FILTER_COEFF_16;
    bme280Dev.settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

    settings_sel = BME280_OSR_PRESS_SEL;
    settings_sel |= BME280_OSR_TEMP_SEL;
    settings_sel |= BME280_OSR_HUM_SEL;
    settings_sel |= BME280_STANDBY_SEL;
    settings_sel |= BME280_FILTER_SEL;

    bme280_set_sensor_settings(settings_sel, &bme280Dev);
    bme280_set_sensor_mode(BME280_NORMAL_MODE, &bme280Dev);
}

/*!
 *  @brief This internal API is used to set the sensor driver interface to
 *  read/write the data.
 *
 *  @param[in] void
 *
 *  @return void
 */
static void init_bme280_sensor_driver_interface(void)
{
#if BME280_INTERFACE_I2C==1
    /* I2C setup */
    /* link read/write/delay function of host system to appropriate
     * bme280 function call prototypes */
    bme280Dev.write = coines_write_i2c;
    bme280Dev.read = coines_read_i2c;
    bme280Dev.delay_ms = coines_delay_msec;
    bme280Dev.intf = BME280_I2C_INTF;
    /* set correct i2c address */
    bme280Dev.dev_id = BME280_DEV_ADDR;
#elif BME280_INTERFACE_SPI==1
    /* SPI setup */
    /* link read/write/delay function of host system to appropriate
     *  bme280 function call prototypes */
    bme280Dev.write = coines_write_spi;
    bme280Dev.read = coines_read_spi;
    bme280Dev.delay_ms = coines_delay_msec;
    bme280Dev.intf = BME280_SPI_INTF;
    bme280Dev.dev_id = BME280_CS_PIN;
#endif
}

/*!
 *  @breif This internal API is used to send the streaming settings to the board
 *
 *  @param[in] void
 *
 *  @return void
 *
 * */
static void send_streaming_settings(void)
{

    struct coines_streaming_config stream_config;

    struct coines_streaming_blocks stream_block;

#if BME280_INTERFACE_I2C==1
    stream_config.intf = COINES_SENSOR_INTF_I2C;
#endif
#if BME280_INTERFACE_SPI==1
    stream_config.intf = COINES_SENSOR_INTF_SPI;
#endif
    stream_config.i2c_bus = COINES_I2C_BUS_0; //if intf is I2C
    stream_config.spi_bus = COINES_SPI_BUS_0; // if intf is SPI
    /*for I2C */
    stream_config.dev_addr = BME280_DEV_ADDR;
    stream_config.cs_pin = BME280_CS_PIN;
    /* Use I2C fast mode when sampling time is less than 1250us */
    stream_config.sampling_time = 10; // 10Hz
    /*1 micro second / 2 milli second*/
    stream_config.sampling_units = COINES_SAMPLING_TIME_IN_MILLI_SEC; //millisecond
    stream_block.no_of_blocks = 1;
    stream_block.reg_start_addr[0] = BME280_DATA_ADDR; //bme280 data start address
    stream_block.no_of_data_bytes[0] = 8;

    coines_config_streaming(1, &stream_config, &stream_block);

}

/*!
 * @brief This internal API is used to read the streaming data in a while loop and
 * print in console
 *
 * @param[in] void
 *
 * @return void
 *
 * */
static void read_sensor_data()
{
    int16_t rslt;
    int times_to_read = 0;
    uint32_t valid_sample_count = 0;
    struct bme280_data bme280_comp_data;
    struct bme280_uncomp_data bme280_uncomp_data;
    while (times_to_read < 1000)
    {
        memset(&bme280_stream_buffer[0], 0, COINES_STREAM_RSP_BUF_SIZE);
        rslt = coines_read_stream_sensor_data(1, 1, &bme280_stream_buffer[0], &valid_sample_count);

        if (rslt == COINES_SUCCESS)
        {
            for (int idx = 0; idx < valid_sample_count; idx++)
            {

                bme280_parse_sensor_data(&bme280_stream_buffer[0], &bme280_uncomp_data);

                bme280_compensate_data(BME280_ALL, &bme280_uncomp_data, &bme280_comp_data, &bme280Dev.calib_data);

                printf("T: %.2f, H: %.2f, P: %.2f \n",
                       (bme280_comp_data.temperature / 100.),
                       (bme280_comp_data.humidity / 1000.),
                       (bme280_comp_data.pressure / 100.));
                fflush(stdout);
            }

            coines_delay_msec(10);

            times_to_read = times_to_read + 1;
        }
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
    init_bme280_sensor_driver_interface();

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
        if (board_info.shuttle_id != BME280_SHUTTLE_ID)
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

    init_bme280();

    send_streaming_settings();

    /*start polling  streaming*/
    coines_start_stop_streaming(COINES_STREAMING_MODE_POLLING, COINES_STREAMING_START);

    read_sensor_data();

    /*stop polling streaming*/
    coines_start_stop_streaming(COINES_STREAMING_MODE_POLLING, COINES_STREAMING_STOP);

    coines_delay_msec(200);

    coines_close_comm_intf(COINES_COMM_INTF_USB);
    return EXIT_SUCCESS;
}

