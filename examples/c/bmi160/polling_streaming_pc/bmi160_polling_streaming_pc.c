/**
 * Copyright (C) 2018 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bmi160_polling_streaming.c
 * @brief 	Sample file to stream bmi160 sensor data using LIB COINES
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
#include "bmi160.h"

/*********************************************************************/
/* local macro definitions */
/*********************************************************************/
/*! i2c interface communication, 1 - Enable; 0- Disable */
#define BMI160_INTERFACE_I2C             1
/*! spi interface communication, 1 - Enable; 0- Disable */
#define BMI160_INTERFACE_SPI             0

#if (!((BMI160_INTERFACE_I2C==1) && (BMI160_INTERFACE_SPI==0)) && \
	(!((BMI160_INTERFACE_I2C==0) && (BMI160_INTERFACE_SPI==1))))
#error "Invalid value given for the macros BMI160_INTERFACE_I2C / BMI160_INTERFACE_SPI"
#endif

/*! bmi160 shuttle id */
#define BMI160_SHUTTLE_ID         0x38

/*! bmi160 Device address */
#define BMI160_DEV_ADDR BMI160_I2C_ADDR

/*********************************************************************/
/* global variables */
/*********************************************************************/

/*! @brief This structure containing relevant bmi160 info */
struct bmi160_dev bmi160dev;
/*! accel streaming response  buffer */
uint8_t bmi160_accel_stream_buffer[COINES_STREAM_RSP_BUF_SIZE];
/*! gyro streaming response buffer */
uint8_t bmi160_gyro_stream_buffer[COINES_STREAM_RSP_BUF_SIZE];

/*********************************************************************/
/* function declarations */
/*********************************************************************/

/*!
 * @brief	internal API is used to initialize the sensor interface
 */
static void init_sensor_interface(void);

/*!
 * @brief 	This internal API is used to initialize the bmi160 sensor with default
 */
static void init_bmi160(void);

/*!
 * @brief	 This internal API is used to initialize the sensor driver interface
 */
static void init_bmi160_sensor_driver_interface(void);

/*!
 * @brief 	This internal API is used to send stream settings
 */
static void send_stream_settings(void);

/*!
 * @brief	This internal API is used to read sensor data
 */
void read_sensor_data(void);

/*********************************************************************/
/* functions */
/*********************************************************************/
/*!
 *  @brief This internal API is used to send stream settings
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void send_stream_settings(void)
{
    struct coines_streaming_config stream_config;

    struct coines_streaming_blocks stream_block;

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
    stream_config.sampling_time = 1250; /* For 800 Hz */
    stream_config.sampling_units = COINES_SAMPLING_TIME_IN_MICRO_SEC; /* Micro second */
    stream_block.no_of_blocks = 1;
    stream_block.reg_start_addr[0] = BMI160_ACCEL_DATA_ADDR; /* Accel data start address */

    stream_block.no_of_data_bytes[0] = 6;

    coines_config_streaming(1, &stream_config, &stream_block);

    stream_config.sampling_time = 625; /* 1.6 kHz */
    stream_block.reg_start_addr[0] = BMI160_GYRO_DATA_ADDR; /* Gyro data start address */

    coines_config_streaming(2, &stream_config, &stream_block);
}

/*!
 *  @brief This internal API is used to read sensor data
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
void read_sensor_data(void)
{
    int16_t rslt;
    int counter = 0;
    uint8_t lsb, msb;
    int16_t ax, ay, az, gx, gy, gz;
    uint32_t valid_sample_count = 0;
    int idx = 0;
    int buffer_index = 0;
    while (counter < 1000)
    {
        memset(&bmi160_accel_stream_buffer[0], 0, COINES_STREAM_RSP_BUF_SIZE);
        rslt = coines_read_stream_sensor_data(1, 1, &bmi160_accel_stream_buffer[0], &valid_sample_count);
        if (rslt == COINES_SUCCESS)
        {
            buffer_index = 0;
            for (idx = 0; idx < valid_sample_count; idx++)
            {
                lsb = bmi160_accel_stream_buffer[buffer_index++];
                msb = bmi160_accel_stream_buffer[buffer_index++];
                ax = (msb << 8) | lsb;

                lsb = bmi160_accel_stream_buffer[buffer_index++];
                msb = bmi160_accel_stream_buffer[buffer_index++];
                ay = (msb << 8) | lsb;

                lsb = bmi160_accel_stream_buffer[buffer_index++];
                msb = bmi160_accel_stream_buffer[buffer_index++];
                az = (msb << 8) | lsb;

                printf("ax:%d\tay:%d\taz:%d\n", ax, ay, az);
                fflush(stdout);
            }
        }
        memset(&bmi160_gyro_stream_buffer[0], 0, COINES_STREAM_RSP_BUF_SIZE);
        rslt = coines_read_stream_sensor_data(2, 1, &bmi160_gyro_stream_buffer[0], &valid_sample_count);
        if (rslt == COINES_SUCCESS)
        {
            buffer_index = 0;
            for (idx = 0; idx < valid_sample_count; idx++)
            {

                lsb = bmi160_gyro_stream_buffer[buffer_index++];
                msb = bmi160_gyro_stream_buffer[buffer_index++];
                gx = (msb << 8) | lsb;

                lsb = bmi160_gyro_stream_buffer[buffer_index++];
                msb = bmi160_gyro_stream_buffer[buffer_index++];
                gy = (msb << 8) | lsb;

                lsb = bmi160_gyro_stream_buffer[buffer_index++];
                msb = bmi160_gyro_stream_buffer[buffer_index++];
                gz = (msb << 8) | lsb;

                printf("gx:%d\tgy:%d\tgz:%d\n", gx, gy, gz);
                fflush(stdout);
            }
        }
        coines_delay_msec(1);
        counter++;
    }
}

/*!
 *  @brief This internal API is used to initialize the sensor interface depending
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
    /* wait until the sensor goes off */
    coines_delay_msec(10);
#if BMI160_INTERFACE_I2C==1
    /* SDO pin is made low for selecting I2C address 0x68 */
    coines_set_pin_config(COINES_SHUTTLE_PIN_15, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
    /* set the sensor interface as I2C */
    coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_FAST_MODE);
    coines_delay_msec(10);
    /* CSB pin is made high for selecting I2C protocol*/
    coines_set_pin_config(COINES_SHUTTLE_PIN_7, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
#endif
#if BMI160_INTERFACE_SPI==1
    /* CSB pin is made low for selecting SPI protocol*/
    coines_set_pin_config(COINES_SHUTTLE_PIN_7, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);

    coines_delay_msec(10);
    coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_5_MHZ, COINES_SPI_MODE3);
#endif
    coines_delay_msec(10);
    /* Switch VDD for sensor on */
    coines_set_shuttleboard_vdd_vddio_config(3300, 3300);

#if BMI160_INTERFACE_SPI==1
    coines_delay_msec(10);
    /* CSB pin is made high for selecting SPI protocol
     * Note: CSB has to see rising after power up, to switch to SPI protocol */
    coines_set_pin_config(COINES_SHUTTLE_PIN_7, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
#endif
}

/*!
 *  @brief This internal API is used to initializes the bmi160 sensor
 *  settings like power mode and OSRS settings.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void init_bmi160(void)
{
    int8_t rslt;
    rslt = bmi160_init(&bmi160dev);

    if (rslt == BMI160_OK)
    {
        printf("BMI160 initialization success !\n");
        printf("Chip ID 0x%X\n", bmi160dev.chip_id);
    }
    else
    {
        printf("BMI160 initialization failure !\n");
        exit(COINES_E_FAILURE);
    }

    /* Select the Output data rate, range of accelerometer sensor */
    bmi160dev.accel_cfg.odr = BMI160_ACCEL_ODR_800HZ;
    bmi160dev.accel_cfg.range = BMI160_ACCEL_RANGE_16G;
    bmi160dev.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

    /* Select the power mode of accelerometer sensor */
    bmi160dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    bmi160dev.gyro_cfg.odr = BMI160_GYRO_ODR_1600HZ;
    bmi160dev.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    bmi160dev.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

    /* Select the power mode of Gyroscope sensor */
    bmi160dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&bmi160dev);
}

/*!
 *  @brief This internal API is used to set the sensor driver interface to
 *  read/write the data.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void init_bmi160_sensor_driver_interface(void)
{
#if BMI160_INTERFACE_I2C==1
    /* I2C setup */
    /* link read/write/delay function of host system to appropriate
     * bmi160 function call prototypes */
    bmi160dev.write = coines_write_i2c;
    bmi160dev.read = coines_read_i2c;
    bmi160dev.delay_ms = coines_delay_msec;
    /* set correct i2c address */
    bmi160dev.id = BMI160_DEV_ADDR;
    bmi160dev.interface = BMI160_I2C_INTF;
#endif
#if BMI160_INTERFACE_SPI==1
    /* SPI setup */
    /* link read/write/delay function of host system to appropriate
     *  bmi160 function call prototypes */
    bmi160dev.write = coines_write_spi;
    bmi160dev.read = coines_read_spi;
    bmi160dev.delay_ms = coines_delay_msec;
    bmi160dev.id = COINES_SHUTTLE_PIN_7;
    bmi160dev.interface = BMI160_SPI_INTF;
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
    struct coines_board_info board_info;
    int16_t rslt;

    init_bmi160_sensor_driver_interface();

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
        if (board_info.shuttle_id != BMI160_SHUTTLE_ID)
        {

            printf("! Warning invalid sensor shuttle \n ,"
                   "This application will not support this sensor \n");
            exit(COINES_E_FAILURE);
        }
    }

    init_sensor_interface();
    /* after sensor init introduce 200 msec sleep */
    coines_delay_msec(200);

    init_bmi160();
    /*send streaming settings*/
    send_stream_settings();
    /*start interrupt streaming*/
    coines_start_stop_streaming(COINES_STREAMING_MODE_POLLING, COINES_STREAMING_START);
    /*read sensor data*/
    read_sensor_data();
    /*stop interrupt streaming*/
    coines_start_stop_streaming(COINES_STREAMING_MODE_POLLING, COINES_STREAMING_STOP);
    /*wait for 100 ms*/
    coines_delay_msec(100);
    /*close the communication*/
    coines_close_comm_intf(COINES_COMM_INTF_USB);
    return EXIT_SUCCESS;
}

