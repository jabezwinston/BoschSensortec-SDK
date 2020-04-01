/**
 * Copyright (C) 2018 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bmi08x_polling_streaming.c
 * @brief 	Sample file to stream bmi08x sensor data using LIB COINES
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
#include "bmi08x.h"

/*********************************************************************/
/* macro definitions */
/*********************************************************************/

/*! i2c interface communication, 1 - Enable; 0- Disable */
#define BMI08x_INTERFACE_I2C             1
/*! spi interface communication, 1 - Enable; 0- Disable */
#define BMI08x_INTERFACE_SPI             0

#if (!((BMI08x_INTERFACE_I2C==1) && (BMI08x_INTERFACE_SPI==0)) && \
	(!((BMI08x_INTERFACE_I2C==0) && (BMI08x_INTERFACE_SPI==1))))
#error "Invalid value given for the macros BMI08x_INTERFACE_I2C / BMI08x_INTERFACE_SPI"
#endif

/*! bmi085 shuttle id*/
#define BMI085_SHUTTLE_ID         0x46
/*! bmi088 shuttle id*/
#define BMI088_SHUTTLE_ID         0x66

/*! bmi08x Accel Device address */
#define BMI08x_ACCEL_DEV_ADDR BMI08X_ACCEL_I2C_ADDR_PRIMARY

/*! bmi08x Gyro Device address */
#define BMI08x_GYRO_DEV_ADDR BMI08X_GYRO_I2C_ADDR_PRIMARY

/*********************************************************************/
/* global variables */
/*********************************************************************/

/*! @brief This structure containing relevant bmi08x info */
struct bmi08x_dev bmi08xdev;
/*! accel streaming response  buffer */
uint8_t bmi08x_accel_stream_buffer[COINES_STREAM_RSP_BUF_SIZE];
/*! gyro streaming response buffer */
uint8_t bmi08x_gyro_stream_buffer[COINES_STREAM_RSP_BUF_SIZE];

/*********************************************************************/
/* function declarations */
/*********************************************************************/

/*!
 * @brief	internal API is used to initialize the sensor interface
 */
static void init_sensor_interface(void);

/*!
 * @brief	 This internal API is used to initialize the bmi08x sensor with default
 */
static void init_bmi08x(void);

/*!
 * @brief	This internal API is used to initialize the sensor driver interface
 */
static void init_bmi08x_sensor_driver_interface(void);

/*!
 * @brief	This internal API is used to send stream settings
 */
static void send_stream_settings(void);

/*! This internal API is used to read sensor data */
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

#if BMI08x_INTERFACE_I2C==1
    stream_config.intf = COINES_SENSOR_INTF_I2C;
#endif
#if BMI08x_INTERFACE_SPI==1
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
    stream_block.reg_start_addr[0] = BMI08X_ACCEL_X_LSB_REG; //accel data start address
#if BMI08x_INTERFACE_I2C==1
    stream_block.no_of_data_bytes[0] = 6;
#endif
#if BMI08x_INTERFACE_SPI==1
    stream_block.no_of_data_bytes[0] = 7; // actual data length is 6 bytes 1 byte needed to initiate the spi communication
#endif

    coines_config_streaming(1, &stream_config, &stream_block);

#if BMI08x_INTERFACE_I2C==1
    stream_config.intf = COINES_SENSOR_INTF_I2C;
#endif
#if BMI08x_INTERFACE_SPI==1
    stream_config.intf = COINES_SENSOR_INTF_SPI;
#endif
    stream_config.i2c_bus = COINES_I2C_BUS_0; //if intf is I2C
    stream_config.spi_bus = COINES_SPI_BUS_0; // if intf is SPI
    /*for I2C */
    stream_config.dev_addr = BMI08x_GYRO_DEV_ADDR;
    stream_config.cs_pin = COINES_SHUTTLE_PIN_14;
    stream_config.sampling_time = 500; //2 Khz
    /*1 micro second / 2 milli second*/
    stream_config.sampling_units = COINES_SAMPLING_TIME_IN_MICRO_SEC; //micro second
    stream_block.no_of_blocks = 1;
    stream_block.reg_start_addr[0] = BMI08X_GYRO_X_LSB_REG; //gyro data start address
    stream_block.no_of_data_bytes[0] = 6;

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
        memset(&bmi08x_accel_stream_buffer[0], 0, COINES_STREAM_RSP_BUF_SIZE);
        rslt = coines_read_stream_sensor_data(1, 1,
                                              &bmi08x_accel_stream_buffer[0],
                                              &valid_sample_count);
        if (rslt == COINES_SUCCESS)
        {
            buffer_index = 0;
            for (idx = 0; idx < valid_sample_count; idx++)
            {
#if BMI08x_INTERFACE_SPI==1
                buffer_index++; //dummy byte; ignore for spi
#endif
                lsb = bmi08x_accel_stream_buffer[buffer_index++];
                msb = bmi08x_accel_stream_buffer[buffer_index++];
                ax = (msb << 8) | lsb;

                lsb = bmi08x_accel_stream_buffer[buffer_index++];
                msb = bmi08x_accel_stream_buffer[buffer_index++];
                ay = (msb << 8) | lsb;

                lsb = bmi08x_accel_stream_buffer[buffer_index++];
                msb = bmi08x_accel_stream_buffer[buffer_index++];
                az = (msb << 8) | lsb;

                printf("ax: %-5d \t ay: %-5d \t az: %-5d\n", ax, ay, az);
                fflush(stdout);
            }
        }
        memset(&bmi08x_gyro_stream_buffer[0], 0, COINES_STREAM_RSP_BUF_SIZE);
        rslt = coines_read_stream_sensor_data(2, 1,
                                              &bmi08x_gyro_stream_buffer[0],
                                              &valid_sample_count);
        if (rslt == COINES_SUCCESS)
        {
            buffer_index = 0;
            for (idx = 0; idx < valid_sample_count; idx++)
            {

                lsb = bmi08x_gyro_stream_buffer[buffer_index++];
                msb = bmi08x_gyro_stream_buffer[buffer_index++];
                gx = (msb << 8) | lsb;

                lsb = bmi08x_gyro_stream_buffer[buffer_index++];
                msb = bmi08x_gyro_stream_buffer[buffer_index++];
                gy = (msb << 8) | lsb;

                lsb = bmi08x_gyro_stream_buffer[buffer_index++];
                msb = bmi08x_gyro_stream_buffer[buffer_index++];
                gz = (msb << 8) | lsb;

                printf("gx: %-5d \t gy: %-5d \t gz: %-5d\n", gx, gy, gz);
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
    coines_delay_msec(10);
#if BMI08x_INTERFACE_I2C==1
    /* set the sensor interface as I2C with 400kHz speed
     Use I2C Fast mode (400kHz) for reliable operation with high ODR/sampling time
     See Readme.txt - NOTE section for details*/
    coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_FAST_MODE);
    coines_delay_msec(10);
    /* PS pin is made high for selecting I2C protocol*/
    coines_set_pin_config(COINES_SHUTTLE_PIN_9, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
#endif
#if BMI08x_INTERFACE_SPI==1
    /* CS pin is made high for selecting SPI protocol*/
    coines_set_pin_config(COINES_SHUTTLE_PIN_8, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
    /* CS pin is made high for selecting SPI protocol*/
    coines_set_pin_config(COINES_SHUTTLE_PIN_14, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
    /* PS pin is made low for selecting SPI protocol*/
    coines_set_pin_config(COINES_SHUTTLE_PIN_9, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
    coines_delay_msec(10);
    coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_5_MHZ, COINES_SPI_MODE3);
#endif
    coines_delay_msec(10);
    /* Switch VDD for sensor on */
    coines_set_shuttleboard_vdd_vddio_config(3300, 3300);
}

/*!
 *  @brief This internal API is used to initializes the bmi08x sensor
 *  settings like power mode and OSRS settings.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void init_bmi08x(void)
{
    if (bmi08a_init(&bmi08xdev) == BMI08X_OK
        && bmi08g_init(&bmi08xdev) == BMI08X_OK)
    {
        printf("BMI08x initialization success !\n");
        printf("Accel chip ID - 0x%x\n", bmi08xdev.accel_chip_id);
        printf("Gyro chip ID - 0x%x\n", bmi08xdev.gyro_chip_id);
    }

    else
    {
        printf("BMI08x initialization failure !\n");
        exit(COINES_E_FAILURE);
    }
    coines_delay_msec(100);

    bmi08xdev.accel_cfg.odr = BMI08X_ACCEL_ODR_1600_HZ;
#if BMI08X_FEATURE_BMI085 == 1
    bmi08xdev.accel_cfg.range = BMI085_ACCEL_RANGE_16G;
#elif BMI08X_FEATURE_BMI088 == 1
    bmi08xdev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
#endif
    bmi08xdev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
    bmi08xdev.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;

    bmi08a_set_power_mode(&bmi08xdev);
    coines_delay_msec(10);
    bmi08a_set_meas_conf(&bmi08xdev);
    coines_delay_msec(10);

    bmi08xdev.gyro_cfg.odr = BMI08X_GYRO_BW_230_ODR_2000_HZ;
    bmi08xdev.gyro_cfg.range = BMI08X_GYRO_RANGE_250_DPS;
    bmi08xdev.gyro_cfg.bw = BMI08X_GYRO_BW_230_ODR_2000_HZ;
    bmi08xdev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;

    bmi08g_set_power_mode(&bmi08xdev);
    coines_delay_msec(10);
    bmi08g_set_meas_conf(&bmi08xdev);
    coines_delay_msec(10);

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
static void init_bmi08x_sensor_driver_interface(void)
{
#if BMI08x_INTERFACE_I2C==1
    /* I2C setup */
    /* link read/write/delay function of host system to appropriate
     * bmi08x function call prototypes */
    bmi08xdev.write = coines_write_i2c;
    bmi08xdev.read = coines_read_i2c;
    bmi08xdev.delay_ms = coines_delay_msec;
    /* set correct i2c address */
    bmi08xdev.accel_id = (unsigned char) BMI08x_ACCEL_DEV_ADDR;
    bmi08xdev.gyro_id = (unsigned char) BMI08x_GYRO_DEV_ADDR;
    bmi08xdev.intf = BMI08X_I2C_INTF;
#endif
#if BMI08x_INTERFACE_SPI==1
    /* SPI setup */
    /* link read/write/delay function of host system to appropriate
     *  bmi08x function call prototypes */
    bmi08xdev.write = coines_write_spi;
    bmi08xdev.read = coines_read_spi;
    bmi08xdev.delay_ms = coines_delay_msec;
    bmi08xdev.intf = BMI08X_SPI_INTF;
    bmi08xdev.accel_id = COINES_SHUTTLE_PIN_8;
    bmi08xdev.gyro_id = COINES_SHUTTLE_PIN_14;
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

    init_bmi08x_sensor_driver_interface();

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
        if ((board_info.shuttle_id != BMI085_SHUTTLE_ID)
            && (board_info.shuttle_id != BMI088_SHUTTLE_ID))
        {

            printf("! Warning invalid sensor shuttle \n ,"
                   "This application will not support this sensor \n");
            exit(COINES_E_FAILURE);
        }
    }

    init_sensor_interface();
    /* after sensor init introduce 200 msec sleep */
    coines_delay_msec(200);

    init_bmi08x();
    /*send streaming settings*/
    send_stream_settings();
    /*start polling streaming*/
    coines_start_stop_streaming(COINES_STREAMING_MODE_POLLING, COINES_STREAMING_START);
    /*read sensor data*/
    read_sensor_data();
    /*stop polling streaming*/
    coines_start_stop_streaming(COINES_STREAMING_MODE_POLLING, COINES_STREAMING_STOP);
    /*wait for 100 ms*/
    coines_delay_msec(100);
    /*close the communication*/
    coines_close_comm_intf(COINES_COMM_INTF_USB);
    return EXIT_SUCCESS;
}

