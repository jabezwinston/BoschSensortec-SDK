/**
 * Copyright (C) 2018 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bmi08x_interrupt_streaming.c
 * @brief	Sample file to stream bmi08x sensor data using LIB COINES
 *
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>
#include <signal.h>
/*********************************************************************/
/* own header files */
/*********************************************************************/
#include "coines.h"
#include "bmi08x.h"

/*! @brief Sample file how to stream bmi08x sensor data based on data ready interrupt using LIB COINES */

/*********************************************************************/
/* macro definitions */
/*********************************************************************/

/*! i2c interface communication, 1 - Enable; 0- Disable */
#define BMI08x_INTERFACE_I2C             0
/*! spi interface communication, 1 - Enable; 0- Disable */
#define BMI08x_INTERFACE_SPI             1

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
/*! bmi08x accel int config */
struct bmi08x_accel_int_channel_cfg accel_int_config;
/*! bmi08x gyro int config */
struct bmi08x_gyro_int_channel_cfg gyro_int_config;
/*!accel streaming configuration */
struct coines_streaming_config accel_stream_config;
/*! gyro stream configuration*/
struct coines_streaming_config gyro_stream_config;
/*! streaming accel sensor register block */
struct coines_streaming_blocks accel_stream_block;
/*! streaming gyro sensor register block */
struct coines_streaming_blocks gyro_stream_block;
/*********************************************************************/
/* function declarations */
/*********************************************************************/

/*!
 * @brief	This internal API is used to send stream settings
 */
static void send_stream_settings(void);
/*!
 * @brief	 This internal API is used to read sensor data
 */
void read_sensor_data(void);
/*!
 * @brief	internal API is used to initialize the sensor interface
 */
static void init_sensor_interface(void);
/*!
 * @brief	This internal API is used to initialize the bmi08x sensor
 */
static void init_bmi08x(void);
/*!
 * @brief	This internal API is used to initialize and map the sensor driver interface
 */
static void init_bmi08x_sensor_driver_interface(void);
/*!
 * @brief	This internal API is used to enable the bmi08x interrupt
 */
static void enable_bmi08x_interrupt();
/*!
 * @brief	 This internal API is used to disable the bmi08x interrupt
 */
static void disable_bmi08x_interrupt();

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
static void send_stream_settings()
{
#if BMI08x_INTERFACE_I2C==1
    accel_stream_config.intf = COINES_SENSOR_INTF_I2C;
#endif
#if BMI08x_INTERFACE_SPI==1
    accel_stream_config.intf = COINES_SENSOR_INTF_SPI;
#endif
    accel_stream_config.i2c_bus = COINES_I2C_BUS_0; //if intf is I2C
    accel_stream_config.spi_bus = COINES_SPI_BUS_0; // if intf is SPI
    /*for I2C */
    accel_stream_config.dev_addr = BMI08x_ACCEL_DEV_ADDR;
    accel_stream_config.cs_pin = COINES_SHUTTLE_PIN_8;
    accel_stream_config.int_pin = COINES_SHUTTLE_PIN_21;
    accel_stream_config.int_timestamp = 1;
    accel_stream_block.no_of_blocks = 1;
    accel_stream_block.reg_start_addr[0] = BMI08X_ACCEL_X_LSB_REG; //accel data start address
#if BMI08x_INTERFACE_I2C==1
            accel_stream_block.no_of_data_bytes[0] = 6;
#endif
#if BMI08x_INTERFACE_SPI==1
    accel_stream_block.no_of_data_bytes[0] = 7; // actual data length is 6 bytes 1 byte needed to initiate the spi communication
#endif
    coines_config_streaming(1, &accel_stream_config, &accel_stream_block);

#if BMI08x_INTERFACE_I2C==1
    gyro_stream_config.intf = COINES_SENSOR_INTF_I2C;
#endif
#if BMI08x_INTERFACE_SPI==1
    gyro_stream_config.intf = COINES_SENSOR_INTF_SPI;
#endif
    gyro_stream_config.i2c_bus = COINES_I2C_BUS_0; //if intf is I2C
    gyro_stream_config.spi_bus = COINES_SPI_BUS_0; // if intf is SPI
    /*for I2C */
    gyro_stream_config.dev_addr = BMI08x_GYRO_DEV_ADDR;
    gyro_stream_config.cs_pin = COINES_SHUTTLE_PIN_14;
    gyro_stream_config.int_pin = COINES_SHUTTLE_PIN_22;
    gyro_stream_config.int_timestamp = 1;
    gyro_stream_block.no_of_blocks = 1;
    gyro_stream_block.reg_start_addr[0] = BMI08X_GYRO_X_LSB_REG; //gyro data start address
    gyro_stream_block.no_of_data_bytes[0] = 6;

    coines_config_streaming(2, &gyro_stream_config, &gyro_stream_block);
}

/*!
 *  @brief This internal API is used to read sensor data
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
void read_sensor_data()
{
    int16_t rslt;
    int counter = 0;
    uint8_t lsb, msb;
    int16_t ax, ay, az, gx, gy, gz;
    uint32_t valid_sample_count = 0;
    uint32_t packet_count = 0;
    uint64_t accel_time_stamp = 0;
    uint64_t gyro_time_stamp = 0;
    int idx = 0;
    int buffer_index = 0;

    while (counter < 1000)
    {
        if (bmi08xdev.accel_cfg.power == BMI08X_ACCEL_PM_ACTIVE)
        {
            memset(&bmi08x_accel_stream_buffer[0], 0,
            COINES_STREAM_RSP_BUF_SIZE);
            rslt = coines_read_stream_sensor_data(1, 1,
                                                  &bmi08x_accel_stream_buffer[0],
                                                  &valid_sample_count);
            if (rslt == COINES_SUCCESS)
            {
                buffer_index = 0;
                for (idx = 0; idx < valid_sample_count; idx++)
                {
                    packet_count = 0;

                    packet_count |= bmi08x_accel_stream_buffer[buffer_index++] << 24;
                    packet_count |= bmi08x_accel_stream_buffer[buffer_index++] << 16;
                    packet_count |= bmi08x_accel_stream_buffer[buffer_index++] << 8;
                    packet_count |= bmi08x_accel_stream_buffer[buffer_index++];
#if BMI08x_INTERFACE_SPI==1
                    buffer_index++; //dummy byte;
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

                    if (accel_stream_config.int_timestamp)
                    {
                        accel_time_stamp = 0;

                        accel_time_stamp |= (uint64_t)bmi08x_accel_stream_buffer[buffer_index++] << 40;
                        accel_time_stamp |= (uint64_t)bmi08x_accel_stream_buffer[buffer_index++] << 32;
                        accel_time_stamp |= (uint64_t)bmi08x_accel_stream_buffer[buffer_index++] << 24;
                        accel_time_stamp |= (uint64_t)bmi08x_accel_stream_buffer[buffer_index++] << 16;
                        accel_time_stamp |= (uint64_t)bmi08x_accel_stream_buffer[buffer_index++] << 8;
                        accel_time_stamp |= (uint64_t)bmi08x_accel_stream_buffer[buffer_index++];
                    }
                    printf("a_pc:%8d   a_t(us):%12"PRIu64" \t ax:%-5d ay:%-5d az:%-5d\n",
                           packet_count, accel_time_stamp / 30, ax, ay, az);
                    fflush(stdout);
                }
            }
        }

        if (bmi08xdev.gyro_cfg.power == BMI08X_GYRO_PM_NORMAL)
        {
            memset(&bmi08x_gyro_stream_buffer[0], 0,
            COINES_STREAM_RSP_BUF_SIZE);
            rslt = coines_read_stream_sensor_data(2, 1,
                                                  &bmi08x_gyro_stream_buffer[0],
                                                  &valid_sample_count);
            if (rslt == COINES_SUCCESS)
            {
                buffer_index = 0;
                for (idx = 0; idx < valid_sample_count; idx++)
                {

                    packet_count = 0;

                    packet_count |= bmi08x_gyro_stream_buffer[buffer_index++] << 24;
                    packet_count |= bmi08x_gyro_stream_buffer[buffer_index++] << 16;
                    packet_count |= bmi08x_gyro_stream_buffer[buffer_index++] << 8;
                    packet_count |= bmi08x_gyro_stream_buffer[buffer_index++];

                    lsb = bmi08x_gyro_stream_buffer[buffer_index++];
                    msb = bmi08x_gyro_stream_buffer[buffer_index++];
                    gx = (msb << 8) | lsb;

                    lsb = bmi08x_gyro_stream_buffer[buffer_index++];
                    msb = bmi08x_gyro_stream_buffer[buffer_index++];
                    gy = (msb << 8) | lsb;

                    lsb = bmi08x_gyro_stream_buffer[buffer_index++];
                    msb = bmi08x_gyro_stream_buffer[buffer_index++];
                    gz = (msb << 8) | lsb;

                    if (gyro_stream_config.int_timestamp)
                    {
                        gyro_time_stamp = 0;
                        gyro_time_stamp |= (uint64_t)bmi08x_gyro_stream_buffer[buffer_index++] << 40;
                        gyro_time_stamp |= (uint64_t)bmi08x_gyro_stream_buffer[buffer_index++] << 32;
                        gyro_time_stamp |= (uint64_t)bmi08x_gyro_stream_buffer[buffer_index++] << 24;
                        gyro_time_stamp |= (uint64_t)bmi08x_gyro_stream_buffer[buffer_index++] << 16;
                        gyro_time_stamp |= (uint64_t)bmi08x_gyro_stream_buffer[buffer_index++] << 8;
                        gyro_time_stamp |= (uint64_t)bmi08x_gyro_stream_buffer[buffer_index++];
                    }
                    printf("g_pc:%8d   g_t(us):%12"PRIu64" \t gx:%-5d gy:%-5d gz:%-5d\n",
                           packet_count, gyro_time_stamp / 30, gx, gy, gz);
                    fflush(stdout);
                }
            }
        }
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
    /* SDO pin is made low for selecting I2C address 0x76*/
    coines_set_pin_config(COINES_SHUTTLE_PIN_8, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
    /* set the sensor interface as I2C with 400kHz speed
     Use I2C Fast mode (400kHz) for reliable operation with high ODR/sampling time
     See Readme.txt - NOTE section for details*/
    coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_FAST_MODE);
    coines_delay_msec(10);
    /* PS pin is made high for selecting I2C protocol (gyroscope)*/
    coines_set_pin_config(COINES_SHUTTLE_PIN_9, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
#endif
#if BMI08x_INTERFACE_SPI==1
    /* CSB1 pin is made high for selecting SPI protocol (accelerometer)*/
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
        fflush(stdout);
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

    if (bmi08xdev.accel_cfg.power == BMI08X_ACCEL_PM_SUSPEND
        && (bmi08xdev.gyro_cfg.power == BMI08X_GYRO_PM_SUSPEND || bmi08xdev.gyro_cfg.power == BMI08X_GYRO_PM_DEEP_SUSPEND))
    {
        printf(
               "Accel. and gyro sensors are in suspend mode\n Use them in active/normal mode !!");
        exit(EXIT_FAILURE);
    }

}
/*!
 *  @brief This API is used to enable bmi08x interrupt
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void enable_bmi08x_interrupt()
{
    int8_t rslt;
    /*set accel interrupt pin configuration*/
    accel_int_config.int_channel = BMI08X_INT_CHANNEL_1;
    accel_int_config.int_type = BMI08X_ACCEL_DATA_RDY_INT;
    accel_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
    accel_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    accel_int_config.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;
    /*Enable accel data ready interrupt channel*/
    rslt = bmi08a_set_int_config((const struct bmi08x_accel_int_channel_cfg*)&accel_int_config, &bmi08xdev);

    if (rslt != BMI08X_OK)
    {
        printf("BMI08x enable accel interrupt configuration failure!\n");
        exit(COINES_E_FAILURE);
    }

    /*set gyro interrupt pin configuration*/
    gyro_int_config.int_channel = BMI08X_INT_CHANNEL_3;
    gyro_int_config.int_type = BMI08X_GYRO_DATA_RDY_INT;
    gyro_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
    gyro_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    gyro_int_config.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;
    /*Enable gyro data ready interrupt channel*/
    bmi08g_set_int_config((const struct bmi08x_gyro_int_channel_cfg *)&gyro_int_config, &bmi08xdev);

    if (rslt != BMI08X_OK)
    {
        printf("BMI08x enable gyro interrupt configuration failure!\n");
        exit(COINES_E_FAILURE);
    }
}

/*!
 *  @brief This API is used to disable bmi08x interrupt
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void disable_bmi08x_interrupt()
{
    int8_t rslt;

    /*set accel interrupt pin configuration*/
    accel_int_config.int_channel = BMI08X_INT_CHANNEL_1;
    accel_int_config.int_type = BMI08X_ACCEL_DATA_RDY_INT;
    accel_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
    accel_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    accel_int_config.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;
    /*Disable accel data ready interrupt channel*/
    rslt = bmi08a_set_int_config((const struct bmi08x_accel_int_channel_cfg*)&accel_int_config, &bmi08xdev);
    if (rslt != BMI08X_OK)
    {
        printf("BMI08x disable accel interrupt configuration failure!\n");
    }
    /*set gyro interrupt pin configuration*/
    gyro_int_config.int_channel = BMI08X_INT_CHANNEL_3;
    gyro_int_config.int_type = BMI08X_GYRO_DATA_RDY_INT;
    gyro_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
    gyro_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    gyro_int_config.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;
    /*Disable gyro data ready interrupt channel*/
    rslt = bmi08g_set_int_config((const struct bmi08x_gyro_int_channel_cfg *)&gyro_int_config, &bmi08xdev);

    if (rslt != BMI08X_OK)
    {
        printf("BMI08x disable gyro interrupt configuration failure!\n");
    }

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
    /*initialize the sensor*/
    init_bmi08x();
    /*send streaming settings*/
    send_stream_settings();
    /*start system timer*/
    coines_trigger_timer(COINES_TIMER_START, COINES_TIMESTAMP_ENABLE);
    /*wait for 10 ms*/
    coines_delay_msec(10);
    /*start interrupt streaming*/
    coines_start_stop_streaming(COINES_STREAMING_MODE_INTERRUPT, COINES_STREAMING_START);
    /*Enable data ready interrupts*/
    enable_bmi08x_interrupt();
    /*read sensor data*/
    read_sensor_data();
    /*disable data ready interrupts*/
    disable_bmi08x_interrupt();
    /*stop interrupt streaming*/
    coines_start_stop_streaming(COINES_STREAMING_MODE_INTERRUPT, COINES_STREAMING_STOP);
    /*stop timer*/
    coines_trigger_timer(COINES_TIMER_STOP, COINES_TIMESTAMP_DISABLE);
    /*wait for 100 ms*/
    coines_delay_msec(100);
    /*close the communication*/
    coines_close_comm_intf(COINES_COMM_INTF_USB);

    return EXIT_SUCCESS;
}

