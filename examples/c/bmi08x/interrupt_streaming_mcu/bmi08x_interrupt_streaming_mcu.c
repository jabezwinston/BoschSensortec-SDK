/**
 * Copyright (C) 2018 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bmi08x_interrupt_streaming.c
 * @brief Sample file how to stream bmi08x sensor data using LIB COINES
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
#include <stdbool.h>
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

/*! @brief This structure containing relevant bmi08x info */
struct bmi08x_dev bmi08xdev;
/*! bmi08x accel int config */
struct bmi08x_accel_int_channel_cfg accel_int_config;
/*! bmi08x gyro int config */
struct bmi08x_gyro_int_channel_cfg gyro_int_config;
/*! @brief variable to hold the bmi08x accel data */
struct bmi08x_sensor_data bmi08x_accel;
/*! @brief variable to hold the bmi08x gyro data */
struct bmi08x_sensor_data bmi08x_gyro;
bool accel_drdy_int = false, gyro_drdy_int = false;

/*********************************************************************/
/* function declarations */
/*********************************************************************/

/*!
 * @brief	Accelerometer data ready interrupt callback
 */
void bmi08x_accel_drdy_int();
/*!
 * @brief	Gyroscope data ready interrupt callback
 */
void bmi08x_gyro_drdy_int();
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
 * @brief	 This internal API is used to enable the bmi08x interrupt
 */
static void enable_bmi08x_interrupt();
/*! This internal API is used to disable the bmi08x interrupt */
static void disable_bmi08x_interrupt();

/*********************************************************************/
/* functions */
/*********************************************************************/

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
    coines_set_pin_config(COINES_SHUTTLE_PIN_8, COINES_PIN_DIRECTION_OUT,
                          COINES_PIN_VALUE_HIGH);
    /* CS pin is made high for selecting SPI protocol*/
    coines_set_pin_config(COINES_SHUTTLE_PIN_14, COINES_PIN_DIRECTION_OUT,
                          COINES_PIN_VALUE_HIGH);
    /* PS pin is made low for selecting SPI protocol*/
    coines_set_pin_config(COINES_SHUTTLE_PIN_9, COINES_PIN_DIRECTION_OUT,
                          COINES_PIN_VALUE_LOW);
    coines_delay_msec(10);
    coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_5_MHZ,
                          COINES_SPI_MODE3);
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

    bmi08xdev.accel_cfg.odr = BMI08X_ACCEL_ODR_200_HZ;
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

    bmi08xdev.gyro_cfg.odr = BMI08X_GYRO_BW_23_ODR_200_HZ;
    bmi08xdev.gyro_cfg.range = BMI08X_GYRO_RANGE_250_DPS;
    bmi08xdev.gyro_cfg.bw = BMI08X_GYRO_BW_23_ODR_200_HZ;
    bmi08xdev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;

    bmi08g_set_power_mode(&bmi08xdev);
    coines_delay_msec(10);
    bmi08g_set_meas_conf(&bmi08xdev);
    coines_delay_msec(10);

    if (bmi08xdev.accel_cfg.power == BMI08X_ACCEL_PM_SUSPEND
        && (bmi08xdev.gyro_cfg.power == BMI08X_GYRO_PM_SUSPEND
            || bmi08xdev.gyro_cfg.power == BMI08X_GYRO_PM_DEEP_SUSPEND))
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
    rslt = bmi08a_set_int_config(
                                 (const struct bmi08x_accel_int_channel_cfg*)&accel_int_config,
                                 &bmi08xdev);

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
    bmi08g_set_int_config(
                          (const struct bmi08x_gyro_int_channel_cfg *)&gyro_int_config,
                          &bmi08xdev);

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
    rslt = bmi08a_set_int_config((const struct bmi08x_accel_int_channel_cfg*)&accel_int_config,
                                 &bmi08xdev);
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
    rslt = bmi08g_set_int_config((const struct bmi08x_gyro_int_channel_cfg *)&gyro_int_config,
                                 &bmi08xdev);

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

    coines_open_comm_intf(COINES_COMM_INTF_USB); /* Wait here till the PC connects to the COM port */

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

    coines_attach_interrupt(COINES_SHUTTLE_PIN_21, bmi08x_accel_drdy_int,
                            COINES_PIN_INTERRUPT_FALLING_EDGE);
    coines_attach_interrupt(COINES_SHUTTLE_PIN_22, bmi08x_gyro_drdy_int,
                            COINES_PIN_INTERRUPT_FALLING_EDGE);

    enable_bmi08x_interrupt();

    uint32_t start_time = coines_get_millis();

    /* Run interrupt streaming for 10s before disabling interrupts */
    while (coines_get_millis() - start_time < 10000)
    {
        if (accel_drdy_int == true
            && bmi08a_get_data(&bmi08x_accel, &bmi08xdev) == BMI08X_OK)
        {
            accel_drdy_int = false;
            printf("ax:%d ay:%d az:%d \tts:%lu\n", bmi08x_accel.x,
                   bmi08x_accel.y,
                   bmi08x_accel.z,
                   coines_get_millis() - start_time);
        }

        if (gyro_drdy_int == true
            && bmi08g_get_data(&bmi08x_gyro, &bmi08xdev) == BMI08X_OK)
        {
            gyro_drdy_int = false;
            printf("gx:%d gy:%d gz:%d \tts:%lu\n", bmi08x_gyro.x, bmi08x_gyro.y,
                   bmi08x_gyro.z,
                   coines_get_millis() - start_time);
        }
    }

    /*disable data ready interrupts*/

    coines_detach_interrupt(COINES_SHUTTLE_PIN_21);
    coines_detach_interrupt(COINES_SHUTTLE_PIN_22);

    disable_bmi08x_interrupt();

    /*close the communication*/
    coines_close_comm_intf(COINES_COMM_INTF_USB);

    return EXIT_SUCCESS;
}

void bmi08x_accel_drdy_int()
{
    accel_drdy_int = true;
}

void bmi08x_gyro_drdy_int()
{
    gyro_drdy_int = true;
}
