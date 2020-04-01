/**
 * Copyright (C) 2018 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    read_synchronized_data_mcu.c
 * @brief 	Sample file how to read bmi08x synchronized sensor data using LIB COINES
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
#include "bmi085.h"
#include "bmi088.h"

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
bool data_sync_int = false;
/*! @brief This structure containing relevant bmi08x info */
struct bmi08x_dev bmi08xdev;
/*! bmi08x int config */
struct bmi08x_int_cfg int_config;
/*Data Sync configuration object*/
struct bmi08x_data_sync_cfg sync_cfg;
/*! @brief variable to hold the bmi08x accel data */
struct bmi08x_sensor_data bmi08x_accel;
/*! @brief variable to hold the bmi08x gyro data */
struct bmi08x_sensor_data bmi08x_gyro;
/*********************************************************************/
/* function declarations */
/*********************************************************************/

/*!
 * @brief	 internal API is used to initialize the sensor interface
 */
static void init_sensor_interface(void);
/*!
 * @brief	 This internal API is used to initialize the bmi08x sensor
 */
static void init_bmi08x(void);
/*!
 * @brief	 This internal API is used to initialize the sensor driver interface
 */
static void init_bmi08x_sensor_driver_interface(void);
/*!
 * @brief	 BMI08x data sync. interrupt callback
 */
void bmi08x_data_sync_int();

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
    /* set the sensor interface as I2C with 400kHz speed
     Use I2C Fast mode (400kHz) for reliable operation with high ODR/sampling time */
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
    coines_set_pin_config(COINES_SHUTTLE_PIN_20, COINES_PIN_DIRECTION_IN, COINES_PIN_VALUE_HIGH);
#endif
    coines_delay_msec(10);
    /* Switch VDD for sensor on */
    coines_set_shuttleboard_vdd_vddio_config(3300, 3300);
}

/*!
 *  @brief This internal API is used to initializes the bmi08x sensor with default.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void init_bmi08x(void)
{
    int16_t rslt;

    /* Initialize bmi08x sensors (accel & gyro)*/
#if BMI08X_FEATURE_BMI085 == 1
    rslt = bmi085_init(&bmi08xdev);
#elif BMI08X_FEATURE_BMI088 == 1
    rslt = bmi088_init(&bmi08xdev);
#endif
    if (rslt == BMI08X_OK)
    {
        printf("BMI08x initialization success!\n");
        printf("Accel chip ID - 0x%x\n", bmi08xdev.accel_chip_id);
        printf("Gyro chip ID - 0x%x\n", bmi08xdev.gyro_chip_id);
        /* Reset the accelerometer */
        rslt = bmi08a_soft_reset(&bmi08xdev);
        /* Wait for 1 ms - delay taken care inside the function*/
    }
    else
    {
        printf("BMI08x initialization failure!\n");
        exit(COINES_E_FAILURE);
    }
    /*! Max read/write length (maximum supported length is 32).
     To be set by the user */
    bmi08xdev.read_write_len = 32;
    /*set accel power mode */
    bmi08xdev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
    rslt = bmi08a_set_power_mode(&bmi08xdev);

    if (rslt == BMI08X_OK)
    {
        bmi08xdev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
        bmi08g_set_power_mode(&bmi08xdev);
    }

    if ((bmi08xdev.accel_cfg.power != BMI08X_ACCEL_PM_ACTIVE)
        || (bmi08xdev.gyro_cfg.power != BMI08X_GYRO_PM_NORMAL))
    {
        printf("Accel/gyro sensor in suspend mode\nUse in active/normal mode !!");
        exit(EXIT_FAILURE);
    }
    printf("Uploading BMI08X data synchronization feature config !\n");

    /*API uploads the bmi08x config file onto the device*/
    if (rslt == BMI08X_OK)
    {
#if BMI08X_FEATURE_BMI085 == 1
        rslt = bmi085_apply_config_file(&bmi08xdev);
#elif BMI08X_FEATURE_BMI088==1
        rslt = bmi088_apply_config_file(&bmi08xdev);
#endif
        /* Wait for 150ms to enable the data synchronization --delay taken care inside the function */
        if (rslt == BMI08X_OK)
        {
            /*for data synchronization bandwidth and odr will be configured based on the selected sync mode  */
#if BMI08X_FEATURE_BMI085 == 1
            /*assign accel range setting*/
            bmi08xdev.accel_cfg.range = BMI085_ACCEL_RANGE_4G;
#elif BMI08X_FEATURE_BMI088 == 1
            bmi08xdev.accel_cfg.range=BMI088_ACCEL_RANGE_24G;
#endif
            /*assign gyro range setting*/
            bmi08xdev.gyro_cfg.range = BMI08X_GYRO_RANGE_2000_DPS;
            /*! Mode (0 = off, 1 = 400Hz, 2 = 1kHz, 3 = 2kHz) */
            sync_cfg.mode = BMI08X_ACCEL_DATA_SYNC_MODE_400HZ;
#if BMI08X_FEATURE_BMI085 == 1
            rslt = bmi085_configure_data_synchronization(sync_cfg, &bmi08xdev);
#elif BMI08X_FEATURE_BMI088 == 1
            rslt = bmi088_configure_data_synchronization(sync_cfg, &bmi08xdev);
#endif
        }
    }
    if (rslt == BMI08X_OK)
    {
        printf("BMI08x data synchronization feature configured !\n");
    }
    else
    {
        printf("BMI08x data synchronization feature configuration failure!\n");
        exit(COINES_E_FAILURE);
    }
}

/*!
 *  @brief This internal API is used to enable data synchronization interrupt.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void enable_bmi08x_data_synchronization_interrupt()
{
    int8_t rslt = BMI08X_OK;

    /*set accel interrupt pin configuration*/
    /*configure host data ready interrupt */
    int_config.accel_int_config_1.int_channel = BMI08X_INT_CHANNEL_1;
    int_config.accel_int_config_1.int_type = BMI08X_ACCEL_SYNC_INPUT;
    int_config.accel_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
    int_config.accel_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

    /*configure Accel syncronization input interrupt pin */
    int_config.accel_int_config_2.int_channel = BMI08X_INT_CHANNEL_2;
    int_config.accel_int_config_2.int_type = BMI08X_ACCEL_SYNC_DATA_RDY_INT;
    int_config.accel_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
    int_config.accel_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

    /*set gyro interrupt pin configuration*/
    int_config.gyro_int_config_1.int_channel = BMI08X_INT_CHANNEL_3;
    int_config.gyro_int_config_1.int_type = BMI08X_GYRO_DATA_RDY_INT;
    int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;
    int_config.gyro_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    int_config.gyro_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

    int_config.gyro_int_config_2.int_channel = BMI08X_INT_CHANNEL_4;
    int_config.gyro_int_config_2.int_type = BMI08X_GYRO_DATA_RDY_INT;
    int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;
    int_config.gyro_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    int_config.gyro_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
#if BMI08X_FEATURE_BMI085 == 1
    /* Enable synchronization interrupt pin */
    rslt = bmi085_set_data_sync_int_config(&int_config, &bmi08xdev);
#elif BMI08X_FEATURE_BMI088==1
    /* Enable synchronization interrupt pin */
    rslt = bmi088_set_data_sync_int_config(&int_config, &bmi08xdev);
#endif

    if (rslt != BMI08X_OK)
    {
        printf("BMI085 data synchronization enable interrupt configuration failure!\n");
        exit(COINES_E_FAILURE);
    }
}

/*!
 *  @brief This internal API is used to disable data synchronization interrupt.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void disable_bmi08x_data_synchronization_interrupt()
{
    int8_t rslt;
    /*turn off the sync feature*/
    sync_cfg.mode = BMI08X_ACCEL_DATA_SYNC_MODE_OFF;
#if BMI08X_FEATURE_BMI085 == 1
    rslt = bmi085_configure_data_synchronization(sync_cfg, &bmi08xdev);
#elif BMI08X_FEATURE_BMI088==1
    rslt = bmi088_configure_data_synchronization(sync_cfg, &bmi08xdev);
#endif
    /* Wait for 150ms to enable the data synchronization --delay taken care inside the function */
    /* configure synchronization interrupt pins */
    if (rslt == BMI08X_OK)
    {
        /*set accel interrupt pin configuration*/
        /*configure host data ready interrupt */
        int_config.accel_int_config_1.int_channel = BMI08X_INT_CHANNEL_1;
        int_config.accel_int_config_1.int_type = BMI08X_ACCEL_SYNC_INPUT;
        int_config.accel_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
        int_config.accel_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
        int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

        /*configure Accel synchronization input interrupt pin */
        int_config.accel_int_config_2.int_channel = BMI08X_INT_CHANNEL_2;
        int_config.accel_int_config_2.int_type = BMI08X_ACCEL_SYNC_DATA_RDY_INT;
        int_config.accel_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
        int_config.accel_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
        int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

        /*set gyro interrupt pin configuration*/
        int_config.gyro_int_config_1.int_channel = BMI08X_INT_CHANNEL_3;
        int_config.gyro_int_config_1.int_type = BMI08X_GYRO_DATA_RDY_INT;
        int_config.gyro_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
        int_config.gyro_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
        int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

        int_config.gyro_int_config_2.int_channel = BMI08X_INT_CHANNEL_4;
        int_config.gyro_int_config_2.int_type = BMI08X_GYRO_DATA_RDY_INT;
        int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;
        int_config.gyro_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
        int_config.gyro_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

        /* Disable synchronization interrupt pin */
#if BMI08X_FEATURE_BMI085 == 1
        rslt = bmi085_set_data_sync_int_config(&int_config, &bmi08xdev);
#elif BMI08X_FEATURE_BMI088==1
        rslt = bmi088_set_data_sync_int_config(&int_config, &bmi08xdev);
#endif
    }
    if (rslt != BMI08X_OK)
    {
        printf(
               "BMI085 data synchronization disable interrupt configuration failure!\n");
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
        printf("Please make sure you have interconnected shuttle board pin id 21(INT1 Accel) and 22(INT3 Gyro) and check INT2 for data ready signal \n");
        /* here we have 2000ms to allow user to read the hint regarding the pin connection */
        coines_delay_msec(2000);
    }
    init_sensor_interface();
    /* after sensor init introduce 200 msec sleep */
    coines_delay_msec(200);
    /*initialize the sensors*/
    init_bmi08x();

    coines_attach_interrupt(COINES_SHUTTLE_PIN_20, bmi08x_data_sync_int, COINES_PIN_INTERRUPT_FALLING_EDGE);

    /*Enable data ready interrupts*/
    enable_bmi08x_data_synchronization_interrupt();
    uint32_t start_time = coines_get_millis();
    /* Run data synchronization for 10s before disabling interrupts */
    while (coines_get_millis() - start_time < 10000)
    {
        if (data_sync_int == true)
        {
            data_sync_int = false;
#if BMI08X_FEATURE_BMI085 == 1
            rslt = bmi085_get_synchronized_data(&bmi08x_accel,&bmi08x_gyro, &bmi08xdev);
#elif BMI08X_FEATURE_BMI088 == 1
            rslt = bmi088_get_synchronized_data(&bmi08x_accel,&bmi08x_gyro, &bmi08xdev);
#endif
            printf("ax:%d \t ay:%d \t az:%d \t gx:%d \t gy:%d \t gz:%d \t ts:%lu\n", bmi08x_accel.x, bmi08x_accel.y,
                    bmi08x_accel.z, bmi08x_gyro.x, bmi08x_gyro.y, bmi08x_gyro.z, coines_get_millis() - start_time);

        }
    }

    /*disable data ready interrupts*/
    disable_bmi08x_data_synchronization_interrupt();

    /*close the communication*/
    coines_close_comm_intf(COINES_COMM_INTF_USB);

    return EXIT_SUCCESS;
}

/* BMI08x data sync. interrupt callback */
void bmi08x_data_sync_int()
{
    data_sync_int = true;
}

