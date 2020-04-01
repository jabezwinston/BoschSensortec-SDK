/**
 * Copyright (C) 2018 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bma400_tap_detection.c
 * @brief   Sample code to configure BMA400 for tap detection using COINES library
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
#include "bma400.h"
/*********************************************************************/
/* macro definitions */
/*********************************************************************/
/*! i2c interface communication, 1 - Enable; 0- Disable */
#define BMA400_INTERFACE_I2C             1
/*! spi interface communication, 1 - Enable; 0- Disable */
#define BMA400_INTERFACE_SPI             0

#if (!((BMA400_INTERFACE_I2C==1) && (BMA400_INTERFACE_SPI==0)) && \
	(!((BMA400_INTERFACE_I2C==0) && (BMA400_INTERFACE_SPI==1))))
#error "Invalid value given for the macros BMA400_INTERFACE_I2C / BMA400_INTERFACE_SPI"
#endif
/*! BMA400 shuttle board ID */
#define BMA400_SHUTTLE_ID           0x1A1
/*! This variable holds the device address of BMA400 */
#define BMA400_DEV_ADDR BMA400_I2C_ADDR_PRIM

/*********************************************************************/
/* global variables */
/*********************************************************************/

/*********************************************************************/
/* function declarations */
/*********************************************************************/

/*!
 * @brief This internal API is used to initialize the sensor interface
 */
static void init_sensor_interface(void);

/*!
 * @brief	This internal API is used to initialize the bma400 sensor with default settings
 */
static void init_bma400(struct bma400_dev *bma400dev);

/*!
 * @brief	 This internal API is used to set the sensor driver interface to read/write the data
 */
static void init_bma400_sensor_driver_interface(struct bma400_dev *bma400dev);

/*!
 * @brief	This internal API is used to enable tap detection
 */
static void enable_tap_detection(struct bma400_dev *bma400dev);

/*!
 *@brief Function to to print the result.
 */
void print_rslt(int8_t rslt);

/*!
 *  @brief This internal API is used to initialize the sensor interface depending
 *   on selection either SPI or I2C.
 *
 *  @param[in] interface
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
#if BMA400_INTERFACE_I2C==1       
    /* set the sensor interface as I2C */
    coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_FAST_MODE);
#elif BMA400_INTERFACE_SPI==1
    /* set the sensor interface as SPI */
    coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_5_MHZ, COINES_SPI_MODE3);
#endif   
    /* Switch VDD for sensor on */
    coines_set_shuttleboard_vdd_vddio_config(3300, 3300);
}

/*!
 * @brief This internal API is used to initializes the bma400 sensor with default
 * settings like power mode and OSRS settings
 *
 * @param[in] void
 *
 * @return void
 *
 */
static void init_bma400(struct bma400_dev *bma400dev)
{
    int8_t rslt;
    rslt = bma400_init(bma400dev);
    if (rslt == BMA400_OK)
    {
        printf("BMA400 Initialization Success!\n");
        printf("Chip ID 0x%x\n", bma400dev->chip_id);
    }
    else
    {
        print_rslt(rslt);
        exit(COINES_E_FAILURE);
    }
    coines_delay_msec(100);
}

/*!
 *  @brief This internal API is used to print the result.
 */
void print_rslt(int8_t rslt)
{
    switch (rslt)
    {
        case BMA400_OK:
            /* Do nothing */
            break;
        case BMA400_E_NULL_PTR:
            printf("Error [%d] : Null pointer\r\n", rslt);
            break;
        case BMA400_E_COM_FAIL:
            printf("Error [%d] : Communication failure\r\n", rslt);
            break;
        case BMA400_E_DEV_NOT_FOUND:
            printf("Error [%d] : Device not found\r\n", rslt);
            break;
        case BMA400_E_INVALID_CONFIG:
            printf("Error [%d] : Invalid configuration\r\n", rslt);
            break;
        case BMA400_W_SELF_TEST_FAIL:
            printf("Warning [%d] : Self test failed\r\n", rslt);
            break;
        default:
            printf("Error [%d] : Unknown error code\r\n", rslt);
            break;
    }
    fflush(stdout);

    if (rslt != BMA400_OK)
    {
        exit(rslt);
    }
}

/*!
 *  @brief This internal API is used to set the sensor driver interface to
 *  read/write the data.
 *
 *  @param[in] bma400dev: device structure
 *
 *  @return void
 *
 */
static void init_bma400_sensor_driver_interface(struct bma400_dev *bma400dev)
{
#if BMA400_INTERFACE_I2C==1
    /* I2C setup */
    /* link read/write/delay function of host system to appropriate
     * bma400 function call prototypes */
    bma400dev->intf_ptr = NULL; /* To attach your interface device reference */
    bma400dev->write = coines_write_i2c;
    bma400dev->read = coines_read_i2c;
    bma400dev->delay_ms = coines_delay_msec;
    /* set correct i2c address */
    bma400dev->dev_id = (unsigned char) BMA400_I2C_ADDRESS_SDO_LOW;
    bma400dev->intf = BMA400_I2C_INTF;

#elif BMA400_INTERFACE_SPI==1
    /* SPI setup */
    /* link read/write/delay function of host system to appropriate
     *  bma400 function call prototypes */
    bma400dev->intf_ptr = NULL; /* To attach your interface device reference */
    bma400dev->write = coines_write_spi;
    bma400dev->read = coines_read_spi;
    bma400dev->delay_ms = coines_delay_msec;
    bma400dev->intf = BMA400_SPI_INTF;
    bma400dev->dev_id = COINES_SHUTTLE_PIN_7;
#endif
}

/*!
 *  @brief This internal API is used to count steps and recognize the activity
 *
 *  @param[in] bma400dev: device structure
 *
 *  @return void
 *
 */
static void enable_tap_detection(struct bma400_dev *bma400dev)
{
    struct bma400_int_enable tap_int[2];
    struct bma400_sensor_conf conf[2];
    int8_t rslt;
    uint32_t poll_period = 2, test_dur_ms = 60000;
    uint16_t int_status;
    /* Doing soft reset */
    rslt = bma400_soft_reset(bma400dev);
    print_rslt(rslt);
    conf[0].type = BMA400_ACCEL;
    conf[1].type = BMA400_TAP_INT;

    rslt = bma400_get_sensor_conf(conf, 2, bma400dev);
    print_rslt(rslt);

    conf[0].param.accel.odr = BMA400_ODR_200HZ;
    conf[0].param.accel.range = BMA400_16G_RANGE;
    conf[0].param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_1;
    conf[0].param.accel.filt1_bw = BMA400_ACCEL_FILT1_BW_1;

    conf[1].param.tap.int_chan = BMA400_UNMAP_INT_PIN;
    conf[1].param.tap.axes_sel = BMA400_Z_AXIS_EN_TAP;
    conf[1].param.tap.sensitivity = BMA400_TAP_SENSITIVITY_0;

    conf[1].param.tap.tics_th = BMA400_TICS_TH_6_DATA_SAMPLES;
    conf[1].param.tap.quiet = BMA400_QUIET_60_DATA_SAMPLES;
    conf[1].param.tap.quiet_dt = BMA400_QUIET_DT_4_DATA_SAMPLES;

    rslt = bma400_set_sensor_conf(conf, 2, bma400dev);
    print_rslt(rslt);

    coines_delay_msec(100);

    tap_int[0].type = BMA400_SINGLE_TAP_INT_EN;
    tap_int[0].conf = BMA400_ENABLE;

    tap_int[1].type = BMA400_DOUBLE_TAP_INT_EN;
    tap_int[1].conf = BMA400_ENABLE;
    /* Enable Interrupt */
    rslt = bma400_enable_interrupt(tap_int, 2, bma400dev);
    print_rslt(rslt);

    coines_delay_msec(100);
    /* Set power mode */
    rslt = bma400_set_power_mode(BMA400_NORMAL_MODE, bma400dev);
    print_rslt(rslt);

    coines_delay_msec(100);

    if (rslt == BMA400_OK)
    {
        printf("Waiting to detect tap... \nGive tap on the shuttle!\r\n");

        while (test_dur_ms)
        {
            coines_delay_msec(poll_period);

            rslt = bma400_get_interrupt_status(&int_status, bma400dev);
            print_rslt(rslt);

            if (int_status & BMA400_S_TAP_INT_ASSERTED)
            {
                printf("Single tap detected!\r\n");
            }

            if (int_status & BMA400_D_TAP_INT_ASSERTED)
            {
                printf("Double tap detected!\r\n");
            }
            fflush(stdout);
            test_dur_ms -= poll_period;
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
    struct bma400_dev bma400dev;

    init_bma400_sensor_driver_interface(&bma400dev);

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
        if (board_info.shuttle_id != BMA400_SHUTTLE_ID)
        {

            printf("! Warning invalid sensor shuttle \n ,"
                   "This application will not support this sensor \n");
            exit(COINES_E_FAILURE);
        }
    }

    init_sensor_interface();

    /* after sensor init introduce 200 msec sleep */
    coines_delay_msec(200);

    init_bma400(&bma400dev);

    enable_tap_detection(&bma400dev);

    coines_close_comm_intf(COINES_COMM_INTF_USB);
    fflush(stdout);

    return EXIT_SUCCESS;
}
