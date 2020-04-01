/**
 * Copyright (C) 2018 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bma400_step_counter.c
 * @brief   Sample code to configure BMA400 for step counting using COINES library
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
 * @brief	This internal API is used to initialize the sensor interface
 */
static void init_sensor_interface(void);

/*!
 * @brief This internal API is used to initialize the bma400 sensor with default settings
 */
static void init_bma400(struct bma400_dev *bma400dev);

/*!
 * @brief	This internal API is used to set the sensor driver interface to read/write the data
 */
static void init_bma400_sensor_driver_interface(struct bma400_dev *bma400dev);

/*!
 * @brief	This internal API is used to perform step counting
 */
static void perform_step_counting(struct bma400_dev *bma400dev);

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
static void perform_step_counting(struct bma400_dev *bma400dev)
{
    int8_t rslt;
    struct bma400_int_enable step_int;
    uint32_t step_count;
    uint8_t activity;
    uint8_t test_dur = 50;

    /* Doing soft reset */
    rslt = bma400_soft_reset(bma400dev);
    print_rslt(rslt);

    step_int.type = BMA400_STEP_COUNTER_INT_EN;
    step_int.conf = BMA400_ENABLE;

    /* Enable interrupt */
    rslt = bma400_enable_interrupt(&step_int, 1, bma400dev);
    print_rslt(rslt);

    /* Set power mode */
    rslt = bma400_set_power_mode(BMA400_NORMAL_MODE, bma400dev);
    print_rslt(rslt);

    printf("Steps counted\tActivity classifier\r\n");
    fflush(stdout);

    while (test_dur)
    {
        coines_delay_msec(1000);

        rslt = bma400_get_steps_counted(&step_count, &activity, bma400dev);
        print_rslt(rslt);

        printf("\t%d\t", step_count);

        switch (activity)
        {
            case BMA400_STILL_ACT:
                printf("Still\r\n");
                break;
            case BMA400_WALK_ACT:
                printf("Walking\r\n");
                break;
            case BMA400_RUN_ACT:
                printf("Running\r\n");
                break;
            default:
                printf("undefined\r\n");
                break;
        }
        fflush(stdout);
        test_dur--;
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

    perform_step_counting(&bma400dev);

    coines_close_comm_intf(COINES_COMM_INTF_USB);
    fflush(stdout);

    return EXIT_SUCCESS;
}
