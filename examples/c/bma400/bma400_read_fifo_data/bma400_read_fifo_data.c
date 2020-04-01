/**
 * Copyright (C) 2018 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bma400_read_fifo_data.c
 * @brief   Sample code to read bma400 fifo data using COINES library
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

#define GRAVITY_EARTH (9.80665f) /* Earth's gravity in m/s^2 */

/* Include 2 additional frames to account for
 * the next frame already being in the FIFO
 * and the sensor time frame
 */
#define N_FRAMES        50
/* 50 Frames result in 50*7, 350 bytes.
 * A few extra for the sensor time frame and
 * in case the next frame is available too.
 */
#define FIFO_SIZE       (357 + BMA400_FIFO_BYTES_OVERREAD)
/* Delay to fill FIFO data At ODR of say 100 Hz,
 * 1 frame gets updated in 1/100 = 0.01s i.e. for
 * 50 frames we need 50 * 0.01 =  0.5 seconds delay
 */
#define WAIT_PERIOD_MS  500
/*********************************************************************/
/* global variables */
/*********************************************************************/
/*! @brief This structure containing relevant bma400 info */
struct bma400_dev bma400dev;

/*********************************************************************/
/* function declarations */
/*********************************************************************/

/*!
 * @brief	This internal API is used to initialize the sensor interface
 */
static void init_sensor_interface(void);

/*!
 * @brief	This internal API is used to initialize the bma400 sensor with default settings
 */
static void init_bma400(void);

/*!
 * @brief	This internal API is used to set the sensor driver interface to read/write the data
 */
static void init_bma400_sensor_driver_interface(void);

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
static void init_bma400(void)
{
    int8_t rslt;
    rslt = bma400_init(&bma400dev);
    if (rslt == BMA400_OK)
    {
        printf("BMA400 Initialization Success!\n");
        printf("Chip ID 0x%x\n", bma400dev.chip_id);
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
}

/*!
 *  @brief This internal API is used to set the sensor driver interface to
 *  read/write the data.
 *
 *  @param[in] interface
 *
 *  @return void
 *
 */
static void init_bma400_sensor_driver_interface(void)
{
#if BMA400_INTERFACE_I2C==1
    /* I2C setup */
    /* link read/write/delay function of host system to appropriate
     * bma400 function call prototypes */
    bma400dev.intf_ptr = NULL; /* To attach your interface device reference */
    bma400dev.write = coines_write_i2c;
    bma400dev.read = coines_read_i2c;
    bma400dev.delay_ms = coines_delay_msec;
    /* set correct i2c address */
    bma400dev.dev_id = (unsigned char) BMA400_I2C_ADDRESS_SDO_LOW;
    bma400dev.intf = BMA400_I2C_INTF;

#elif BMA400_INTERFACE_SPI==1
    /* SPI setup */
    /* link read/write/delay function of host system to appropriate
     *  bma400 function call prototypes */
    bma400dev.intf_ptr = NULL; /* To attach your interface device reference */
    bma400dev.write = coines_write_spi;
    bma400dev.read = coines_read_spi;
    bma400dev.delay_ms = coines_delay_msec;
    bma400dev.intf = BMA400_SPI_INTF;
    bma400dev.dev_id = COINES_SHUTTLE_PIN_7;
#endif
}

float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width)
{
    float half_scale = (float)(1 << bit_width) / 2.0f;

    return GRAVITY_EARTH * val * g_range / half_scale;
}

float sensor_ticks_to_s(uint32_t sensor_time)
{
    return (float)sensor_time * 0.0000390625f;
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
    struct bma400_sensor_data accel_data[N_FRAMES] = {};
    struct bma400_fifo_data fifo_frame;
    struct bma400_device_conf fifo_conf;
    struct bma400_sensor_conf conf;
    uint16_t i;
    uint8_t fifo_buff[FIFO_SIZE] = { 0 };
    uint16_t accel_frames_req = N_FRAMES;
    float x, y, z, t;
    uint8_t read_times = 10; /* Read the FIFO 10 times */
    int16_t rslt;
    struct coines_board_info board_info;

    init_bma400_sensor_driver_interface();

    rslt = coines_open_comm_intf(COINES_COMM_INTF_USB);
    if (rslt < 0)
    {
        printf("\n Unable to connect with Application Board ! \n"
               " 1. Check if the board is connected and powered on. \n"
               " 2. Check if Application Board USB driver is installed. \n"
               " 3. Check if board is in use by another application. (Insufficient permissions to access USB) \n");
        fflush(stdout);
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

    init_bma400();

    rslt = bma400_soft_reset(&bma400dev);
    print_rslt(rslt);

    /* Select the type of configuration to be modified */
    conf.type = BMA400_ACCEL;

    /* Get the accelerometer configurations which are set in the sensor */
    rslt = bma400_get_sensor_conf(&conf, 1, &bma400dev);
    print_rslt(rslt);

    /* Modify the desired configurations as per macros
     * available in bma400_defs.h file */
    conf.param.accel.odr = BMA400_ODR_100HZ;
    conf.param.accel.range = BMA400_2G_RANGE;
    conf.param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_1;

    /* Set the desired configurations to the sensor */
    rslt = bma400_set_sensor_conf(&conf, 1, &bma400dev);
    print_rslt(rslt);

    fifo_conf.type = BMA400_FIFO_CONF;

    rslt = bma400_get_device_conf(&fifo_conf, 1, &bma400dev);
    print_rslt(rslt);

    fifo_conf.param.fifo_conf.conf_regs = BMA400_FIFO_X_EN | BMA400_FIFO_Y_EN
                                          | BMA400_FIFO_Z_EN
                                          | BMA400_FIFO_TIME_EN;
    fifo_conf.param.fifo_conf.conf_status = BMA400_ENABLE;

    rslt = bma400_set_device_conf(&fifo_conf, 1, &bma400dev);
    print_rslt(rslt);

    rslt = bma400_set_power_mode(BMA400_NORMAL_MODE, &bma400dev);
    print_rslt(rslt);

    while ((rslt == BMA400_OK) && read_times)
    {
        fifo_frame.data = fifo_buff;
        fifo_frame.length = FIFO_SIZE;
        bma400dev.delay_ms(WAIT_PERIOD_MS);

        printf("Requested FIFO length : %d\r\n", fifo_frame.length);

        rslt = bma400_get_fifo_data(&fifo_frame, &bma400dev);
        print_rslt(rslt);

        if (rslt != BMA400_OK)
        {
            printf("FIFO read failed\r\n");
        }

        printf("Available FIFO length : %d \r\n", fifo_frame.length);
        fflush(stdout);
        do
        {
            accel_frames_req = N_FRAMES;
            rslt = bma400_extract_accel(&fifo_frame, accel_data,
                                        &accel_frames_req,
                                        &bma400dev);
            print_rslt(rslt);

            if (rslt != BMA400_OK)
            {
                printf("Accelerometer data extraction failed\r\n");
            }

            if (accel_frames_req)
            {
                printf("Extracted FIFO frames : %d \r\n", accel_frames_req);

                printf("Frame index\tAx[m/s2]\tAy[m/s2]\tAz[m/s2]\r\n");
                for (i = 0; i < accel_frames_req; i++)
                {
                    bma400dev.delay_ms(10); /* Wait for 10ms as ODR is set to 100Hz */

                    /* 12-bit accelerometer at range 2G */
                    x = lsb_to_ms2(accel_data[i].x, 2, 12);
                    y = lsb_to_ms2(accel_data[i].y, 2, 12);
                    z = lsb_to_ms2(accel_data[i].z, 2, 12);

                    printf("     %d\t\t%.2f\t\t%.2f\t\t%.2f\r\n", i, x, y, z);
                    fflush(stdout);
                }
            }
        }
        while (accel_frames_req);

        if (fifo_frame.fifo_sensor_time)
        {
            t = sensor_ticks_to_s(fifo_frame.fifo_sensor_time);

            printf("FIFO sensor time : %.4fs\r\n", t);
        }

        if (fifo_frame.conf_change)
        {
            printf("FIFO configuration change: 0x%X\r\n",
                   fifo_frame.conf_change);

            if (fifo_frame.conf_change & BMA400_FIFO_CONF0_CHANGE)
            {
                printf("FIFO data source configuration changed \r\n");
            }

            if (fifo_frame.conf_change & BMA400_ACCEL_CONF0_CHANGE)
            {
                printf("Accel filt1_bw configuration changed \r\n");
            }

            if (fifo_frame.conf_change & BMA400_ACCEL_CONF1_CHANGE)
            {
                printf("Accel odr/osr/range configuration changed \r\n");
            }
        }

        read_times--;
    }
    coines_close_comm_intf(COINES_COMM_INTF_USB);
    fflush(stdout);

    return EXIT_SUCCESS;
}
