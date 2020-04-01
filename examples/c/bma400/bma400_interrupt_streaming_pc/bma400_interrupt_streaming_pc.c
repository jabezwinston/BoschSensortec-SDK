/**
 * Copyright (C) 2019 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bma400_interrupt_streaming_pc.c
 * @brief   Sample code to read accel. data and tap feature from BMA400
 *          using interrupts INT1 and INT2 respectively
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include "coines.h"
#include "bma400.h"

/*********************************************************************/
/* macro definitions */
/*********************************************************************/

/*! i2c interface communication, 1 - Enable; 0- Disable */
#define BMA400_INTERFACE_I2C             0
/*! spi interface communication, 1 - Enable; 0- Disable */
#define BMA400_INTERFACE_SPI             1

#if (!((BMA400_INTERFACE_I2C==1) && (BMA400_INTERFACE_SPI==0)) && \
		(!((BMA400_INTERFACE_I2C==0) && (BMA400_INTERFACE_SPI==1))))
#error "Invalid value given for the macros BMA400_INTERFACE_I2C / BMA400_INTERFACE_SPI"
#endif
/*! BMA400 shuttle board ID */
#define BMA400_SHUTTLE_ID           0x1A1
/*! This variable holds the device address of BMA400 */
#define BMA400_DEV_ADDR BMA400_I2C_ADDRESS_SDO_LOW

/*********************************************************************/
/* global definitions */
/*********************************************************************/

uint8_t bma400_accel_stream_buffer[COINES_STREAM_RSP_BUF_SIZE];
struct coines_streaming_config accel_stream_config;
struct coines_streaming_blocks accel_stream_block;

/*********************************************************************/
/* Function prototype */
/*********************************************************************/

/*!
 *@brief Function to to print the result.
 */
void print_rslt(int8_t rslt);

/*********************************************************************/
/* Function definitions */
/*********************************************************************/

/*!
 *  @brief This API is used to configure stream settings
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void send_stream_settings()
{
#if BMA400_INTERFACE_I2C==1
    accel_stream_config.intf = COINES_SENSOR_INTF_I2C;
#endif
#if BMA400_INTERFACE_SPI==1
    accel_stream_config.intf = COINES_SENSOR_INTF_SPI;
#endif
    accel_stream_config.i2c_bus = COINES_I2C_BUS_0; //if intf is I2C
    accel_stream_config.spi_bus = COINES_SPI_BUS_0; // if intf is SPI
    /*for I2C */
    accel_stream_config.dev_addr = BMA400_DEV_ADDR;
    accel_stream_config.cs_pin = COINES_SHUTTLE_PIN_7;
    accel_stream_config.int_pin = COINES_SHUTTLE_PIN_20;
    accel_stream_config.int_timestamp = 1;
    accel_stream_block.no_of_blocks = 1;
    accel_stream_block.reg_start_addr[0] = BMA400_ACCEL_DATA_ADDR; //Accel data start address
#if BMA400_INTERFACE_I2C==1
    accel_stream_block.no_of_data_bytes[0] = 6;
#endif
#if BMA400_INTERFACE_SPI==1
    accel_stream_block.no_of_data_bytes[0] = 7; // actual data length is 6 bytes 1 byte needed to initiate the spi communication
#endif
    coines_config_streaming(1, &accel_stream_config, &accel_stream_block);

#if BMA400_INTERFACE_I2C==1
    accel_stream_config.intf = COINES_SENSOR_INTF_I2C;
#endif
#if BMA400_INTERFACE_SPI==1
    accel_stream_config.intf = COINES_SENSOR_INTF_SPI;
#endif
    accel_stream_config.i2c_bus = COINES_I2C_BUS_0; //if intf is I2C
    accel_stream_config.spi_bus = COINES_SPI_BUS_0; // if intf is SPI
    /*for I2C */
    accel_stream_config.dev_addr = BMA400_DEV_ADDR;
    accel_stream_config.cs_pin = COINES_SHUTTLE_PIN_7;
    accel_stream_config.int_pin = COINES_SHUTTLE_PIN_21;
    accel_stream_config.int_timestamp = 1;
    accel_stream_block.no_of_blocks = 1;
    accel_stream_block.reg_start_addr[0] = BMA400_INT_STAT0_ADDR; //Accel interrupt status register
#if BMA400_INTERFACE_I2C==1
    accel_stream_block.no_of_data_bytes[0] = 3;
#endif
#if BMA400_INTERFACE_SPI==1
    accel_stream_block.no_of_data_bytes[0] = 4; // actual data length is 3 bytes 1 byte needed to initiate the spi communication
#endif
    coines_config_streaming(2, &accel_stream_config, &accel_stream_block);
}

/*!
 *  @brief This API is used to read sensor data
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void read_sensor_data()
{
    int16_t rslt;
    int counter = 0;
    uint8_t lsb, msb;
    int16_t ax, ay, az;
    uint32_t valid_sample_count = 0;
    uint32_t packet_count = 0;
    uint64_t accel_time_stamp = 0;
    int idx = 0;
    int buffer_index = 0;
    uint8_t status_reg[3] = { 0, 0, 0 };
    printf("Give single or double tap to read the status of HW interrupt channel 2 while streaming\n");
    coines_delay_msec(3000);

    while (counter < 1000)
    {
        memset(&bma400_accel_stream_buffer[0], 0, COINES_STREAM_RSP_BUF_SIZE);
        rslt = coines_read_stream_sensor_data(1, 1, &bma400_accel_stream_buffer[0], &valid_sample_count);
        if (rslt == COINES_SUCCESS)
        {
            buffer_index = 0;
            for (idx = 0; idx < valid_sample_count; idx++)
            {
                packet_count = 0;

                packet_count |= bma400_accel_stream_buffer[buffer_index++] << 24;
                packet_count |= bma400_accel_stream_buffer[buffer_index++] << 16;
                packet_count |= bma400_accel_stream_buffer[buffer_index++] << 8;
                packet_count |= bma400_accel_stream_buffer[buffer_index++];
#if BMA400_INTERFACE_SPI==1
                buffer_index++; //dummy byte;
#endif
                lsb = bma400_accel_stream_buffer[buffer_index++];
                msb = bma400_accel_stream_buffer[buffer_index++];
                ax = (msb << 8) | lsb;
                if (ax > 2047)
                    ax -= 4096;

                lsb = bma400_accel_stream_buffer[buffer_index++];
                msb = bma400_accel_stream_buffer[buffer_index++];
                ay = (msb << 8) | lsb;
                if (ay > 2047)
                    ay -= 4096;

                lsb = bma400_accel_stream_buffer[buffer_index++];
                msb = bma400_accel_stream_buffer[buffer_index++];
                az = (msb << 8) | lsb;
                if (az > 2047)
                    az -= 4096;

                if (accel_stream_config.int_timestamp)
                {
                    accel_time_stamp = 0;
                    accel_time_stamp |= (uint64_t)bma400_accel_stream_buffer[buffer_index++] << 40;
                    accel_time_stamp |= (uint64_t)bma400_accel_stream_buffer[buffer_index++] << 32;
                    accel_time_stamp |= (uint64_t)bma400_accel_stream_buffer[buffer_index++] << 24;
                    accel_time_stamp |= (uint64_t)bma400_accel_stream_buffer[buffer_index++] << 16;
                    accel_time_stamp |= (uint64_t)bma400_accel_stream_buffer[buffer_index++] << 8;
                    accel_time_stamp |= (uint64_t)bma400_accel_stream_buffer[buffer_index++];
                }
                printf("a_pc:%8d   a_t(us):%12"PRIu64" \t ax:%-5d ay:%-5d az:%-5d\n",
                       packet_count, accel_time_stamp / 30, ax, ay, az);
                fflush(stdout);
            }
        }
        memset(&bma400_accel_stream_buffer[0], 0, COINES_STREAM_RSP_BUF_SIZE);
        rslt = coines_read_stream_sensor_data(2, 1, &bma400_accel_stream_buffer[0], &valid_sample_count);

        if (rslt == COINES_SUCCESS)
        {
            buffer_index = 0;
            for (idx = 0; idx < valid_sample_count; idx++)
            {
                packet_count = 0;

                packet_count |= bma400_accel_stream_buffer[buffer_index++] << 24;
                packet_count |= bma400_accel_stream_buffer[buffer_index++] << 16;
                packet_count |= bma400_accel_stream_buffer[buffer_index++] << 8;
                packet_count |= bma400_accel_stream_buffer[buffer_index++];
#if BMA400_INTERFACE_SPI==1
                buffer_index++; //dummy byte;
#endif
                status_reg[0] = bma400_accel_stream_buffer[buffer_index++];
                status_reg[1] = bma400_accel_stream_buffer[buffer_index++];
                status_reg[2] = bma400_accel_stream_buffer[buffer_index++];

                if (accel_stream_config.int_timestamp)
                {
                    accel_time_stamp = 0;
                    accel_time_stamp |= (uint64_t)bma400_accel_stream_buffer[buffer_index++] << 40;
                    accel_time_stamp |= (uint64_t)bma400_accel_stream_buffer[buffer_index++] << 32;
                    accel_time_stamp |= (uint64_t)bma400_accel_stream_buffer[buffer_index++] << 24;
                    accel_time_stamp |= (uint64_t)bma400_accel_stream_buffer[buffer_index++] << 16;
                    accel_time_stamp |= (uint64_t)bma400_accel_stream_buffer[buffer_index++] << 8;
                    accel_time_stamp |= (uint64_t)bma400_accel_stream_buffer[buffer_index++];
                }
                printf("a_pc:%8d   a_t(us):%12"PRIu64" \t status[0]:%d   status[1]:%d, status[2]:%d\n",
                       packet_count, accel_time_stamp / 30, status_reg[0], status_reg[1], status_reg[2]);
                fflush(stdout);
            }
        }
        counter++;
    }

}

/*!
 *  @brief This API is used to enable bma400 interrupt
 *
 *  @param[in] bma400dev: device structure
 *
 *  @return void
 *
 */
static void enable_bma400_interrupt(struct bma400_dev *bma400dev)
{
    struct bma400_int_enable tap_int[2];
    struct bma400_sensor_conf conf;
    int8_t rslt;
    struct bma400_int_enable drdy_int;
    struct bma400_device_conf dev_conf;

    /* Configure Data ready interrupt for INT channel 1 */
    dev_conf.type = BMA400_INT_PIN_CONF;
    dev_conf.param.int_conf.int_chan = BMA400_INT_CHANNEL_1;
    dev_conf.param.int_conf.pin_conf = BMA400_INT_PUSH_PULL_ACTIVE_1;
    rslt = bma400_set_device_conf(&dev_conf, 1, bma400dev);

    drdy_int.type = BMA400_DRDY_INT_EN;
    drdy_int.conf = BMA400_ENABLE;
    rslt = bma400_enable_interrupt(&drdy_int, 1, bma400dev);
    print_rslt(rslt);

    /* Configure Tap interrupt for INT channel 2 */
    conf.type = BMA400_TAP_INT;
    rslt = bma400_get_sensor_conf(&conf, 1, bma400dev);
    print_rslt(rslt);

    conf.param.tap.int_chan = BMA400_INT_CHANNEL_2;
    conf.param.tap.axes_sel = BMA400_Z_AXIS_EN_TAP;
    conf.param.tap.sensitivity = BMA400_TAP_SENSITIVITY_0;

    conf.param.tap.tics_th = BMA400_TICS_TH_6_DATA_SAMPLES;
    conf.param.tap.quiet = BMA400_QUIET_60_DATA_SAMPLES;
    conf.param.tap.quiet_dt = BMA400_QUIET_DT_4_DATA_SAMPLES;

    rslt = bma400_set_sensor_conf(&conf, 1, bma400dev);
    print_rslt(rslt);

    coines_delay_msec(100);

    tap_int[0].type = BMA400_SINGLE_TAP_INT_EN;
    tap_int[0].conf = BMA400_ENABLE;

    tap_int[1].type = BMA400_DOUBLE_TAP_INT_EN;
    tap_int[1].conf = BMA400_ENABLE;
    /* Enable Interrupt */
    rslt = bma400_enable_interrupt(tap_int, 2, bma400dev);
    print_rslt(rslt);
}

/*!
 *  @brief This API is used to disable bma400 interrupt
 *
 *  @param[in] bma400dev: device structure
 *
 *  @return void
 *
 */
static void disable_bma400_interrupt(struct bma400_dev *bma400dev)
{
    int8_t rslt;
    struct bma400_int_enable drdy_int;
    struct bma400_device_conf dev_conf;

    drdy_int.type = BMA400_DRDY_INT_EN;
    drdy_int.conf = BMA400_DISABLE;
    rslt = bma400_enable_interrupt(&drdy_int, 1, bma400dev);
    print_rslt(rslt);

    drdy_int.type = BMA400_SINGLE_TAP_INT_EN;
    drdy_int.conf = BMA400_DISABLE;
    rslt = bma400_enable_interrupt(&drdy_int, 1, bma400dev);
    print_rslt(rslt);

    dev_conf.param.int_conf.int_chan = BMA400_UNMAP_INT_PIN;
    dev_conf.param.int_conf.pin_conf = BMA400_INT_PUSH_PULL_ACTIVE_1;
    rslt = bma400_set_device_conf(&dev_conf, 1, bma400dev);
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
 *  @param[in] bma400dev: device structure
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
    }
    coines_delay_msec(100);
}

/*!
 *  @brief This internal API is used to print the result.
 *
 *  @param[in] rslt: result
 *  @retval	None
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
        exit(COINES_E_FAILURE);
    }
}

/*!
 *  @brief This internal API is used to set the sensor driver interface to
 *  read/write the data.
 *
 *  @param[in] bma400dev: bma400 device structure
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
 *  @brief This internal API is used to configure the sensor 
 *
 *  @param[in] bma400dev: bma400 device structure
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
static int8_t set_sensor_config(struct bma400_dev *bma400dev)
{
    int8_t rslt;
    struct bma400_sensor_conf conf;

    /* Select the type of configuration to be modified */
    conf.type = BMA400_ACCEL;

    /* Get the accelerometer configurations which are set in the sensor */
    rslt = bma400_get_sensor_conf(&conf, 1, bma400dev);
    print_rslt(rslt);

    /* Modify the desired configurations as per macros
     * available in bma400_defs.h file */
    conf.param.accel.odr = BMA400_ODR_100HZ;
    conf.param.accel.range = BMA400_2G_RANGE;
    conf.param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_1;
    conf.param.accel.int_chan = BMA400_INT_CHANNEL_1;

    /* Set the desired configurations to the sensor */
    rslt = bma400_set_sensor_conf(&conf, 1, bma400dev);
    print_rslt(rslt);

    rslt = bma400_set_power_mode(BMA400_NORMAL_MODE, bma400dev);
    print_rslt(rslt);

    return rslt;
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
int main(int argc, char const *argv[])
{
    struct coines_board_info board_info;
    struct bma400_dev bma;
    int8_t rslt;

    init_bma400_sensor_driver_interface(&bma);

    rslt = coines_open_comm_intf(COINES_COMM_INTF_USB);
    if (rslt < 0)
    {
        printf("\n Unable to connect with Application Board ! \n"
               " 1. Check if the board is connected and powered on. \n"
               " 2. Check if Application Board USB driver is installed. \n"
               " 3. Check if board is in use by another application. (Insufficient permissions to access USB) \n");
        exit(rslt);
    }

    /* Check if the right board is connected (implicitly check if board was reset) */
    rslt = coines_get_board_info(&board_info);
    if (rslt == COINES_SUCCESS)
    {
        if (board_info.shuttle_id != BMA400_SHUTTLE_ID)
        {
            printf("Invalid sensor shuttle ID (not a BMA400 shuttle)\n(sometimes board power off-power on helps.)\n");
            fflush(stdout);
            exit(COINES_E_FAILURE);
        }
    }

    init_sensor_interface();

    /* after sensor init introduce 200 msec sleep */
    coines_delay_msec(200);

    init_bma400(&bma);

    rslt = bma400_soft_reset(&bma);
    print_rslt(rslt);

    rslt = set_sensor_config(&bma);
    if (rslt != BMA400_OK)
    {
        printf("Error setting bma400 config.\n");
        fflush(stdout);
        exit(rslt);
    }

    send_stream_settings();
    enable_bma400_interrupt(&bma);

    coines_trigger_timer(COINES_TIMER_START, COINES_TIMESTAMP_ENABLE);
    coines_start_stop_streaming(COINES_STREAMING_MODE_INTERRUPT, COINES_STREAMING_START);

    read_sensor_data();

    disable_bma400_interrupt(&bma);
    coines_start_stop_streaming(COINES_STREAMING_MODE_INTERRUPT, COINES_STREAMING_STOP);
    coines_trigger_timer(COINES_TIMER_STOP, COINES_TIMESTAMP_DISABLE);

    return 0;
}

