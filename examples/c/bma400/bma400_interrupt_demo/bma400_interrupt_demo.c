/**
 * Copyright (C) 2018 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bma400_interrupt_demo.c
 * @brief   Sample code to configure different interrupts of BMA400 using COINES library
 *
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <inttypes.h>
/*********************************************************************/
/* own header files */
/*********************************************************************/
#include "coines.h"
#include "bma400.h"

/*********************************************************************/
/* macro definitions */
/*********************************************************************/
/*! i2c interface communication, 1 - Enable; 0- Disable */
#define BMA400_INTERFACE_I2C             0
/*! spi interface communication, 1 - Enable; 0- Disable */
#define BMA400_INTERFACE_SPI             1
/*! BMA400 shuttle board ID */
#define BMA400_SHUTTLE_ID           0x1A1

#if (!((BMA400_INTERFACE_I2C==1) && (BMA400_INTERFACE_SPI==0)) && \
    (!((BMA400_INTERFACE_I2C==0) && (BMA400_INTERFACE_SPI==1))))
#error "Invalid value given for the macros BMA400_INTERFACE_I2C / BMA400_INTERFACE_SPI"
#endif

/*********************************************************************/
/* global variables */
/*********************************************************************/
/*! @brief This structure containing relevant bma400 info */
struct bma400_dev bma400dev;
/*! @brief variable to hold the bma400 accel data */
struct bma400_sensor_data bma400_accel;

/*********************************************************************/
/* functions */
/*********************************************************************/

/*!
 *  @brief This internal API is used to initialize the sensor interface depending
 *   on selection either SPI or I2C.
 *
 *  @param[in] interface
 *
 *  @return void
 *
 */
static void init_sensor_interface(int interface)
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
 *  @brief This internal API is used to initializes the bma400 sensor
 *  settings like power mode and OSRS settings.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void init_bma400(void)
{
    if (bma400_init(&bma400dev) == BMA400_OK)
    {
        printf("BMA400 initialization successful! Chip ID - 0x%x\n",
               bma400dev.chip_id);
    }
    else
    {
        printf("BMA400 initialization failure! Did you reset the board?\n");
        exit(COINES_E_FAILURE);
    }
}

/*!
 *  @brief This internal API is used to condifure the BMA400
 *  including sensor settings (ranga, bandwidth, ..) and interrupts
 *
 *  @param[in] odr 
 *
 *  @return void
 *
 */
static void set_sensor_config(uint8_t odr)
{
    struct bma400_device_conf dev_conf[3];
    struct bma400_sensor_conf sensor_conf[3];
    struct bma400_int_enable bma400_interrupt[4];

    /* Configure the sensor data output: filter, ODR, ... */
    sensor_conf[0].type = BMA400_ACCEL;
    sensor_conf[0].param.accel.odr = odr;
    sensor_conf[0].param.accel.range = BMA400_2G_RANGE;
    sensor_conf[0].param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_2;
    sensor_conf[0].param.accel.osr = BMA400_ACCEL_OSR_SETTING_1;

    /* configure the auto-wakeup interupt
     - do not route signal to any INT pin
     - update "ONE_TIME", i.e. every time the device goes to sleep, the
     reference values are updated with the latest position. */
    dev_conf[0].type = BMA400_AUTOWAKEUP_INT;
    dev_conf[0].param.wakeup.int_chan = BMA400_UNMAP_INT_PIN;
    dev_conf[0].param.wakeup.wakeup_axes_en = BMA400_XYZ_AXIS_EN;
    dev_conf[0].param.wakeup.int_wkup_threshold = 10;
    dev_conf[0].param.wakeup.wakeup_ref_update = BMA400_ONE_TIME_UPDATE;
    dev_conf[0].param.wakeup.sample_count = BMA400_SAMPLE_COUNT_2;

    /* Configure both INT pins to output, push-pull characteristic, active=high
     This configuration is just shown here, but the INT pins are not observed in the example. */
    dev_conf[1].type = BMA400_INT_PIN_CONF;
    dev_conf[1].param.int_conf.pin_conf = BMA400_INT_PUSH_PULL_ACTIVE_1;
    dev_conf[1].param.int_conf.int_chan = BMA400_MAP_BOTH_INT_PINS;

    /* Go to low power automatically one second after last activity was detected
     (aka generic interrupt 2 was asserted) */
    dev_conf[2].type = BMA400_AUTO_LOW_POWER;
    dev_conf[2].param.auto_lp.auto_low_power_trigger =
    BMA400_AUTO_LP_TIME_RESET_EN;
    dev_conf[2].param.auto_lp.auto_lp_timeout_threshold = 400; // 2.5ms/LSB -> 1s

    /* Generic interrupt 2 configuration, so that it keeps sensor in normal mode when there is still movement,
     i.e. avoid auto low power while moving */
    sensor_conf[1].type = BMA400_GEN2_INT;
    sensor_conf[1].param.gen_int.int_chan = BMA400_INT_CHANNEL_2;
    sensor_conf[1].param.gen_int.axes_sel = BMA400_XYZ_AXIS_EN;
    sensor_conf[1].param.gen_int.criterion_sel = BMA400_ACTIVITY_INT;
    sensor_conf[1].param.gen_int.evaluate_axes = BMA400_ANY_AXES_INT;
    sensor_conf[1].param.gen_int.ref_update = BMA400_MANUAL_UPDATE;
    sensor_conf[1].param.gen_int.data_src = BMA400_DATA_SRC_ACC_FILT2;
    sensor_conf[1].param.gen_int.gen_int_thres = 10; // 1 LSB = 8mg (independent from range) --> 80mg threshold
    sensor_conf[1].param.gen_int.gen_int_dur = 1; // 1 LSB = 1 data ready tick, i.e. depends on filter/ODR setting
    sensor_conf[1].param.gen_int.hysteresis = BMA400_HYST_0_MG;
    /* Refernce position (int_thres_ref_[x|y|z]): (0,0,1024) in LSB or (0,0,1) in G (in 2G mode)
     --> horizonal position on flat surface */
    sensor_conf[1].param.gen_int.int_thres_ref_x = 0;
    sensor_conf[1].param.gen_int.int_thres_ref_y = 0;
    sensor_conf[1].param.gen_int.int_thres_ref_z = 1024;

    /* Configure orientation change interrupt
     - Interrupt will not be triggered if board is moved only slowly, only faster movements, followed by
     a 1s stability phase leads to triggered orientation change interrupt */
    sensor_conf[2].type = BMA400_ORIENT_CHANGE_INT;
    sensor_conf[2].param.orient.axes_sel = BMA400_XYZ_AXIS_EN;
    sensor_conf[2].param.orient.data_src = BMA400_DATA_SRC_ACC_FILT2;
    /* the reference orientation shall be updated with values from the FILT_LP signal */
    sensor_conf[2].param.orient.ref_update = BMA400_ORIENT_REFU_ACC_FILT_LP;
    /* Orientation interrupt condition 1: see at least 80mg between last 2 values of ACC_FILT_LP data */
    sensor_conf[2].param.orient.orient_thres = 5; // 1 LSB = 8mg -> thres = 40mg
    /* Orientation interrupt condition 2: new signal within 80mg for at least 1s (ACC_FILT_LP data) */
    sensor_conf[2].param.orient.stability_thres = 10; // 1 LSB = 8mg -> thres = 80mg
    sensor_conf[2].param.orient.orient_int_dur = 100; // 1 LSB = 10ms -> duration = 1s
    /* Check how stable last orientation is based on last ACC_FILT_LP value */
    sensor_conf[2].param.orient.stability_mode = BMA400_STABILITY_ACC_FILT_LP;
    /* Initial value: horizonal position on flat surface, will be overwritten after new orientation is detected
     (since BMA400_ORIENT_REFU_ACC_FILT_LP is chosen) */
    sensor_conf[2].param.orient.orient_ref_x = 0;
    sensor_conf[2].param.orient.orient_ref_y = 0;
    sensor_conf[2].param.orient.orient_ref_z = 1024;   // Placed flat on table
    sensor_conf[2].param.orient.int_chan = BMA400_INT_CHANNEL_1; // Interrupt on INT1

    /* Here the actual intterupts are enabled, which were configured in the previous sections */
    bma400_interrupt[0].conf = BMA400_ENABLE;
    bma400_interrupt[0].type = BMA400_ORIENT_CHANGE_INT_EN;
    bma400_interrupt[1].conf = BMA400_ENABLE;
    bma400_interrupt[1].type = BMA400_GEN2_INT_EN;
    bma400_interrupt[2].conf = BMA400_ENABLE;
    bma400_interrupt[2].type = BMA400_AUTO_WAKEUP_EN;
    bma400_interrupt[3].conf = BMA400_ENABLE;
    bma400_interrupt[3].type = BMA400_LATCH_INT_EN;

    if (bma400_set_sensor_conf(sensor_conf, 3, &bma400dev) == BMA400_OK)
    {
        if (bma400_set_device_conf(dev_conf, 3, &bma400dev) == BMA400_OK)
        {
            bma400_enable_interrupt(bma400_interrupt, 4, &bma400dev);
        }
    }
    else
        printf("Error setting bma400 config.\n");

    bma400_set_power_mode(BMA400_LOW_POWER_MODE, &bma400dev);

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
static void init_bma400_sensor_driver_interface(int interface)
{
#if BMA400_INTERFACE_I2C==1
    /* I2C setup */
    /* link read/write/delay function of host system to appropriate
     * bma400 function call prototypes */
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
    bma400dev.write = coines_write_spi;
    bma400dev.read = coines_read_spi;
    bma400dev.delay_ms = coines_delay_msec;
    bma400dev.intf = BMA400_SPI_INTF;
    bma400dev.dev_id = COINES_SHUTTLE_PIN_7;
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
    int interface = BMA400_INTERFACE_SPI;
    uint8_t odr = BMA400_ODR_400HZ;
    int16_t rslt;
    uint16_t bma400_int_status = 0;
    uint8_t power_mode = -1;
    int sensor_status = -2;

    init_bma400_sensor_driver_interface(interface);

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
            exit(COINES_E_FAILURE);
        }
    }

    init_sensor_interface(interface);

    coines_delay_msec(500);

    init_bma400();
    set_sensor_config(odr);

    while (1)
    {
        bma400_get_power_mode(&power_mode, &bma400dev);
        if (power_mode == BMA400_LOW_POWER_MODE)
        {
            if (sensor_status != power_mode)
            {
                sensor_status = power_mode;
                printf("BMA400 entered low power mode\n");
                fflush(stdout);
            }
        }
        else
        {
            sensor_status = power_mode;
            bma400_get_accel_data(BMA400_DATA_SENSOR_TIME, &bma400_accel,
                                  &bma400dev);
            bma400_get_interrupt_status(&bma400_int_status, &bma400dev);
            if (bma400_int_status & BMA400_ORIENT_CH_INT_ASSERTED)
            {
                printf("\n\n\nOrientation changed interrupt asserted!\n\n\n");
                //coines_delay_msec(500); // may be added in order to recognize the occurance of the interrupt
            }
            else if (bma400_int_status & BMA400_INT_OVERRUN_ASSERTED)
            {
                printf(
                       "\n\n\nWarning: Interrupt engine overrun detected! Reduce odr.\n\n\n");
                coines_delay_msec(500);
            }
            else
            {
                printf("t: %d\tax:%05d\tay:%05d\taz:%05d\tint:%04x\n",
                       bma400_accel.sensortime,
                       bma400_accel.x, bma400_accel.y,
                       bma400_accel.z,
                       bma400_int_status);
                fflush(stdout);
            }
        }
    }

    coines_close_comm_intf(COINES_COMM_INTF_USB);
    return EXIT_SUCCESS;
}
