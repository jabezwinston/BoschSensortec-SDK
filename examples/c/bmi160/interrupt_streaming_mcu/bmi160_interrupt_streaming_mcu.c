/**
 * Copyright (C) 2019 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bmi160_interrupt_streaming_mcu.c
 * @brief   Sample file demonstrates how to configure data ready interrupt 
 *          and read accel,gyro data from BMI160
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
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

#define BMI160_ACCEL_DATA_READY_MASK    0x80
#define BMI160_GYRO_DATA_READY_MASK     0x40

/*********************************************************************/
/* global variables */
/*********************************************************************/

/*! @brief This structure containing relevant bmi160 info */
struct bmi160_dev bmi160dev;
/*! bmi160 int config */
struct bmi160_int_settg sensor_int_config;
bool drdy_int = false;

/*********************************************************************/
/* function declarations */
/*********************************************************************/

/*!
 * @brief	internal API is used to initialize the sensor interface
 */
static void init_sensor_interface(void);
/*!
 * @brief	 This internal API is used to initialize the bmi160 sensor
 */
static void init_bmi160(void);
/*!
 * @brief	 This internal API is used to initialize and map the sensor driver interface
 */
static void init_bmi160_sensor_driver_interface(void);
/*!
 * @brief	 This internal API is used to enable the bmi160 interrupt
 */
static void enable_bmi160_interrupt();
/*!
 * @brief	 This internal API is used to disable the bmi160 interrupt
 */
static void disable_bmi160_interrupt();
/*!
 * @brief	 Data ready interrupt callback
 */
void bmi160_drdy_int();

struct bmi160_sensor_data bmi160_accel;
struct bmi160_sensor_data bmi160_gyro;

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
        fflush(stdout);
    }

    else
    {
        printf("BMI160 initialization failure !\n");
        exit(COINES_E_FAILURE);
    }
    coines_delay_msec(100);

    /* Select the Output data rate, range of accelerometer sensor */
    bmi160dev.accel_cfg.odr = BMI160_ACCEL_ODR_100HZ;
    bmi160dev.accel_cfg.range = BMI160_ACCEL_RANGE_16G;
    bmi160dev.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

    /* Select the power mode of accelerometer sensor */
    bmi160dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    bmi160dev.gyro_cfg.odr = BMI160_GYRO_ODR_200HZ;
    bmi160dev.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    bmi160dev.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

    /* Select the power mode of Gyroscope sensor */
    bmi160dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&bmi160dev);

    if ((bmi160dev.accel_cfg.power == BMI160_ACCEL_SUSPEND_MODE)
        && (bmi160dev.gyro_cfg.power == BMI160_GYRO_SUSPEND_MODE))
    {
        printf("Accel and gyro sensors are in suspend mode\n Use them in normal/Low power/Fast start-up mode !!");
        exit(EXIT_FAILURE);
    }
}
/*!
 *  @brief This API is used to enable bmi160 interrupt
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void enable_bmi160_interrupt()
{
    int8_t rslt;

    /* Select the Interrupt channel/pin */
    sensor_int_config.int_channel = BMI160_INT_CHANNEL_1;
    /* Select the Interrupt type */
    sensor_int_config.int_type = BMI160_ACC_GYRO_DATA_RDY_INT; /* Choosing Data ready interrupt */

    sensor_int_config.int_pin_settg.output_en = BMI160_ENABLE; /* Enabling interrupt pins to act as output pin */
    sensor_int_config.int_pin_settg.output_mode = BMI160_DISABLE; /* Choosing push-pull mode for interrupt pin */
    sensor_int_config.int_pin_settg.output_type = BMI160_DISABLE; /* Choosing active low output */
    sensor_int_config.int_pin_settg.edge_ctrl = BMI160_ENABLE; /* Choosing edge triggered output */
    sensor_int_config.int_pin_settg.input_en = BMI160_DISABLE; /* Disabling interrupt pin to act as input */
    sensor_int_config.int_pin_settg.latch_dur = BMI160_LATCH_DUR_NONE; /* non-latched output */

    /* Configure the Data ready interrupt */
    rslt = bmi160_set_int_config(&sensor_int_config, &bmi160dev); /* sensor is an instance of the structure bmi160_dev  */

    if (rslt != BMI160_OK)
    {
        printf("BMI160 enable interrupt configuration failure!\n");
        exit(COINES_E_FAILURE);
    }
}

/*!
 *  @brief This API is used to disable bmi160 interrupt
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void disable_bmi160_interrupt()
{
    int8_t rslt;

    /* Select the Interrupt channel/pin */
    sensor_int_config.int_channel = BMI160_INT_CHANNEL_NONE;
    sensor_int_config.int_pin_settg.output_en = BMI160_DISABLE; /* Disabling interrupt pins to act as output pin */
    sensor_int_config.int_pin_settg.edge_ctrl = BMI160_DISABLE; /* Choosing edge triggered output */

    /* Set the Data ready interrupt */
    rslt = bmi160_set_int_config(&sensor_int_config, &bmi160dev); /* sensor is an instance of the structure bmi160_dev  */

    /* Disable accel data ready interrupt channel */
    if (rslt != BMI160_OK)
    {
        printf("BMI160 disable interrupt configuration failure!\n");
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

    coines_open_comm_intf(COINES_COMM_INTF_USB); /* Wait here till the PC connects to the COM port */

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

    coines_attach_interrupt(COINES_SHUTTLE_PIN_20, bmi160_drdy_int, COINES_PIN_INTERRUPT_FALLING_EDGE);
    /*Enable data ready interrupts*/
    enable_bmi160_interrupt();

    uint8_t int_status = 0, sensor_select = 0;
    uint32_t start_time = coines_get_millis();

    /* Run interrupt streaming for 10s before disabling interrupts */
    while (coines_get_millis() - start_time < 10000)
    {
        if (drdy_int == true)
        {
            drdy_int = false;

            bmi160_get_regs(BMI160_STATUS_ADDR, &int_status, 1, &bmi160dev);

            if (int_status & BMI160_ACCEL_DATA_READY_MASK)
                sensor_select |= BMI160_ACCEL_SEL;

            if (int_status & BMI160_GYRO_DATA_READY_MASK)
                sensor_select |= BMI160_GYRO_SEL;

            bmi160_get_sensor_data(sensor_select, &bmi160_accel, &bmi160_gyro, &bmi160dev);

            if (int_status & BMI160_ACCEL_DATA_READY_MASK)
                printf("ax:%d\tay:%d\taz:%d ", bmi160_accel.x, bmi160_accel.y, bmi160_accel.z);

            if (int_status & BMI160_GYRO_DATA_READY_MASK)
                printf("gx:%d\tgy:%d\tgz:%d", bmi160_gyro.x, bmi160_gyro.y, bmi160_gyro.z);

            printf("\tts:%lu\n", coines_get_millis() - start_time);
            int_status = sensor_select = 0;
        }
    }

    coines_detach_interrupt(COINES_SHUTTLE_PIN_20);
    /*disable data ready interrupts*/
    disable_bmi160_interrupt();

    /*close the communication*/
    coines_close_comm_intf(COINES_COMM_INTF_USB);

    return EXIT_SUCCESS;
}

/* BMI160 data ready interrupt callback */

void bmi160_drdy_int()
{
    drdy_int = true;
}
