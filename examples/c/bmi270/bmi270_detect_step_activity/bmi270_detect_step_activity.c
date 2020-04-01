/**
 * Copyright (c) 2020 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bmi270_detect_step_activity.c
 *
 */

/*!
 * @ingroup bmi2xyGroupExample
 * @defgroup bmi2xyGroupExample Step activity
 * @brief example to showcase step activity output
 */

/******************************************************************************/
/*!            Header Files                                  */
#include <stdio.h>
#include "bmi2.h"
#include "bmi2xy_hal_interface.h"
#include "bmi270.h"
#include "bmi2_error_codes.h"

/******************************************************************************/
/*!            Macros                                        */

/*! I2C interface communication, 1 - Enable; 0- Disable */
#define BMI270_INTERFACE_I2C UINT8_C(0)

/*! SPI interface communication, 1 - Enable; 0- Disable */
#define BMI270_INTERFACE_SPI UINT8_C(1)

#if (!((BMI270_INTERFACE_I2C==1) && (BMI270_INTERFACE_SPI==0)) && \
    (!((BMI270_INTERFACE_I2C==0) && (BMI270_INTERFACE_SPI==1))))
#error "Invalid value given for the macros BMI270_INTERFACE_I2C / BMI270_INTERFACE_SPI"
#endif

/******************************************************************************/
/*!         Static Function Declarations                          */

/*!
 *  @brief This internal API is used to set the sensor configuration.
 *
 *  @param[in] bmi2_dev     : Structure instance of bmi2_dev.
 *
 *  @return Status of execution.
 */
static int8_t bmi2_set_config(struct bmi2_dev *bmi2_dev);

/******************************************************************************/
/*!            Functions                                        */

/* This function starts the execution of program */
int main(void)
{
    /* Structure to define bmi2 sensor configurations */
    struct bmi2_dev bmi2;

    /* Structure to define bmi2 sensor data */
    struct bmi2_sensor_data sensor_data;

    /* Variable to define result */
    int8_t rslt;

    /* Variable to define the duration of the loop */
    uint16_t wait_for_steps = 300;

    /* Sensor type to get data */
    sensor_data.type = BMI2_STEP_ACTIVITY;

    /* Bus configuration : I2C */
    #if BMI270_INTERFACE_I2C == 1

    /* To initialize the hal function */
    bmi2xy_hal_i2c_init();
    bmi2.dev_id = BMI2_I2C_PRIM_ADDR;
    bmi2.intf = BMI2_I2C_INTERFACE;
    bmi2.read = bmi2xy_hal_i2c_bus_read;
    bmi2.write = bmi2xy_hal_i2c_bus_write;

    /* Bus configuration : SPI */
    #elif BMI270_INTERFACE_SPI == 1

    /* To initialize the hal function */
    bmi2xy_hal_spi_init();
    bmi2.dev_id = SPI_CS;
    bmi2.intf = BMI2_SPI_INTERFACE;
    bmi2.read = bmi2xy_hal_spi_bus_read;
    bmi2.write = bmi2xy_hal_spi_bus_write;
    #endif

    bmi2.read_write_len = 32;
    bmi2.delay_us = bmi2xy_hal_delay_usec;

    /* Config file pointer should be assigned to NULL, so that default file address is assigned in bmi270_init */
    bmi2.config_file_ptr = NULL;

    const char *activity_output[4] = { "BMI2_STILL", "BMI2_WALK", "BMI2_RUN", "BMI2_UNKNOWN" };

    /* Initialize bmi270 */
    rslt = bmi270_init(&bmi2);
    print_rslt(rslt);

    if (rslt == BMI2_OK)
    {
        /* Set the sensor configuration */
        rslt = bmi2_set_config(&bmi2);
        print_rslt(rslt);

        if (rslt == BMI2_OK)
        {
            printf("Move the board for step activity\n");

            /* Loop to print the step activity data when interrupt occurs */
            while (wait_for_steps--)
            {
                /* Get step activity output */
                rslt = bmi2_get_sensor_data(&sensor_data, 1, &bmi2);
                print_rslt(rslt);

                printf("Step activity  = %s\n", activity_output[sensor_data.sens_data.activity_output]);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This internal API sets the sensor configuration
 */
static int8_t bmi2_set_config(struct bmi2_dev *bmi2_dev)
{
    /* Variable to define result */
    int8_t rslt;

    /* List the sensors which are required to enable */
    uint8_t sensor_sel[2] = { BMI2_ACCEL, BMI2_STEP_ACTIVITY };

    /* Select features and their pins to be mapped to */
    struct bmi2_sens_int_config sens_int = { .type = BMI2_STEP_ACTIVITY, .hw_int_pin = BMI2_INT2 };

    /* Structure to define the type of the sensor and its configurations */
    struct bmi2_sens_config config;

    /* Update the type of sensor for setting the configurations */
    config.type = BMI2_STEP_ACTIVITY;

    /* Enable the accelerometer and step-counter sensor */
    rslt = bmi2_sensor_enable(sensor_sel, 2, bmi2_dev);
    print_rslt(rslt);

    if (rslt == BMI2_OK)
    {
        /* Get default configurations for the type of feature selected */
        rslt = bmi2_get_sensor_config(&config, 1, bmi2_dev);
        print_rslt(rslt);

        if (rslt == BMI2_OK)
        {
            /* Set the configurations */
            rslt = bmi2_set_sensor_config(&config, 1, bmi2_dev);
            print_rslt(rslt);

            if (rslt == BMI2_OK)
            {
                /* Map the feature interrupt */
                rslt = bmi2_map_feat_int(&sens_int, 1, bmi2_dev);
                print_rslt(rslt);
            }
        }
    }

    return rslt;
}
