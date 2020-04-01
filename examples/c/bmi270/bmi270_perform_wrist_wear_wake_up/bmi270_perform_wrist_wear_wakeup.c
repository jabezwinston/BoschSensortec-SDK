/**
 * Copyright (C) 2020 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file  bmi270_perform_wrist_wear_wakeup.c
 *
 */

/*!
 * @ingroup bmi2xyGroupExample
 * @defgroup bmi2xyGroupExample Wrist wear wakeup
 * @brief example to showcase wrist wear wakeup output
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
    /* Structure to define BMI2 sensor configurations */
    struct bmi2_dev bmi2;

    /* Variable to define result */
    int8_t rslt;

    /* Initialize status of wrist wear wakeup interrupt */
    uint16_t int_status = 0;

    /* Select features and their pins to be mapped to */
    struct bmi2_sens_int_config sens_int = { .type = BMI2_WRIST_WEAR_WAKE_UP, .hw_int_pin = BMI2_INT1 };

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

    /* Initialize BMI270 */
    rslt = bmi270_init(&bmi2);
    print_rslt(rslt);

    if (rslt == BMI2_OK)
    {
        /* Set the sensor configuration */
        rslt = bmi2_set_config(&bmi2);
        print_rslt(rslt);

        if (rslt == BMI2_OK)
        {
            /* Map the feature interrupt */
            rslt = bmi2_map_feat_int(&sens_int, 1, &bmi2);
            print_rslt(rslt);

            if (rslt == BMI2_OK)
            {
                printf("Tilt the board in landscape position to trigger wrist wear wakeup\n");

                /* Loop to print the wrist wear wakeup data when interrupt occurs */
                while (1)
                {
                    int_status = 0;

                    /* To get the interrupt status of the wrist wear wakeup */
                    rslt = bmi2_get_int_status(&int_status, &bmi2);
                    print_rslt(rslt);

                    /* To check the interrupt status of the wrist gesture */
                    if ((rslt == BMI2_OK) && (int_status & BMI270_WRIST_WAKE_UP_STATUS_MASK))
                    {
                        printf("Wrist wear wakeup detected\n");
                        break;
                    }
                }
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
    uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_WRIST_WEAR_WAKE_UP };

    /* Structure to define the type of the sensor and its configurations */
    struct bmi2_sens_config config;

    /* Configure type of feature */
    config.type = BMI2_WRIST_WEAR_WAKE_UP;

    /* Enable the selected sensors */
    rslt = bmi2_sensor_enable(sens_list, 2, bmi2_dev);
    print_rslt(rslt);

    if (rslt == BMI2_OK)
    {
        /* Get default configurations for the type of feature selected */
        rslt = bmi2_get_sensor_config(&config, 1, bmi2_dev);
        print_rslt(rslt);

        if (rslt == BMI2_OK)
        {
            /* Set the new configuration */
            rslt = bmi2_set_sensor_config(&config, 1, bmi2_dev);
            print_rslt(rslt);
        }
    }

    return rslt;
}
