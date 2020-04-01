/**
 * Copyright (C) 2020 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file  bmi270_perform_wrist_gesture.c
 *
 */

/*!
 * @ingroup bmi2xyGroupExample
 * @defgroup bmi2xyGroupExample Wrist gesture
 * @brief example to showcase wrist gesture output
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

    /* Variable to define result */
    int8_t rslt;

    /* Initialize status of wrist gesture interrupt */
    uint16_t int_status = 0;

    /* Select features and their pins to be mapped to */
    struct bmi2_sens_int_config sens_int = { .type = BMI2_WRIST_GESTURE, .hw_int_pin = BMI2_INT1 };

    /* Sensor data structure */
    struct bmi2_sensor_data sens_data = { 0 };

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

    /* Pointer to the configuration data buffer address */
    bmi2.config_file_ptr = NULL;

    /* The gesture movements are listed in array */
    const char *gesture_output[6] =
    { "unknown_gesture", "push_arm_down", "pivot_up", "wrist_shake_jiggle", "flick_in", "flick_out" };

    /* Initialize BMI270 */
    rslt = bmi270_init(&bmi2);
    print_rslt(rslt);

    if (rslt == BMI2_OK)
    {
        sens_data.type = BMI2_WRIST_GESTURE;

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
                printf("Flip the board in portrait/landscape mode:\n");

                /* Loop to print the wrist gesture data when interrupt occurs */
                while (1)
                {
                    int_status = 0;

                    /* Get the interrupt status of the wrist gesture */
                    rslt = bmi2_get_int_status(&int_status, &bmi2);
                    print_rslt(rslt);

                    if ((rslt == BMI2_OK) && (int_status & BMI270_WRIST_GEST_STATUS_MASK))
                    {
                        printf("Wrist gesture detected\n");

                        /* Get wrist gesture output */
                        rslt = bmi2_get_sensor_data(&sens_data, 1, &bmi2);
                        print_rslt(rslt);

                        printf("Wrist gesture = %d\r\n", sens_data.sens_data.wrist_gesture_output);

                        printf("Gesture output = %s\n", gesture_output[sens_data.sens_data.wrist_gesture_output]);
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
    uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_WRIST_GESTURE };

    /* Structure to define the type of the sensor and its configurations */
    struct bmi2_sens_config config;

    /* Configure type of feature */
    config.type = BMI2_WRIST_GESTURE;

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
            config.cfg.wrist_gest.wearable_arm = BMI2_ARM_LEFT;

            /* Set the new configuration along with interrupt mapping */
            rslt = bmi2_set_sensor_config(&config, 1, bmi2_dev);
            print_rslt(rslt);
        }
    }

    return rslt;
}
