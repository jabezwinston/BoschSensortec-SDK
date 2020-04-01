/**
 * Copyright (C) 2020 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bmi270_read_gyro.c
 *
 */

/*!
 * @ingroup bmi2xyGroupExample
 * @defgroup bmi2xyGroupExample Gyro
 * @brief example to showcase gyroscope output
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
/*!         Static Function Declarations                     */

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
    /* Variable to define result */
    int8_t rslt;

    /* Structure to define BMI2 sensor configurations */
    struct bmi2_dev bmi2;

    /* Create an instance of sensor data structure */
    struct bmi2_sensor_data sensor_data = { 0 };

    /* Initialize status */
    uint16_t int_status = 0;

    /* Initialize loop */
    uint32_t loop = 1000;

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
            printf("Gyro data\n");

            /* Loop to print the gyro data when interrupt occurs */
            while (loop--)
            {
                /* Get the gyro data */
                rslt = bmi2_get_int_status(&int_status, &bmi2);

                /* To check the gyro status */
                if ((rslt == BMI2_OK) && (int_status & BMI2_GYR_DRDY_INT_MASK))
                {
                    sensor_data.type = BMI2_GYRO;

                    /* Get gyro data (raw LSB) */
                    rslt = bmi2_get_sensor_data(&sensor_data, 1, &bmi2);

                    if (rslt == BMI2_OK)
                    {
                        printf("x = %+06d\t", sensor_data.sens_data.gyr.x);
                        printf("y = %+06d\t", sensor_data.sens_data.gyr.y);
                        printf("z = %+06d\r", sensor_data.sens_data.gyr.z);
                    }
                }
            }
        }
    }

    printf("\n");

    return rslt;
}

/*!
 * @brief This internal API sets the sensor configuration
 */
static int8_t bmi2_set_config(struct bmi2_dev *bmi2_dev)
{
    /* Variable to define result */
    int8_t rslt;

    /* Initialize interrupts for gyroscope */
    uint8_t sens_int = BMI2_DRDY_INT;

    /* List the sensors which are required to enable */
    uint8_t sens_list = BMI2_GYRO;

    /* Structure to define the type of the sensor and its configurations */
    struct bmi2_sens_config config;

    /* Configure type of feature */
    config.type = BMI2_GYRO;

    /* Enable the selected sensors */
    rslt = bmi2_sensor_enable(&sens_list, 1, bmi2_dev);
    print_rslt(rslt);

    if (rslt == BMI2_OK)
    {
        /* Get default configurations for the type of feature selected */
        rslt = bmi2_get_sensor_config(&config, 1, bmi2_dev);
        print_rslt(rslt);

        if (rslt == BMI2_OK)
        {
            /* The user can change the following configuration parameter according to their requirement */
            /* Output data Rate. By default ODR is set as 200Hz for gyro */
            config.cfg.gyr.odr = BMI2_GYR_ODR_200HZ;

            /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps */
            config.cfg.gyr.range = BMI2_GYR_RANGE_2000;

            /* Gyroscope Bandwidth parameters. By default the gyro bandwidth is in normal mode */
            config.cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;

            /* Enable/Disable the noise performance mode for precision yaw rate sensing
             * There are two modes
             *  0 -> Ultra low power mode(Default)
             *  1 -> High performance mode/Power optimized mode(BMI2_POWER_OPT_MODE)
             */
            config.cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;

            /* Enable/Disable filter performance mode where averaging of samples will be done based on above set
             * bandwidth and ODR.
             * There are two modes
             *  0 -> Ultra low power mode
             *  1 -> High performance mode(Default)/ Performance optimized mode(BMI2_PERF_OPT_MODE)
             */
            config.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

            /* Set the gyro configurations */
            rslt = bmi2_set_sensor_config(&config, 1, bmi2_dev);

            if (rslt == BMI2_OK)
            {
                /* Map interrupt to pins */
                rslt = bmi2_map_data_int(sens_int, BMI2_INT2, bmi2_dev);
                print_rslt(rslt);
            }
        }
    }

    return rslt;
}
