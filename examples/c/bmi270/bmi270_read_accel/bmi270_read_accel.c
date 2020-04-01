/**
 * Copyright (C) 2020 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bmi270_read_accel.c
 *
 */

/*!
 * @ingroup bmi2xyGroupExample
 * @defgroup bmi2xyGroupExample Accel
 * @brief example to showcase accelerometer output
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
/*!           Functions                                        */

/* This function starts the execution of program */
int main(void)
{
    /* Structure to define BMI2 sensor configurations */
    struct bmi2_dev bmi2;

    /* Variable to define result */
    int8_t rslt;

    uint32_t loop = 1000;

    /* Create an instance of sensor data structure */
    struct bmi2_sensor_data sensor_data = { 0 };

    /* Initialize status of accel data interrupt */
    uint16_t int_status = 0;

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
        rslt = bmi2_set_config(&bmi2);
        print_rslt(rslt);

        if (rslt == BMI2_OK)
        {
            printf("Accel data \n\r");

            /* Loop to print the accel data when interrupt occurs */
            while (loop--)
            {
                /* To get the status of interrupt */
                rslt = bmi2_get_int_status(&int_status, &bmi2);

                /* To check the accel data interrupt status */
                if ((rslt == BMI2_OK) && (int_status & BMI2_ACC_DRDY_INT_MASK))
                {
                    /* Get accelerometer data (raw LSB) */
                    rslt = bmi2_get_sensor_data(&sensor_data, 1, &bmi2);
                    print_rslt(rslt);

                    if (rslt == BMI2_OK)
                    {
                        printf("x = %+06d\t", sensor_data.sens_data.acc.x);
                        printf("y = %+06d\t", sensor_data.sens_data.acc.y);
                        printf("z = %+06d\r", sensor_data.sens_data.acc.z);
                    }
                }
            }
        }
    }

    printf("\n");

    return rslt;
}

static int8_t bmi2_set_config(struct bmi2_dev *bmi2_dev)
{
    /* Variable to define result */
    int8_t rslt;

    /* List the sensors which are required to enable */
    uint8_t sensor_list = BMI2_ACCEL;

    /* Create an instance of sensor configuration structure */
    struct bmi2_sens_config config = { 0 };

    /* Enable the selected sensors */
    rslt = bmi2_sensor_enable(&sensor_list, 1, bmi2_dev);
    print_rslt(rslt);

    if (rslt == BMI2_OK)
    {
        /* The user can change the following configuration parameter according to their requirement */
        /* By default ODR is set as 100Hz for accel */
        config.cfg.acc.odr = BMI2_ACC_ODR_100HZ;

        /* G range of the sensor (+/- 2G, 4G, 8G, 16G) */
        config.cfg.acc.range = BMI2_ACC_RANGE_2G;

        /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
         * if it is set to 2, then 2^(bandwidth parameter) samples
         * are averaged, resulting in 4 averaged samples
         * Note1 : For more information, refer the datasheet.
         * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
         * this has an adverse effect on the power consumed.
         */
        config.cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;

        /* Enable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)/ Performance optimized mode(BMI2_PERF_OPT_MODE)
         * For more info refer datasheet.
         */
        config.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

        /* Set the accel configurations */
        rslt = bmi2_set_sensor_config(&config, 1, bmi2_dev);
        print_rslt(rslt);

        if (rslt == BMI2_OK)
        {
            /* Map interrupt to pins */
            rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, bmi2_dev);
            print_rslt(rslt);

        }
    }

    return rslt;
}
