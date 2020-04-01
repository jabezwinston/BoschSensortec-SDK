/**
 * Copyright (c) 2020 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file  bmi270_execute_crt.c
 *
 */

/*!
 * @ingroup bmi2xyGroupExample
 * @defgroup bmi2xyGroupExample Component Retrim
 * @brief example to showcase component retrim output
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
/*!            Functions                                        */

/* This function starts the execution of program */
int main(void)
{
    /* Structure to define BMI2 sensor configurations */
    struct bmi2_dev bmi2;

    /* Variable to define result */
    int8_t rslt;

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

    /* To Initialize bmi270 */
    rslt = bmi270_init(&bmi2);
    print_rslt(rslt);

    if (rslt == BMI2_OK)
    {
        /* This API is to run the CRT process */
        /* Do not shake the board for CRT test. If so, it will throw an abort error */
        rslt = bmi2_do_crt(&bmi2);
        print_rslt(rslt);

        if (rslt == BMI2_OK)
        {
            printf("CRT completed successfully\n");
        }
    }

    return rslt;
}
