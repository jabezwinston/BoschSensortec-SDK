/**
 * Copyright (C) 2020 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bmi2xy_hal_interface.c
 *
 */

/******************************************************************************/
/*!              Header Files                                  */
#include <stdio.h>
#include <stdlib.h>
#include "bmi2xy_hal_interface.h"

/******************************************************************************/
/*!              Macros                                  */
#define BMI2XY_SHUTTLE_ID UINT16_C(0x1B8)

/******************************************************************************/
/*!              Static Function Prototype                                  */

/*!
 *  @brief This internal API is used to initialize the hal function.
 *
 *  @return void.
 */
static void bmi2xy_hal_init(void);

/******************************************************************************/
/*!              Functions                                  */

/*! This internal API is used to initialize the hal function */
static void bmi2xy_hal_init(void)
{
    struct coines_board_info board_info;
    int16_t rslt = 0;

    coines_open_comm_intf(COINES_COMM_INTF_USB);

    rslt = coines_get_board_info(&board_info);

#if defined(PC)
    setbuf(stdout, NULL);
#endif

    if (rslt == COINES_SUCCESS)
    {
        if ((board_info.shuttle_id != BMI2XY_SHUTTLE_ID))
        {
            printf("! Warning invalid sensor shuttle \n ," "This application will not support this sensor \n");
            exit(COINES_E_FAILURE);
        }
    }
}

/*! This API is used to initialize the I2C configuration */
void bmi2xy_hal_i2c_init(void)
{
    bmi2xy_hal_init();

    /* Switch VDD for sensor off */
    coines_set_shuttleboard_vdd_vddio_config(0, 0);
    coines_delay_msec(50);

    coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_FAST_MODE);

    coines_delay_msec(50);
    /* Switch VDD for sensor on */
    coines_set_shuttleboard_vdd_vddio_config(3300, 3300);
    coines_delay_msec(50);
}

/*! This API is used to initialize the SPI configuration */
void bmi2xy_hal_spi_init(void)
{
    bmi2xy_hal_init();

    /* Switch VDD for sensor off */
    coines_set_shuttleboard_vdd_vddio_config(0, 0);
    coines_delay_msec(50);

    coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_5_MHZ, COINES_SPI_MODE3);


    /* Switch VDD for sensor on */
    coines_delay_msec(50);
    coines_set_shuttleboard_vdd_vddio_config(3300, 3300);

    coines_delay_msec(50);
    coines_set_pin_config(COINES_SHUTTLE_PIN_7, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);

}

/*! This API is used to perform I2C read operation with sensor */
int8_t bmi2xy_hal_i2c_bus_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    int8_t rslt = 0;

    rslt = coines_read_i2c(dev_id, reg_addr, reg_data, length);

    return rslt;
}

/*! This API is used to perform SPI read operation with sensor */
int8_t bmi2xy_hal_spi_bus_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    int8_t rslt = 0;

    rslt = coines_read_spi(dev_id, reg_addr, reg_data, length);

    return rslt;
}

/*! This API is used to perform I2C write operations with sensor */
int8_t bmi2xy_hal_i2c_bus_write(uint8_t dev_id, uint8_t reg_addr, const uint8_t *reg_data, uint16_t length)
{
    int8_t rslt = 0;

    rslt = coines_write_i2c(dev_id, reg_addr, (uint8_t *)reg_data, length);

    return rslt;
}

/*! This API is used to perform SPI write operations with sensor */
int8_t bmi2xy_hal_spi_bus_write(uint8_t dev_id, uint8_t reg_addr, const uint8_t *reg_data, uint16_t length)
{
    int8_t rslt = 0;

    rslt = coines_write_spi(dev_id, reg_addr, (uint8_t *)reg_data, length);

    return rslt;
}

/*! This API provides the mentioned delay in microseconds */
void bmi2xy_hal_delay_usec(uint32_t us)
{
	coines_delay_usec(us);
}
