/**
 * Copyright (C) 2020 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bmi2xy_hal_interface.h
 *
 */

#ifndef __BMI2XY_HAL_INTERFACE_H__
#define __BMI2XY_HAL_INTERFACE_H__

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/*!              Header Files                                  */
#include "bmi270.h"
#include "coines.h"
/******************************************************************************/

/*! Macro to define SPI CS Pin */
#define SPI_CS               COINES_SHUTTLE_PIN_7

/*!            Functions                                        */

/*!
 * @brief This API is used to initialize the I2C configuration.
 */
void bmi2xy_hal_i2c_init(void);

/*!
 * @brief This API is used to initialize the SPI configuration.
 */
void bmi2xy_hal_spi_init(void);

/*!
 * @brief This API is used to perform I2C read operation with sensor.
 *
 * @param[in] dev_id           : I2C address identification
 * @param[in] reg_addr           : Register address.
 * @param[out] reg_data          : Read buffer to store data from register.
 * @param[in] length           : Read data length.
 *
 * @return Result of API execution status.
 * @retval 0 Success
 * @retval >0 Warning
 * @retval <0 Error / failure
 */
int8_t bmi2xy_hal_i2c_bus_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);

/*!
 * @brief This API is used to perform SPI read operation with sensor.
 *
 * @param[in] dev_id           : Used for SPI chip select identification
 * @param[in] reg_addr           : Register address.
 * @param[out] reg_data          : Read buffer to store data from register.
 * @param[in] length           : Read data length.
 *
 * @return Result of API execution status.
 * @retval 0 Success
 * @retval >0 Warning
 * @retval <0 Error / failure
 */
int8_t bmi2xy_hal_spi_bus_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);

/*!
 * @brief This API is used to perform I2C write operations with sensor.
 *
 * @param[in] dev_id           : I2C address identification
 * @param[in] reg_addr           : Register address.
 * @param[out] reg_data          : Write buffer
 * @param[in] length           : Write data length.
 *
 * @return Result of API execution status.
 * @retval 0 Success
 * @retval >0 Warning
 * @retval <0 Error / failure
 */
int8_t bmi2xy_hal_i2c_bus_write(uint8_t dev_id, uint8_t reg_addr, const uint8_t *reg_data, uint16_t length);

/*!
 * @brief This API is used to perform SPI write operations with sensor.
 *
 * @param[in] dev_id           : SPI address identification
 * @param[in] reg_addr           : Register address.
 * @param[out] reg_data          : Write buffer
 * @param[in] length           : Write data length.
 *
 * @return Result of API execution status.
 * @retval 0 Success
 * @retval >0 Warning
 * @retval <0 Error / failure
 */
int8_t bmi2xy_hal_spi_bus_write(uint8_t dev_id, uint8_t reg_addr, const uint8_t *reg_data, uint16_t length);

/*!
 * @brief This API provides the mentioned delay in microseconds.
 *
 * @param[in] ms            : Microseconds to delay.
 *
 * @return None.
 */
void bmi2xy_hal_delay_usec(uint32_t us);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* __BMI2XY_HAL_INTERFACE_H__ */
