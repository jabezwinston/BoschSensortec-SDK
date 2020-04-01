/**
 * Copyright (C) 2019 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    hal.c
 * @date    Feb-05-2019
 * @brief   Hardware abstraction layer for the bhy2 command line interface
 *
 */

#ifndef BHY_HAL_H
#define BHY_HAL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/**
 * @brief Start the communication interface
 * @param[in] intf: Select the interface with the sensor
 */
void start_board_communication();

/**
 * @brief End the communication with the sensor
 */
void end_board_communication(void);

/*!
* @brief Read registers over SPI
* @param[in] reg: register addr.
* @param[out] buf: data from register.
* @param[in] len: data length.
* @param[in] private_data: private data.
* @return 0 on success, negative on failure.
*/
int32_t spi_read(uint8_t reg, uint8_t *buf, uint32_t len, void *private_data);

/*!
* @brief Write registers over SPI
* @param[in] reg: register addr.
* @param[in] buf: data written into register.
* @param[in] len: data length.
* @param[in] private_data: private data.
* @return 0 on success, negative on failure.
*/

int32_t spi_write(uint8_t reg, const uint8_t *buf, uint32_t len, void *private_data);

/*!
* @brief Delay function in microsecond.
* @param[in] private_data: private data.
*/
void delay_us(uint32_t us, void *private_data);
/*!
* @brief Delay function in millisecond.
* @param[in] private_data: private data.
*/
void delay_ms(uint32_t ms, void *private_data);
/*!
* @brief get hardware platform systick.
* @param[in] private_data: private data.
* @return tmp_systick: return platform systick, unit is millisecond.
*/
uint32_t get_systick_ms(void *private_data);
/*!
* @brief get interrupt state.
* @param[in] private_data: private data.
* @return tmp_int: return interrupt state, 0: no int is assert, 1: int is assert.
*/
uint8_t get_int_state(void *private_data);

/**
 * @brief Get the status of the interrupt pin
 * @return  true if the pin is high and false if low.
 */
bool get_intr_pin_status(void);

/**
 * @brief Get the status of the pins connected on the board
 * @param[in] payload argument passed from the command line
 */
void get_pin_state(const char *payload);

#endif
