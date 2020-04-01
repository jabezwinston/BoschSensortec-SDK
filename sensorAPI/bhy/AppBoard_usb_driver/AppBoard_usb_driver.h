/**
 * Copyright (C) 2017 - 2018 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    AppBoard_usb_driver.h
 * @date    March-31-2017
 * @version 1.0.0
 * @brief
 *
 */
/*! @file AppBoard_usb_driver.h
 @brief APP2.0 Board USB driver Module */
/*!
 * @defgroup APPBOARD_USB_DRIVER
 * @{*
 */

/*********************************************************************/
/* header files */

#include <stdint.h>

/*********************************************************************/
/* function prototype declarations */

/*!
 * @brief		Initializes the usb driver
 *
 * @return		returns if the initialization is successful or not
 *
 */
int AppBoard_usb_driver_init(void);

/*!
 * @brief		Closes the usb communication
 *
 * @return		returns if the process is successful or not
 *
 */
int AppBoard_usb_driver_shutdown(void);

/*!
 * @brief       Get the polling interrupt status via BHI register 0x36 (when reading from FIFO)
 *
 * @param[in] dev_addr    Device address for I2C communication.
 *
 * @return      returns if the process is successful or not
 *
 */
int AppBoard_get_int_status(uint16_t dev_addr);
