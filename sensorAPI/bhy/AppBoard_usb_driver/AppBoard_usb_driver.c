/**\
 * Copyright (C) 2017 - 2018 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * File   	AppBoard_usb_driver.c
 * Date   	March-31-2017
 * Version  1.0.0
 *
 */
/*! @file AppBoard_usb_driver.c
 @brief This file takes care of APP2.0 board USB driver initialization*/

#include "coines.h"
#include "AppBoard_usb_driver.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>

/*********************************************************************/
/*! data buffer size */
#define DATABUF_SIZE    128
/* functions */

/* @brief This API is used to establish the USB communication of APP2.0 board*/
int AppBoard_usb_driver_init(void)
{
    int16_t rslt;
    struct coines_board_info board_info;

    rslt = coines_open_comm_intf(COINES_COMM_INTF_USB);
    if (rslt < 0)
    {
        printf("\n Unable to connect with Application Board ! \n"
               " 1. Check if the board is connected and powered on. \n"
               " 2. Check if Application Board USB driver is installed. \n"
               " 3. Check if board is in use by another application. (Insufficient permissions to access USB) \n");
        exit(rslt);
    }

    rslt = coines_get_board_info(&board_info);

    return rslt;
}

/* @brief This API is used to close the USB communication of APP2.0 board */
int AppBoard_usb_driver_shutdown(void)
{
    coines_close_comm_intf(COINES_COMM_INTF_USB);
    return EXIT_SUCCESS;
}
/*!
 * @brief  Get the polling interrupt status via BHI register 0x36 (when reading from FIFO)
 */
int AppBoard_get_int_status(uint16_t dev_addr)
{
    static uint8_t buf[DATABUF_SIZE];
    int numBytes = 1;
    int16_t rslt;
    rslt = coines_read_i2c(dev_addr, 0x36, buf, numBytes);
    return ((buf[0] & 0x01) && (rslt == 0));
}
