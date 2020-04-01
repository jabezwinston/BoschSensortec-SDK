/**
 * Copyright (C) Robert Bosch. All Rights Reserved. Confidential.
 *
 * Disclaimer
 *
 * Common:
 * Bosch Sensortec products are developed for the consumer goods industry. They may only be used
 * within the parameters of the respective valid product data sheet.  Bosch Sensortec products are
 * provided with the express understanding that there is no warranty of fitness for a particular purpose.
 * They are not fit for use in life-sustaining, safety or security sensitive systems or any system or device
 * that may lead to bodily harm or property damage if the system or device malfunctions. In addition,
 * Bosch Sensortec products are not fit for use in products which interact with motor vehicle systems.
 * The resale and/or use of products are at the purchasers own risk and his own responsibility. The
 * examination of fitness for the intended use is the sole responsibility of the Purchaser.
 *
 * The purchaser shall indemnify Bosch Sensortec from all third party claims, including any claims for
 * incidental, or consequential damages, arising from any product use not covered by the parameters of
 * the respective valid product data sheet or not approved by Bosch Sensortec and reimburse Bosch
 * Sensortec for all costs in connection with such claims.
 *
 * The purchaser must monitor the market for the purchased products, particularly with regard to
 * product safety and inform Bosch Sensortec without delay of all security relevant incidents.
 *
 * Engineering Samples are marked with an asterisk (*) or (e). Samples may vary from the valid
 * technical specifications of the product series. They are therefore not intended or fit for resale to third
 * parties or for use in end products. Their sole purpose is internal client testing. The testing of an
 * engineering sample may in no way replace the testing of a product series. Bosch Sensortec
 * assumes no liability for the use of engineering samples. By accepting the engineering samples, the
 * Purchaser agrees to indemnify Bosch Sensortec from all claims arising from the use of engineering
 * samples.
 *
 * Special:
 * This software module (hereinafter called "Software") and any information on application-sheets
 * (hereinafter called "Information") is provided free of charge for the sole purpose to support your
 * application work. The Software and Information is subject to the following terms and conditions:
 *
 * The Software is specifically designed for the exclusive use for Bosch Sensortec products by
 * personnel who have special experience and training. Do not use this Software if you do not have the
 * proper experience or training.
 *
 * This Software package is provided `` as is `` and without any expressed or implied warranties,
 * including without limitation, the implied warranties of merchantability and fitness for a particular
 * purpose.
 *
 * Bosch Sensortec and their representatives and agents deny any liability for the functional impairment
 * of this Software in terms of fitness, performance and safety. Bosch Sensortec and their
 * representatives and agents shall not be liable for any direct or indirect damages or injury, except as
 * otherwise stipulated in mandatory applicable law.
 *
 * The Information provided is believed to be accurate and reliable. Bosch Sensortec assumes no
 * responsibility for the consequences of use of such Information nor for any infringement of patents or
 * other rights of third parties which may result from its use. No license is granted by implication or
 * otherwise under any patent or patent rights of Bosch. Specifications mentioned in the Information are
 * subject to change without notice.
 *
 * It is not allowed to deliver the source code of the Software to any third party without permission of
 * Bosch Sensortec.
 *
 * @file        bhy2_hal.h
 *
 * @brief
 *
 *
 */

/*!
 * @defgroup bhy2_hal
 * @{*/

#ifndef BHY_HAL_H
#define BHY_HAL_H
#include "typedef.h"


#ifndef UNUSED
#define UNUSED(x) (void)(x)
#endif

//#define DBG_MSG_EN

#ifdef DBG_MSG_EN
#define DBG_PRINT(format, ...)       printf("[DBG]"format, ##__VA_ARGS__)
#define DBG_PRINT_TEXT(format, ...)     printf(format, ##__VA_ARGS__)
#define DBG_PRINT_FILE_LINE(format,...) printf("[DBG]FILE: "__FILE__", LINE: %d: "format, __LINE__, ##__VA_ARGS__)
#else
#define DBG_PRINT(format, ...)
#define DBG_PRINT_TEXT(format, ...)
#define DBG_PRINT_FILE_LINE(format,...)
#endif
#define TST_PRINT(format, ...)       printf("[Info]"format, ##__VA_ARGS__)
#define TST_PRINT_TEXT(format, ...)     printf(format, ##__VA_ARGS__)


#define BHA260_CS_PIN COINES_SHUTTLE_PIN_7

/*!
* @brief Read bus interface.
* @param[in] reg: register addr.
* @param[out] buf: data from register.
* @param[in] len: data length.
* @param[in] private_data: private data.
* @return 0 on success, negative on failure.
*/
int bhy2_bus_read(u8 reg, u8 *buf, u32 len, void *private_data);
/*!
* @brief Write bus interface.
* @param[in] reg: register addr.
* @param[in] buf: data written into register.
* @param[in] len: data length.
* @param[in] private_data: private data.
* @return 0 on success, negative on failure.
*/
int bhy2_bus_write(u8 reg, const u8 *buf, u32 len, void *private_data);
/*!
* @brief Delay function in microsecond.
* @param[in] private_data: private data.
*/
void bhy2_delay_us(u32 us, void *private_data);
/*!
* @brief Delay function in millisecond.
* @param[in] private_data: private data.
*/
void bhy2_delay_ms(u32 ms, void *private_data);
/*!
* @brief get hardware platform systick.
* @param[in] private_data: private data.
* @return tmp_systick: return platform systick, unit is millisecond.
*/
u32 bhy2_get_systick_ms(void *private_data);
/*!
* @brief get interrupt state.
* @param[in] private_data: private data.
* @return tmp_int: return interrupt state, 0: no int is assert, 1: int is assert.
*/
u8 bhy2_get_int_state(void *private_data);
#endif
