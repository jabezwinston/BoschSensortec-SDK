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
 * @file        bhy2_hal.c
 *
 * @brief
 *
 *
 */

/*!
 * @defgroup bhy2_hal.c
 * @{*/

#include "bhy2_hal.h"
#include "coines.h"
#include <time.h>

/*!
* @brief Read bus interface.
* @param[in] reg: register addr.
* @param[out] buf: data from register.
* @param[in] len: data length.
* @param[in] private_data: private data.
* @return 0 on success, negative on failure.
*/
int bhy2_bus_read(u8 reg, u8 *buf, u32 len, void *private_data)
{
    UNUSED(private_data);
    return coines_read_spi(BHA260_CS_PIN, reg, buf, len);
}

/*!
* @brief Write bus interface.
* @param[in] reg: register addr.
* @param[in] buf: data written into register.
* @param[in] len: data length.
* @param[in] private_data: private data.
* @return 0 on success, negative on failure.
*/
int bhy2_bus_write(u8 reg, const u8 *buf, u32 len, void *private_data)
{

    UNUSED(private_data);
    return coines_write_spi(BHA260_CS_PIN, reg, (u8*)buf, len);
}

/*!
* @brief Delay function in microsecond.
* @param[in] private_data: private data.
*/
void bhy2_delay_us(u32 us, void *private_data)
{
    UNUSED(private_data);
#ifdef PC
    struct timespec req, rem;
    req.tv_sec = 0;
    req.tv_nsec = us*1000;
    nanosleep(&req, &rem);
#endif

#ifdef MCU_APP20
    /*Approx delay*/
    for(volatile uint32_t us_c= us; us_c > 0; us_c --)
    for(volatile uint8_t c=0;c<10;c++);
#endif

}

/*!
* @brief Delay function in millisecond.
* @param[in] private_data: private data.
*/
void bhy2_delay_ms(u32 ms, void *private_data)
{
#ifdef PC
    bhy2_delay_us(ms*1000, private_data);
#endif

#ifdef MCU_APP20
    coines_delay_msec(ms);
#endif
}

/*!
* @brief get hardware platform systick.
* @param[in] private_data: private data.
* @return tmp_systick: return platform systick, unit is millisecond.
*/
u32 bhy2_get_systick_ms(void *private_data)
{
    UNUSED(private_data);
#ifdef PC
    clock_t currentTime = clock();

    return (u32)(currentTime / (CLOCKS_PER_SEC / 1000));
#endif

#ifdef MCU_APP20
    return coines_get_millis();
#endif
}
