/**\
 * Copyright (C) 2017 - 2018 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * File   	BHy_support.c
 * Date   	March-31-2017
 * Version  1.0.0
 *
 */
/*! @file BHy_support.c
 @brief This file takes care of supporting BHY read/write functions*/

/*********************************************************************/
/* header files */

#include "coines.h"
#include "BHy_support.h"

extern void mdelay(uint32_t ul_dly_ticks);

/*********************************************************************/
/* functions */

/* @brief This API is used for initializing the BHY sensor */
void bhy_initialize_support(void)
{
	bhy.bus_write = coines_write_i2c;
	bhy.bus_read = coines_read_i2c;
	bhy.delay_msec = coines_delay_msec;
	bhy.device_addr = BHY_I2C_SLAVE_ADDRESS;
	coines_delay_msec(5);
	coines_set_shuttleboard_vdd_vddio_config(0, 0);
	coines_delay_msec(5);
	/* set the sensor interface as I2C */
	coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_STANDARD_MODE);
	coines_delay_msec(5);
	coines_set_shuttleboard_vdd_vddio_config(3300, 3300);
	coines_delay_msec(200);
	bhy_reset();
	bhy_init(&bhy);
}

/* @brief This API is used for configuring the values of BHY sensor */
int8_t bhy_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
	int r = coines_write_i2c(dev_addr, reg_addr, reg_data, (uint8_t)length);
	return (r < 0 ? BHY_ERROR : BHY_SUCCESS);
}

/* @brief This API is used for reading values from BHY sensor */
int8_t bhy_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *rx_data, uint16_t length)
{
	int r = coines_read_i2c(dev_addr, reg_addr, rx_data, (uint8_t)length);
	return (r < 0 ? BHY_ERROR : BHY_SUCCESS);
}

/* @brief This API is used for inserting a delay in terms of milli seconds */
void bhy_delay_msec(u32 msec)
{
	mdelay(msec);
}

/* @brief This API is used for re-setting BHY sensor */
void bhy_reset(void)
{
	bhy_set_reset_request(BHY_RESET_ENABLE);
}
