/**
 * Copyright (C) 2018 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bhy2_bme_env.c
 * @date    Dec-10-2018
 * @version 1.1
 * @brief   Sample file how to display bme using BHY2 
 *          with LIB COINES
 *
 */
/*********************************************************************/
/* system header files */
#include <stdlib.h>
#include <stdio.h>

/*********************************************************************/
/* own header files */
#include "coines.h"

#include "bhy2_api.h"
#include "bhy_host_interface.h"
#include "bhy2_hal.h"
#include "bhy2_cus.h"

#include "Bosch_SHUTTLE_BHI260_AK09915_BME280_altitude.h"

/*********************************************************************/
/* macro definitions */

#define BHA260_SHUTTLE_ID 0x139
#define BHI260_SHUTTLE_ID 0x119
#define VDD_VDDIO_VALUE   3300



#define ASSERT(expr) \
    if (0 != (expr)) \
        fail_and_exit(__LINE__, #expr, expr)

/*********************************************************************/
/* global variables */
bhy2_hub_t bhy2_hub_obj;


/*!
 * @brief exit function 
 *
 * @param[in] line: line number
 * @param[in] fct: function name
 * @param[in] val: value
 *
 * @return void
 */        
void fail_and_exit( int line, const char * fct, int val) 
{
    printf("\n\rFailed at line %d. \n\r The function %s returned %d\r\n", line, fct, val);
    exit(1);
}
        
/*!
 * @brief function init application board
 *
 * @param[in] void
 *
 * @return void
 */
void init_appboard_communication(void) 
{
    int16_t rslt = COINES_SUCCESS;
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

  if (rslt == COINES_SUCCESS)
  {
    if(BHA260_SHUTTLE_ID == board_info.shuttle_id)
    {
        printf("Found BHA260 Shuttleboard Successfully\n\r");
    } 
    else if(BHI260_SHUTTLE_ID == board_info.shuttle_id)
    {
        printf("Found BHI260 Shuttleboard Successfully\n\r");
    }
    else
    {    
        printf("Expecting BHA260 Shuttle ID 0x%04X or BHI260 Shuttle ID 0x%04X but found 0x%04X instead, aborting...\n\r", BHA260_SHUTTLE_ID, BHI260_SHUTTLE_ID,board_info.shuttle_id);    
        exit(0);    
    }
  }    
    /* Switch VDD for sensor off */
	ASSERT( coines_set_shuttleboard_vdd_vddio_config(0, 0) );

	coines_delay_msec(10);
	/* set the sensor interface as SPI */
	ASSERT( coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_10_MHZ, COINES_SPI_MODE3) );

    coines_delay_msec(10);
	/* Switch VDD for sensor on */
	ASSERT( coines_set_shuttleboard_vdd_vddio_config(VDD_VDDIO_VALUE, VDD_VDDIO_VALUE) );
    
	coines_delay_msec(10);
	/* CS pin is made low for selecting SPI protocol*/
	ASSERT( coines_set_pin_config(COINES_SHUTTLE_PIN_7, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW) );
	coines_delay_msec(10);
    coines_set_pin_config(COINES_SHUTTLE_PIN_7, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
    
    /* config INT pin */
    coines_set_pin_config(COINES_SHUTTLE_PIN_21, COINES_PIN_DIRECTION_IN, COINES_PIN_VALUE_LOW);
    
    
}

/*********************************************************************/
/* functions */
/*!
 * @brief install virtual sensor process callbacks
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[out] none.
 *
 * @return res: operation result
 */
s16 bhy2_install_generic_callbacks( bhy2_hub_t*  p_bhy2_hub )
{
    bhy2_virt_sensor_callback_register( p_bhy2_hub, BHY_SYS_SENSOR_ID_TS_SMALL_DELTA, bhy2_parse_frame_timestamp_small_delta);
    bhy2_virt_sensor_callback_register( p_bhy2_hub, BHY_SYS_SENSOR_ID_TS_LARGE_DELTA, bhy2_parse_frame_timestamp_large_delta);
    bhy2_virt_sensor_callback_register( p_bhy2_hub, BHY_SYS_SENSOR_ID_TS_FULL, bhy2_parse_frame_full_timestamp);
    bhy2_virt_sensor_callback_register( p_bhy2_hub, BHY_SYS_SENSOR_ID_META_EVENT, bhy2_parse_frame_meta_event);

    bhy2_virt_sensor_callback_register( p_bhy2_hub, BHY_SYS_SENSOR_ID_TS_SMALL_DELTA_WU, bhy2_parse_frame_timestamp_small_delta_wkup );
    bhy2_virt_sensor_callback_register( p_bhy2_hub, BHY_SYS_SENSOR_ID_TS_LARGE_DELTA_WU, bhy2_parse_frame_timestamp_large_delta_wkup );
    bhy2_virt_sensor_callback_register( p_bhy2_hub, BHY_SYS_SENSOR_ID_TS_FULL_WU, bhy2_parse_frame_full_timestamp_wkup );
    bhy2_virt_sensor_callback_register( p_bhy2_hub, BHY_SYS_SENSOR_ID_META_EVENT_WU, bhy2_parse_frame_meta_event_wkup );

    bhy2_virt_sensor_callback_register( p_bhy2_hub, BHY_SYS_SENSOR_ID_DEBUG_MSG, bhy2_parse_frame_debug_message);

    TST_PRINT ( "Install generic callbacks completely.\r\n");

    return 0;
}

/*!
 * @brief function to install sensor callbacks
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[out] none.
 *
 * @return res: operation result
 */
s16 bhy2_install_required_sensor_callbacks( bhy2_hub_t*  p_bhy2_hub )
{
    bhy2_virt_sensor_callback_register( p_bhy2_hub, BHY_SENSOR_ID_TEMP, bhy2_parse_frame_temperature);
    bhy2_virt_sensor_callback_register( p_bhy2_hub, BHY_SENSOR_ID_BARO, bhy2_parse_frame_barometer);
    bhy2_virt_sensor_callback_register( p_bhy2_hub, BHY_SENSOR_ID_HUM, bhy2_parse_frame_humidity);
    
    TST_PRINT ( "Install required sensor callbacks completely.\r\n");
    
    return 0;
}

/*!
 *  @brief Main Function where the execution getting started to test the code.
 *
 *  @param[in] void
 *
 *  @return status
 *
 */
int main(void)
{
    
    u8 tu8_data = 0;
    u16 tu16_data = 0;
    int32_t ts32_ret_val = 0;   
    u8 pin_value = 0;
    u8 pin_direction = 0;

    init_appboard_communication();

    /* init bhy2 obj */
    ASSERT ( bhy2_hub_init( &bhy2_hub_obj, bhy2_bus_read, bhy2_bus_write, bhy2_delay_us, 0 ) );
    /* reset hub */
    ASSERT ( bhy2_hub_reset( &bhy2_hub_obj ) );
    /* check hub state */
    ASSERT ( bhy2_product_id_get( &bhy2_hub_obj, &tu8_data) );
    ASSERT ( bhy2_fpga_version_get( &bhy2_hub_obj, &tu8_data) );
    ASSERT ( bhy2_rom_version_get( &bhy2_hub_obj, &tu16_data ) );
    /* set trigger mode on int pin */
    ASSERT ( bhy2_host_interrupt_ctrl_set( &bhy2_hub_obj, /*BHY_ICTL_EDGE|*/BHY_ICTL_DISABLE_STATUS) );
    ASSERT ( bhy2_register_get( &bhy2_hub_obj, BHY_REG_HOST_INTERRUPT_CTRL, &tu8_data) );
    printf("BHY_REG_HOST_INTERRUPT_CTRL: 0x%02X!\r\n", tu8_data);
    /* config status channel */
    ASSERT ( bhy2_host_interface_ctrl_set( &bhy2_hub_obj, BHY_IFCTL_ASYNC_STATUS_CHANNEL) );
    ASSERT ( bhy2_register_get( &bhy2_hub_obj, BHY_REG_HOST_INTERFACE_CTRL, &tu8_data) );

    ASSERT ( bhy2_register_get ( &bhy2_hub_obj, BHY_REG_BOOT_STATUS, &tu8_data) );
    TST_PRINT ( "Boot status: 0x%02X.\r\n", tu8_data);
    

    if((tu8_data & BHY_BST_HOST_INTERFACE_READY) == BHY_BST_HOST_INTERFACE_READY)
    {
        TST_PRINT ( "Host interface is ready.\r\n");
        if((tu8_data & BHY_BST_FLASH_DETECTED) == BHY_BST_FLASH_DETECTED)
        {
            TST_PRINT ( "Ext-flash is detected.\r\n");
            if((tu8_data & BHY_BST_FLASH_VERIFY_DONE) == BHY_BST_FLASH_VERIFY_DONE)//boot from flash is ok
            {
                if((tu8_data & BHY_BST_FLASH_VERIFY_ERROR) == BHY_BST_FLASH_VERIFY_ERROR)
                {
                    TST_PRINT ( "Verify ext-flash error.\r\n");
                }
                else
                {
                    TST_PRINT ( "Boot from ext-flash is OK.\r\n");
                }
            }
            else//no patch in flash
            {
                TST_PRINT ( "No patch is in ext-flash, need to write patch to ext-flash.\r\n");
            }
        }
        else//no flash, need to upload ram patch
        {
            TST_PRINT ( "No ext-flash is detected, need to upload ram patch.\r\n");

        }
    }
    else
    {
        //hub is not ready, need reset hub
        TST_PRINT ( "host interface is not ready, need reset hub.\r\n");

        bhy2_hub_reset( &bhy2_hub_obj );
    }
	fflush(stdout);

    /* upload ram ptach to ram */
    ts32_ret_val = bhy2_patch_upload_to_ram( &bhy2_hub_obj, bhy_firmware_image, sizeof(bhy_firmware_image));
    if(ts32_ret_val < 0)
    {
        TST_PRINT(" fw download status: %02d!\r\n", ts32_ret_val);
        ASSERT(ts32_ret_val);
    }
    /* if success, boot hub from ram */
    ts32_ret_val = bhy2_patch_boot_from_ram( &bhy2_hub_obj);
    if(ts32_ret_val < 0)
    {
        TST_PRINT("reboot status: %02d!\r\n", ts32_ret_val);
        ASSERT(ts32_ret_val);
    }

    bhy2_install_generic_callbacks( &bhy2_hub_obj );

    /* install virtual sensor callbacks */
    bhy2_install_required_sensor_callbacks( &bhy2_hub_obj );
	fflush(stdout);

    /* check ram version */
    ASSERT ( bhy2_ram_version_get( &bhy2_hub_obj, &tu16_data ));
    /* get current firmware's sensors and print them */
    ASSERT ( bhy2_virt_sensor_present_get ( &bhy2_hub_obj, NULL));

    float sample_rate_t = 1.0;
    ASSERT ( bhy2_virt_sensor_cfg_set(&bhy2_hub_obj,BHY_SENSOR_ID_TEMP,sample_rate_t, 0) ); 
    float sample_rate_b = 1.0;
    ASSERT ( bhy2_virt_sensor_cfg_set(&bhy2_hub_obj,BHY_SENSOR_ID_BARO,sample_rate_b, 0) );
    float sample_rate_h = 1.0;
    ASSERT ( bhy2_virt_sensor_cfg_set(&bhy2_hub_obj,BHY_SENSOR_ID_HUM, sample_rate_h, 0) );
    
    while (1) {

        do {
        
            ASSERT( coines_get_pin_config(COINES_SHUTTLE_PIN_21,
                                        (enum coines_pin_direction *)&pin_direction,
                                        (enum coines_pin_value *)&pin_value) );                  
            
        } while (pin_value == 0);

      /* parse & execute bhi160 fifo data */
      bhy2_parse_fifos( &bhy2_hub_obj);
      fflush(stdout);
    }
    
    return 0;
}



