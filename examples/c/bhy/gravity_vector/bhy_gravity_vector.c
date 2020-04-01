/**\
 * Copyright (C)  2018 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * File     gravity_vector_example.c
 * @brief This demo streams the gravity vector to a terminal program on the computer at 25hz sampling rate
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "bhy_uc_driver.h"

#ifdef BHI160
#include "BHI160fw.h"
#endif
#ifdef BHI160B
#include "BHI160Bfw.h"
#endif
#ifdef BHA250
#include "BHA250fw.h"
#endif
/*********************************************************************/
/* own header files */
/*********************************************************************/
#include "coines.h"

/*********************************************************************/
/* macro definitions */
/*********************************************************************/
#define ARRAYSIZE 69    //should be greater or equal to 69 bytes, page size (50) + maximum packet size(18) + 1

/* @brief This API is used for parsing the gesture data from BHY and to display in terms
 * Glance, Pickup, significant motion
 *
 * @param[in] sensor_data
 * @param[in] sensor_id : Sensor Identifier for the virtual sensor.
 *
 * @return        NULL
 *
 */
void sensors_callback(bhy_data_generic_t * sensor_data,
                      bhy_virtual_sensor_t sensor_id);

/*! buffer to hold  the gravity vector values in terms of string*/
char outBuffer[60] = " X: 0.999  Y: 0.999  Z: 0.999   \r";

/*********************************************************************/
/* functions */
/*********************************************************************/

#define BHI160_DEV_ADDR 0x28

/* @brief This function is used to introduce delay in ms.
 *
 * @param[in] ul_dly_ticks :  delay in ticks.
 *
 * @return        NULL
 *
 */
void mdelay(u32 ul_dly_ticks)
{
    coines_delay_msec(ul_dly_ticks);
}

/* @brief This API is used for parsing the activity recognition data from BHY
 *
 * @param[in] sensor_data: bhy data
 * @param[in] sensor_id: bhy id
 *
 * @return  void
 *
 */
void sensors_callback(bhy_data_generic_t * sensor_data,
                      bhy_virtual_sensor_t sensor_id)
{
    float temp;
    u8 index;

    temp = sensor_data->data_vector.x / 8192.;
    outBuffer[3] = temp < 0 ? '-' : ' ';
    temp = temp < 0 ? -temp : temp;
    outBuffer[4] = floorf(temp) + '0';

    for (index = 6; index <= 8; index++)
    {
        temp = (temp - floorf(temp)) * 10;
        outBuffer[index] = floorf(temp) + '0';
    }

    temp = sensor_data->data_vector.y / 8192.;
    outBuffer[13] = temp < 0 ? '-' : ' ';
    temp = temp < 0 ? -temp : temp;
    outBuffer[14] = floorf(temp) + '0';

    for (index = 16; index <= 18; index++)
    {
        temp = (temp - floorf(temp)) * 10;
        outBuffer[index] = floorf(temp) + '0';
    }

    temp = sensor_data->data_vector.z / 8192.;
    outBuffer[23] = temp < 0 ? '-' : ' ';
    temp = temp < 0 ? -temp : temp;
    outBuffer[24] = floorf(temp) + '0';

    for (index = 26; index <= 28; index++)
    {
        temp = (temp - floorf(temp)) * 10;
        outBuffer[index] = floorf(temp) + '0';
    }

    printf("%s", outBuffer);
    fflush(stdout);
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
    u8 array[ARRAYSIZE], *fifoptr;
    u16 bytes_left_in_fifo = 0;
    u16 bytes_remaining, bytes_read;
    bhy_data_generic_t fifo_packet;
    bhy_data_type_t packet_type;
    BHY_RETURN_FUNCTION_TYPE result;

    AppBoard_usb_driver_init();

    printf("uploading RAM patch...\r");
    fflush(stdout);
    /* initializes the BHI160 and loads the RAM patch */
    bhy_driver_init(bhy_firmware_image, sizeof(bhy_firmware_image));

    mdelay(10); // precaution: wait a few ms (i2C might not be ready yet after init / RAM patch upload)
    while (AppBoard_get_int_status(BHI160_DEV_ADDR))
        ;
    while (!AppBoard_get_int_status(BHI160_DEV_ADDR))
        ;

    /* enables the absolute orientation vector and assigns the callback */
    bhy_enable_virtual_sensor(VS_TYPE_GRAVITY, VS_WAKEUP, 25, 0, VS_FLUSH_NONE, 0, 0);
    bhy_install_sensor_callback(VS_TYPE_GRAVITY, VS_WAKEUP, sensors_callback);

    /* continuously read and parse the fifo */
    while (true)
    {
        while (!AppBoard_get_int_status(BHI160_DEV_ADDR) && !bytes_remaining)
            ;

        bhy_read_fifo(array + bytes_left_in_fifo,
        ARRAYSIZE - bytes_left_in_fifo,
                      &bytes_read, &bytes_remaining);

        bytes_read += bytes_left_in_fifo;

        fifoptr = array;
        packet_type = BHY_DATA_TYPE_PADDING;

        do
        {
            /* this function will call callbacks that are registered */
            result = bhy_parse_next_fifo_packet(&fifoptr, &bytes_read,
                                                &fifo_packet,
                                                &packet_type);
            /* the logic here is that if doing a partial parsing of the fifo, then we should not parse      */
            /* the last 18 bytes (max length of a packet) so that we don't try to parse an incomplete       */
            /* packet                                                                                                                                                                       */
        }
        while ((result == BHY_SUCCESS)
               && (bytes_read > (bytes_remaining ? 18 : 0)));
        bytes_left_in_fifo = 0;

        if (result == BHY_SUCCESS)
        {

            if (bytes_remaining)
            {
                /* shifts the remaining bytes to the beginning of the buffer */
                while (bytes_left_in_fifo < bytes_read)
                    array[bytes_left_in_fifo++] = *(fifoptr++);
            }
        }
        else
        {
            /* if there is an error, flush the fifo */
            bhy_set_fifo_flush(0xFF);

        }
    }
}
