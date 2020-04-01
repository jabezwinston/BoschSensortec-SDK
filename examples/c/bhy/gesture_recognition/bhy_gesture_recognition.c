/**\
 * Copyright (C) 2018 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * File     gesture_recognition_example.c
 * @brief    This demo showcases the gesture recognition in the context of a mobile device.
 * 			Every time the Android Gestures "Pickup","Glance" or "Significant motion" are detected, it sends them along with the system timestamp.
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
#define ARRAYSIZE 69	//should be greater or equal to 69 bytes, page size (50) + maximum packet size(18) + 1

/*********************************************************************/
/* function declarations */
/*********************************************************************/
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

/* @brief This function is a callback function for updating the timestamp.
 *
 * @param[in] new_timestamp :  time stamp.
 *
 * @return        NULL
 *
 */

void timestamp_callback(bhy_data_scalar_u16_t *new_timestamp);

/*! Variable to hold the system time stamp */
u32 g_system_timestamp = 0;
/*! buffer to hold  the rotation vector values in terms of string*/
char outBuffer[80] = " Time=xxx.xxxs Gesture: xxxxxxxxxx   \r\n";

#define BHI160_DEV_ADDR 0x28

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
    /* Since a timestamp is always sent before every new data, and that the callbacks	*/
    /* are called while the parsing is done, then the system timestamp is always equal	*/
    /* to the sample timestamp. (in callback mode only)									*/
    temp = g_system_timestamp / 3200000.;

    for (index = 6; index <= 8; index++)
    {
        outBuffer[index] = floorf(temp) + '0';
        temp = (temp - floorf(temp)) * 10;
    }

    for (index = 10; index <= 12; index++)
    {
        outBuffer[index] = floorf(temp) + '0';
        temp = (temp - floorf(temp)) * 10;
    }

    sensor_id &= 0x1F;
    /* gesture recognition sensors are always one-shot, so you need to	*/
    /* re-enable them every time if you want to catch every event		*/
    bhy_enable_virtual_sensor(sensor_id, VS_WAKEUP, 1, 0, VS_FLUSH_NONE, 0, 0);

    switch (sensor_id)
    {
        case VS_TYPE_GLANCE:
            strcpy(&outBuffer[24], "Glance    \r\n");
            break;
        case VS_TYPE_PICKUP:
            strcpy(&outBuffer[24], "Pickup    \r\n");
            break;
        case VS_TYPE_SIGNIFICANT_MOTION:
            strcpy(&outBuffer[24], "Sig motion\r\n");
            break;
        default:
            strcpy(&outBuffer[24], "Unknown   \r\n");
            break;
    }

    printf("%s", outBuffer);
    fflush(stdout);
}

/* @brief This is a timestamp callback function
 *
 * @param[in] new_timestamp : timestamp
 *
 * @return  void
 *
 */
void timestamp_callback(bhy_data_scalar_u16_t *new_timestamp)
{
    /* updates the system timestamp	*/
    bhy_update_system_timestamp(new_timestamp, &g_system_timestamp);

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
    u8 array[ARRAYSIZE], *fifoptr, bytes_left_in_fifo = 0;
    u16 bytes_remaining, bytes_read;
    bhy_data_generic_t fifo_packet;
    bhy_data_type_t packet_type;
    BHY_RETURN_FUNCTION_TYPE result;

    AppBoard_usb_driver_init();

    printf("uploading RAM patch...\r");
    fflush(stdout);

    /* initializes the BHI160 and loads the RAM patch */
    bhy_driver_init(bhy_firmware_image, sizeof(bhy_firmware_image));

    mdelay(5); // precaution: wait a few ms (i2C might not be ready yet after init / RAM patch upload)
    while (AppBoard_get_int_status(BHI160_DEV_ADDR))
        ;
    while (!AppBoard_get_int_status(BHI160_DEV_ADDR))
        ;

    /* enables the gesture recognition and assigns the callback */
    bhy_enable_virtual_sensor(VS_TYPE_GLANCE, VS_WAKEUP, 1, 0, VS_FLUSH_NONE, 0, 0);
    bhy_enable_virtual_sensor(VS_TYPE_PICKUP, VS_WAKEUP, 1, 0, VS_FLUSH_NONE, 0, 0);
    bhy_enable_virtual_sensor(VS_TYPE_SIGNIFICANT_MOTION, VS_WAKEUP, 1, 0, VS_FLUSH_NONE, 0, 0);

    bhy_install_sensor_callback(VS_TYPE_GLANCE, VS_WAKEUP, sensors_callback);
    bhy_install_sensor_callback(VS_TYPE_PICKUP, VS_WAKEUP, sensors_callback);
    bhy_install_sensor_callback(VS_TYPE_SIGNIFICANT_MOTION, VS_WAKEUP, sensors_callback);

    bhy_install_timestamp_callback(VS_WAKEUP, timestamp_callback);

    printf("System is now monitoring glance, pickup and sig motion, move the device! \r\n");
    fflush(stdout);

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
            /* the logic here is that if doing a partial parsing of the fifo, then we should not parse	*/
            /* the last 18 bytes (max length of a packet) so that we don't try to parse an incomplete	*/
            /* packet																					*/
        }
        while ((result == BHY_SUCCESS)
               && (bytes_read > (bytes_remaining ? 18 : 0)));
        bytes_left_in_fifo = 0;

        if (bytes_remaining)
        {
            /* shifts the remaining bytes to the beginning of the buffer */
            while (bytes_left_in_fifo < bytes_read)
                array[bytes_left_in_fifo++] = *(fifoptr++);
        }
    }
}
