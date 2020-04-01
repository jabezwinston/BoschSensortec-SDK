/**\
 * Copyright (C) 2018  Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @File    activity_recognition_example.c
 * @brief 	This demo show cases the BHI160 activity recognition. It records all of the user's activity into the non-wakeup FIFO.
 **/

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
/*! should be greater or equal to 69 bytes, page size (50) + maximum packet size(18) + 1 */
#define ARRAYSIZE 69

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
/*! buffer to hold  the activity recognition values in terms of string*/
char outBuffer[100] =
                      " Time=xxx.xxxs Still: X   Walking: X   Running: X   Bicycle: X   Vehicle: X   Tilting: X \r\n";

/** BHI160 device address */
#define BHI160_DEV_ADDR 0x28

/*********************************************************************/
/* functions */
/*********************************************************************/
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
    /* if there are no changes then it will read X */
    outBuffer[22] = 'X';
    outBuffer[35] = 'X';
    outBuffer[48] = 'X';
    outBuffer[61] = 'X';
    outBuffer[74] = 'X';
    outBuffer[87] = 'X';

    /* '0' means "end of activity and '1' means start of activity */
    if (sensor_data->data_scalar_u16.data & 0b0000000000000001)
        outBuffer[22] = '0';
    if (sensor_data->data_scalar_u16.data & 0b0000000000000010)
        outBuffer[35] = '0';
    if (sensor_data->data_scalar_u16.data & 0b0000000000000100)
        outBuffer[48] = '0';
    if (sensor_data->data_scalar_u16.data & 0b0000000000001000)
        outBuffer[61] = '0';
    if (sensor_data->data_scalar_u16.data & 0b0000000000010000)
        outBuffer[74] = '0';
    if (sensor_data->data_scalar_u16.data & 0b0000000000100000)
        outBuffer[87] = '0';
    if (sensor_data->data_scalar_u16.data & 0b0000000100000000)
        outBuffer[22] = '1';
    if (sensor_data->data_scalar_u16.data & 0b0000001000000000)
        outBuffer[35] = '1';
    if (sensor_data->data_scalar_u16.data & 0b0000010000000000)
        outBuffer[48] = '1';
    if (sensor_data->data_scalar_u16.data & 0b0000100000000000)
        outBuffer[61] = '1';
    if (sensor_data->data_scalar_u16.data & 0b0001000000000000)
        outBuffer[74] = '1';
    if (sensor_data->data_scalar_u16.data & 0b0010000000000000)
        outBuffer[87] = '1';

    printf("%s", outBuffer);
    /* activity recognition is not time critical, so let's wait a little bit */
    mdelay(200);
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
    fflush(stderr);
    /* initializes the BHI160 and loads the RAM patch */
    bhy_driver_init(bhy_firmware_image, sizeof(bhy_firmware_image));

    mdelay(5); // precaution: wait a few ms (i2C might not be ready yet after init / RAM patch upload)
    while (AppBoard_get_int_status(BHI160_DEV_ADDR))
        ;
    while (!AppBoard_get_int_status(BHI160_DEV_ADDR))
        ;

    /* enables the activity recognition and assigns the callback */
    bhy_enable_virtual_sensor(VS_TYPE_ACTIVITY_RECOGNITION, VS_NON_WAKEUP, 1, 0, VS_FLUSH_NONE, 0, 0);

    bhy_install_sensor_callback(VS_TYPE_ACTIVITY_RECOGNITION, VS_NON_WAKEUP, sensors_callback);

    bhy_install_timestamp_callback(VS_NON_WAKEUP, timestamp_callback);

    printf("System is now monitoring activity ... \r\n");
    printf("Here is what has been recorded \r\n");
    fflush(stdout);

    /* continuously read and parse the fifo after the push of the button*/
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

        if (bytes_read)
        {
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
        else
        {
            /* activity recognition is not time critical, so let's wait a little bit */
            mdelay(100);
        }
        fflush(stdout);
    }
}
