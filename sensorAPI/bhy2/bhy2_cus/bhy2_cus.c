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
 * @file        bhy2_cus.c
 *
 * @brief
 *
 *
 */

/*********************************************************************/
/* system header files */
#include <string.h>
#include <stdio.h>


/*********************************************************************/
/* own header files */
#include "typedef.h"
#include "bhy2_hal.h"
#include "bhy_host_interface.h"
#include "bhy2_api.h"
#include "bhy2_cus.h"


/*********************************************************************/
/* local macro definitions */


/*********************************************************************/
/* constant definitions */


/*********************************************************************/
/* global variables */


/*********************************************************************/
/* static variables */
data_gps_t g_gps_data = {0};


/*********************************************************************/
/* static function delarations */


/*********************************************************************/
/* functions */
/*!
 * @brief parse acc corrected frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_acc_corrected(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tf_x = 0;
    float tf_y = 0;
    float tf_z = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ACC].info.scale_factor;
    s16 ts16_x = 0;
    s16 ts16_y = 0;
    s16 ts16_z = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;
    u8  tu8_accuracy = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ACC].info.accuracy;

    if((p_fifo_buffer->read_pos + 7) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    ts16_x = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    ts16_y = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    ts16_z = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 6)] << 8);

    //p_bhy2_sensor->acc_corrected.id = BHY_SENSOR_ID_ACC;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ACC].data.three_axis.x = ts16_x;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ACC].data.three_axis.y = ts16_y;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ACC].data.three_axis.z = ts16_z;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */
    tf_x = (float)ts16_x * tmp_scale_factor;
    tf_y = (float)ts16_y * tmp_scale_factor;
    tf_z = (float)ts16_z * tmp_scale_factor;
    p_fifo_buffer->read_pos += 7;

    TST_PRINT("sensor: acc_corrected timestamp: %13.6fs x: %7.3fg y: %7.3fg z: %7.3fg accuracy: %1d.\r\n",\
                tmp_time_stamp, tf_x, tf_y, tf_z, tu8_accuracy);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup acc corrected frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_acc_corrected_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tf_x = 0;
    float tf_y = 0;
    float tf_z = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ACC_WU].info.scale_factor;
    s16 ts16_x = 0;
    s16 ts16_y = 0;
    s16 ts16_z = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;
    u8  tu8_accuracy = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ACC_WU].info.accuracy;

    if((p_fifo_buffer->read_pos + 7) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    ts16_x = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    ts16_y = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    ts16_z = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 6)] << 8);

    //p_bhy2_sensor->acc_corrected_wkup.id = BHY_SENSOR_ID_ACC_WU;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ACC_WU].data.three_axis.x = ts16_x;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ACC_WU].data.three_axis.y = ts16_y;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ACC_WU].data.three_axis.z = ts16_z;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /* 1 LSB = 1/64000 sec */ /* 1 LSB = 1/64000 sec */
    tf_x = (float)ts16_x * tmp_scale_factor;
    tf_y = (float)ts16_y * tmp_scale_factor;
    tf_z = (float)ts16_z * tmp_scale_factor;
    p_fifo_buffer->read_pos += 7;

    TST_PRINT("sensor: wkup_acc_corrected timestamp: %13.6fs x: %7.3fg y: %7.3fg z: %7.3fg accuracy: %1d.\r\n",\
            tmp_time_stamp, tf_x, tf_y, tf_z, tu8_accuracy);
    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse acc passthrough frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_acc_passthrough(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tf_x = 0;
    float tf_y = 0;
    float tf_z = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ACC_PASS].info.scale_factor;
    s16 ts16_x = 0;
    s16 ts16_y = 0;
    s16 ts16_z = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 7) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    ts16_x = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    ts16_y = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    ts16_z = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 6)] << 8);

    //p_bhy2_sensor->acc_passthrougth.id = BHY_SENSOR_ID_ACC_PASS;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ACC_PASS].data.three_axis.x = ts16_x;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ACC_PASS].data.three_axis.y = ts16_y;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ACC_PASS].data.three_axis.z = ts16_z;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */ /* 1 LSB = 1/64000 sec */
    tf_x = (float)ts16_x * tmp_scale_factor;
    tf_y = (float)ts16_y * tmp_scale_factor;
    tf_z = (float)ts16_z * tmp_scale_factor;
    p_fifo_buffer->read_pos += 7;

    TST_PRINT("sensor: acc_passthrough timestamp: %13.6fs x: %7.3fg y: %7.3fg z: %7.3fg.\r\n",\
            tmp_time_stamp, tf_x, tf_y, tf_z);
    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse acc raw data frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_acc_raw(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tf_x = 0;
    float tf_y = 0;
    float tf_z = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ACC_RAW].info.scale_factor;
    s16 ts16_x = 0;
    s16 ts16_y = 0;
    s16 ts16_z = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 7) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    ts16_x = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    ts16_y = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    ts16_z = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 6)] << 8);

    //p_bhy2_sensor->acc_raw.id = BHY_SENSOR_ID_ACC_RAW;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ACC_RAW].data.three_axis.x = ts16_x;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ACC_RAW].data.three_axis.y = ts16_y;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ACC_RAW].data.three_axis.z = ts16_z;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */ /* 1 LSB = 1/64000 sec */
    tf_x = (float)ts16_x * tmp_scale_factor;
    tf_y = (float)ts16_y * tmp_scale_factor;
    tf_z = (float)ts16_z * tmp_scale_factor;
    p_fifo_buffer->read_pos += 7;

    TST_PRINT("sensor: acc_raw timestamp: %13.6fs x: %7.3fg y: %7.3fg z: %7.3fg.\r\n",\
            tmp_time_stamp, tf_x, tf_y, tf_z);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup acc raw data frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_acc_raw_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tf_x = 0;
    float tf_y = 0;
    float tf_z = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ACC_RAW_WU].info.scale_factor;
    s16 ts16_x = 0;
    s16 ts16_y = 0;
    s16 ts16_z = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 7) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    ts16_x = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    ts16_y = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    ts16_z = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 6)] << 8);

    //p_bhy2_sensor->acc_raw_wkup.id = BHY_SENSOR_ID_ACC_RAW_WU;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ACC_RAW_WU].data.three_axis.x = ts16_x;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ACC_RAW_WU].data.three_axis.y = ts16_y;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ACC_RAW_WU].data.three_axis.z = ts16_z;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /* 1 LSB = 1/64000 sec */ /* 1 LSB = 1/64000 sec */
    tf_x = (float)ts16_x * tmp_scale_factor;
    tf_y = (float)ts16_y * tmp_scale_factor;
    tf_z = (float)ts16_z * tmp_scale_factor;
    p_fifo_buffer->read_pos += 7;

    TST_PRINT("sensor: wkup_acc_raw timestamp: %13.6fs x: %7.3fg y: %7.3fg z: %7.3fg.\r\n", \
            tmp_time_stamp, tf_x, tf_y, tf_z);
    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse acc offset frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_acc_offset(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tf_x = 0;
    float tf_y = 0;
    float tf_z = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ACC_BIAS].info.scale_factor;
    s16 ts16_x = 0;
    s16 ts16_y = 0;
    s16 ts16_z = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 7) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    ts16_x = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    ts16_y = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    ts16_z = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 6)] << 8);

    //p_bhy2_sensor->acc_raw.id = BHY_SENSOR_ID_ACC_RAW;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ACC_BIAS].data.three_axis.x = ts16_x;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ACC_BIAS].data.three_axis.y = ts16_y;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ACC_BIAS].data.three_axis.z = ts16_z;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */ /* 1 LSB = 1/64000 sec */
    tf_x = (float)ts16_x * tmp_scale_factor;
    tf_y = (float)ts16_y * tmp_scale_factor;
    tf_z = (float)ts16_z * tmp_scale_factor;
    p_fifo_buffer->read_pos += 7;

    TST_PRINT("sensor: acc_offset timestamp: %13.6fs x: %7.3fg y: %7.3fg z: %7.3fg.\r\n",\
            tmp_time_stamp, tf_x, tf_y, tf_z);
    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse magnetometer corrected frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_mag_corrected(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tf_x = 0;
    float tf_y = 0;
    float tf_z = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_MAG].info.scale_factor;
    s16 ts16_x = 0;
    s16 ts16_y = 0;
    s16 ts16_z = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;
    u8  tu8_accuracy = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_MAG].info.accuracy;

    if((p_fifo_buffer->read_pos + 7) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    ts16_x = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    ts16_y = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    ts16_z = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 6)] << 8);

    //p_bhy2_sensor->acc_corrected.id = BHY_SENSOR_ID_ACC;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_MAG].data.three_axis.x = ts16_x;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_MAG].data.three_axis.y = ts16_y;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_MAG].data.three_axis.z = ts16_z;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */ /* 1 LSB = 1/64000 sec */
    tf_x = (float)ts16_x * tmp_scale_factor;
    tf_y = (float)ts16_y * tmp_scale_factor;
    tf_z = (float)ts16_z * tmp_scale_factor;
    p_fifo_buffer->read_pos += 7;

    TST_PRINT("sensor: mag_corrected timestamp: %13.6fs x: %7.1fuT y: %7.1fuT z: %7.1fuT accuracy: %1d.\r\n",\
            tmp_time_stamp, tf_x, tf_y, tf_z, tu8_accuracy);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup magnetometer corrected frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_mag_corrected_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tf_x = 0;
    float tf_y = 0;
    float tf_z = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_MAG_WU].info.scale_factor;
    s16 ts16_x = 0;
    s16 ts16_y = 0;
    s16 ts16_z = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;
    u8  tu8_accuracy = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_MAG_WU].info.accuracy;

    if((p_fifo_buffer->read_pos + 7) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    ts16_x = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    ts16_y = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    ts16_z = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 6)] << 8);

    //p_bhy2_sensor->acc_corrected.id = BHY_SENSOR_ID_ACC;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_MAG_WU].data.three_axis.x = ts16_x;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_MAG_WU].data.three_axis.y = ts16_y;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_MAG_WU].data.three_axis.z = ts16_z;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /* 1 LSB = 1/64000 sec */ /* 1 LSB = 1/64000 sec */
    tf_x = (float)ts16_x * tmp_scale_factor;
    tf_y = (float)ts16_y * tmp_scale_factor;
    tf_z = (float)ts16_z * tmp_scale_factor;
    p_fifo_buffer->read_pos += 7;

    TST_PRINT("sensor: wkup_mag_corrected timestamp: %13.6fs x: %7.1fuT y: %7.1fuT z: %7.1fuT accuracy: %1d.\r\n",\
            tmp_time_stamp, tf_x, tf_y, tf_z, tu8_accuracy);
    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse magnetometer passthrough frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_mag_passthrough(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tf_x = 0;
    float tf_y = 0;
    float tf_z = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_MAG_PASS].info.scale_factor;
    s16 ts16_x = 0;
    s16 ts16_y = 0;
    s16 ts16_z = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 7) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    ts16_x = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    ts16_y = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    ts16_z = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 6)] << 8);

    //p_bhy2_sensor->acc_corrected.id = BHY_SENSOR_ID_ACC;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_MAG_PASS].data.three_axis.x = ts16_x;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_MAG_PASS].data.three_axis.y = ts16_y;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_MAG_PASS].data.three_axis.z = ts16_z;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */ /* 1 LSB = 1/64000 sec */
    tf_x = (float)ts16_x * tmp_scale_factor;
    tf_y = (float)ts16_y * tmp_scale_factor;
    tf_z = (float)ts16_z * tmp_scale_factor;
    p_fifo_buffer->read_pos += 7;

    TST_PRINT("sensor: mag_passthrough timestamp: %13.6fs x: %7.1fuT y: %7.1fuT z: %7.1fuT.\r\n",\
            tmp_time_stamp, tf_x, tf_y, tf_z);
    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse magnetometer raw data frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_mag_raw(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tf_x = 0;
    float tf_y = 0;
    float tf_z = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_MAG_RAW].info.scale_factor;
    s16 ts16_x = 0;
    s16 ts16_y = 0;
    s16 ts16_z = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 7) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    ts16_x = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    ts16_y = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    ts16_z = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 6)] << 8);

    //p_bhy2_sensor->acc_corrected.id = BHY_SENSOR_ID_ACC;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_MAG_RAW].data.three_axis.x = ts16_x;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_MAG_RAW].data.three_axis.y = ts16_y;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_MAG_RAW].data.three_axis.z = ts16_z;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */
    tf_x = (float)ts16_x * tmp_scale_factor;
    tf_y = (float)ts16_y * tmp_scale_factor;
    tf_z = (float)ts16_z * tmp_scale_factor;
    p_fifo_buffer->read_pos += 7;

    TST_PRINT("sensor: mag_raw timestamp: %13.6fs x: %7.1fuT y: %7.1fuT z: %7.1fuT.\r\n",\
            tmp_time_stamp, tf_x, tf_y, tf_z);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup magnetometer raw data frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_mag_raw_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tf_x = 0;
    float tf_y = 0;
    float tf_z = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_MAG_RAW_WU].info.scale_factor;
    s16 ts16_x = 0;
    s16 ts16_y = 0;
    s16 ts16_z = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 7) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    ts16_x = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    ts16_y = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    ts16_z = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 6)] << 8);
    //p_bhy2_sensor->acc_corrected.id = BHY_SENSOR_ID_ACC;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_MAG_RAW_WU].data.three_axis.x = ts16_x;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_MAG_RAW_WU].data.three_axis.y = ts16_y;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_MAG_RAW_WU].data.three_axis.z = ts16_z;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /* 1 LSB = 1/64000 sec */
    tf_x = (float)ts16_x * tmp_scale_factor;
    tf_y = (float)ts16_y * tmp_scale_factor;
    tf_z = (float)ts16_z * tmp_scale_factor;
    p_fifo_buffer->read_pos += 7;

    TST_PRINT("sensor: wkup_mag_raw timestamp: %13.6fs x: %7.1fuT y: %7.1fuT z: %7.1fuT.\r\n",\
            tmp_time_stamp, tf_x, tf_y, tf_z);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse magnetometer offset frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_mag_offset(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tf_x = 0;
    float tf_y = 0;
    float tf_z = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_MAG_BIAS].info.scale_factor;
    s16 ts16_x = 0;
    s16 ts16_y = 0;
    s16 ts16_z = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 7) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    ts16_x = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    ts16_y = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    ts16_z = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 6)] << 8);

    //p_bhy2_sensor->acc_corrected.id = BHY_SENSOR_ID_ACC;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_MAG_BIAS].data.three_axis.x = ts16_x;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_MAG_BIAS].data.three_axis.y = ts16_y;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_MAG_BIAS].data.three_axis.z = ts16_z;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */
    tf_x = (float)ts16_x * tmp_scale_factor;
    tf_y = (float)ts16_y * tmp_scale_factor;
    tf_z = (float)ts16_z * tmp_scale_factor;
    p_fifo_buffer->read_pos += 7;

    TST_PRINT("sensor: mag_offset timestamp: %13.6fs x: %7.1fuT y: %7.1fuT z: %7.1fuT.\r\n",\
            tmp_time_stamp, tf_x, tf_y, tf_z);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse orientation frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_orientation(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tmp_heading_f = 0;
    float tmp_pitch_f = 0;
    float tmp_roll_f = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ORI].info.scale_factor;
    s16 tmp_heading = 0;
    s16 tmp_pitch = 0;
    s16 tmp_roll = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;
    u8  tu8_accuracy = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ORI].info.accuracy;

    if((p_fifo_buffer->read_pos + 7) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_heading = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    tmp_pitch = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    tmp_roll = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 6)] << 8);

    //p_bhy2_sensor->acc_corrected.id = BHY_SENSOR_ID_ACC;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ORI].data.orientation.heading = tmp_heading;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ORI].data.orientation.pitch = tmp_pitch;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ORI].data.orientation.roll = tmp_roll;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */
    tmp_heading_f = (float)tmp_heading * tmp_scale_factor;
    tmp_pitch_f = (float)tmp_pitch * tmp_scale_factor;
    tmp_roll_f = (float)tmp_roll * tmp_scale_factor;
    p_fifo_buffer->read_pos += 7;

    TST_PRINT("sensor: orientation timestamp: %13.6fs x: %6.1fdeg y: %6.1fdeg z: %6.1fdeg accuracy: %1d.\r\n",\
            tmp_time_stamp, tmp_heading_f, tmp_pitch_f, tmp_roll_f, tu8_accuracy);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup orientation frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_orientation_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tmp_heading_f = 0;
    float tmp_pitch_f = 0;
    float tmp_roll_f = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ORI_WU].info.scale_factor;
    s16 tmp_heading = 0;
    s16 tmp_pitch = 0;
    s16 tmp_roll = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;
    u8  tu8_accuracy = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ORI_WU].info.accuracy;

    if((p_fifo_buffer->read_pos + 7) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_heading = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    tmp_pitch = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    tmp_roll = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 6)] << 8);

    //p_bhy2_sensor->acc_corrected.id = BHY_SENSOR_ID_ACC;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ORI_WU].data.orientation.heading = tmp_heading;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ORI_WU].data.orientation.pitch = tmp_pitch;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_ORI_WU].data.orientation.roll = tmp_roll;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /* 1 LSB = 1/64000 sec */
    tmp_heading_f = (float)tmp_heading * tmp_scale_factor;
    tmp_pitch_f = (float)tmp_pitch * tmp_scale_factor;
    tmp_roll_f = (float)tmp_roll * tmp_scale_factor;
    p_fifo_buffer->read_pos += 7;

    TST_PRINT("sensor: wkup_orientation timestamp: %13.6fs x: %6.1fdeg y: %6.1fdeg z: %6.1fdeg accuracy: %1d.\r\n",\
            tmp_time_stamp, tmp_heading_f, tmp_pitch_f, tmp_roll_f, tu8_accuracy);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse gyroscope corrected frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_gyro_corrected(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tf_time_stamp = 0;
    float tf_x = 0;
    float tf_y = 0;
    float tf_z = 0;
    float tf_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GYRO].info.scale_factor;
    s16 ts16_x = 0;
    s16 ts16_y = 0;
    s16 ts16_z = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;
    u8  tu8_accuracy = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GYRO].info.accuracy;

    if((p_fifo_buffer->read_pos + 7) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    ts16_x = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    ts16_y = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    ts16_z = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 6)] << 8);

    //p_bhy2_sensor->acc_corrected.id = BHY_SENSOR_ID_ACC;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GYRO].data.three_axis.x = ts16_x;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GYRO].data.three_axis.y = ts16_y;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GYRO].data.three_axis.z = ts16_z;
    tf_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */
    tf_x = (float)ts16_x * tf_scale_factor;
    tf_y = (float)ts16_y * tf_scale_factor;
    tf_z = (float)ts16_z * tf_scale_factor;
    p_fifo_buffer->read_pos += 7;

    TST_PRINT("sensor: gyro_corrected timestamp: %13.6fs x: %6.1fdeg y: %6.1fdeg z: %6.1fdeg accuracy: %1d.\r\n",\
            tf_time_stamp, tf_x, tf_y, tf_z, tu8_accuracy);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup gyroscope corrected frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_gyro_corrected_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tf_x = 0;
    float tf_y = 0;
    float tf_z = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GYRO_WU].info.scale_factor;
    s16 ts16_x = 0;
    s16 ts16_y = 0;
    s16 ts16_z = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;
    u8  tu8_accuracy = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GYRO_WU].info.accuracy;

    if((p_fifo_buffer->read_pos + 7) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    ts16_x = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    ts16_y = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    ts16_z = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 6)] << 8);

    //p_bhy2_sensor->acc_corrected.id = BHY_SENSOR_ID_ACC;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GYRO_WU].data.three_axis.x = ts16_x;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GYRO_WU].data.three_axis.y = ts16_y;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GYRO_WU].data.three_axis.z = ts16_z;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /* 1 LSB = 1/64000 sec */
    tf_x = (float)ts16_x * tmp_scale_factor;
    tf_y = (float)ts16_y * tmp_scale_factor;
    tf_z = (float)ts16_z * tmp_scale_factor;
    p_fifo_buffer->read_pos += 7;

    TST_PRINT("sensor: wkup_gyro_corrected timestamp: %13.6fs x: %6.1fdeg y: %6.1fdeg z: %6.1fdeg accuracy: %1d.\r\n",\
            tmp_time_stamp, tf_x, tf_y, tf_z, tu8_accuracy);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse gyroscope passthrough frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_gyro_passthrough(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tf_x = 0;
    float tf_y = 0;
    float tf_z = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GYRO_PASS].info.scale_factor;
    s16 ts16_x = 0;
    s16 ts16_y = 0;
    s16 ts16_z = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 7) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    ts16_x = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    ts16_y = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    ts16_z = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 6)] << 8);

    //p_bhy2_sensor->acc_corrected.id = BHY_SENSOR_ID_ACC;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GYRO_PASS].data.three_axis.x = ts16_x;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GYRO_PASS].data.three_axis.y = ts16_y;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GYRO_PASS].data.three_axis.z = ts16_z;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */
    tf_x = (float)ts16_x * tmp_scale_factor;
    tf_y = (float)ts16_y * tmp_scale_factor;
    tf_z = (float)ts16_z * tmp_scale_factor;
    p_fifo_buffer->read_pos += 7;

    TST_PRINT("sensor: gyro_passthrough timestamp: %13.6fs x: %6.1fdeg y: %6.1fdeg z: %6.1fdeg.\r\n",\
            tmp_time_stamp, tf_x, tf_y, tf_z);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse gyroscope raw data frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_gyro_raw(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tf_x = 0;
    float tf_y = 0;
    float tf_z = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GYRO_RAW].info.scale_factor;
    s16 ts16_x = 0;
    s16 ts16_y = 0;
    s16 ts16_z = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 7) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    ts16_x = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    ts16_y = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    ts16_z = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 6)] << 8);

    //p_bhy2_sensor->acc_corrected.id = BHY_SENSOR_ID_ACC;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GYRO_RAW].data.three_axis.x = ts16_x;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GYRO_RAW].data.three_axis.y = ts16_y;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GYRO_RAW].data.three_axis.z = ts16_z;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */
    tf_x = (float)ts16_x * tmp_scale_factor;
    tf_y = (float)ts16_y * tmp_scale_factor;
    tf_z = (float)ts16_z * tmp_scale_factor;
    p_fifo_buffer->read_pos += 7;

    TST_PRINT("sensor: gyro_raw timestamp: %13.6fs x: %6.1fdeg y: %6.1fdeg z: %6.1fdeg.\r\n",\
            tmp_time_stamp, tf_x, tf_y, tf_z);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup gyroscope raw data frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_gyro_raw_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tf_x = 0;
    float tf_y = 0;
    float tf_z = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GYRO_RAW_WU].info.scale_factor;
    s16 ts16_x = 0;
    s16 ts16_y = 0;
    s16 ts16_z = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 7) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    ts16_x = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    ts16_y = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    ts16_z = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 6)] << 8);

    //p_bhy2_sensor->acc_corrected.id = BHY_SENSOR_ID_ACC;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GYRO_RAW_WU].data.three_axis.x = ts16_x;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GYRO_RAW_WU].data.three_axis.y = ts16_y;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GYRO_RAW_WU].data.three_axis.z = ts16_z;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /* 1 LSB = 1/64000 sec */
    tf_x = (float)ts16_x * tmp_scale_factor;
    tf_y = (float)ts16_y * tmp_scale_factor;
    tf_z = (float)ts16_z * tmp_scale_factor;
    p_fifo_buffer->read_pos += 7;

    TST_PRINT("sensor: wkup_gyro_raw timestamp: %13.6fs x: %6.1fdeg y: %6.1fdeg z: %6.1fdeg.\r\n",\
            tmp_time_stamp, tf_x, tf_y, tf_z);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse gyroscope offset frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_gyro_offset(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tf_x = 0;
    float tf_y = 0;
    float tf_z = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GYRO_BIAS].info.scale_factor;
    s16 ts16_x = 0;
    s16 ts16_y = 0;
    s16 ts16_z = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 7) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    ts16_x = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    ts16_y = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    ts16_z = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 6)] << 8);

    //p_bhy2_sensor->acc_corrected.id = BHY_SENSOR_ID_ACC;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GYRO_BIAS].data.three_axis.x = ts16_x;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GYRO_BIAS].data.three_axis.y = ts16_y;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GYRO_BIAS].data.three_axis.z = ts16_z;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */
    tf_x = (float)ts16_x * tmp_scale_factor;
    tf_y = (float)ts16_y * tmp_scale_factor;
    tf_z = (float)ts16_z * tmp_scale_factor;
    p_fifo_buffer->read_pos += 7;

    TST_PRINT("sensor: gyro_offset timestamp: %13.6fs x: %6.1fdeg y: %6.1fdeg z: %6.1fdeg.\r\n",\
            tmp_time_stamp, tf_x, tf_y, tf_z);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse light frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_light(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tmp_temp_f = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_LIGHT].info.scale_factor;
    u16 tmp_temp = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 3) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_temp = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);

    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_LIGHT].data.scalar_u16.data_u16 = tmp_temp;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */
    tmp_temp_f = (float)tmp_temp * tmp_scale_factor;
    p_fifo_buffer->read_pos += 3;

    TST_PRINT("sensor: light timestamp: %13.6fs L: %7.1fLux %d.\r\n", tmp_time_stamp, tmp_temp_f, tmp_temp);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup light frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_light_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tmp_temp_f = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_LIGHT_WU].info.scale_factor;
    u16 tmp_temp = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 3) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_temp = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);

    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_LIGHT_WU].data.scalar_u16.data_u16 = tmp_temp;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /* 1 LSB = 1/64000 sec */
    tmp_temp_f = (float)tmp_temp * tmp_scale_factor;
    p_fifo_buffer->read_pos += 3;

    TST_PRINT("sensor: wkup_light timestamp: %13.6fs L: %7.1fLux.\r\n", tmp_time_stamp, tmp_temp_f);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse proximity frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_proximity(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{

    float tmp_time_stamp = 0;
    u8 tmp_temp = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 2) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_temp = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)];

    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_PROX].data.scalar_u8.data_u8 = tmp_temp;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */
    p_fifo_buffer->read_pos += 2;

    TST_PRINT("sensor: proximity timestamp: %13.6fs V: %1d.\r\n", tmp_time_stamp, tmp_temp);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup proximity frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_proximity_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{

    float tmp_time_stamp = 0;
    u8 tmp_temp = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 2) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_temp = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)];

    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_PROX_WU].data.scalar_u8.data_u8 = tmp_temp;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /* 1 LSB = 1/64000 sec */
    p_fifo_buffer->read_pos += 2;

    TST_PRINT("sensor: wkup_proximity timestamp: %13.6fs V: %1d.\r\n", tmp_time_stamp, tmp_temp);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse humidity frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_humidity(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{

    float tmp_time_stamp = 0;
    u8 tmp_temp = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 2) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_temp = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)];

    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_HUM].data.scalar_u8.data_u8 = tmp_temp;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */
    p_fifo_buffer->read_pos += 2;

    TST_PRINT("sensor: humidity timestamp: %13.6fs RH: %3d%%.\r\n", tmp_time_stamp, tmp_temp);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup humidity frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_humidity_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{

    float tmp_time_stamp = 0;
    u8 tmp_temp = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 2) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_temp = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)];

    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_HUM_WU].data.scalar_u8.data_u8 = tmp_temp;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /* 1 LSB = 1/64000 sec */
    p_fifo_buffer->read_pos += 2;

    TST_PRINT("sensor: wkup_humidity timestamp: %13.6fs RH: %3d%%.\r\n", tmp_time_stamp, tmp_temp);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse temperature frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_temperature(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tmp_temp_f = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_TEMP].info.scale_factor;
    s16 tmp_temp = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 3) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_temp = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);

    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_TEMP].data.scalar_s16.data_s16 = tmp_temp;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */
    tmp_temp_f = (float)tmp_temp * tmp_scale_factor;
    p_fifo_buffer->read_pos += 3;

    TST_PRINT("sensor: temperature timestamp: %13.6fs T: %6.1f degC.\r\n", tmp_time_stamp, tmp_temp_f);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup temperature frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_temperature_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tmp_temp_f = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_TEMP_WU].info.scale_factor;
    s16 tmp_temp = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 3) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_temp = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);

    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_TEMP_WU].data.scalar_s16.data_s16 = tmp_temp;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /* 1 LSB = 1/64000 sec */
    tmp_temp_f = (float)tmp_temp * tmp_scale_factor;
    p_fifo_buffer->read_pos += 3;

    TST_PRINT("sensor: wkup_temperature timestamp: %13.6fs T: %6.1f degC.\r\n", tmp_time_stamp, tmp_temp_f);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse barometer frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_barometer(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tmp_temp_f = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_BARO].info.scale_factor;
    u32 tmp_temp = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 4) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_temp = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((u32)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8)
                | ((u32)p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] << 16);

    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_BARO].data.scalar_u32.data_u32 = tmp_temp;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */
    tmp_temp_f = (float)tmp_temp * tmp_scale_factor;
    p_fifo_buffer->read_pos += 4;

    TST_PRINT("sensor: barometer timestamp: %13.6fs P: %6.1f hPa.\r\n", tmp_time_stamp, tmp_temp_f);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup barometer frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_barometer_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tmp_temp_f = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_BARO_WU].info.scale_factor;
    u32 tmp_temp = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 4) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_temp = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((u32)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8)
                | ((u32)p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] << 16);

    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_BARO_WU].data.scalar_u32.data_u32 = tmp_temp;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /* 1 LSB = 1/64000 sec */
    tmp_temp_f = (float)tmp_temp * tmp_scale_factor;
    p_fifo_buffer->read_pos += 4;

    TST_PRINT("sensor: wkup_barometer timestamp: %13.6fs P: %6.1f hPa.\r\n", tmp_time_stamp, tmp_temp_f);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse gas frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_gas(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    u32 tmp_temp = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 5) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_temp = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((u32)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8)
                | ((u32)p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] << 16) | ((u32)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 24);

    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GAS].data.scalar_u32.data_u32 = tmp_temp;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */
    p_fifo_buffer->read_pos += 5;

    TST_PRINT("sensor: gas timestamp: %13.6fs Gas: %7d Ohms\r\n", tmp_time_stamp, tmp_temp);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup gas frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_gas_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    u32 tmp_temp = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 5) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_temp = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((u32)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8)
                | ((u32)p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] << 16) | ((u32)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 24);

    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GAS_WU].data.scalar_u32.data_u32 = tmp_temp;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /* 1 LSB = 1/64000 sec */
    p_fifo_buffer->read_pos += 5;

    TST_PRINT("sensor: wkup_gas timestamp: %13.6fs Gas: %7d Ohms\r\n", tmp_time_stamp, tmp_temp);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse gravity frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_gravity(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tf_x = 0;
    float tf_y = 0;
    float tf_z = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GRA].info.scale_factor;
    s16 ts16_x = 0;
    s16 ts16_y = 0;
    s16 ts16_z = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 7) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    ts16_x = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    ts16_y = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    ts16_z = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 6)] << 8);

    //p_bhy2_sensor->acc_corrected.id = BHY_SENSOR_ID_ACC;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GRA].data.three_axis.x = ts16_x;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GRA].data.three_axis.y = ts16_y;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GRA].data.three_axis.z = ts16_z;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */
    tf_x = (float)ts16_x * tmp_scale_factor;
    tf_y = (float)ts16_y * tmp_scale_factor;
    tf_z = (float)ts16_z * tmp_scale_factor;
    p_fifo_buffer->read_pos += 7;

    TST_PRINT("sensor: gravity timestamp: %13.6fs x: %7.3fg y: %7.3fg z: %7.3fg.\r\n",
                tmp_time_stamp, tf_x, tf_y, tf_z);
    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup gravity frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_gravity_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tf_x = 0;
    float tf_y = 0;
    float tf_z = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GRA_WU].info.scale_factor;
    s16 ts16_x = 0;
    s16 ts16_y = 0;
    s16 ts16_z = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 7) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    ts16_x = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    ts16_y = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    ts16_z = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 6)] << 8);

    //p_bhy2_sensor->acc_corrected.id = BHY_SENSOR_ID_ACC;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GRA_WU].data.three_axis.x = ts16_x;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GRA_WU].data.three_axis.y = ts16_y;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GRA_WU].data.three_axis.z = ts16_z;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /* 1 LSB = 1/64000 sec */
    tf_x = (float)ts16_x * tmp_scale_factor;
    tf_y = (float)ts16_y * tmp_scale_factor;
    tf_z = (float)ts16_z * tmp_scale_factor;
    p_fifo_buffer->read_pos += 7;

    TST_PRINT("sensor: wkup_gravity timestamp: %13.6fs x: %7.3fg y: %7.3fg z: %7.3fg.\r\n",
                tmp_time_stamp, tf_x, tf_y, tf_z);
    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse linear acc frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */

s16 bhy2_parse_frame_linear_acc(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tf_x = 0;
    float tf_y = 0;
    float tf_z = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_LACC].info.scale_factor;
    s16 ts16_x = 0;
    s16 ts16_y = 0;
    s16 ts16_z = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 7) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    ts16_x = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    ts16_y = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    ts16_z = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 6)] << 8);

    //p_bhy2_sensor->acc_corrected.id = BHY_SENSOR_ID_ACC;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_LACC].data.three_axis.x = ts16_x;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_LACC].data.three_axis.y = ts16_y;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_LACC].data.three_axis.z = ts16_z;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */
    tf_x = (float)ts16_x * tmp_scale_factor;
    tf_y = (float)ts16_y * tmp_scale_factor;
    tf_z = (float)ts16_z * tmp_scale_factor;
    p_fifo_buffer->read_pos += 7;

    TST_PRINT("sensor: linear_acc timestamp: %13.6fs x: %7.3fg y: %7.3fg z: %7.3fg.\r\n",
                tmp_time_stamp, tf_x, tf_y, tf_z);
    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup linear acc frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_linear_acc_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tf_x = 0;
    float tf_y = 0;
    float tf_z = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_LACC_WU].info.scale_factor;
    s16 ts16_x = 0;
    s16 ts16_y = 0;
    s16 ts16_z = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 7) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    ts16_x = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    ts16_y = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    ts16_z = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 6)] << 8);

    //p_bhy2_sensor->acc_corrected.id = BHY_SENSOR_ID_ACC;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_LACC_WU].data.three_axis.x = ts16_x;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_LACC_WU].data.three_axis.y = ts16_y;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_LACC_WU].data.three_axis.z = ts16_z;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /* 1 LSB = 1/64000 sec */
    tf_x = (float)ts16_x * tmp_scale_factor;
    tf_y = (float)ts16_y * tmp_scale_factor;
    tf_z = (float)ts16_z * tmp_scale_factor;
    p_fifo_buffer->read_pos += 7;

    TST_PRINT("sensor: wkup_linear_acc timestamp: %13.6fs x: %7.3fg y: %7.3fg z: %7.3fg.\r\n",
                tmp_time_stamp, tf_x, tf_y, tf_z);
    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse rotation frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_rotation(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tf_x = 0;
    float tf_y = 0;
    float tf_z = 0;
    float tmp_w_f = 0;
    float tmp_accuracy_f = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_RV].info.scale_factor;
    s16 ts16_x = 0;
    s16 ts16_y = 0;
    s16 ts16_z = 0;
    s16 tmp_w = 0;
    s16 tmp_accuracy = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 11) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    ts16_x = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    ts16_y = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    ts16_z = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 6)] << 8);
    tmp_w = p_fifo_buffer->p_buffer[(ts32_read_pos + 7)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 8)] << 8);
    tmp_accuracy = p_fifo_buffer->p_buffer[(ts32_read_pos + 9)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 10)] << 8);
    //p_bhy2_sensor->acc_corrected.id = BHY_SENSOR_ID_ACC;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_RV].data.quaternion.x = ts16_x;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_RV].data.quaternion.y = ts16_y;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_RV].data.quaternion.z = ts16_z;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_RV].data.quaternion.w = tmp_w;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_RV].data.quaternion.accuracy = tmp_accuracy;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */
    tf_x = (float)ts16_x * tmp_scale_factor;
    tf_y = (float)ts16_y * tmp_scale_factor;
    tf_z = (float)ts16_z * tmp_scale_factor;
    tmp_w_f = (float)tmp_w * tmp_scale_factor;
    tmp_accuracy_f = (float)tmp_accuracy * tmp_scale_factor;
    p_fifo_buffer->read_pos += 11;

    TST_PRINT("sensor: rotation timestamp: %13.6fs x: %6.3f y: %6.3f z: %6.3f w: %6.3f accuracy: %6.3f.\r\n",
                tmp_time_stamp, tf_x, tf_y, tf_z, tmp_w_f, tmp_accuracy_f);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup rotation frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_rotation_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tf_x = 0;
    float tf_y = 0;
    float tf_z = 0;
    float tmp_w_f = 0;
    float tmp_accuracy_f = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_RV_WU].info.scale_factor;
    s16 ts16_x = 0;
    s16 ts16_y = 0;
    s16 ts16_z = 0;
    s16 tmp_w = 0;
    s16 tmp_accuracy = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 11) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    ts16_x = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    ts16_y = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    ts16_z = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 6)] << 8);
    tmp_w = p_fifo_buffer->p_buffer[(ts32_read_pos + 7)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 8)] << 8);
    tmp_accuracy = p_fifo_buffer->p_buffer[(ts32_read_pos + 9)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 10)] << 8);
    //p_bhy2_sensor->acc_corrected.id = BHY_SENSOR_ID_ACC;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_RV_WU].data.quaternion.x = ts16_x;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_RV_WU].data.quaternion.y = ts16_y;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_RV_WU].data.quaternion.z = ts16_z;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_RV_WU].data.quaternion.w = tmp_w;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_RV_WU].data.quaternion.accuracy = tmp_accuracy;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /* 1 LSB = 1/64000 sec */
    tf_x = (float)ts16_x * tmp_scale_factor;
    tf_y = (float)ts16_y * tmp_scale_factor;
    tf_z = (float)ts16_z * tmp_scale_factor;
    tmp_w_f = (float)tmp_w * tmp_scale_factor;
    tmp_accuracy_f = (float)tmp_accuracy * tmp_scale_factor;
    p_fifo_buffer->read_pos += 11;

    TST_PRINT("sensor: wkup_rotation timestamp: %13.6fs x: %6.3f y: %6.3f z: %6.3f w: %6.3f accuracy: %6.3f.\r\n",
                tmp_time_stamp, tf_x, tf_y, tf_z, tmp_w_f, tmp_accuracy_f);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse game rotation frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_game_rotation(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tf_x = 0;
    float tf_y = 0;
    float tf_z = 0;
    float tmp_w_f = 0;
    float tmp_accuracy_f = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GAMERV].info.scale_factor;
    s16 ts16_x = 0;
    s16 ts16_y = 0;
    s16 ts16_z = 0;
    s16 tmp_w = 0;
    s16 tmp_accuracy = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 11) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    ts16_x = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    ts16_y = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    ts16_z = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 6)] << 8);
    tmp_w = p_fifo_buffer->p_buffer[(ts32_read_pos + 7)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 8)] << 8);
    tmp_accuracy = p_fifo_buffer->p_buffer[(ts32_read_pos + 9)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 10)] << 8);
    //p_bhy2_sensor->acc_corrected.id = BHY_SENSOR_ID_ACC;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GAMERV].data.quaternion.x = ts16_x;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GAMERV].data.quaternion.y = ts16_y;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GAMERV].data.quaternion.z = ts16_z;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GAMERV].data.quaternion.w = tmp_w;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GAMERV].data.quaternion.accuracy = tmp_accuracy;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */
    tf_x = (float)ts16_x * tmp_scale_factor;
    tf_y = (float)ts16_y * tmp_scale_factor;
    tf_z = (float)ts16_z * tmp_scale_factor;
    tmp_w_f = (float)tmp_w * tmp_scale_factor;
    tmp_accuracy_f = (float)tmp_accuracy * tmp_scale_factor;
    p_fifo_buffer->read_pos += 11;

    TST_PRINT("sensor: game_rotation timestamp: %13.6fs x: %6.3f y: %6.3f z: %6.3f w: %6.3f accuracy: %6.3f.\r\n",
                tmp_time_stamp, tf_x, tf_y, tf_z, tmp_w_f, tmp_accuracy_f);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup game rotation frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_game_rotation_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tf_x = 0;
    float tf_y = 0;
    float tf_z = 0;
    float tmp_w_f = 0;
    float tmp_accuracy_f = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GAMERV_WU].info.scale_factor;
    s16 ts16_x = 0;
    s16 ts16_y = 0;
    s16 ts16_z = 0;
    s16 tmp_w = 0;
    s16 tmp_accuracy = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 11) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    ts16_x = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    ts16_y = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    ts16_z = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 6)] << 8);
    tmp_w = p_fifo_buffer->p_buffer[(ts32_read_pos + 7)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 8)] << 8);
    tmp_accuracy = p_fifo_buffer->p_buffer[(ts32_read_pos + 9)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 10)] << 8);
    //p_bhy2_sensor->acc_corrected.id = BHY_SENSOR_ID_ACC;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GAMERV_WU].data.quaternion.x = ts16_x;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GAMERV_WU].data.quaternion.y = ts16_y;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GAMERV_WU].data.quaternion.z = ts16_z;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GAMERV_WU].data.quaternion.w = tmp_w;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GAMERV_WU].data.quaternion.accuracy = tmp_accuracy;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /* 1 LSB = 1/64000 sec */
    tf_x = (float)ts16_x * tmp_scale_factor;
    tf_y = (float)ts16_y * tmp_scale_factor;
    tf_z = (float)ts16_z * tmp_scale_factor;
    tmp_w_f = (float)tmp_w * tmp_scale_factor;
    tmp_accuracy_f = (float)tmp_accuracy * tmp_scale_factor;
    p_fifo_buffer->read_pos += 11;

    TST_PRINT("sensor: wkup_game_rotation timestamp: %13.6fs x: %6.3f y: %6.3f z: %6.3f w: %6.3f accuracy: %6.3f.\r\n",
                tmp_time_stamp, tf_x, tf_y, tf_z, tmp_w_f, tmp_accuracy_f);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse geomagnetic rotation frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_geo_rotation(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tf_x = 0;
    float tf_y = 0;
    float tf_z = 0;
    float tmp_w_f = 0;
    float tmp_accuracy_f = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GEORV].info.scale_factor;
    s16 ts16_x = 0;
    s16 ts16_y = 0;
    s16 ts16_z = 0;
    s16 tmp_w = 0;
    s16 tmp_accuracy = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 11) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    ts16_x = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    ts16_y = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    ts16_z = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 6)] << 8);
    tmp_w = p_fifo_buffer->p_buffer[(ts32_read_pos + 7)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 8)] << 8);
    tmp_accuracy = p_fifo_buffer->p_buffer[(ts32_read_pos + 9)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 10)] << 8);
    //p_bhy2_sensor->acc_corrected.id = BHY_SENSOR_ID_ACC;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GEORV].data.quaternion.x = ts16_x;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GEORV].data.quaternion.y = ts16_y;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GEORV].data.quaternion.z = ts16_z;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GEORV].data.quaternion.w = tmp_w;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GEORV].data.quaternion.accuracy = tmp_accuracy;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */
    tf_x = (float)ts16_x * tmp_scale_factor;
    tf_y = (float)ts16_y * tmp_scale_factor;
    tf_z = (float)ts16_z * tmp_scale_factor;
    tmp_w_f = (float)tmp_w * tmp_scale_factor;
    tmp_accuracy_f = (float)tmp_accuracy * tmp_scale_factor;
    p_fifo_buffer->read_pos += 11;

    TST_PRINT("sensor: geo_rotation timestamp: %13.6fs x: %6.3f y: %6.3f z: %6.3f w: %6.3f accuracy: %6.3f.\r\n",
                tmp_time_stamp, tf_x, tf_y, tf_z, tmp_w_f, tmp_accuracy_f);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup geomagnetic rotation frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_geo_rotation_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    float tf_x = 0;
    float tf_y = 0;
    float tf_z = 0;
    float tmp_w_f = 0;
    float tmp_accuracy_f = 0;
    float tmp_scale_factor = p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GEORV_WU].info.scale_factor;
    s16 ts16_x = 0;
    s16 ts16_y = 0;
    s16 ts16_z = 0;
    s16 tmp_w = 0;
    s16 tmp_accuracy = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 11) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    ts16_x = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    ts16_y = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    ts16_z = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 6)] << 8);
    tmp_w = p_fifo_buffer->p_buffer[(ts32_read_pos + 7)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 8)] << 8);
    tmp_accuracy = p_fifo_buffer->p_buffer[(ts32_read_pos + 9)] | ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 10)] << 8);
    //p_bhy2_sensor->acc_corrected.id = BHY_SENSOR_ID_ACC;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GEORV_WU].data.quaternion.x = ts16_x;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GEORV_WU].data.quaternion.y = ts16_y;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GEORV_WU].data.quaternion.z = ts16_z;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GEORV_WU].data.quaternion.w = tmp_w;
    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_GEORV_WU].data.quaternion.accuracy = tmp_accuracy;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /* 1 LSB = 1/64000 sec */
    tf_x = (float)ts16_x * tmp_scale_factor;
    tf_y = (float)ts16_y * tmp_scale_factor;
    tf_z = (float)ts16_z * tmp_scale_factor;
    tmp_w_f = (float)tmp_w * tmp_scale_factor;
    tmp_accuracy_f = (float)tmp_accuracy * tmp_scale_factor;
    p_fifo_buffer->read_pos += 11;

    TST_PRINT("sensor: wkup_geo_rotation timestamp: %13.6fs x: %6.3f y: %6.3f z: %6.3f w: %6.3f accuracy: %6.3f.\r\n",
                tmp_time_stamp, tf_x, tf_y, tf_z, tmp_w_f, tmp_accuracy_f);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup significant motion frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_sig_motion_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;

    if((p_fifo_buffer->read_pos + 1) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /* 1 LSB = 1/64000 sec */
    p_fifo_buffer->read_pos += 1;

    TST_PRINT("sensor: wkup_sig_motion timestamp: %13.6fs.\r\n", tmp_time_stamp);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse step detect frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_step_detect(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;

    if((p_fifo_buffer->read_pos + 1) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */
    p_fifo_buffer->read_pos += 1;

    TST_PRINT("sensor: step_detect timestamp: %13.6fs.\r\n", tmp_time_stamp);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup step detect frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_step_detect_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;

    if((p_fifo_buffer->read_pos + 1) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /* 1 LSB = 1/64000 sec */
    p_fifo_buffer->read_pos += 1;

    TST_PRINT("sensor: wkup_step_detect timestamp: %13.6fs.\r\n", tmp_time_stamp);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup tilt detect frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_tilt_detect_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;

    if((p_fifo_buffer->read_pos + 1) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /* 1 LSB = 1/64000 sec */
    p_fifo_buffer->read_pos += 1;

    TST_PRINT("sensor: wkup_tilt_detect timestamp: %13.6fs.\r\n", tmp_time_stamp);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup wake gesture frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_wake_gesture_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;

    if((p_fifo_buffer->read_pos + 1) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /* 1 LSB = 1/64000 sec */
    p_fifo_buffer->read_pos += 1;

    TST_PRINT("sensor: wkup_wake_gesture timestamp: %13.6fs.\r\n", tmp_time_stamp);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup glance gesture frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_glance_gesture_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;

    if((p_fifo_buffer->read_pos + 1) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /* 1 LSB = 1/64000 sec */
    p_fifo_buffer->read_pos += 1;

    TST_PRINT("sensor: wkup_glance_gesture timestamp: %13.6fs.\r\n", tmp_time_stamp);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup pickup gesture frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_pickup_gesture_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;

    if((p_fifo_buffer->read_pos + 1) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /* 1 LSB = 1/64000 sec */
    p_fifo_buffer->read_pos += 1;

    TST_PRINT("sensor: wkup_pickup_gesture timestamp: %13.6fs.\r\n", tmp_time_stamp);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup hardware significant motion frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_sig_motion_hw_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;

    if((p_fifo_buffer->read_pos + 1) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /* 1 LSB = 1/64000 sec */
    p_fifo_buffer->read_pos += 1;

    TST_PRINT("sensor: wkup_sig_motion_hw timestamp: %13.6fs.\r\n", tmp_time_stamp);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse hardware step detect frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_step_detect_hw(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;

    if((p_fifo_buffer->read_pos + 1) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */
    p_fifo_buffer->read_pos += 1;

    TST_PRINT("sensor: step_detect_hw timestamp: %13.6fs.\r\n", tmp_time_stamp);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup hardware step detect frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_step_detect_hw_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;

    if((p_fifo_buffer->read_pos + 1) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /* 1 LSB = 1/64000 sec */
    p_fifo_buffer->read_pos += 1;

    TST_PRINT("sensor: wkup_step_detect_hw timestamp: %13.6fs.\r\n", tmp_time_stamp);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse hardware any motion frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_any_motion_hw(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;

    if((p_fifo_buffer->read_pos + 1) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */
    p_fifo_buffer->read_pos += 1;

    TST_PRINT("sensor: any_motion_hw timestamp: %13.6fs.\r\n", tmp_time_stamp);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup hardware any motion frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_any_motion_hw_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;

    if((p_fifo_buffer->read_pos + 1) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /* 1 LSB = 1/64000 sec */
    p_fifo_buffer->read_pos += 1;

    TST_PRINT("sensor: wkup_any_motion_hw timestamp: %13.6fs.\r\n", tmp_time_stamp);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse step counter frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_step_counter(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    u32 tmp_steps = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 5) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_steps = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)]
                | ((u32)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8)
                | ((u32)p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] << 16)
                | ((u32)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 24);

    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_STC].data.scalar_u32.data_u32 = tmp_steps;

    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */
    p_fifo_buffer->read_pos += 5;

    TST_PRINT("sensor: step_counter timestamp: %13.6fs steps: %6d.\r\n", tmp_time_stamp, tmp_steps);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup step counter frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_step_counter_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    u32 tmp_steps = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 5) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_steps = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)]
                | ((u32)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8)
                | ((u32)p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] << 16)
                | ((u32)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 24);

    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_STC_WU].data.scalar_u32.data_u32 = tmp_steps;

    tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /* 1 LSB = 1/64000 sec */
    p_fifo_buffer->read_pos += 5;

    TST_PRINT("sensor: wkup_step_counter timestamp: %13.6fs steps: %6d.\r\n", tmp_time_stamp, tmp_steps);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse hardware step counter frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_step_counter_hw(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    u32 tmp_steps = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 5) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_steps = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)]
                | ((u32)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8)
                | ((u32)p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] << 16)
                | ((u32)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 24);

    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_STC_HW].data.scalar_u32.data_u32 = tmp_steps;
    p_fifo_buffer->read_pos += 5;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */

    TST_PRINT("sensor: step_counter_hw timestamp: %13.6fs steps: %6d.\r\n", tmp_time_stamp, tmp_steps);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse activity frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_step_counter_hw_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    u32 tmp_steps = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 5) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_steps = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)]
                | ((u32)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8)
                | ((u32)p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] << 16)
                | ((u32)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 24);

    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_STC_HW_WU].data.scalar_u32.data_u32 = tmp_steps;
    p_fifo_buffer->read_pos += 5;
    tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /* 1 LSB = 1/64000 sec */

    TST_PRINT("sensor: wkup_step_counter_hw timestamp: %13.6fs steps: %6d.\r\n", tmp_time_stamp, tmp_steps);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup activity frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_activity_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    u16 tmp_activity = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 3) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_activity = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((u16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);

    p_bhy2_hub->virt_sensor[BHY_SENSOR_ID_AR_WU].data.scalar_u16.data_u16 = tmp_activity;

    tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /* 1 LSB = 1/64000 sec */

    TST_PRINT("sensor: wkup_activity timestamp: %13.6fs data: %6d.\r\n", tmp_time_stamp, tmp_activity);

    if(BHY_STILL_ACTIVITY_ENDED == (tmp_activity & BHY_STILL_ACTIVITY_ENDED))
    {
        DBG_PRINT("still activity ended.\r\n");
    }
    if(BHY_WALKING_ACTIVITY_ENDED == (tmp_activity & BHY_WALKING_ACTIVITY_ENDED))
    {
        DBG_PRINT("walking activity ended.\r\n");
    }
    if(BHY_RUNNING_ACTIVITY_ENDED == (tmp_activity & BHY_RUNNING_ACTIVITY_ENDED))
    {
        DBG_PRINT("running activity ended.\r\n");
    }
    if(BHY_ON_BICYCLE_ACTIVITY_ENDED == (tmp_activity & BHY_ON_BICYCLE_ACTIVITY_ENDED))
    {
        DBG_PRINT("on bicycle activity ended.\r\n");
    }
    if(BHY_IN_VEHICLE_ACTIVITY_ENDED == (tmp_activity & BHY_IN_VEHICLE_ACTIVITY_ENDED))
    {
        DBG_PRINT("in vehicle activity ended.\r\n");
    }
    if(BHY_TILTING_ACTIVITY_ENDED == (tmp_activity & BHY_TILTING_ACTIVITY_ENDED))
    {
        DBG_PRINT("tilting activity ended.\r\n");
    }
    if(BHY_STILL_ACTIVITY_STARTED == (tmp_activity & BHY_STILL_ACTIVITY_STARTED))
    {
        DBG_PRINT("still activity started.\r\n");
    }
    if(BHY_WALKING_ACTIVITY_STARTED == (tmp_activity & BHY_WALKING_ACTIVITY_STARTED))
    {
        DBG_PRINT("walking activity started.\r\n");
    }
    if(BHY_RUNNING_ACTIVITY_STARTED == (tmp_activity & BHY_RUNNING_ACTIVITY_STARTED))
    {
        DBG_PRINT("running activity started.\r\n");
    }
    if(BHY_ON_BICYCLE_ACTIVITY_STARTED == (tmp_activity & BHY_ON_BICYCLE_ACTIVITY_STARTED))
    {
        DBG_PRINT("on bicycle activity started.\r\n");
    }
    if(BHY_IN_VEHICLE_ACTIVITY_STARTED == (tmp_activity & BHY_IN_VEHICLE_ACTIVITY_STARTED))
    {
        DBG_PRINT("in vehicle activity started.\r\n");
    }
    if(BHY_TILTING_ACTIVITY_STARTED == (tmp_activity & BHY_TILTING_ACTIVITY_STARTED))
    {
        DBG_PRINT("tilting activity started.\r\n");
    }
    p_fifo_buffer->read_pos += 3;

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse small delta timestamp frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_timestamp_small_delta(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 2) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    p_bhy2_hub->time_stamp += p_fifo_buffer->p_buffer[(ts32_read_pos + 1)];
    //printf("small delta\r\n");
    //printf("timp stamp: %.3fs.\r\n", (float)p_bhy2_hub->time_stamp / 64000);
    p_fifo_buffer->read_pos += 2;
    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup small delta timestamp frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_timestamp_small_delta_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    s32 ts32_read_pos = p_fifo_buffer->read_pos;

    if((p_fifo_buffer->read_pos + 2) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    p_bhy2_hub->time_stamp_wkup += p_fifo_buffer->p_buffer[(ts32_read_pos + 1)];
    //printf("wkup small delta\r\n");
    //printf("timp stamp: %.3fs.\r\n", (float)p_bhy2_hub->time_stamp_wkup / 64000);
    p_fifo_buffer->read_pos += 2;
    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse large delta timestampframe
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_timestamp_large_delta(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    s32 ts32_read_pos = p_fifo_buffer->read_pos;
    u16 large_delta = 0;

    if((p_fifo_buffer->read_pos + 3) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    large_delta = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((u16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    p_bhy2_hub->time_stamp += large_delta;
    //printf("large delta\r\n");
    //printf("timp stamp: %.3fs.\r\n", (float)p_bhy2_hub->time_stamp / 64000);
    p_fifo_buffer->read_pos += 3;

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup large delta timestamp frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_timestamp_large_delta_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    s32 ts32_read_pos = p_fifo_buffer->read_pos;
    u16 large_delta = 0;

    if((p_fifo_buffer->read_pos + 3) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    large_delta = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)] | ((u16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    p_bhy2_hub->time_stamp_wkup += large_delta;
    //printf("wkup large delta\r\n");
    //printf("timp stamp: %.3fs.\r\n", (float)p_bhy2_hub->time_stamp_wkup / 64000);
    p_fifo_buffer->read_pos += 3;
    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse full timestamp frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_full_timestamp(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    s32 ts32_read_pos = p_fifo_buffer->read_pos;
    u64 full_timestamp = 0;

    if((p_fifo_buffer->read_pos + 6) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    full_timestamp = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)]
                    + ((u64)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    full_timestamp += ((u64)p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] << 16)
                    + ((u64)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 24);
    full_timestamp += ((u64)p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] << 32);
    p_bhy2_hub->time_stamp = full_timestamp;
    //printf("full\r\n");
    //printf("timp stamp: %.3fs.\r\n", (float)p_bhy2_hub->time_stamp / 64000);
    p_fifo_buffer->read_pos += 6;
    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup full timestamp frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_full_timestamp_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    s32 ts32_read_pos = p_fifo_buffer->read_pos;
    u64 full_timestamp = 0;

    if((p_fifo_buffer->read_pos + 6) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    full_timestamp = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)]
                    + ((u64)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    full_timestamp += ((u64)p_fifo_buffer->p_buffer[(ts32_read_pos + 3)] << 16)
                    + ((u64)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 24);
    full_timestamp += ((u64)p_fifo_buffer->p_buffer[(ts32_read_pos + 5)] << 32);
    p_bhy2_hub->time_stamp_wkup = full_timestamp;
    //printf("wkup full\r\n");
    //printf("timp stamp: %.3fs.\r\n", (float)p_bhy2_hub->time_stamp_wkup / 64000);
    p_fifo_buffer->read_pos += 6;
    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse meta event frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_meta_event(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    s32 ts32_read_pos = p_fifo_buffer->read_pos;
    u8 tmp_meta_event_type = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)];
    u8 tmp_byte1 = p_fifo_buffer->p_buffer[(ts32_read_pos + 2)];
    u8 tmp_byte2 = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)];

    if((p_fifo_buffer->read_pos + 4) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    p_bhy2_hub->meta_event.meta_event_type = tmp_meta_event_type;
    p_bhy2_hub->meta_event.byte1 = tmp_byte1;
    p_bhy2_hub->meta_event.byte2 = tmp_byte2;

    switch(tmp_meta_event_type)
    {
        case BHY_META_EVENT_FLUSH_COMPLETE:
            TST_PRINT("BHY_META_EVENT_FLUSH_COMPLETE: sensor type: %3d.\r\n", tmp_byte1);
            break;
        case BHY_META_EVENT_SAMPLE_RATE_CHANGED:
            TST_PRINT("BHY_META_EVENT_SAMPLE_RATE_CHANGED: sensor type: %3d.\r\n", tmp_byte1);
            break;
        case BHY_META_EVENT_POWER_MODE_CHANGED:
            TST_PRINT("BHY_META_EVENT_POWER_MODE_CHANGED: sensor type: %3d.\r\n", tmp_byte1);
            break;
        case BHY_META_EVENT_ALGORITHM_EVENTS:
            TST_PRINT("BHY_META_EVENT_ALGORITHM_EVENTS.\r\n");
            break;
        case BHY_META_EVENT_SENSOR_STATUS:
            p_bhy2_hub->virt_sensor[tmp_byte1].info.accuracy = tmp_byte2;
            TST_PRINT("BHY_META_EVENT_SENSOR_STATUS: sensor type: %3d status: %1d.\r\n", tmp_byte1, tmp_byte2);
            break;
        case BHY_META_EVENT_BSX_DO_STEPS_MAIN:
            TST_PRINT("BHY_META_EVENT_BSX_DO_STEPS_MAIN.\r\n");
            break;
        case BHY_META_EVENT_BSX_DO_STEPS_CALIB:
            TST_PRINT("BHY_META_EVENT_BSX_DO_STEPS_CALIB.\r\n");
            break;
        case BHY_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
            TST_PRINT("BHY_META_EVENT_BSX_GET_OUTPUT_SIGNAL.\r\n");
            break;
        case BHY_META_EVENT_SENSOR_ERROR:
            TST_PRINT("BHY_META_EVENT_SENSOR_ERROR: sensor type: %3d err reg: 0x%02X.\r\n", tmp_byte1, tmp_byte2);
            break;
        case BHY_META_EVENT_FIFO_OVERFLOW:
            TST_PRINT("BHY_META_EVENT_FIFO_OVERFLOW.\r\n");
            break;
        case BHY_META_EVENT_DYNAMIC_RANGE_CHANGED:
            TST_PRINT("BHY_META_EVENT_DYNAMIC_RANGE_CHANGED: sensor type: %3d.\r\n", tmp_byte1);
            break;
        case BHY_META_EVENT_FIFO_WATERMARK:
            TST_PRINT("BHY_META_EVENT_FIFO_WATERMARK.\r\n");
            break;
        case BHY_META_EVENT_INITIALIZED:
            TST_PRINT("BHY_META_EVENT_INITIALIZED: ram ver: %4d.\r\n", ((u16)tmp_byte2 << 8)|tmp_byte1);
            break;
        case BHY_META_TRANSFER_CAUSE:
            TST_PRINT("BHY_META_TRANSFER_CAUSE: sensor type: %3d.\r\n", tmp_byte1);
            break;
        case BHY_META_EVENT_SENSOR_FRAMEWORK:
            TST_PRINT("BHY_META_EVENT_SENSOR_FRAMEWORK: sensor type: %3d.\r\n", tmp_byte1);
            break;
        case BHY_META_EVENT_RESET:
            TST_PRINT("BHY_META_EVENT_RESET.\r\n");
            break;
        case BHY_META_EVENT_SPACER:
            //printf("BHY_META_EVENT_SPACER.\r\n");
            break;
        default:
            TST_PRINT("meta event unknow.\r\n");
            TST_PRINT("meta event id: %2d.\r\n", tmp_meta_event_type);
            break;
    }
    p_fifo_buffer->read_pos += 4;

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse wakeup meta event frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_meta_event_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    s32 ts32_read_pos = p_fifo_buffer->read_pos;
    u8 tmp_meta_event_type = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)];
    u8 tmp_byte1 = p_fifo_buffer->p_buffer[(ts32_read_pos + 2)];
    u8 tmp_byte2 = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)];

    if((p_fifo_buffer->read_pos + 4) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    p_bhy2_hub->meta_event_wkup.meta_event_type = tmp_meta_event_type;
    p_bhy2_hub->meta_event_wkup.byte1 = tmp_byte1;
    p_bhy2_hub->meta_event_wkup.byte2 = tmp_byte2;

    switch(tmp_meta_event_type)
    {
        case BHY_META_EVENT_FLUSH_COMPLETE:
            TST_PRINT("BHY_WKUP_META_EVENT_FLUSH_COMPLETE: sensor type: %3d.\r\n", tmp_byte1);
            break;
        case BHY_META_EVENT_SAMPLE_RATE_CHANGED:
            TST_PRINT("BHY_WKUP_META_EVENT_SAMPLE_RATE_CHANGED: sensor type: %3d.\r\n", tmp_byte1);
            break;
        case BHY_META_EVENT_POWER_MODE_CHANGED:
            TST_PRINT("BHY_WKUP_META_EVENT_POWER_MODE_CHANGED: sensor type: %3d.\r\n", tmp_byte1);
            break;
        case BHY_META_EVENT_ALGORITHM_EVENTS:
            TST_PRINT("BHY_WKUP_META_EVENT_ALGORITHM_EVENTS.\r\n");
            break;
        case BHY_META_EVENT_SENSOR_STATUS:
            p_bhy2_hub->virt_sensor[tmp_byte1].info.accuracy = tmp_byte2;
            TST_PRINT("BHY_WKUP_META_EVENT_SENSOR_STATUS: sensor type: %3d status: %1d.\r\n", tmp_byte1, tmp_byte2);
            break;
        case BHY_META_EVENT_BSX_DO_STEPS_MAIN:
            TST_PRINT("BHY_WKUP_META_EVENT_BSX_DO_STEPS_MAIN.\r\n");
            break;
        case BHY_META_EVENT_BSX_DO_STEPS_CALIB:
            TST_PRINT("BHY_WKUP_META_EVENT_BSX_DO_STEPS_CALIB.\r\n");
            break;
        case BHY_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
            TST_PRINT("BHY_WKUP_META_EVENT_BSX_GET_OUTPUT_SIGNAL.\r\n");
            break;
        case BHY_META_EVENT_SENSOR_ERROR:
            TST_PRINT("BHY_WKUP_META_EVENT_SENSOR_ERROR: sensor type: %3d.\r\n", tmp_byte1);
            break;
        case BHY_META_EVENT_FIFO_OVERFLOW:
            TST_PRINT("BHY_WKUP_META_EVENT_FIFO_OVERFLOW.\r\n");
            break;
        case BHY_META_EVENT_DYNAMIC_RANGE_CHANGED:
            TST_PRINT("BHY_WKUP_META_EVENT_DYNAMIC_RANGE_CHANGED: sensor type: %3d.\r\n", tmp_byte1);
            break;
        case BHY_META_EVENT_FIFO_WATERMARK:
            TST_PRINT("BHY_WKUP_META_EVENT_FIFO_WATERMARK.\r\n");
            break;
        case BHY_META_EVENT_INITIALIZED:
            TST_PRINT("BHY_WKUP_META_EVENT_INITIALIZED: ram ver: %4d.\r\n", ((u16)tmp_byte2 << 8)|tmp_byte1);
            break;
        case BHY_META_TRANSFER_CAUSE:
            TST_PRINT("BHY_WKUP_META_TRANSFER_CAUSE: sensor type: %3d.\r\n", tmp_byte1);
            break;
        case BHY_META_EVENT_SENSOR_FRAMEWORK:
            TST_PRINT("BHY_WKUP_META_EVENT_SENSOR_FRAMEWORK: sensor type: %3d.\r\n", tmp_byte1);
            break;
        case BHY_META_EVENT_RESET:
            TST_PRINT("BHY_WKUP_META_EVENT_RESET.\r\n");
            break;
        case BHY_META_EVENT_SPACER:
            //printf("BHY_META_EVENT_SPACER.\r\n");
            break;
        default:
            TST_PRINT("meta event unknow.\r\n");
            TST_PRINT("wkup meta event id: %2d.\r\n", tmp_meta_event_type);
            break;
    }
    p_fifo_buffer->read_pos += 4;

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse debug message frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_debug_message(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    s32 ts32_read_pos = p_fifo_buffer->read_pos;
    data_debug_t tmp_debug = {0};
    u8 i = 0;

    if((p_fifo_buffer->read_pos + 18) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    tmp_debug.flag = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)];
    memset(tmp_debug.buffer, 0, sizeof(tmp_debug.buffer));
    for(i = 0; i < 16; i++)
    {
        tmp_debug.buffer[i] = p_fifo_buffer->p_buffer[(ts32_read_pos + 2 + i)];
    }

    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */
    p_fifo_buffer->read_pos += 18;
    TST_PRINT("dbg_msg: %13.6fs flag: 0x%02X data: %s\r\n", tmp_time_stamp, tmp_debug.flag, tmp_debug.buffer);

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse gps frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer to parse
 * @param[in] p_bhy2_hub: pointer to bhu2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_gps(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    float tmp_time_stamp = 0;
    u32 ts32_read_pos = p_fifo_buffer->read_pos;
    //data_gps tmp_gps_data = {0};
    u8 i = 0;

    if((p_fifo_buffer->read_pos + 27) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

    g_gps_data.id = p_fifo_buffer->p_buffer[(ts32_read_pos)];
    g_gps_data.total_length = p_fifo_buffer->p_buffer[(ts32_read_pos + 1)]
                                + ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 2)] << 8);
    g_gps_data.bytes_transferred = p_fifo_buffer->p_buffer[(ts32_read_pos + 3)]
                                    + ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + 4)] << 8);
    g_gps_data.bytes_to_transfer = p_fifo_buffer->p_buffer[(ts32_read_pos + 5)];
    g_gps_data.protocol_type = p_fifo_buffer->p_buffer[(ts32_read_pos + 6)];
    if(g_gps_data.bytes_transferred == 0)
    {
        memset(g_gps_data.buffer, 0, sizeof(g_gps_data.buffer));
    }
#if 0
    for(i = 0; i < tmp_gps_data.bytes_to_transfer; i++)
    {
        tmp_gps_data.p_buffer[i] = p_fifo_buffer->p_buffer[(ts32_read_pos + 7 + i)];
    }

    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */
    p_fifo_buffer->read_pos += 27;

    TST_PRINT("sensor: gps timestamp: %13.6fs %s.\r\n", tmp_time_stamp, tmp_gps_data.p_buffer);
#else
    for(i = 0; i < g_gps_data.bytes_to_transfer; i++)
    {
        g_gps_data.buffer[g_gps_data.bytes_transferred + i] = p_fifo_buffer->p_buffer[(ts32_read_pos + 7 + i)];
    }

    tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /* 1 LSB = 1/64000 sec */
    p_fifo_buffer->read_pos += 27;
    //TST_PRINT("sensor: gps timestamp: %13.6fs %s.\r\n", tmp_time_stamp, &tmp_gps_data.p_buffer[tmp_gps_data.bytes_transferred]);
    if( g_gps_data.total_length == (g_gps_data.bytes_transferred + g_gps_data.bytes_to_transfer))
    {
        /* CR LF is contained in GPS message */
        TST_PRINT("sensor: gps timestamp: %13.6fs %s", tmp_time_stamp, g_gps_data.buffer);
    }
#endif

    return BHY_HIF_E_SUCCESS;
}

