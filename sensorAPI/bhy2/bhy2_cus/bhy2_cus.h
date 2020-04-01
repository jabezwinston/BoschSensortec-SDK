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
 * @file        bhy2_cus.h
 *
 * @brief
 *
 *
 */

/*!
 * @defgroup bhy2_cus
 * @{*/

#ifndef __BHY2_CUS_H__
#define __BHY2_CUS_H__

#include "typedef.h"
#include "bhy_host_interface.h"


s16 bhy2_parse_frame_acc_corrected(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_acc_corrected_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_acc_passthrough(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_acc_raw(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_acc_raw_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_acc_offset(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_mag_corrected(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_mag_corrected_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_mag_passthrough(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_mag_raw(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_mag_raw_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_mag_offset(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_orientation(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_orientation_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_gyro_corrected(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_gyro_corrected_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_gyro_passthrough(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_gyro_raw(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_gyro_raw_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_gyro_offset(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_light(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_light_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_proximity(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_proximity_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_humidity(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_humidity_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_temperature(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_temperature_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_barometer(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_barometer_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_gas(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_gas_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_gravity(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_gravity_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_linear_acc(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_linear_acc_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_rotation(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_rotation_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_game_rotation(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_game_rotation_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_geo_rotation(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_geo_rotation_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_sig_motion_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_step_detect(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_step_detect_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_tilt_detect_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_wake_gesture_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_glance_gesture_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_pickup_gesture_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_sig_motion_hw_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_step_detect_hw(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_step_detect_hw_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_any_motion_hw(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_any_motion_hw_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_step_counter(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_step_counter_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_step_counter_hw(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_step_counter_hw_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_activity_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_timestamp_small_delta(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_timestamp_small_delta_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_timestamp_large_delta(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_timestamp_large_delta_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_meta_event(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_meta_event_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_full_timestamp(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_full_timestamp_wkup(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_debug_message(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
s16 bhy2_parse_frame_gps(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
#endif

