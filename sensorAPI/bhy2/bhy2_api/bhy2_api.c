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
 * @file        bhy2_api.c
 *
 * @brief
 *
 *
 */

/*!
 * @addtogroup ${file_base}
 * @brief
 * @{*/


/*********************************************************************/
/* system header files */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/*********************************************************************/
/* own header files */
#include "typedef.h"
#include "bhy2_hal.h"
#include "bhy_host_interface.h"
#include "bhy2_api.h"

/*********************************************************************/
/* local macro definitions */


/*********************************************************************/
/* constant definitions */


/*********************************************************************/
/* global variables */
u8 wkup_fifo_buffer[DATA_FIFO_BUFFER_LENGTH] = {0};
u8 nonwkup_fifo_buffer[DATA_FIFO_BUFFER_LENGTH] = {0};
u8 status_fifo_buffer[STATUS_FIFO_BUFFER_LENGTH] = {0};


/*********************************************************************/
/* static variables */
u8 sensor_table[][30] =
{
    "padding",/* 0: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "acc_passthrough",/* 1: BHY_SENSOR_ID_ACC_PASS */
    "reserved2",/* 2: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "acc_raw",/* 3: BHY_SENSOR_ID_ACC_RAW */
    "acc_corrected",/* 4: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "acc_offset",/* 5: BHY_SENSOR_ID_ACC_BIAS */
    "wkup_acc_corrected",/* 6: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "wkup_acc_raw",/* 7: BHY_SENSOR_ID_ACC_RAW_WU */
    "reserved8",/* 8: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved9",/* 9: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "gyro_passthrough",/* 10: BHY_SENSOR_ID_GYRO_PASS */
    "reserved11",/* 11: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "gyro_raw",/* 12: BHY_SENSOR_ID_GYRO_RAW */
    "gyro_corrected",/* 13: BHY_SENSOR_ID_GYRO */
    "gyro_offset",/* 14: BHY_SENSOR_ID_GYRO_BIAS */
    "wkup_gyro_corrected",/* 15: BHY_SENSOR_ID_GYRO_WU */
    "wkup_gyro_raw",/* 16: BHY_SENSOR_ID_GYRO_RAW_WU */
    "reserved17",/* 17: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved18",/* 18: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "mag_passthrough",/* 19: BHY_SENSOR_ID_MAG_PASS */
    "reserved20",/* 20: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "mag_raw",/* 21: BHY_SENSOR_ID_MAG_RAW */
    "mag_corrected",/* 22: BHY_SENSOR_ID_MAG */
    "mag_offset",/* 23: BHY_SENSOR_ID_MAG_BIAS */
    "wkup_mag_corrected",/* 24: BHY_SENSOR_ID_MAG_WU */
    "wkup_mag_raw",/* 25: BHY_SENSOR_ID_MAG_RAW_WU */
    "reserved26",/* 26: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved27",/* 27: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "gravity",/* 28: BHY_SENSOR_ID_GRA */
    "wkup_gravity",/* 29: BHY_SENSOR_ID_GRA_WU */
    "reserved30",/* 30: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "linear_acc",/* 31: BHY_SENSOR_ID_LACC */
    "wkup_linear_acc",/* 32: BHY_SENSOR_ID_LACC_WU */
    "reserved33",/* 33: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "rotation",/* 34: BHY_SENSOR_ID_RV */
    "wkup_rotation",/* 35: BHY_SENSOR_ID_RV_WU */
    "reserved36",/* 36: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "game_rotation",/* 37: BHY_SENSOR_ID_GAMERV */
    "wkup_game_rotation",/* 38: BHY_SENSOR_ID_GAMERV_WU */
    "reserved39",/* 39: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "geo_rotation",/* 40: BHY_SENSOR_ID_GEORV */
    "wkup_geo_rotation",/* 41: BHY_SENSOR_ID_GEORV_WU */
    "reserved42",/* 42: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "orient",/* 43: BHY_SENSOR_ID_ORI */
    "wkup_orient",/* 44: BHY_SENSOR_ID_ORI_WU */
    "reserved45",/* 45: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved46",/* 46: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved47",/* 47: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "wkup_tilt_detect",/* 48: BHY_SENSOR_ID_TILT_WU */
    "reserved49",/* 49: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "step detect",/* 50: BHY_SENSOR_ID_STD */
    "reserved51",/* 51: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "step count",/* 52: BHY_SENSOR_ID_STC */
    "wkup_step_count",/* 53: BHY_SENSOR_ID_STC_WU */
    "reserved54",/* 54: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "wkup_sig_motion",/* 55: BHY_SENSOR_ID_SIG_WU */
    "reserved56",/* 56: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "wkup_wake_gesture",/* 57: BHY_SENSOR_ID_WAKE_WU */
    "reserved58",/* 58: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "wkup_glance_gesture",/* 59: BHY_SENSOR_ID_GLANCE_WU */
    "reserved60",/* 60: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "wkup_pickup_gesture",/* 61: BHY_SENSOR_ID_PICKUP_WU */
    "reserved62",/* 62: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "wkup_activity",/* 63: BHY_SENSOR_ID_AR_WU */
    "reserved64",/* 64: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved65",/* 65: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved66",/* 66: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved67",/* 67: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved68",/* 68: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved69",/* 69: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved70",/* 70: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved71",/* 71: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved72",/* 72: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved73",/* 73: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved74",/* 74: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved75",/* 75: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved76",/* 76: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved77",/* 77: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved78",/* 78: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved79",/* 79: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved80",/* 80: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved81",/* 81: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved82",/* 82: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved83",/* 83: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved84",/* 84: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved85",/* 85: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved86",/* 86: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved87",/* 87: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved88",/* 88: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved89",/* 89: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved90",/* 90: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved91",/* 91: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved92",/* 92: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved93",/* 93: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "wkup_step_detect",/* 94: BHY_SENSOR_ID_STD_WU */
    "reserved95",/* 95: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved96",/* 96: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved97",/* 97: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved98",/* 98: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved99",/* 99: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved100",/* 100: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved101",/* 101: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved102",/* 102: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved103",/* 103: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved104",/* 104: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved105",/* 105: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved106",/* 106: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved107",/* 107: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved108",/* 108: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved109",/* 109: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved110",/* 110: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved111",/* 111: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved112",/* 112: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved113",/* 113: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved114",/* 114: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved115",/* 115: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved116",/* 116: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved117",/* 117: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved118",/* 118: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved119",/* 119: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved120",/* 120: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved121",/* 121: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved122",/* 122: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved123",/* 123: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved124",/* 124: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved125",/* 125: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved126",/* 126: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved127",/* 127: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "temperature",/* 128: BHY_SENSOR_ID_TEMP */
    "barometer",/* 129: BHY_SENSOR_ID_BARO */
    "humidity",/* 130: BHY_SENSOR_ID_HUM */
    "gas",/* 131: BHY_SENSOR_ID_GAS */
    "wkup_temperature",/* 132: BHY_SENSOR_ID_TEMP_WU */
    "wkup_barometer",/* 133: BHY_SENSOR_ID_BARO_WU */
    "wkup_humidity",/* 134: BHY_SENSOR_ID_HUM_WU */
    "wkup_gas",/* 135: BHY_SENSOR_ID_GAS_WU */
    "step_count_hw",/* 136: BHY_SENSOR_ID_STC_HW */
    "step_detect_hw",/* 137: BHY_SENSOR_ID_STD_HW */
    "sig_motion_hw",/* 138: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "wkup_step_count_hw",/* 139: BHY_SENSOR_ID_STC_HW_WU */
    "wkup_step_detect_hw",/* 140: BHY_SENSOR_ID_STD_HW_WU */
    "wkup_sig_motion_hw",/* 141: BHY_SENSOR_ID_SIG_HW_WU */
    "any_motion_hw",/* 142: BHY_SENSOR_ID_ANY_MOTION */
    "wkup_any_motion_hw",/* 143: BHY_SENSOR_ID_ANY_MOTION_WU */
    "ex_camera",/* 144: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "gps",/* 145: BHY_SENSOR_ID_GPS */
    "light",/* 146: BHY_SENSOR_ID_LIGHT */
    "proximity",/* 147: BHY_SENSOR_ID_PROX */
    "wkup_light",/* 148: BHY_SENSOR_ID_LIGHT_WU */
    "wkup_proximity",/* 149: BHY_SENSOR_ID_PROX_WU */
    "reserved150",/* 150: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved151",/* 151: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved152",/* 152: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved153",/* 153: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved154",/* 154: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved155",/* 155: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved156",/* 156: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved157",/* 157: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved158",/* 158: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved159",/* 159: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved160",/* 160: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved161",/* 161: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved162",/* 162: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved163",/* 163: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved164",/* 164: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved165",/* 165: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved166",/* 166: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved167",/* 167: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved168",/* 168: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved169",/* 169: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved170",/* 170: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved171",/* 171: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved172",/* 172: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved173",/* 173: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved174",/* 174: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved175",/* 175: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved176",/* 176: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved177",/* 177: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved178",/* 178: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved179",/* 179: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved180",/* 180: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved181",/* 181: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved182",/* 182: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved183",/* 183: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved184",/* 184: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved185",/* 185: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved186",/* 186: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved187",/* 187: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved188",/* 188: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved189",/* 189: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved190",/* 190: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved191",/* 191: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved192",/* 192: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved193",/* 193: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved194",/* 194: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved195",/* 195: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved196",/* 196: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved197",/* 197: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved198",/* 198: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved199",/* 199: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved200",/* 200: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved201",/* 201: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved202",/* 202: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved203",/* 203: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved204",/* 204: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved205",/* 205: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved206",/* 206: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved207",/* 207: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved208",/* 208: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved209",/* 209: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved210",/* 210: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved211",/* 211: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved212",/* 212: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved213",/* 213: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved214",/* 214: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved215",/* 215: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved216",/* 216: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved217",/* 217: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved218",/* 218: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved219",/* 219: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved220",/* 220: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved221",/* 221: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved222",/* 222: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved223",/* 223: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved224",/* 224: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved225",/* 225: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved226",/* 226: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved227",/* 227: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved228",/* 228: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved229",/* 229: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved230",/* 230: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved231",/* 231: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved232",/* 232: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved233",/* 233: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved234",/* 234: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved235",/* 235: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved236",/* 236: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved237",/* 237: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved238",/* 238: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved239",/* 239: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved240",/* 240: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved241",/* 241: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "reserved242",/* 242: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "bsx4_update_sub_log",/* 243: BHY_SYS_SENSOR_ID_BSX_LOG_UPDATE_SUB */
    "bsx4_do_steps_log",/* 244: BHY_SYS_SENSOR_ID_BSX_LOG_DOSTEP */
    "wkup_timestamp_small_delta",/* 245: BHY_SYS_SENSOR_ID_TS_SMALL_DELTA_WU */
    "wkup_timestamp_large_delta",/* 246: BHY_SYS_SENSOR_ID_TS_LARGE_DELTA_WU */
    "wkup_full_timestamp",/* 247: BHY_SYS_SENSOR_ID_TS_FULL_WU */
    "wkup_meta_event",/* 248: BHY_SYS_SENSOR_ID_META_EVENT_WU */
    "reserved249",/* 249: BSX_VIRTUAL_SENSOR_ID_INVALID */
    "debug_msg",/* 250: BHY_SYS_SENSOR_ID_DEBUG_MSG */
    "timestamp_small_delta",/* 251: BHY_SYS_SENSOR_ID_TS_SMALL_DELTA */
    "timestamp_large_delta",/* 252: BHY_SYS_SENSOR_ID_TS_LARGE_DELTA */
    "full_timestamp",/* 253: BHY_SYS_SENSOR_ID_TS_FULL */
    "meta_event",/* 254: BHY_SYS_SENSOR_ID_META_EVENT */
    "filler",/* 255: BHY_SYS_SENSOR_ID_FILLER */
};


/*!
 * @brief get register data
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] reg: register to get
 * @param[out] p_data: pointer to register data
 *
 * @return res: operation result
 */
s16 bhy2_register_get( bhy2_hub_t* p_bhy2_hub, u8 reg, u8* p_data)
{
    s16 res = BHY_HIF_E_SUCCESS;

    res = bhy_hif_bus_read( &p_bhy2_hub->handle, reg, p_data, 1);

    return res;
}

/*!
 * @brief set register data
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] reg: register to set
 * @param[in] data: data to set
 *
 * @return res: operation result
 */
s16 bhy2_register_set( bhy2_hub_t* p_bhy2_hub, u8 reg, u8 data)
{
    s16 res = BHY_HIF_E_SUCCESS;

    res = bhy_hif_bus_write( &p_bhy2_hub->handle, reg, &data, 1);

    return res;
}

/*!
 * @brief update bhy2 obj sample rate
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 *
 * @return res: operation result
 */
s16 bhy2_update_sample_rate( bhy2_hub_t* p_bhy2_hub )
{
    s16 res = BHY_HIF_E_SUCCESS;
    u16 i = 0;

    p_bhy2_hub->max_sample_rate = 0;
    p_bhy2_hub->min_sample_rate = 0;
    p_bhy2_hub->sensor_type_check = 0;

    for(i = 0; i < BHY_SYS_SENSOR_ID_MAX; i++)
    {
        /* only check enabled virtual sensor */
        if( p_bhy2_hub->virt_sensor[i].info.id > 0)
        {
            /* get maximum sample rate */
            if(p_bhy2_hub->max_sample_rate < p_bhy2_hub->virt_sensor[i].info.sample_rate)
            {
                p_bhy2_hub->max_sample_rate = p_bhy2_hub->virt_sensor[i].info.sample_rate;
            }
            /* get minimum sample rate */
            if((u32)p_bhy2_hub->min_sample_rate == 0)
            {
                p_bhy2_hub->min_sample_rate = p_bhy2_hub->virt_sensor[i].info.sample_rate;
            }
            else
            {
                if(p_bhy2_hub->min_sample_rate > p_bhy2_hub->virt_sensor[i].info.sample_rate)
                {
                    p_bhy2_hub->min_sample_rate = p_bhy2_hub->virt_sensor[i].info.sample_rate;
                }
            }
            /* check enabled sensor type */
            p_bhy2_hub->sensor_type_check |= p_bhy2_hub->virt_sensor[i].info.type;
        }
    }
    /* update enabled sensor type */

    DBG_PRINT("max_sample_rate:%6.2fHz\r\n", p_bhy2_hub->max_sample_rate);
    DBG_PRINT("min_sample_rate:%6.2fHz\r\n", p_bhy2_hub->min_sample_rate);
    DBG_PRINT("sensor_type_check:%1d\r\n", p_bhy2_hub->sensor_type_check);
    return res;
}

/*!
 * @brief update bhy2 obj latency
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 *
 * @return res: operation result
 */
s16 bhy2_update_latency( bhy2_hub_t* p_bhy2_hub )
{
    s16 res = BHY_HIF_E_SUCCESS;
    u16 i = 0;

    p_bhy2_hub->max_latency = 0;

    for(i = 0; i < BHY_SYS_SENSOR_ID_MAX; i++)
    {
        /* only check enabled virtual sensor */
        if( p_bhy2_hub->virt_sensor[i].info.id > 0)
        {
            /* get maximum latency */
            if(p_bhy2_hub->max_latency < p_bhy2_hub->virt_sensor[i].info.latency)
            {
                p_bhy2_hub->max_latency = p_bhy2_hub->virt_sensor[i].info.latency;
            }
            /* get minimum latency */
            if((u32)p_bhy2_hub->min_latency == 0)
            {
                p_bhy2_hub->min_latency = p_bhy2_hub->virt_sensor[i].info.latency;
            }
            else
            {
                if(p_bhy2_hub->min_latency > p_bhy2_hub->virt_sensor[i].info.latency)
                {
                    p_bhy2_hub->min_latency = p_bhy2_hub->virt_sensor[i].info.latency;
                }
            }
        }
    }
    DBG_PRINT("max_latency:%6dms\r\n", p_bhy2_hub->max_latency);
    DBG_PRINT("min_latency:%6dms\r\n", p_bhy2_hub->min_latency);

    return res;
}

/*!
 * @brief dump registers' data
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 *
 * @return res: operation result
 */
s16 bhy2_dump_registers(bhy2_hub_t* p_bhy2_hub)
{
    s16 res = BHY_HIF_E_SUCCESS;
    u8 tmp_buf[45] = {0};

    bhy_hif_bus_read(&p_bhy2_hub->handle, BHY_REG_CHIP_CTRL, tmp_buf, sizeof(tmp_buf));
    TST_PRINT("dump data from register 0x05 to 0x31:\r\n");
    TST_PRINT("Reg: 0x05 0x06 0x07 0x08 0x09 0x0A 0x0B 0x0C 0x0D 0x0E 0x0F\r\n");
    TST_PRINT("Val: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X,\r\n",
    tmp_buf[0], tmp_buf[1], tmp_buf[2], tmp_buf[3], tmp_buf[4],
    tmp_buf[5], tmp_buf[6], tmp_buf[7], tmp_buf[8], tmp_buf[9], tmp_buf[10]);
    TST_PRINT("Reg: 0x10 0x11 0x12 0x13 0x14 0x15 0x16 0x17 0x18 0x19 0x1A 0x1B 0x1C 0x1D 0x1E 0x1F\r\n");
    TST_PRINT("Val: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X,\r\n",
    tmp_buf[11], tmp_buf[12], tmp_buf[13], tmp_buf[14], tmp_buf[15], tmp_buf[16], tmp_buf[17], tmp_buf[18],
    tmp_buf[19], tmp_buf[20], tmp_buf[21], tmp_buf[22], tmp_buf[23], tmp_buf[24], tmp_buf[25], tmp_buf[26]);
    TST_PRINT("Reg: 0x20 0x21 0x22 0x23 0x24 0x25 0x26 0x27 0x28 0x29 0x2A 0x2B 0x2C 0x2D 0x2E 0x2F\r\n");
    TST_PRINT("Val: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X,\r\n",
    tmp_buf[27], tmp_buf[28], tmp_buf[29], tmp_buf[30], tmp_buf[31], tmp_buf[32], tmp_buf[33], tmp_buf[34],
    tmp_buf[35], tmp_buf[36], tmp_buf[37], tmp_buf[38], tmp_buf[39], tmp_buf[40], tmp_buf[41], tmp_buf[42]);
    TST_PRINT("Reg: 0x30 0x31\r\n");
    TST_PRINT("Val: 0x%02X 0x%02X,\r\n", tmp_buf[43], tmp_buf[44]);

    return res;
}

/*!
 * @brief init software watchdog
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 *
 * @return res: operation result
 */
s16 bhy2_softwatchdog_init(bhy2_hub_t* p_bhy2_hub)
{
    s16 res = BHY_HIF_E_SUCCESS;
    p_bhy2_hub->swdog_delay_ms = 0;
    p_bhy2_hub->sys_time_ms = 0;

    return res;
}

/*!
 * @brief clean soft-watchdog value
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 *
 * @return res: operation result
 */
s16 bhy2_softwatchdog_clean(bhy2_hub_t* p_bhy2_hub)
{
    s16 res = BHY_HIF_E_SUCCESS;
    p_bhy2_hub->swdog_delay_ms = 0;
    p_bhy2_hub->sys_time_ms = bhy2_get_systick_ms(0);
    /*
    DBG_PRINT_FILE_LINE("sys_time_ms:%dms.\r\n", current_time_ms);
    DBG_PRINT("sys_time_ms:%dms.\r\n", current_time_ms);
    */
    return res;
}

/*!
 * @brief soft-watchdog error process
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 *
 * @return res: operation result
 */
s16 bhy2_softwatchdog_error_process(bhy2_hub_t* p_bhy2_hub)
{
    u16 tmp_data = 0;
    u16 i = 0;
    s16 res = BHY_HIF_E_SUCCESS;

    p_bhy2_hub->swdog_delay_ms = 0;
    p_bhy2_hub->sys_time_ms = bhy2_get_systick_ms(0);
    DBG_PRINT("swdog error process!\r\n");
    res = bhy2_hub_reset( p_bhy2_hub );
    if(res < BHY_HIF_E_SUCCESS)
    {
        DBG_PRINT("swdog error process, hub reset fail: %d!\r\n", res);
        goto SOFT_WATCHDOG_ERROR_RETURN;
    }

    res = bhy2_rom_version_get( p_bhy2_hub, &tmp_data);
    if(res < BHY_HIF_E_SUCCESS)
    {
        DBG_PRINT("swdog error process, get rom version fail: %d!\r\n", res);
        goto SOFT_WATCHDOG_ERROR_RETURN;
    }

    if( (p_bhy2_hub->p_patch == 0) || (p_bhy2_hub->patch_size == 0))
    {
        res = BHY_HIF_E_HANDLE;
        DBG_PRINT("swdog error process, miss patch: %d!\r\n", res);
        goto SOFT_WATCHDOG_ERROR_RETURN;
    }

    res = bhy2_patch_upload_to_ram( p_bhy2_hub, p_bhy2_hub->p_patch, p_bhy2_hub->patch_size);
    if(res < BHY_HIF_E_SUCCESS)
    {
        DBG_PRINT("swdog error process, fw download fail: %02d!\r\n", res);
        goto SOFT_WATCHDOG_ERROR_RETURN;
    }

    res = bhy2_patch_boot_from_ram( p_bhy2_hub);
    if(res < BHY_HIF_E_SUCCESS)
    {
        DBG_PRINT("swdog error process, reboot fail: %02d!\r\n", res);
        goto SOFT_WATCHDOG_ERROR_RETURN;
    }

    res = bhy2_ram_version_get( p_bhy2_hub, 0 );
    if(res < BHY_HIF_E_SUCCESS)
    {
        DBG_PRINT("swdog error process, get ram version fail: %02d!\r\n", res);
        goto SOFT_WATCHDOG_ERROR_RETURN;
    }
    /* recover sensor hub configurations -s */
    for(i = 0; i < sizeof(p_bhy2_hub->reg_conf_save.reg_buf); i++)
    {
        /* only check modefied reg */
        if(CHK_BIT(p_bhy2_hub->reg_conf_chk.data, i))
        {
            res = bhy2_register_set ( p_bhy2_hub, (BHY_REG_CHIP_CTRL+i), p_bhy2_hub->reg_conf_save.reg_buf[i]);
            if(res < BHY_HIF_E_SUCCESS)
            {
                goto SOFT_WATCHDOG_ERROR_RETURN;
            }
        }
    }
    /* recover sensor hub configurations -e */
    /* recover enabled virtual sensors -s*/
    for(i = 0; i < BHY_SYS_SENSOR_ID_MAX; i++)
    {
        /* only check enabled virtual sensor */
        if(p_bhy2_hub->virt_sensor[i].info.id)
        {
            res = bhy2_virt_sensor_cfg_set( p_bhy2_hub,
                                            p_bhy2_hub->virt_sensor[i].info.id,
                                            p_bhy2_hub->virt_sensor[i].info.sample_rate,
                                            p_bhy2_hub->virt_sensor[i].info.latency);
            if(res < BHY_HIF_E_SUCCESS)
            {
                DBG_PRINT("swdog error process, re-enable virtual sensor fail, sen_id: %02d!\r\n", p_bhy2_hub->virt_sensor[i].info.id);
                goto SOFT_WATCHDOG_ERROR_RETURN;
            }
        }
    }
    /* recover enabled virtual sensors -e*/

    DBG_PRINT_FILE_LINE("sys_time_ms:%dms.\r\n", p_bhy2_hub->sys_time_ms);

SOFT_WATCHDOG_ERROR_RETURN:
    return res;
}

/*!
 * @brief soft-watchdog schedule call
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 *
 * @return res: operation result
 */
s16 bhy2_softwatchdog_schedule( bhy2_hub_t* p_bhy2_hub)
{
    s16 res = BHY_HIF_E_SUCCESS;
    float max_interval = 0;
    u8 tmp_error_reg_val = 0;
    u8 tmp_boot_status_reg_val = 0;
    u16 tmp_ram_version = 0;
    u16 tmp_error_code = 0;
    u32 tmp_post_mortem_data_len = 0;
    u8* p_buf = 0;

    p_bhy2_hub->swdog_delay_ms = bhy2_get_systick_ms(0) - p_bhy2_hub->sys_time_ms;

    if(p_bhy2_hub->sensor_type_check > 0)/* if some sensors are enabled, excute watchdog */
    {
        /* continuous mode */
        if((p_bhy2_hub->sensor_type_check & SENSOR_TYPE_CONTINUOUS_MODE) == SENSOR_TYPE_CONTINUOUS_MODE)
        {
            if(p_bhy2_hub->min_latency)
            {
                if( (1000 / p_bhy2_hub->min_sample_rate) > p_bhy2_hub->min_latency)
                {
                    max_interval = (1000 / p_bhy2_hub->min_sample_rate);
                }
                else
                {
                    max_interval = (float)p_bhy2_hub->min_latency;
                }
            }
            else
            {
                max_interval = (1000 / p_bhy2_hub->min_sample_rate);
            }

            /* compare in the same unit ms, softwatchdog delay should less than 5*longest interval */
            if(p_bhy2_hub->swdog_delay_ms > (5 * max_interval))
            {
                /* hub error, need reset sensorhub */
                DBG_PRINT_FILE_LINE("hub error, need reset!\r\n");
                res = BHY_HIF_E_HANDLE;
            }
        }
        /* on change mode */
        else if((p_bhy2_hub->sensor_type_check&SENSOR_TYPE_ON_CHANGE_MODE) == SENSOR_TYPE_ON_CHANGE_MODE)
        {
            //need to implemented here
            if(p_bhy2_hub->swdog_delay_ms > (5000))
            {
                /* read error register */
                bhy_hif_bus_read(&p_bhy2_hub->handle, BHY_REG_ERROR, &tmp_error_reg_val, 1);
                if(tmp_error_reg_val > 0)
                {
                    DBG_PRINT("swdog err det, err reg = 0x%02X!\r\n", tmp_error_reg_val);
                    /* bsx algorithm error will be ignored -s */
                    if((tmp_error_reg_val & 0x50) == 0x50)
                    {

                    }
                    /* bsx algorithm error will be ignored -e */
                    else
                    {
                        res = BHY_HIF_E_HANDLE;
                    }
                }
                /* read boot status register */
                bhy_hif_bus_read(&p_bhy2_hub->handle, BHY_REG_BOOT_STATUS, &tmp_boot_status_reg_val, 1);
                if((tmp_boot_status_reg_val & BHY_BST_HOST_FW_IDLE)
                    || ((tmp_boot_status_reg_val & BHY_BST_HOST_INTERFACE_READY) == 0))
                {
                    DBG_PRINT("swdog err det, boot sta reg = 0x%02X!\r\n", tmp_error_reg_val);
                    res = BHY_HIF_E_HANDLE;
                }
                /* read ram version register */
                bhy2_ram_version_get(p_bhy2_hub, &tmp_ram_version);
                if( tmp_ram_version == 0 )
                {
                    res = BHY_HIF_E_HANDLE;
                }

                if( res < BHY_HIF_E_SUCCESS )
                {
                    /* hub error, need reset sensorhub */
                    DBG_PRINT_FILE_LINE("hub error, need reset!\r\n");
                }
                else
                {
                    /* clean softwatchdog here, if get irq */
                    bhy2_softwatchdog_clean( p_bhy2_hub );
                }
            }
        }
        /* error process */
        if( res < BHY_HIF_E_SUCCESS)
        {
            /* dump registers & save data in ram, need further improvement */
            bhy2_dump_registers(p_bhy2_hub);
            /* dump post mortem data */
            p_buf = malloc(5120);
            memset(p_buf, 0, 5120);
            res = bhy2_post_mortem_data_get ( p_bhy2_hub, &tmp_error_code, &tmp_post_mortem_data_len, p_buf, 5120);
            free(p_buf);
            /* need restore configuration of sensor hub */
            bhy2_softwatchdog_error_process( p_bhy2_hub );
        }
    }
    else
    {
        /* clean softwatchdog here, if no sensors are enabled */
        bhy2_softwatchdog_clean( p_bhy2_hub );
    }

    return res;
}

/*!
 * @brief parse padding frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_padding(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
    s16 res = BHY_HIF_E_SUCCESS;
    switch(p_fifo_buffer->p_buffer[p_fifo_buffer->read_pos])
    {
        case BSX_VIRTUAL_SENSOR_ID_INVALID:
        case BHY_SENSOR_ID_SIG_WU:
        case BHY_SENSOR_ID_SIG_HW_WU:
        case BHY_SENSOR_ID_STD:
        case BHY_SENSOR_ID_STD_WU:
        case BHY_SENSOR_ID_STD_HW:
        case BHY_SENSOR_ID_STD_HW_WU:
        case BHY_SENSOR_ID_TILT_WU:
        case BHY_SENSOR_ID_WAKE_WU:
        case BHY_SENSOR_ID_GLANCE_WU:
        case BHY_SENSOR_ID_PICKUP_WU:
        case BHY_SENSOR_ID_ANY_MOTION:
        case BHY_SENSOR_ID_ANY_MOTION_WU:
        case BHY_SYS_SENSOR_ID_FILLER:
            if((p_fifo_buffer->read_pos + 1) <= p_fifo_buffer->read_length)
            {
                p_fifo_buffer->read_pos += 1;/* sizeof(ID + data) */
            }
            else
            {
                res = BHY_HIF_E_BUF;
            }
            break;
        case BHY_SENSOR_ID_PROX:
        case BHY_SENSOR_ID_PROX_WU:
        case BHY_SENSOR_ID_HUM:
        case BHY_SENSOR_ID_HUM_WU:
        case BHY_SYS_SENSOR_ID_TS_SMALL_DELTA:
        case BHY_SYS_SENSOR_ID_TS_SMALL_DELTA_WU:
        case BHY_SENSOR_ID_EXCAMERA:
            if((p_fifo_buffer->read_pos + 2) <= p_fifo_buffer->read_length)
            {
                p_fifo_buffer->read_pos += 2;/* sizeof(ID + data) */
            }
            else
            {
                res = BHY_HIF_E_BUF;
            }
            break;
        case BHY_SENSOR_ID_LIGHT:
        case BHY_SENSOR_ID_LIGHT_WU:
        case BHY_SENSOR_ID_TEMP:
        case BHY_SENSOR_ID_TEMP_WU:
        //case BHY_SENSOR_ID_ATEMP:
        //case BHY_SENSOR_ID_ATEMP_WU:
        case BHY_SENSOR_ID_AR_WU:
        case BHY_SYS_SENSOR_ID_TS_LARGE_DELTA:
        case BHY_SYS_SENSOR_ID_TS_LARGE_DELTA_WU:
            if((p_fifo_buffer->read_pos + 3) <= p_fifo_buffer->read_length)
            {
                p_fifo_buffer->read_pos += 3;/* sizeof(ID + data) */
            }
            else
            {
                res = BHY_HIF_E_BUF;
            }
            break;
        case BHY_SENSOR_ID_BARO:
        case BHY_SENSOR_ID_BARO_WU:
        case BHY_SYS_SENSOR_ID_META_EVENT:
        case BHY_SYS_SENSOR_ID_META_EVENT_WU:
            if((p_fifo_buffer->read_pos + 4) <= p_fifo_buffer->read_length)
            {
                p_fifo_buffer->read_pos += 4;/* sizeof(ID + data) */
            }
            else
            {
                res = BHY_HIF_E_BUF;
            }
            break;
        case BHY_SENSOR_ID_GAS:
        case BHY_SENSOR_ID_GAS_WU:
            if((p_fifo_buffer->read_pos + 5) <= p_fifo_buffer->read_length)
            {
                p_fifo_buffer->read_pos += 5;/* sizeof(ID + data) */
            }
            else
            {
                res = BHY_HIF_E_BUF;
            }
            break;
        case BHY_SYS_SENSOR_ID_TS_FULL:
        case BHY_SYS_SENSOR_ID_TS_FULL_WU:
            if((p_fifo_buffer->read_pos + 6) <= p_fifo_buffer->read_length)
            {
                p_fifo_buffer->read_pos += 6;/* sizeof(ID + data) */
            }
            else
            {
                res = BHY_HIF_E_BUF;
            }
            break;
        case BHY_SENSOR_ID_ACC:
        case BHY_SENSOR_ID_ACC_WU:
        case BHY_SENSOR_ID_ACC_PASS:
        case BHY_SENSOR_ID_ACC_RAW:
        case BHY_SENSOR_ID_ACC_RAW_WU:
        case BHY_SENSOR_ID_ACC_BIAS:
        case BHY_SENSOR_ID_MAG:
        case BHY_SENSOR_ID_MAG_WU:
        case BHY_SENSOR_ID_MAG_PASS:
        case BHY_SENSOR_ID_MAG_RAW:
        case BHY_SENSOR_ID_MAG_RAW_WU:
        case BHY_SENSOR_ID_MAG_BIAS:
        case BHY_SENSOR_ID_ORI:
        case BHY_SENSOR_ID_ORI_WU:
        case BHY_SENSOR_ID_GYRO:
        case BHY_SENSOR_ID_GYRO_WU:
        case BHY_SENSOR_ID_GYRO_PASS:
        case BHY_SENSOR_ID_GYRO_RAW:
        case BHY_SENSOR_ID_GYRO_RAW_WU:
        case BHY_SENSOR_ID_GYRO_BIAS:
        case BHY_SENSOR_ID_GRA:
        case BHY_SENSOR_ID_GRA_WU:
        case BHY_SENSOR_ID_LACC:
            if((p_fifo_buffer->read_pos + 7) <= p_fifo_buffer->read_length)
            {
                p_fifo_buffer->read_pos += 7;/* sizeof(ID + data) */
            }
            else
            {
                res = BHY_HIF_E_BUF;
            }
            break;
        case BHY_SENSOR_ID_LACC_WU:
            break;
        case BHY_SENSOR_ID_RV:
        case BHY_SENSOR_ID_RV_WU:
        case BHY_SENSOR_ID_GAMERV:
        case BHY_SENSOR_ID_GAMERV_WU:
        case BHY_SENSOR_ID_GEORV:
        case BHY_SENSOR_ID_GEORV_WU:
            if((p_fifo_buffer->read_pos + 11) <= p_fifo_buffer->read_length)
            {
                p_fifo_buffer->read_pos += 11;/* sizeof(ID + data) */
            }
            else
            {
                res = BHY_HIF_E_BUF;
            }
            break;
        //case BHY_SENSOR_ID_HEART:
        //case BHY_SENSOR_ID_HEART_WU:
            //p_fifo_buffer->read_pos += 2;
            //break;
        case BHY_SENSOR_ID_GPS:
            if((p_fifo_buffer->read_pos + 27) <= p_fifo_buffer->read_length)
            {
                p_fifo_buffer->read_pos += 27;/* sizeof(ID + data) */
            }
            else
            {
                res = BHY_HIF_E_BUF;
            }
            break;
        case BHY_SYS_SENSOR_ID_BSX_LOG_UPDATE_SUB:
        case BHY_SYS_SENSOR_ID_BSX_LOG_DOSTEP:
            if((p_fifo_buffer->read_pos + 23) <= p_fifo_buffer->read_length)
            {
                p_fifo_buffer->read_pos += 23;/* sizeof(ID + data) */
            }
            else
            {
                res = BHY_HIF_E_BUF;
            }
            break;
        case BHY_SYS_SENSOR_ID_DEBUG_MSG:
            if((p_fifo_buffer->read_pos + 18) <= p_fifo_buffer->read_length)
            {
                p_fifo_buffer->read_pos += 18;/* sizeof(ID + data) */
            }
            else
            {
                res = BHY_HIF_E_BUF;
            }
            break;
    }

    return res;
}

/*!
 * @brief parse fifos
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_fifos(bhy2_hub_t* p_bhy2_hub)
{
    u8 irq_status = 0;
    u16 tmp_sensor_id = 0;
    u32 tu32_read_length = 0;
    u16 i = 0;
    fifo_buffer_t* p_wkup_fifo = &p_bhy2_hub->fifos.wkup_fifo;
    fifo_buffer_t* p_nonwkup_fifo= &p_bhy2_hub->fifos.nonwkup_fifo;
    fifo_buffer_t* p_status_fifo= &p_bhy2_hub->fifos.status_fifo;

    /* get irq status */
    bhy_hif_read_irq_status( &p_bhy2_hub->handle, &irq_status);

    while(irq_status || (p_wkup_fifo->remain_length) || (p_nonwkup_fifo->remain_length) || (p_status_fifo->remain_length))
    {
        DBG_PRINT("irq_status: 0x%02X!\r\n", irq_status);
        /* clean softwatchdog here, if get irq */
        bhy2_softwatchdog_clean( p_bhy2_hub);

        if( (BHY_IS_IRQ_FIFO_W(irq_status) == BHY_IST_FIFO_W_DRDY)
            || (BHY_IS_IRQ_FIFO_W(irq_status) == BHY_IST_FIFO_W_LTCY)
            || (BHY_IS_IRQ_FIFO_W(irq_status) == BHY_IST_FIFO_W_WM)
            || (p_wkup_fifo->remain_length))
        {
            /* reset read pos to the begin of buffer */
            p_wkup_fifo->read_pos = 0;
            bhy_hif_read_wakeup_fifo( &p_bhy2_hub->handle,
                                        &tu32_read_length,
                                        &p_wkup_fifo->p_buffer[p_wkup_fifo->read_length],
                                        (p_wkup_fifo->buffer_size - p_wkup_fifo->read_length),
                                        &p_wkup_fifo->remain_length);
            p_wkup_fifo->read_pos = 0;
            p_wkup_fifo->read_length += tu32_read_length;
            DBG_PRINT("irq_status: 0x%02X wkup_fifo read_length: %d remain_length:%d!\r\n", irq_status, p_wkup_fifo->read_length, p_wkup_fifo->remain_length);
        }
        if( (BHY_IS_IRQ_FIFO_NW(irq_status) == BHY_IST_FIFO_NW_DRDY)
            || (BHY_IS_IRQ_FIFO_NW(irq_status) == BHY_IST_FIFO_NW_LTCY)
            || (BHY_IS_IRQ_FIFO_NW(irq_status) == BHY_IST_FIFO_NW_WM)
            || (p_nonwkup_fifo->remain_length))
        {
            p_nonwkup_fifo->read_pos = 0;
            DBG_PRINT("irq_status: 0x%02X nonwkup_fifo 1 read_length_all: %d remain_length:%d!\r\n", irq_status, p_nonwkup_fifo->read_length, p_nonwkup_fifo->remain_length);
            DBG_PRINT("nonwkup_fifo buffer can be read size: %d.\r\n", (p_nonwkup_fifo->buffer_size - p_nonwkup_fifo->read_length));
            bhy_hif_read_nonwakeup_fifo( &p_bhy2_hub->handle,
                                            &tu32_read_length,
                                            &p_nonwkup_fifo->p_buffer[p_nonwkup_fifo->read_length],
                                            (p_nonwkup_fifo->buffer_size - p_nonwkup_fifo->read_length),
                                            &p_nonwkup_fifo->remain_length);
            p_nonwkup_fifo->read_length += tu32_read_length;
            DBG_PRINT("irq_status: 0x%02X nonwkup_fifo 2 read_length: %d read_length_all: %d remain_length:%d!\r\n", irq_status, tu32_read_length, p_nonwkup_fifo->read_length, p_nonwkup_fifo->remain_length);
        }
        if((BHY_IS_IRQ_ASYNC_STATUS(irq_status) == BHY_IST_MASK_DEBUG) || (p_status_fifo->remain_length))
        {
            p_status_fifo->read_pos = 0;
            bhy_hif_read_status_async( &p_bhy2_hub->handle,
                                        &tu32_read_length,
                                        &p_status_fifo->p_buffer[p_status_fifo->read_length],
                                        (p_status_fifo->buffer_size - p_status_fifo->read_length),
                                        &p_status_fifo->remain_length);
            p_status_fifo->read_length += tu32_read_length;
            DBG_PRINT("BHY_IST_MASK_DEBUG!\r\n");
            DBG_PRINT("bhy_hif_read_status_async()!\r\n");
        }
        if(BHY_IS_IRQ_STATUS(irq_status) == BHY_IST_MASK_STATUS)//will not parse result of command here
        {
            //bhy_hif_read_status_async( handle, &fifo_len, status_fifo_buf, sizeof(status_fifo_buf));
            DBG_PRINT("BHY_IST_MASK_STATUS!\r\n");
        }
        if( BHY_IS_IRQ_RESET(irq_status) == BHY_IST_MASK_RESET_FAULT )
        {
            u16 temp_code = 0;
            //bhy_hif_read_wkup_fifo( handle, &fifo_len, wkup_fifo_buf, sizeof(wkup_fifo_buf));
            bhy2_post_mortem_data_get(  p_bhy2_hub,
                                        &temp_code,
                                        &p_status_fifo->read_length,
                                        p_status_fifo->p_buffer,
                                        p_status_fifo->buffer_size);
            DBG_PRINT("BHY_IST_MASK_RESET_FAULT!\r\n");
        }
        /* clear irq status */
        irq_status = 0;
        /* parse wakeup fifo -s*/
        for( ; p_wkup_fifo->read_pos < p_wkup_fifo->read_length; )
        {
            tmp_sensor_id = p_wkup_fifo->p_buffer[p_wkup_fifo->read_pos];
            DBG_PRINT("wkup frame id: %d, read_pos:%d, buf len:%d!\r\n",
                        tmp_sensor_id, p_wkup_fifo->read_pos, p_wkup_fifo->read_length);
            if(tmp_sensor_id < BHY_SYS_SENSOR_ID_MAX)
            {
                if(p_bhy2_hub->virt_sensor[tmp_sensor_id].frame.frame_parse_func != 0)
                {
                    if(BHY_HIF_E_SUCCESS == p_bhy2_hub->virt_sensor[tmp_sensor_id].frame.frame_parse_func(p_wkup_fifo, p_bhy2_hub))
                    {
                    }
                    else
                    {
                        DBG_PRINT("wakeup frame_parse_func() fail!\r\n");
                        break;
                    }
                }
                else
                {
                    DBG_PRINT("no wkup frame parse function is found!\r\n");
                }
            }
            else
            {
                DBG_PRINT("no sensor id matched in wkup fifo!\r\n");
            }
        }
        if(p_wkup_fifo->read_length)
        {
            p_wkup_fifo->read_length -= p_wkup_fifo->read_pos;
            if(p_wkup_fifo->read_length)
            {
                for( i = 0; i < p_wkup_fifo->read_length; i++)
                {
                    p_wkup_fifo->p_buffer[i]= p_wkup_fifo->p_buffer[p_wkup_fifo->read_pos + i];
                }
            }
        }
        /* parse wakeup fifo -e*/
        /* parse non-wakeup fifo -s*/
        for( ; p_nonwkup_fifo->read_pos < p_nonwkup_fifo->read_length; )
        {
            tmp_sensor_id = p_nonwkup_fifo->p_buffer[p_nonwkup_fifo->read_pos];
            DBG_PRINT("non-wkup frame id: %d, read_pos:%d, buf len:%d!\r\n",
                        tmp_sensor_id, p_nonwkup_fifo->read_pos, p_nonwkup_fifo->read_length);
            if(tmp_sensor_id < BHY_SYS_SENSOR_ID_MAX)
            {
                if(p_bhy2_hub->virt_sensor[tmp_sensor_id].frame.frame_parse_func != 0)
                {
                    if(BHY_HIF_E_SUCCESS == p_bhy2_hub->virt_sensor[tmp_sensor_id].frame.frame_parse_func(p_nonwkup_fifo, p_bhy2_hub))
                    {
                    }
                    else
                    {
                        DBG_PRINT("non-wakeup frame_parse_func() fail!\r\n");
                        break;
                    }
                }
                else
                {
                    DBG_PRINT("no non-wkup frame parse function is found!\r\n");
                }
            }
            else
            {
                DBG_PRINT("no sensor id matched in non-wkup fifo!\r\n");
            }
        }
        if(p_nonwkup_fifo->read_length)
        {
            p_nonwkup_fifo->read_length -= p_nonwkup_fifo->read_pos;
            if(p_nonwkup_fifo->read_length)
            {
                for( i = 0; i < p_nonwkup_fifo->read_length; i++)
                {
                    p_nonwkup_fifo->p_buffer[i]= p_nonwkup_fifo->p_buffer[p_nonwkup_fifo->read_pos + i];
                }
            }
        }
        /* parse non-wakeup fifo -e*/
        /* parse status fifo -s*/
        for( ; p_status_fifo->read_pos < p_status_fifo->read_length; )
        {
            tmp_sensor_id = p_status_fifo->p_buffer[p_status_fifo->read_pos];
            DBG_PRINT("status frame id: %d, read_pos:%d, buf len:%d!\r\n",
                        tmp_sensor_id, p_status_fifo->read_pos, p_status_fifo->read_length);
            if(tmp_sensor_id < BHY_SYS_SENSOR_ID_MAX)
            {
                if(p_bhy2_hub->virt_sensor[tmp_sensor_id].frame.frame_parse_func != 0)
                {
                    if(BHY_HIF_E_SUCCESS == p_bhy2_hub->virt_sensor[tmp_sensor_id].frame.frame_parse_func(p_status_fifo, p_bhy2_hub))
                    {
                    }
                    else
                    {
                        DBG_PRINT("status frame_parse_func() fail!\r\n");
                        break;
                    }
                }
                else
                {
                    DBG_PRINT("no status frame parse function is found!\r\n");
                }
            }
            else
            {
                DBG_PRINT("no sensor id matched in status fifo!\r\n");
            }
        }
        if(p_status_fifo->read_length)
        {
            p_status_fifo->read_length -= p_status_fifo->read_pos;
            if(p_status_fifo->read_length)
            {
                for( i = 0; i < p_status_fifo->read_length; i++)
                {
                    p_status_fifo->p_buffer[i]= p_status_fifo->p_buffer[p_status_fifo->read_pos + i];
                }
            }
        }
        /* parse status fifo -e*/
    }
    return 0;
}

/*!
 * @brief get presented virtual sensors
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[out] p_param: pointer to buffer to get virtual sensor status
 *                      the buffer should be u8*32 bytes
 *
 * @return res: operation result
 */
s16 bhy2_virt_sensor_present_get (bhy2_hub_t* p_bhy2_hub, u8* p_param)
{
    s16 res = BHY_HIF_E_SUCCESS;
    u32 tmp_ret_len = 0;

    res = bhy_hif_read_parameter( &p_bhy2_hub->handle, BHY_PARAM_SYS_VIRT_SENSOR_PRESENT,
                                    p_bhy2_hub->virt_sen_present, sizeof(p_bhy2_hub->virt_sen_present), &tmp_ret_len);

    for(u16 i = 0; i < sizeof(p_bhy2_hub->virt_sen_present); i++)
    {
        if(p_param != NULL)
        {
            p_param[i] = p_bhy2_hub->virt_sen_present[i];
        }
        for(u8 j = 0; j < 8; j++)
        {
            p_bhy2_hub->virt_sensor[i * 8 + j].info.present = ((p_bhy2_hub->virt_sen_present[i] >> j) & 0x01);
        }
        DBG_PRINT("Virt sensor present[%2d]: 0x%02X\r\n", i, p_bhy2_hub->virt_sen_present[i]);
    }
    TST_PRINT("Virtual sensor present:\r\n");
    for(u16 i = 0; i < BHY_SYS_SENSOR_ID_MAX; i++)
    {
        if(p_bhy2_hub->virt_sensor[i].info.present)
        {
            TST_PRINT("Sensor ID: %3d, %s.\r\n", i, (char*)&sensor_table[i]);
            switch(i)
            {
                case BHY_SENSOR_ID_ACC:
                case BHY_SENSOR_ID_ACC_WU:
                case BHY_SENSOR_ID_ACC_PASS:
                case BHY_SENSOR_ID_ACC_RAW:
                case BHY_SENSOR_ID_ACC_RAW_WU:
                case BHY_SENSOR_ID_ACC_BIAS:
                case BHY_SENSOR_ID_GRA:
                case BHY_SENSOR_ID_GRA_WU:
                case BHY_SENSOR_ID_LACC:
                case BHY_SENSOR_ID_LACC_WU:
                case BHY_SENSOR_ID_MAG:
                case BHY_SENSOR_ID_MAG_WU:
                case BHY_SENSOR_ID_MAG_PASS:
                case BHY_SENSOR_ID_MAG_RAW:
                case BHY_SENSOR_ID_MAG_RAW_WU:
                case BHY_SENSOR_ID_MAG_BIAS:
                case BHY_SENSOR_ID_ORI:
                case BHY_SENSOR_ID_ORI_WU:
                case BHY_SENSOR_ID_GYRO:
                case BHY_SENSOR_ID_GYRO_WU:
                case BHY_SENSOR_ID_GYRO_PASS:
                case BHY_SENSOR_ID_GYRO_RAW:
                case BHY_SENSOR_ID_GYRO_RAW_WU:
                case BHY_SENSOR_ID_GYRO_BIAS:
                case BHY_SENSOR_ID_LIGHT:
                case BHY_SENSOR_ID_LIGHT_WU:
                case BHY_SENSOR_ID_BARO:
                case BHY_SENSOR_ID_BARO_WU:
                case BHY_SENSOR_ID_TEMP:
                case BHY_SENSOR_ID_TEMP_WU:
                case BHY_SENSOR_ID_PROX:
                case BHY_SENSOR_ID_PROX_WU:
                case BHY_SENSOR_ID_RV:
                case BHY_SENSOR_ID_RV_WU:
                case BHY_SENSOR_ID_GAMERV:
                case BHY_SENSOR_ID_GAMERV_WU:
                case BHY_SENSOR_ID_GEORV:
                case BHY_SENSOR_ID_GEORV_WU:
                case BHY_SENSOR_ID_HUM:
                case BHY_SENSOR_ID_HUM_WU:
                    p_bhy2_hub->virt_sensor[i].info.type = SENSOR_TYPE_CONTINUOUS_MODE;
                    break;
                /*
                case BHY_SENSOR_ID_ATEMP:
                    break;
                case BHY_SENSOR_ID_ATEMP_WU:
                    break;
                case BHY_SENSOR_ID_HEART:
                    break;
                case BHY_SENSOR_ID_HEART_WU:
                    break;
                */
                case BHY_SENSOR_ID_AR_WU:
                case BHY_SENSOR_ID_SIG_WU:
                case BHY_SENSOR_ID_SIG_HW_WU:
                case BHY_SENSOR_ID_STD:
                case BHY_SENSOR_ID_STD_WU:
                case BHY_SENSOR_ID_STD_HW:
                case BHY_SENSOR_ID_STD_HW_WU:
                case BHY_SENSOR_ID_TILT_WU:
                case BHY_SENSOR_ID_WAKE_WU:
                case BHY_SENSOR_ID_GLANCE_WU:
                case BHY_SENSOR_ID_PICKUP_WU:
                case BHY_SENSOR_ID_ANY_MOTION:
                case BHY_SENSOR_ID_ANY_MOTION_WU:
                    p_bhy2_hub->virt_sensor[i].info.type = SENSOR_TYPE_ON_CHANGE_MODE;
                    break;
                case BHY_SENSOR_ID_GAS:
                case BHY_SENSOR_ID_GAS_WU:
                    p_bhy2_hub->virt_sensor[i].info.type = SENSOR_TYPE_CONTINUOUS_MODE;
                    break;
                case BHY_SENSOR_ID_GPS:
                    p_bhy2_hub->virt_sensor[i].info.type = SENSOR_TYPE_ON_CHANGE_MODE;
                    break;
                case BHY_SENSOR_ID_EXCAMERA:
                    p_bhy2_hub->virt_sensor[i].info.type = SENSOR_TYPE_ON_CHANGE_MODE;
                    break;
            }
        }
    }

    return res;
}

/*!
 * @brief get virtual sensor current configuration
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] sensor_id: virtual sensor id
 * @param[out] virt_sensor_conf: pointer to buffer to get configuration
 *
 * @return res: operation result
 */
s16 bhy2_virt_sensor_cfg_get(bhy2_hub_t* p_bhy2_hub, u8 sensor_id, virt_sensor_conf_t* virt_sensor_conf)
{
    s16 res = 0;
    u32 tmp_ret_len = 0;
    u32 tmp_sample_rate = 0;

    res = bhy_hif_read_parameter( &p_bhy2_hub->handle, BHY_PARAM_SENSOR_CONF_0 + sensor_id,
                                    virt_sensor_conf->buf, sizeof(virt_sensor_conf->buf), &tmp_ret_len);
    tmp_sample_rate = virt_sensor_conf->buf[0] + (virt_sensor_conf->buf[1] << 8)
                    + (virt_sensor_conf->buf[2] << 16) + (virt_sensor_conf->buf[3] << 24);
    virt_sensor_conf->sample_rate = *(float*)&tmp_sample_rate;
    virt_sensor_conf->latency = virt_sensor_conf->buf[4] + (virt_sensor_conf->buf[5] << 8)
                                + (virt_sensor_conf->buf[6] << 16) + (virt_sensor_conf->buf[7] << 24);
    virt_sensor_conf->sensitivity = virt_sensor_conf->buf[8] + (virt_sensor_conf->buf[9] << 8);
    virt_sensor_conf->range = virt_sensor_conf->buf[10] + (virt_sensor_conf->buf[11] << 8);

    return res;
}

/*!
 * @brief set virtual sensor configuration
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] sensor_id: virtual sensor id
 * @param[in] sample_rate: sample rate
 * @param[in] latency: latency
 *
 * @return res: operation result
 */
s16 bhy2_virt_sensor_cfg_set(bhy2_hub_t* p_bhy2_hub, u8 sensor_id, float sample_rate, u32 latency)
{
    s16 res = 0;
    virt_sensor_conf_t tmp_virt_sensor_conf = {0};
    res = bhy_hif_exec_sensor_conf_cmd( &p_bhy2_hub->handle, sensor_id, *(bhy_u32*)&sample_rate, latency);
    bhy2_virt_sensor_cfg_get( p_bhy2_hub, sensor_id, &tmp_virt_sensor_conf);
    /* if odr > 0, record sensor id -s */
    if(sample_rate)
    {
        p_bhy2_hub->virt_sensor[sensor_id].info.id = sensor_id;
    }
    else
    {
        p_bhy2_hub->virt_sensor[sensor_id].info.id = 0;
    }
    /* if odr > 0, record sensor id -e */
    if(BHY_HIF_E_SUCCESS == res)
    {
        p_bhy2_hub->virt_sensor[sensor_id].info.sample_rate = sample_rate; /* Hz */
        p_bhy2_hub->virt_sensor[sensor_id].info.latency = latency; /* ms */
        #if 1
        switch(sensor_id)
        {
            case BHY_SENSOR_ID_ACC:
            case BHY_SENSOR_ID_ACC_WU:
            case BHY_SENSOR_ID_ACC_PASS:
            case BHY_SENSOR_ID_ACC_RAW:
            case BHY_SENSOR_ID_ACC_RAW_WU:
            case BHY_SENSOR_ID_ACC_BIAS:
            case BHY_SENSOR_ID_GRA:
            case BHY_SENSOR_ID_GRA_WU:
            case BHY_SENSOR_ID_LACC:
            case BHY_SENSOR_ID_LACC_WU:
                p_bhy2_hub->virt_sensor[sensor_id].info.range = tmp_virt_sensor_conf.range; /* g */
                p_bhy2_hub->virt_sensor[sensor_id].info.scale_factor = (float)tmp_virt_sensor_conf.range / MAX_SINT16;
                p_bhy2_hub->virt_sensor[sensor_id].info.type = SENSOR_TYPE_CONTINUOUS_MODE;
                break;
            case BHY_SENSOR_ID_MAG:
            case BHY_SENSOR_ID_MAG_WU:
            case BHY_SENSOR_ID_MAG_PASS:
            case BHY_SENSOR_ID_MAG_RAW:
            case BHY_SENSOR_ID_MAG_RAW_WU:
            case BHY_SENSOR_ID_MAG_BIAS:
                p_bhy2_hub->virt_sensor[sensor_id].info.range = tmp_virt_sensor_conf.range; /* uT */
                p_bhy2_hub->virt_sensor[sensor_id].info.scale_factor = (float)tmp_virt_sensor_conf.range / MAX_SINT16;
                p_bhy2_hub->virt_sensor[sensor_id].info.type = SENSOR_TYPE_CONTINUOUS_MODE;
                break;
            case BHY_SENSOR_ID_ORI:
            case BHY_SENSOR_ID_ORI_WU:
                p_bhy2_hub->virt_sensor[sensor_id].info.range = 360; /* degree */
                p_bhy2_hub->virt_sensor[sensor_id].info.scale_factor = (float)360 / MAX_SINT16;
                p_bhy2_hub->virt_sensor[sensor_id].info.type = SENSOR_TYPE_CONTINUOUS_MODE;
                break;
            case BHY_SENSOR_ID_GYRO:
            case BHY_SENSOR_ID_GYRO_WU:
            case BHY_SENSOR_ID_GYRO_PASS:
            case BHY_SENSOR_ID_GYRO_RAW:
            case BHY_SENSOR_ID_GYRO_RAW_WU:
            case BHY_SENSOR_ID_GYRO_BIAS:
                p_bhy2_hub->virt_sensor[sensor_id].info.range = tmp_virt_sensor_conf.range; /* degree */
                p_bhy2_hub->virt_sensor[sensor_id].info.scale_factor = (float)tmp_virt_sensor_conf.range / MAX_SINT16;
                p_bhy2_hub->virt_sensor[sensor_id].info.type = SENSOR_TYPE_CONTINUOUS_MODE;
                break;
            case BHY_SENSOR_ID_LIGHT:
            case BHY_SENSOR_ID_LIGHT_WU:
                p_bhy2_hub->virt_sensor[sensor_id].info.range = tmp_virt_sensor_conf.range; /* lux */
                p_bhy2_hub->virt_sensor[sensor_id].info.scale_factor = (float)1;
                p_bhy2_hub->virt_sensor[sensor_id].info.type = SENSOR_TYPE_CONTINUOUS_MODE;
                break;
            case BHY_SENSOR_ID_BARO:
            case BHY_SENSOR_ID_BARO_WU:
                p_bhy2_hub->virt_sensor[sensor_id].info.range = tmp_virt_sensor_conf.range; /* Pa */
                p_bhy2_hub->virt_sensor[sensor_id].info.scale_factor = (float)1 / 128;  /* 1 LSB = 1/128 Pa */
                p_bhy2_hub->virt_sensor[sensor_id].info.type = SENSOR_TYPE_CONTINUOUS_MODE;
                break;
            case BHY_SENSOR_ID_TEMP:
            case BHY_SENSOR_ID_TEMP_WU:
                p_bhy2_hub->virt_sensor[sensor_id].info.range = tmp_virt_sensor_conf.range; /* centigrade */
                p_bhy2_hub->virt_sensor[sensor_id].info.scale_factor = (float)1 / 100;  /* 1 LSB = 1/100 ¡æ */
                p_bhy2_hub->virt_sensor[sensor_id].info.type = SENSOR_TYPE_CONTINUOUS_MODE;
                break;
            case BHY_SENSOR_ID_PROX:
            case BHY_SENSOR_ID_PROX_WU:
                p_bhy2_hub->virt_sensor[sensor_id].info.range = tmp_virt_sensor_conf.range; /* no unit */
                p_bhy2_hub->virt_sensor[sensor_id].info.scale_factor = (float)1;
                p_bhy2_hub->virt_sensor[sensor_id].info.type = SENSOR_TYPE_CONTINUOUS_MODE;
                break;
            case BHY_SENSOR_ID_RV:
            case BHY_SENSOR_ID_RV_WU:
            case BHY_SENSOR_ID_GAMERV:
            case BHY_SENSOR_ID_GAMERV_WU:
            case BHY_SENSOR_ID_GEORV:
            case BHY_SENSOR_ID_GEORV_WU:
                p_bhy2_hub->virt_sensor[sensor_id].info.range = 1; /* no unit */
                p_bhy2_hub->virt_sensor[sensor_id].info.scale_factor = (float)1 / 16384;/* 1 LSB = 1/16384 */
                p_bhy2_hub->virt_sensor[sensor_id].info.type = SENSOR_TYPE_CONTINUOUS_MODE;
                break;
            case BHY_SENSOR_ID_HUM:
            case BHY_SENSOR_ID_HUM_WU:
                p_bhy2_hub->virt_sensor[sensor_id].info.range = tmp_virt_sensor_conf.range; /* % */
                p_bhy2_hub->virt_sensor[sensor_id].info.scale_factor = (float)1;
                p_bhy2_hub->virt_sensor[sensor_id].info.type = SENSOR_TYPE_CONTINUOUS_MODE;
                break;
            /*
            case BHY_SENSOR_ID_ATEMP:
                break;
            case BHY_SENSOR_ID_ATEMP_WU:
                break;
            case BHY_SENSOR_ID_HEART:
                break;
            case BHY_SENSOR_ID_HEART_WU:
                break;
            */
            case BHY_SENSOR_ID_AR_WU:
            case BHY_SENSOR_ID_SIG_WU:
            case BHY_SENSOR_ID_SIG_HW_WU:
            case BHY_SENSOR_ID_STD:
            case BHY_SENSOR_ID_STD_WU:
            case BHY_SENSOR_ID_STD_HW:
            case BHY_SENSOR_ID_STD_HW_WU:
            case BHY_SENSOR_ID_TILT_WU:
            case BHY_SENSOR_ID_WAKE_WU:
            case BHY_SENSOR_ID_GLANCE_WU:
            case BHY_SENSOR_ID_PICKUP_WU:
            case BHY_SENSOR_ID_ANY_MOTION:
            case BHY_SENSOR_ID_ANY_MOTION_WU:
                p_bhy2_hub->virt_sensor[sensor_id].info.type = SENSOR_TYPE_ON_CHANGE_MODE;
                break;
            case BHY_SENSOR_ID_GAS:
            case BHY_SENSOR_ID_GAS_WU:
                p_bhy2_hub->virt_sensor[sensor_id].info.range = tmp_virt_sensor_conf.range; /* ohm */
                p_bhy2_hub->virt_sensor[sensor_id].info.scale_factor = (float)1;
                p_bhy2_hub->virt_sensor[sensor_id].info.type = SENSOR_TYPE_CONTINUOUS_MODE;
                break;
            case BHY_SENSOR_ID_GPS:
                p_bhy2_hub->virt_sensor[sensor_id].info.type = SENSOR_TYPE_ON_CHANGE_MODE;
                break;
            case BHY_SENSOR_ID_EXCAMERA:
                p_bhy2_hub->virt_sensor[sensor_id].info.range = tmp_virt_sensor_conf.range; /* no unit */
                p_bhy2_hub->virt_sensor[sensor_id].info.scale_factor = (float)1;
                p_bhy2_hub->virt_sensor[sensor_id].info.type = SENSOR_TYPE_ON_CHANGE_MODE;
                break;
        }
        #endif
        bhy2_update_sample_rate( p_bhy2_hub );
        bhy2_update_latency( p_bhy2_hub );
        /* clean softwatchdog here, if no sensors are enabled */
        bhy2_softwatchdog_clean( p_bhy2_hub );
    }

    return res;
}

/*!
 * @brief set virtual sensor sample range
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] sensor_id: virtual sensor id
 * @param[in] range: range to pass in
 *
 * @return res: operation result
 */
s16 bhy2_virt_sensor_range_set(bhy2_hub_t* p_bhy2_hub, u8 sensor_id, u16 range)
{
    s16 res = 0;
    u8 tmp_buf[2] = {0};

    tmp_buf[0] = (bhy_u8)(range & 0xFF);
    tmp_buf[1] = (bhy_u8)((range >> 8) & 0xFF);
    res = bhy_hif_exec_cmd( &p_bhy2_hub->handle, BHY_CMD_CHANGE_RANGE, tmp_buf, sizeof(tmp_buf));

    if(BHY_HIF_E_SUCCESS == res)
    {
        p_bhy2_hub->virt_sensor[sensor_id].info.range = range;
    }
    return res;
}

/*!
 * @brief register virtual sensor callback function
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] sensor_id: virtual sensor id
 * @param[in] frame_parse_func: pointer to virtual sensor parse function
 *
 * @return res: operation result
 */
s16 bhy2_virt_sensor_callback_register(bhy2_hub_t* p_bhy2_hub, u8 sensor_id,
                                s16 (*frame_parse_func)(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub))
{
    s16 res = BHY_HIF_E_HANDLE;

    if((sensor_id > BSX_VIRTUAL_SENSOR_ID_INVALID) && (sensor_id < BHY_SYS_SENSOR_ID_MAX))
    {
        p_bhy2_hub->virt_sensor[sensor_id].frame.frame_parse_func = frame_parse_func;
        res = BHY_HIF_E_SUCCESS;
    }

    return res;
}

/*!
 * @brief get fifo control configuration
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[out] p_fifo_ctrl: pointer to buffer to get fifo control configuration
 *              the buffer size is u32*4bytes
 *
 * @return res: operation result
 */
s16 bhy2_fifo_ctrl_get( bhy2_hub_t* p_bhy2_hub, u32* p_fifo_ctrl)
{
    s16 res = 0;
    u32 tmp_ret_len = 0;
    u8 tmp_buf[16] = {0};
    u32 wkup_fifo_watermark = 0;
    u32 wkup_fifo_size = 0;
    u32 nonwkup_fifo_watermark = 0;
    u32 nonwkup_fifo_size = 0;

    res = bhy_hif_read_parameter( &p_bhy2_hub->handle, BHY_PARAM_FIFO_CTRL, tmp_buf, sizeof(tmp_buf), &tmp_ret_len);
    wkup_fifo_watermark = tmp_buf[0] + (tmp_buf[1] << 8) + (tmp_buf[2] << 16) + (tmp_buf[3] << 24);
    wkup_fifo_size = tmp_buf[4] + (tmp_buf[5] << 8) + (tmp_buf[6] << 16) + (tmp_buf[7] << 24);
    nonwkup_fifo_watermark = tmp_buf[8] + (tmp_buf[9] << 8) + (tmp_buf[10] << 16) + (tmp_buf[11] << 24);
    nonwkup_fifo_size = tmp_buf[12] + (tmp_buf[13] << 8) + (tmp_buf[14] << 16) + (tmp_buf[15] << 24);

    if(p_fifo_ctrl != NULL)
    {
        p_fifo_ctrl[0] = wkup_fifo_watermark;
        p_fifo_ctrl[1] = wkup_fifo_size;
        p_fifo_ctrl[2] = nonwkup_fifo_watermark;
        p_fifo_ctrl[3] = nonwkup_fifo_size;
    }
    DBG_PRINT("wkup watermark: %5d, wkup fifo size:%5d, nonwkup watermark: %5d, nonwkup fifo size:%5d!\r\n",
            wkup_fifo_watermark, wkup_fifo_size, nonwkup_fifo_watermark, nonwkup_fifo_size);

    return res;
}

/*!
 * @brief set wakeup fifo watermark
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] watermark: watermark to pass in
 *
 * @return res: operation result
 */
s16 bhy2_wkup_fifo_watermark_set( bhy2_hub_t* p_bhy2_hub , u32 watermark)
{
    s16 res = BHY_HIF_E_SUCCESS;
    u32 tmp_ret_len = 0;
    u8 tmp_buf[16] = {0};
    u32 tmp_watermark = 0;

    res = bhy_hif_read_parameter( &p_bhy2_hub->handle, BHY_PARAM_FIFO_CTRL, tmp_buf, sizeof(tmp_buf), &tmp_ret_len);

    tmp_buf[0] = (watermark & 0xFF);
    tmp_buf[1] = ((watermark >> 8) & 0xFF);
    tmp_buf[2] = ((watermark >> 16) & 0xFF);
    tmp_buf[3] = ((watermark >> 24) & 0xFF);

    res = bhy_hif_write_parameter( &p_bhy2_hub->handle, BHY_PARAM_FIFO_CTRL, tmp_buf, sizeof(tmp_buf));

    res = bhy_hif_read_parameter( &p_bhy2_hub->handle, BHY_PARAM_FIFO_CTRL, tmp_buf, sizeof(tmp_buf), &tmp_ret_len);
    tmp_watermark = tmp_buf[0] + (tmp_buf[1] << 8) + (tmp_buf[2] << 16) + (tmp_buf[3] << 24);
    if(tmp_watermark != watermark)
    {
        res = BHY_HIF_E_HANDLE;
    }

    return res;
}

/*!
 * @brief get wakeup fifo watermark
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[out] watermark: pointer to get watermark
 *
 * @return res: operation result
 */
s16 bhy2_wkup_fifo_watermark_get( bhy2_hub_t* p_bhy2_hub , u32* watermark)
{
    s16 res = 0;
    u32 tmp_ret_len = 0;
    u8 tmp_buf[16] = {0};
    u32 wkup_fifo_watermark = 0;

    res = bhy_hif_read_parameter( &p_bhy2_hub->handle, BHY_PARAM_FIFO_CTRL, tmp_buf, sizeof(tmp_buf), &tmp_ret_len);
    wkup_fifo_watermark = tmp_buf[0] + (tmp_buf[1] << 8) + (tmp_buf[2] << 16) + (tmp_buf[3] << 24);

    *watermark = wkup_fifo_watermark;

    return res;
}

/*!
 * @brief set non-wakeup fifo watermark
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] watermark: watermark to pass in
 *
 * @return res: operation result
 */
s16 bhy2_nonwkup_fifo_watermark_set( bhy2_hub_t* p_bhy2_hub , u32 watermark)
{
    s16 res = BHY_HIF_E_SUCCESS;
    u32 tmp_ret_len = 0;
    u8 tmp_buf[16] = {0};
    u32 tmp_watermark = 0;

    res = bhy_hif_read_parameter( &p_bhy2_hub->handle, BHY_PARAM_FIFO_CTRL, tmp_buf, sizeof(tmp_buf), &tmp_ret_len);

    tmp_buf[8] = (watermark & 0xFF);
    tmp_buf[9] = ((watermark >> 8) & 0xFF);
    tmp_buf[10] = ((watermark >> 16) & 0xFF);
    tmp_buf[11] = ((watermark >> 24) & 0xFF);

    res = bhy_hif_write_parameter( &p_bhy2_hub->handle, BHY_PARAM_FIFO_CTRL, tmp_buf, sizeof(tmp_buf));

    res = bhy_hif_read_parameter( &p_bhy2_hub->handle, BHY_PARAM_FIFO_CTRL, tmp_buf, sizeof(tmp_buf), &tmp_ret_len);
    tmp_watermark = tmp_buf[0] + (tmp_buf[1] << 8) + (tmp_buf[2] << 16) + (tmp_buf[3] << 24);
    if(tmp_watermark != watermark)
    {
        res = BHY_HIF_E_HANDLE;
    }

    return res;
}

/*!
 * @brief get non-wakeup fifo watermark
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[out] watermark: pointer to get watermark
 *
 * @return res: operation result
 */
s16 bhy2_nonwkup_fifo_watermark_get( bhy2_hub_t* p_bhy2_hub , u32* watermark)
{
    s16 res = 0;
    u32 tmp_ret_len = 0;
    u8 tmp_buf[16] = {0};
    u32 nonwkup_fifo_watermark = 0;

    res = bhy_hif_read_parameter( &p_bhy2_hub->handle, BHY_PARAM_FIFO_CTRL, tmp_buf, sizeof(tmp_buf), &tmp_ret_len);
    nonwkup_fifo_watermark = tmp_buf[8] + (tmp_buf[9] << 8) + (tmp_buf[10] << 16) + (tmp_buf[11] << 24);

    *watermark = nonwkup_fifo_watermark;

    return res;
}

/*!
 * @brief get product id
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[out] p_prod_id: pointer to get product id
 *
 * @return res: operation result
 */
s16 bhy2_product_id_get(bhy2_hub_t* p_bhy2_hub, u8* p_prod_id)
{
    s16 res = BHY_HIF_E_SUCCESS;

    res = bhy_hif_read_product_id( &p_bhy2_hub->handle, &p_bhy2_hub->product_id);
    *p_prod_id = p_bhy2_hub->product_id;
    DBG_PRINT("product id: 0x%02X!\r\n", *p_prod_id);

    return res;
}

/*!
 * @brief get revision id
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[out] p_revision_id: pointer to get revision id
 *
 * @return res: operation result
 */
s16 bhy2_revision_id_get(bhy2_hub_t* p_bhy2_hub, u8* p_revision_id)
{
    bhy_u8 buf;
    s16 ret = BHY_HIF_E_SUCCESS;

    if (!&p_bhy2_hub->handle)
    {
        return BHY_HIF_E_HANDLE;
    }
    ret = bhy_hif_bus_read(&p_bhy2_hub->handle, BHY_REG_REVISION_ID, &buf, sizeof(buf));
    if (ret < 0)
    {
        return BHY_HIF_E_IO;
    }
    *p_revision_id = buf;
    DBG_PRINT("revision id: 0x%02X!\r\n", *p_revision_id);
    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief get rom version
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[out] p_rom_version: pointer to get rom version
 *
 * @return res: operation result
 */
s16 bhy2_rom_version_get(bhy2_hub_t* p_bhy2_hub, u16* p_rom_version)
{
    s16 res = BHY_HIF_E_SUCCESS;

    res = bhy_hif_read_rom_version( &p_bhy2_hub->handle, &p_bhy2_hub->rom_version);
    *p_rom_version = p_bhy2_hub->rom_version;
    DBG_PRINT("rom version: %04d!\r\n", *p_rom_version);

    return res;
}

/*!
 * @brief get ram version
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[out] p_ram_version: pointer to get ram version
 *
 * @return res: operation result
 */
s16 bhy2_ram_version_get(bhy2_hub_t* p_bhy2_hub, u16* p_ram_version)
{
    s16 res = BHY_HIF_E_SUCCESS;

    res = bhy_hif_read_ram_kernel_version( &p_bhy2_hub->handle, &p_bhy2_hub->ram_version);
    *p_ram_version = p_bhy2_hub->ram_version;
    DBG_PRINT("ram version: %04d!\r\n", *p_ram_version);

    return res;
}

/*!
 * @brief upload ram patch to ram
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] p_patch: pointer to ram patch
 * @param[in] length: size of ram patch
 *
 * @return res: operation result
 */
s16 bhy2_patch_upload_to_ram(bhy2_hub_t* p_bhy2_hub, const u8* p_patch, u32 length)
{
    s16 res = BHY_HIF_E_SUCCESS;

    res = bhy_hif_upload_to_ram( &p_bhy2_hub->handle, p_patch, length);
    /* get ram patch pointer & size -s */
    p_bhy2_hub->p_patch = p_patch;
    p_bhy2_hub->patch_size = length;
    /* get ram patch pointer & size -e */
    DBG_PRINT("upload to ram status: %2d!\r\n", res);

    return res;
}

/*!
 * @brief upload ram patch to ram part by part
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] p_buf: pointer to ram patch buffer
 * @param[in] total_len: total size of ram patch
 * @param[in] cur_pos: current write position
 * @param[in] pack_len: packet length
 *
 * @return res: operation result
 */
s16 bhy2_patch_upload_to_ram_partly(bhy2_hub_t* p_bhy2_hub, const u8* p_buf, u32 total_len, u32 cur_pos, u32 pack_len)
{
    s16 res = BHY_HIF_E_SUCCESS;

    res = bhy_hif_upload_to_ram_partly( &p_bhy2_hub->handle, p_buf, total_len, cur_pos, pack_len);
    /* get ram patch pointer & size -s */
    p_bhy2_hub->p_patch = p_buf;
    p_bhy2_hub->patch_size = total_len;
    /* get ram patch pointer & size -e */
    DBG_PRINT("upload to ram status: %2d!\r\n", res);

    return res;
}

/*!
 * @brief boot hub from ram
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 *
 * @return res: operation result
 */
s16 bhy2_patch_boot_from_ram(bhy2_hub_t* p_bhy2_hub)
{
    s16 res = BHY_HIF_E_SUCCESS;

    res = bhy_hif_boot_program_ram( &p_bhy2_hub->handle);
    DBG_PRINT("boot from ram status: %2d!\r\n", res);

    return res;
}

/*!
 * @brief erase qspi flash
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 *
 * @return res: operation result
 */
s16 bhy2_patch_erase_flash(bhy2_hub_t* p_bhy2_hub)
{
    s16 res = BHY_HIF_E_SUCCESS;
    u8 tmp_buf[256] = {0};
    u32 tmp_ret_len = 0;

    res = bhy_hif_erase_flash( &p_bhy2_hub->handle, tmp_buf, 256, &tmp_ret_len );

    DBG_PRINT("erase flash status: %2d!\r\n", res);

    return res;
}

/*!
 * @brief upload ram patch to qspi flash
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] p_patch: pointer to ram patch
 * @param[in] length: size of ram patch
 *
 * @return res: operation result
 */
s16 bhy2_patch_upload_to_flash(bhy2_hub_t* p_bhy2_hub, const u8* p_patch, u32 length)
{
    s16 res = BHY_HIF_E_SUCCESS;
    u8 tmp_buf[20] = {0};
    u32 tmp_ret_len = 0;
    res = bhy_hif_write_flash( &p_bhy2_hub->handle, p_patch, length, tmp_buf, sizeof(tmp_buf), &tmp_ret_len);

    /* get ram patch pointer & size -s */
    p_bhy2_hub->p_patch = p_patch;
    p_bhy2_hub->patch_size = length;
    /* get ram patch pointer & size -e */
    DBG_PRINT("upload to flash status: %2d, patch size: %d !\r\n", res, length);

    return res;
}

/*!
 * @brief upload ram patch to ram part by part
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] p_buf: pointer to ram patch buffer
 * @param[in] total_len: total size of ram patch
 * @param[in] cur_pos: current write position
 * @param[in] pack_len: packet length
 *
 * @return res: operation result
 */
s16 bhy2_patch_upload_to_flash_partly(bhy2_hub_t* p_bhy2_hub, const u8* p_buf, u32 total_len, u32 cur_pos, u32 pack_len)
{
    s16 res = BHY_HIF_E_SUCCESS;

    res = bhy_hif_upload_to_flash_partly( &p_bhy2_hub->handle, p_buf, total_len, cur_pos, pack_len);
    /* get ram patch pointer & size -s */
    p_bhy2_hub->p_patch = p_buf;
    p_bhy2_hub->patch_size = total_len;
    /* get ram patch pointer & size -e */
    DBG_PRINT("upload to flash status: %2d!\r\n", res);

    return res;
}

/*!
 * @brief boot from qspi flash
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 *
 * @return res: operation result
 */
s16 bhy2_patch_boot_from_flash(bhy2_hub_t* p_bhy2_hub)
{
    s16 res = BHY_HIF_E_SUCCESS;

    res = bhy_hif_boot_from_flash( &p_bhy2_hub->handle);
    DBG_PRINT("boot from flash status: %2d!\r\n", res);

    return res;
}

/*!
 * @brief set host interrupt control register
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] param: param to set
 *
 * @return res: operation result
 */
s16 bhy2_host_interrupt_ctrl_set(bhy2_hub_t* p_bhy2_hub, u8 param)
{
    s16 res = BHY_HIF_E_SUCCESS;
    res = bhy_hif_bus_write( &p_bhy2_hub->handle, BHY_REG_HOST_INTERRUPT_CTRL, &param, 1);
    p_bhy2_hub->reg_conf_chk.bit.host_interrupt_ctrl_bit = 1;
    p_bhy2_hub->reg_conf_save.reg.host_interrupt_ctrl = param;
    return res;
}

/*!
 * @brief set host interface control register
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] param: param to set
 *
 * @return res: operation result
 */
s16 bhy2_host_interface_ctrl_set(bhy2_hub_t* p_bhy2_hub, u8 param)
{
    s16 res = BHY_HIF_E_SUCCESS;
    res = bhy_hif_bus_write( &p_bhy2_hub->handle, BHY_REG_HOST_INTERFACE_CTRL, &param, 1);
    p_bhy2_hub->reg_conf_chk.bit.host_interface_ctrl_bit = 1;
    p_bhy2_hub->reg_conf_save.reg.host_interface_ctrl = param;
    return res;
}

/*!
 * @brief configure output timestamp in fifo
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] param: param to set
 *              the value should be: BHY_FIFO_FORMAT_CTRL_ENABLE_TIMPSTAMP          (0)
 *                                   BHY_FIFO_FORMAT_CTRL_DISABLE_TIMPSTAMP         (1 << 0)
 *                                   BHY_FIFO_FORMAT_CTRL_DISABLE_FULL_TIMPSTAMP     (1 << 1)
 *
 * @return res: operation result
 */
s16 bhy2_timestamp_output_ctrl_set(bhy2_hub_t* p_bhy2_hub, u8 param)
{
    s16 res = BHY_HIF_E_SUCCESS;
    u8 tmp_buf[4] = {0};

    tmp_buf[0] = param & 0x03;
    //tmp_buf[0] = 0x0F;
    res = bhy_hif_exec_cmd( &p_bhy2_hub->handle, BHY_CMD_FIFO_FORMAT_CTRL, tmp_buf, sizeof(tmp_buf));
    return res;
}

/*!
 * @brief configure hardware timestamp
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] param: param to set
 *
 * @return res: operation result
 */
s16 bhy2_hw_timestamp_ctrl_set (bhy2_hub_t* p_bhy2_hub, u8 param)
{
    s16 res = BHY_HIF_E_SUCCESS;
    u8 tmp_buf[1] = {0};
    res = bhy_hif_bus_read( &p_bhy2_hub->handle, BHY_REG_EV_TIME_REQ, tmp_buf, 1);
    if(param == 1)
    {
        tmp_buf[0] = tmp_buf[0] | 0x01;
    }
    else
    {
        tmp_buf[0] = tmp_buf[0] & 0xFE;
    }
    res = bhy_hif_bus_write( &p_bhy2_hub->handle, BHY_REG_EV_TIME_REQ, tmp_buf, 1);
    p_bhy2_hub->reg_conf_chk.bit.event_irq_request_bit = 1;
    p_bhy2_hub->reg_conf_save.reg.event_irq_request = tmp_buf[0];

    return res;
}

/*!
 * @brief get hardware timestamp configuration
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[out] p_param: pointer to get configuration
 *
 * @return res: operation result
 */
s16 bhy2_hw_timestamp_ctrl_get (bhy2_hub_t* p_bhy2_hub, u8* p_param)
{
    s16 res = BHY_HIF_E_SUCCESS;
    u8 tmp_buf[1] = {0};
    res = bhy_hif_bus_read( &p_bhy2_hub->handle, BHY_REG_EV_TIME_REQ, tmp_buf, 1);
    *p_param = tmp_buf[0] & 0x01;

    return res;
}

/*!
 * @brief get hardware timestamp
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[out] p_timestamp: pointer to get timestamp
 *
 * @return res: operation result
 */
s16 bhy2_hw_timestamp_get (bhy2_hub_t* p_bhy2_hub, u64* p_timestamp)
{
    s16 res = BHY_HIF_E_SUCCESS;
    u64 tmp_data = 0;

    res = bhy_hif_read_hw_timestamp( &p_bhy2_hub->handle, &tmp_data);
    *p_timestamp = tmp_data;

    return res;
}

/*!
 * @brief get fpga version
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[out] p_param: pointer to get fpga version
 *
 * @return res: operation result
 */
s16 bhy2_fpga_version_get (bhy2_hub_t* p_bhy2_hub, u8* p_param)
{
    s16 res = BHY_HIF_E_SUCCESS;
    u8 tmp_buf[1] = {0};
    res = bhy_hif_bus_read( &p_bhy2_hub->handle, BHY_REG_EV_TIME_REQ, tmp_buf, 1);
    *p_param = tmp_buf[0] & 0xFE;
    DBG_PRINT("fpga version: 0x%02X!\r\n", *p_param);
    return res;
}

/*!
 * @brief set host control register
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] param: data to set
 *
 * @return res: operation result
 */
s16 bhy2_host_ctrl_set (bhy2_hub_t* p_bhy2_hub, u8 param)
{
    s16 res = BHY_HIF_E_SUCCESS;

    res = bhy_hif_bus_write( &p_bhy2_hub->handle, BHY_REG_HOST_CTRL, &param, 1);

    p_bhy2_hub->reg_conf_chk.bit.host_ctrl_bit = 1;
    p_bhy2_hub->reg_conf_save.reg.host_ctrl = param;

    return res;
}

/*!
 * @brief get host control register
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[out] p_param: pointer to register data
 *
 * @return res: operation result
 */
s16 bhy2_host_ctrl_get (bhy2_hub_t* p_bhy2_hub, u8* p_param)
{
    s16 res = BHY_HIF_E_SUCCESS;
    u8 tmp_buf[1] = {0};
    res = bhy_hif_bus_read( &p_bhy2_hub->handle, BHY_REG_HOST_CTRL, tmp_buf, 1);
    *p_param = tmp_buf[0];

    return res;
}



/*!
 * @brief dump registers' data
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] p_conf: pointer to soft-passthrough configuration
 * @param[in] reg: register to operate(read/write)
 * @param[in] trans_num: number of data to transfer(read/write)
 * @param[in,out] p_buf: pointer to buffer to operate(read/write)
 *
 * @return res: operation result
 */
s16 bhy2_soft_passthrough_operation (bhy2_hub_t* p_bhy2_hub, soft_passthrough_conf_t* p_conf,
                                    u8 reg, u8 trans_num, u8* p_buf)
{
    u32 tmp_ret_len = 0;
    s16 res = BHY_HIF_E_SUCCESS;
    u8* p_tmp_buf = 0;
    p_conf->conf.trans_count = trans_num;
    p_conf->conf.reg = reg;
    if(p_conf->conf.direction == SPASS_READ)
    {
        u8 tmp_rd_buf_size = 0;
        if((9 + trans_num) % 4)
        {
            tmp_rd_buf_size = ((9 + trans_num) / 4 + 1) * 4;
        }
        else
        {
            tmp_rd_buf_size = (9 + trans_num);
        }
        p_tmp_buf = malloc(tmp_rd_buf_size);
        res = bhy_hif_exec_soft_passthrough( &p_bhy2_hub->handle, p_conf->data, 8, p_tmp_buf, tmp_rd_buf_size, &tmp_ret_len);
        memcpy(p_buf, &p_tmp_buf[9], trans_num);
        free(p_tmp_buf);
    }
    else
    {
        u8 tmp_wr_buf_size = 0;
        if((8 + trans_num) % 4)
        {
            tmp_wr_buf_size = ((8 + trans_num) / 4 + 1) * 4;
        }
        else
        {
            tmp_wr_buf_size = (8 + trans_num);
        }
        u8 tmp_rd_buf[12] = {0};
        p_tmp_buf = malloc(8 + trans_num);
        memcpy(p_tmp_buf, p_conf->data, 8);
        memcpy(&p_tmp_buf[8], p_buf, trans_num);
        res = bhy_hif_exec_soft_passthrough( &p_bhy2_hub->handle, p_tmp_buf, tmp_wr_buf_size, tmp_rd_buf, 12, &tmp_ret_len);
        free(p_tmp_buf);
    }
    return res;
}

/*!
 * @brief flush specific virtual sensor data in fifos
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] sensor_id: virtual sensor id
 *
 * @return res: operation result
 */
s16 bhy2_fifo_flush(bhy2_hub_t* p_bhy2_hub, u8 sensor_id)
{
    s16 res = BHY_HIF_E_SUCCESS;
    res = bhy_hif_write_fifo_flush( &p_bhy2_hub->handle, sensor_id);
    return res;
}

/*!
 * @brief reset sensor hub,
 *          after reset, the hub is in boot mode and need to upload ram patch to it again
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 *
 * @return res: operation result
 */
s16 bhy2_hub_reset( bhy2_hub_t* p_bhy2_hub )
{
    s16 res = BHY_HIF_E_SUCCESS;
    res = bhy_hif_reset( &p_bhy2_hub->handle );
    DBG_PRINT ("Sensor hub reset.\r\n");
    return res;
}

/*!
 * @brief requset physical sensor to do self-test.
 * physical sensor id:
 *                      BHY_PHYS_SENSOR_ID_ACC,
 *                      BHY_PHYS_SENSOR_ID_MAG,
 *                      BHY_PHYS_SENSOR_ID_GYRO,
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] sensor_id: Sensor ID for the sensor that needed to do self test
 * @param[out] ret_status: Returned completion status
 * @param[out] offset: Returned self-test offset the the 3 axis if applicable
 *
 * @return res: operation result
 */
s16 bhy2_do_self_test( bhy2_hub_t* p_bhy2_hub, u8 sensor_id, u8 *ret_status, s16 *offset )
{
    s16 res = BHY_HIF_E_SUCCESS;

    DBG_PRINT ("do self-test\r\n");
    res = bhy_hif_do_self_test(&p_bhy2_hub->handle, sensor_id, ret_status, offset);
    switch(sensor_id)
    {
        case BHY_PHYS_SENSOR_ID_ACC:
            DBG_PRINT ("Accelerometer.\r\n");
            break;
        case BHY_PHYS_SENSOR_ID_MAG:
            DBG_PRINT ("Magnetometer.\r\n");
            break;
        case BHY_PHYS_SENSOR_ID_GYRO:
            DBG_PRINT ("Gyroscope.\r\n");
            break;
        default:
            DBG_PRINT ("Unknown physical sensor id.\r\n");
            break;
    }
    DBG_PRINT ("Offset value: %d %d %d.\r\n", offset[0], offset[1], offset[2]);
    switch(*ret_status)
    {
        case SELFTEST_PASSED:
            DBG_PRINT ("Self-test result: passed.\r\n");
            break;
        case SELFTEST_X_FAILED:
            DBG_PRINT ("Self-test result: x axis failed.\r\n");
            break;
        case SELFTEST_Y_FAILED:
            DBG_PRINT ("Self-test result: y axis failed.\r\n");
            break;
        case SELFTEST_X_Y_FAILED:
            DBG_PRINT ("Self-test result: x&y axis failed.\r\n");
            break;
        case SELFTEST_Z_FAILED:
            DBG_PRINT ("Self-test result: z axis failed.\r\n");
            break;
        case SELFTEST_X_Z_FAILED:
            DBG_PRINT ("Self-test result: x&z axis failed.\r\n");
            break;
        case SELFTEST_Y_Z_FAILED:
            DBG_PRINT ("Self-test result: y&z axis failed.\r\n");
            break;
        case SELFTEST_X_Y_Z_FAILED:
            DBG_PRINT ("Self-test result: x&y&z axis failed.\r\n");
            break;
        case SELFTEST_UNSUPPORTED:
            DBG_PRINT ("Self-test result: doesn't implement.\r\n");
            break;
        case SELFTEST_NO_DEVICE:
            DBG_PRINT ("Self-test result: unknow physical sensor id.\r\n");
            break;
        default:
            DBG_PRINT ("Self-test result: unknow error.\r\n");
            break;
    }

    return res;
}

/*!
 * @brief requset physical sensor to do fast offset calibration.
 * physical sensor id:
 *                      BHY_PHYS_SENSOR_ID_ACC,
 *                      BHY_PHYS_SENSOR_ID_MAG,
 *                      BHY_PHYS_SENSOR_ID_GYRO,
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] sensor_id: Sensor ID for the sensor that needed to do self test
 * @param[out] ret_status: Returned completion status
 * @param[out] offset: Returned self-test offset the the 3 axis if applicable
 *
 * @return res: operation result
 */
s16 bhy2_do_foc( bhy2_hub_t* p_bhy2_hub, u8 sensor_id, u8 *ret_status, s16 *offset )
{
    s16 res = BHY_HIF_E_SUCCESS;

    DBG_PRINT ("do foc\r\n");
    res = bhy_hif_do_foc(&p_bhy2_hub->handle, sensor_id, ret_status, offset);
    switch(sensor_id)
    {
        case BHY_PHYS_SENSOR_ID_ACC:
            DBG_PRINT ("Accelerometer.\r\n");
            break;
        case BHY_PHYS_SENSOR_ID_MAG:
            DBG_PRINT ("Magnetometer.\r\n");
            break;
        case BHY_PHYS_SENSOR_ID_GYRO:
            DBG_PRINT ("Gyroscope.\r\n");
            break;
        default:
            DBG_PRINT ("Unknown physical sensor id.\r\n");
            break;
    }

    DBG_PRINT ("Offset value: %d %d %d.\r\n", offset[0], offset[1], offset[2]);
    switch(*ret_status)
    {
        case FOC_RESULT_SUCCESS:
            DBG_PRINT ("FOC result: success.\r\n");
            break;
        case FOC_RESULT_FAIL:
            DBG_PRINT ("FOC result: failed.\r\n");
            break;
        case FOC_RESULT_ERR_UNKNOWN:
        default:
            DBG_PRINT ("FOC result: unknown error.\r\n");
            break;
    }

    return res;
}

/*!
 * @brief set orientation matrix to physical sensor.
 * physical sensor id:
 *                      BHY_PHYS_SENSOR_ID_ACC,
 *                      BHY_PHYS_SENSOR_ID_MAG,
 *                      BHY_PHYS_SENSOR_ID_GYRO,
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] sensor_id: Sensor ID for the sensor that needed to do self test
 * @param[in] matrix: matrix buffer
 * @param[in] matrix_len: fix value of 9
 *
 * @return res: operation result
 */
s16 bhy2_orientation_matrix_set( bhy2_hub_t* p_bhy2_hub, u8 sensor_id, s8 *matrix, u8 matrix_len )
{
    u8 tmp_matrix_buf[8];
    s16 res = BHY_HIF_E_SUCCESS;

    DBG_PRINT ("Set orientation matrix.\r\n");

    if(matrix_len < 9)
    {
        DBG_PRINT ("Error value of matrix_len.\r\n");
        res = BHY_HIF_E_INVAL;
    }
    else
    {
        memset(tmp_matrix_buf, 0, sizeof(tmp_matrix_buf));
        tmp_matrix_buf[0] = (matrix[0] & 0x0F) | ((matrix[1] << 4) & 0xF0);
        tmp_matrix_buf[1] = (matrix[2] & 0x0F) | ((matrix[3] << 4) & 0xF0);
        tmp_matrix_buf[2] = (matrix[4] & 0x0F) | ((matrix[5] << 4) & 0xF0);
        tmp_matrix_buf[3] = (matrix[6] & 0x0F) | ((matrix[7] << 4) & 0xF0);
        tmp_matrix_buf[4] = (matrix[8] & 0x0F) ;

        res = bhy_hif_write_orientation_matrix(&p_bhy2_hub->handle, sensor_id, tmp_matrix_buf, 8);

        switch(sensor_id)
        {
            case BHY_PHYS_SENSOR_ID_ACC:
                DBG_PRINT ("Accelerometer.\r\n");
                break;
            case BHY_PHYS_SENSOR_ID_MAG:
                DBG_PRINT ("Magnetometer.\r\n");
                break;
            case BHY_PHYS_SENSOR_ID_GYRO:
                DBG_PRINT ("Gyroscope.\r\n");
                break;
            default:
                DBG_PRINT ("Unknown physical sensor id.\r\n");
                break;
        }
        DBG_PRINT ("Matrix : C0 C1 C2 C3 C4 C5 C6 C7 C8.\r\n");
        DBG_PRINT ("Value:   %2d %2d %2d %2d %2d %2d %2d %2d %2d.\r\n",
                    matrix[0], matrix[1], matrix[2], matrix[3], matrix[4], matrix[5], matrix[6], matrix[7], matrix[8]);
        DBG_PRINT ("Set Value: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X.\r\n",
                    tmp_matrix_buf[0], tmp_matrix_buf[1], tmp_matrix_buf[2], tmp_matrix_buf[3], tmp_matrix_buf[4]);
    }
    return res;
}

/*!
 * @brief get orientation matrix to physical sensor.
 * physical sensor id:
 *                      BHY_PHYS_SENSOR_ID_ACC,
 *                      BHY_PHYS_SENSOR_ID_MAG,
 *                      BHY_PHYS_SENSOR_ID_GYRO,
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] sensor_id: Sensor ID for the sensor that needed to do self test
 * @param[out] p_matrix: matrix buffer
 * @param[out] p_matrix_len: fix value of 9
 *
 * @return res: operation result
 */
s16 bhy2_orientation_matrix_get( bhy2_hub_t* p_bhy2_hub, u8 sensor_id, s8 *p_matrix, u8* p_matrix_len )
{
    s16 res = BHY_HIF_E_SUCCESS;
    union bhy_phys_sensor_info tmp_phy_sen_info;

    DBG_PRINT ("Get orientation matrix.\r\n");

    bhy_hif_read_phys_sensor_info( &p_bhy2_hub->handle, sensor_id, &tmp_phy_sen_info);

    p_matrix[0] = (s8)((tmp_phy_sen_info.info.orientation_matrix[0] & 0x0F) << 4) >> 4;
    p_matrix[1] = (s8)(tmp_phy_sen_info.info.orientation_matrix[0] & 0xF0) >> 4;
    p_matrix[2] = (s8)((tmp_phy_sen_info.info.orientation_matrix[1] & 0x0F) << 4) >> 4;
    p_matrix[3] = (s8)(tmp_phy_sen_info.info.orientation_matrix[1] & 0xF0) >> 4;
    p_matrix[4] = (s8)((tmp_phy_sen_info.info.orientation_matrix[2] & 0x0F) << 4) >> 4;
    p_matrix[5] = (s8)(tmp_phy_sen_info.info.orientation_matrix[2] & 0xF0) >> 4;
    p_matrix[6] = (s8)((tmp_phy_sen_info.info.orientation_matrix[3] & 0x0F) << 4) >> 4;
    p_matrix[7] = (s8)(tmp_phy_sen_info.info.orientation_matrix[3] & 0xF0) >> 4;
    p_matrix[8] = (s8)((tmp_phy_sen_info.info.orientation_matrix[4] & 0x0F) << 4) >> 4;

    *p_matrix_len = 9;

    DBG_PRINT ("Matrix : C0 C1 C2 C3 C4 C5 C6 C7 C8.\r\n");
    DBG_PRINT ("Value:   %2d %2d %2d %2d %2d %2d %2d %2d %2d.\r\n",
                p_matrix[0], p_matrix[1], p_matrix[2], p_matrix[3],
                p_matrix[4], p_matrix[5], p_matrix[6], p_matrix[7], p_matrix[8]);

    return res;
}

/*!
 * @brief get orientation matrix to physical sensor.
 * physical sensor id:
 *                      BHY_PHYS_SENSOR_ID_ACC,
 *                      BHY_PHYS_SENSOR_ID_MAG,
 *                      BHY_PHYS_SENSOR_ID_GYRO,
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] p_buf: pointer of a buffer to get sic matrix
 * @param[in] buf_len: length of buffer
 * @param[out] p_ret_len: actual length of sic matrix
 *
 * @return res: operation result
 */
s16 bhy2_sic_matrix_get( bhy2_hub_t* p_bhy2_hub, u8 *p_buf, u16 buf_len, u32* p_ret_len )
{
    s16 res = BHY_HIF_E_SUCCESS;

    DBG_PRINT ("Get sic matrix.\r\n");

    res = bhy_hif_read_bsx_state(&p_bhy2_hub->handle, BHY_PARAM_SIC, p_buf, buf_len, p_ret_len);

    DBG_PRINT ("total length: %d.\r\n", *p_ret_len);

    for(u16 i = 0; i < *p_ret_len; i++)
    {
        if((i % 10) == 0)
        {
            DBG_PRINT ("%4d", p_buf[i]);
            if((i + 1) == *p_ret_len)
            {
                DBG_PRINT_TEXT ("\r\n");
            }
        }
        else
        {
            DBG_PRINT_TEXT ("%4d", p_buf[i]);
            if(((i + 1) % 10) == 0)
            {
                DBG_PRINT_TEXT ("\r\n");
            }
            else if((i + 1) == *p_ret_len)
            {
                DBG_PRINT_TEXT ("\r\n");
            }
        }
    }

    return res;
}

/*!
 * @brief get orientation matrix to physical sensor.
 * physical sensor id:
 *                      BHY_PHYS_SENSOR_ID_ACC,
 *                      BHY_PHYS_SENSOR_ID_MAG,
 *                      BHY_PHYS_SENSOR_ID_GYRO,
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] p_buf: pointer of a buffer to set sic matrix
 * @param[in] buf_len: length of sic matrix
 *
 * @return res: operation result
 */
s16 bhy2_sic_matrix_set( bhy2_hub_t* p_bhy2_hub, u8 *p_buf, u16 buf_len )
{
    s16 res = BHY_HIF_E_SUCCESS;

    DBG_PRINT ("Set sic matrix.\r\n");

    res = bhy_hif_write_bsx_state(&p_bhy2_hub->handle, BHY_PARAM_SIC, p_buf, buf_len);

    DBG_PRINT ("total length: %d.\r\n", buf_len);

    for(u16 i = 0; i < buf_len; i++)
    {
        if((i % 10) == 0)
        {
            DBG_PRINT ("%4d", p_buf[i]);
            if((i + 1) == buf_len)
            {
                DBG_PRINT_TEXT ("\r\n");
            }
        }
        else
        {
            DBG_PRINT_TEXT ("%4d", p_buf[i]);
            if(((i + 1) % 10) == 0)
            {
                DBG_PRINT_TEXT ("\r\n");
            }
            else if((i + 1) == buf_len)
            {
                DBG_PRINT_TEXT ("\r\n");
            }
        }
    }

    return res;
}

/*!
 * @brief get orientation matrix to physical sensor.
 * physical sensor id:
 *                      BHY_PHYS_SENSOR_ID_ACC,
 *                      BHY_PHYS_SENSOR_ID_MAG,
 *                      BHY_PHYS_SENSOR_ID_GYRO,
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] sensor_id: physical sensor id
 * @param[in] p_buf: pointer of a buffer to get sic matrix
 * @param[in] buf_len: length of buffer
 * @param[out] p_ret_len: actual length of sic matrix
 *
 * @return res: operation result
 */
s16 bhy2_calibration_profile_get( bhy2_hub_t* p_bhy2_hub, u8 sensor_id, u8 *p_buf, u16 buf_len, u32* p_ret_len )
{
    s16 res = BHY_HIF_E_SUCCESS;

    DBG_PRINT ("Get calibration profile.\r\n");

    res = bhy_hif_read_bsx_state(&p_bhy2_hub->handle, BHY_PARAM_CALIB_STATE_BASE | sensor_id, p_buf, buf_len, p_ret_len);

    DBG_PRINT ("total length: %d.\r\n", *p_ret_len);

    for(u16 i = 0; i < *p_ret_len; i++)
    {
        if((i % 10) == 0)
        {
            DBG_PRINT ("%4d", p_buf[i]);
            if((i + 1) == *p_ret_len)
            {
                DBG_PRINT_TEXT ("\r\n");
            }
        }
        else
        {
            DBG_PRINT_TEXT ("%4d", p_buf[i]);
            if(((i + 1) % 10) == 0)
            {
                DBG_PRINT_TEXT ("\r\n");
            }
            else if((i + 1) == *p_ret_len)
            {
                DBG_PRINT_TEXT ("\r\n");
            }
        }
    }

    return res;
}

/*!
 * @brief set calibration profile to physical sensor.
 * physical sensor id:
 *                      BHY_PHYS_SENSOR_ID_ACC,
 *                      BHY_PHYS_SENSOR_ID_MAG,
 *                      BHY_PHYS_SENSOR_ID_GYRO,
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] sensor_id: physical sensor id
 * @param[in] p_buf: pointer of a buffer to set sic matrix
 * @param[in] buf_len: length of sic matrix
 *
 * @return res: operation result
 */
s16 bhy2_calibration_profile_set( bhy2_hub_t* p_bhy2_hub, u8 sensor_id, u8 *p_buf, u16 buf_len )
{
    s16 res = BHY_HIF_E_SUCCESS;

    DBG_PRINT ("Set calibration profile.\r\n");

    res = bhy_hif_write_bsx_state(&p_bhy2_hub->handle, BHY_PARAM_CALIB_STATE_BASE | sensor_id, p_buf, buf_len);

    DBG_PRINT ("total length: %d.\r\n", buf_len);

    for(u16 i = 0; i < buf_len; i++)
    {
        if((i % 10) == 0)
        {
            DBG_PRINT ("%4d", p_buf[i]);
            if((i + 1) == buf_len)
            {
                DBG_PRINT_TEXT ("\r\n");
            }
        }
        else
        {
            DBG_PRINT_TEXT ("%4d", p_buf[i]);
            if(((i + 1) % 10) == 0)
            {
                DBG_PRINT_TEXT ("\r\n");
            }
            else if((i + 1) == buf_len)
            {
                DBG_PRINT_TEXT ("\r\n");
            }
        }
    }

    return res;
}

/*!
 * @brief get post mortem data.
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[out] p_code: error code
 * @param[out] p_ret_len: actual length of post mortem data
 * @param[out] p_buf: pointer of buffer to receive post mortem data.
 * @param[out] buf_len: length of buffer
 *
 * @return res: operation result
 */
s16 bhy2_post_mortem_data_get( bhy2_hub_t* p_bhy2_hub, u16 *p_code, u32 *p_ret_len, u8 *p_buf, u32 buf_len)
{
    s16 res = BHY_HIF_E_SUCCESS;

    TST_PRINT ("Get post mortem data.\r\n");

    res = bhy_hif_read_post_mortem( &p_bhy2_hub->handle, p_code, p_ret_len, p_buf, buf_len);

    TST_PRINT ("Error code: 0x%02X, total length: %d.\r\n", *p_code, *p_ret_len);

    for(u16 i = 0; i < *p_ret_len; i++)
    {
        if((i % 10) == 0)
        {
            TST_PRINT (" 0x%02X", p_buf[i]);
            if((i + 1) == buf_len)
            {
                TST_PRINT ("\r\n");
            }
            /* must add delay here to let printf send out message and clear up buffer for next message */
            bhy2_delay_ms(5, 0);
        }
        else
        {
            TST_PRINT_TEXT (" 0x%02X", p_buf[i]);
            if(((i + 1) % 10) == 0)
            {
                TST_PRINT_TEXT ("\r\n");
            }
            else if((i + 1) == buf_len)
            {
                TST_PRINT_TEXT ("\r\n");
            }
        }
    }
    TST_PRINT_TEXT ("\r\n");
    return res;
}

/*!
 * @brief trigger fatal error.
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] error_type: error type
 * @param[in] flag: flag for error type
 *
 * @return res: operation result
 */
s16 bhy2_trigger_fatal_error( bhy2_hub_t* p_bhy2_hub, u8 error_type, u8 flag)
{
    s16 res = BHY_HIF_E_SUCCESS;
    u8 tmp_buf[4] = {0};
    TST_PRINT ("trigger fatal error.\r\n");

    tmp_buf[0] = error_type;
    tmp_buf[1] = flag;
    res = bhy_hif_write_parameter( &p_bhy2_hub->handle, BHY_CMD_TRIG_FATAL_ERROR, tmp_buf, sizeof(tmp_buf));

    TST_PRINT ("Error type: 0x%02X, flag: %d.\r\n", error_type, flag);

    return res;
}

/*!
 * @brief init bhy2 api operation
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] bus_read_func: pointer to read function
 * @param[in] bus_write_func: pointer to write function
 * @param[in] delay_us_func: pointer to delay function
 * @param[in] private_data: reserved for further use
 *
 * @return res: operation result
 */
s16 bhy2_hub_init( bhy2_hub_t* p_bhy2_hub ,
                    int (*bus_read_func)(bhy_u8, bhy_u8 *, u32, void *),
                    int (*bus_write_func)(bhy_u8, const bhy_u8 *, u32, void *),
                    void (*delay_us_func)(u32, void *),
                    void *private_data)
{
    s16 res = BHY_HIF_E_SUCCESS;
    u16 i = 0;
    /* initial bhy2 hub obj -s*/
    memset( p_bhy2_hub, 0, sizeof(bhy2_hub_t));

    memset( wkup_fifo_buffer, 0, sizeof(wkup_fifo_buffer));
    p_bhy2_hub->fifos.wkup_fifo.read_pos = 0;
    p_bhy2_hub->fifos.wkup_fifo.read_length = 0;
    p_bhy2_hub->fifos.wkup_fifo.p_buffer = wkup_fifo_buffer;
    p_bhy2_hub->fifos.wkup_fifo.buffer_size = sizeof(wkup_fifo_buffer);
    memset( nonwkup_fifo_buffer, 0, sizeof(nonwkup_fifo_buffer));
    p_bhy2_hub->fifos.nonwkup_fifo.read_pos = 0;
    p_bhy2_hub->fifos.nonwkup_fifo.read_length = 0;
    p_bhy2_hub->fifos.nonwkup_fifo.p_buffer = nonwkup_fifo_buffer;
    p_bhy2_hub->fifos.nonwkup_fifo.buffer_size = sizeof(nonwkup_fifo_buffer);
    memset( status_fifo_buffer, 0, sizeof(status_fifo_buffer));
    p_bhy2_hub->fifos.status_fifo.read_pos = 0;
    p_bhy2_hub->fifos.status_fifo.read_length = 0;
    p_bhy2_hub->fifos.status_fifo.p_buffer = status_fifo_buffer;
    p_bhy2_hub->fifos.status_fifo.buffer_size = sizeof(status_fifo_buffer);

    for(i = 0; i< BHY_SYS_SENSOR_ID_MAX; i++)
    {
        p_bhy2_hub->virt_sensor[i].frame.frame_parse_func = bhy2_parse_frame_padding;
    }
    /* initial bhy2 hub obj -e*/

    bhy_hif_init_handle( &p_bhy2_hub->handle, bus_read_func, bus_write_func, delay_us_func, private_data);
    DBG_PRINT ("bhy2_hub_init.\r\n");
    return res;
}

/** @}*/
