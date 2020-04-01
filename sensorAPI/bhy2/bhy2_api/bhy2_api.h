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
 * @file        bhy2_api.h
 *
 * @brief
 *
 *
 */

/*!
 * @defgroup hostinterface BHy host interface definition
 * @{*/

#ifndef __BHY2_API_H__
#define __BHY2_API_H__

#include "typedef.h"
#include "bhy_host_interface.h"

#define GPS_BUFFER_LENGTH               512//256//512
#define DATA_FIFO_BUFFER_LENGTH            44//  4096//4096//2048//10240//2048//512
#define STATUS_FIFO_BUFFER_LENGTH          44//44//44//44//44//44//44//44//44//44//44//44//44//44//44//44//44//44//44//44//44//44//44//44//44//44//44//44//44//44//44//44//44//44//44//44//44//44//44//44//44//44//44//44//    5120//2048//2048//10240//2048//512
#define SENSOR_TYPE_CONTINUOUS_MODE     0x01
#define SENSOR_TYPE_ON_CHANGE_MODE      0x02
/*! define for soft passthrough feature */
#define SPASS_READ                      0
#define SPASS_WRITE                     1
#define SPASS_SINGLE_TRANS              0
#define SPASS_MULTI_TRANS               1
#define SPASS_DELAY_DIS                 0
#define SPASS_DELAY_EN                  1
#define SPASS_SIF1                      1
#define SPASS_SIF2                      2
#define SPASS_SIF3                      3
#define SPASS_SPI_4_WIRE                0
#define SPASS_SPI_3_WIRE                1
#define SPASS_SPI_CPOL_0                0
#define SPASS_SPI_CPOL_1                1
#define SPASS_SPI_CPHA_0                0
#define SPASS_SPI_CPHA_1                1
#define SPASS_SPI_CS_LOW                0
#define SPASS_SPI_CS_HIGH               1
#define SPASS_SPI_LSB_FIRST_DIS         0
#define SPASS_SPI_LSB_FIRST_EN          1
#define SPASS_SPI_READ_BIT_POL_LOW      0
#define SPASS_SPI_READ_BIT_POL_HIGH     1
#define SPASS_SPI_READ_BIT_POS_0        0
#define SPASS_SPI_READ_BIT_POS_1        1
#define SPASS_SPI_READ_BIT_POS_2        2
#define SPASS_SPI_READ_BIT_POS_3        3
#define SPASS_SPI_READ_BIT_POS_4        4
#define SPASS_SPI_READ_BIT_POS_5        5
#define SPASS_SPI_READ_BIT_POS_6        6
#define SPASS_SPI_READ_BIT_POS_7        7

/*! define self-test result */
#define SELFTEST_PASSED             0
#define SELFTEST_X_FAILED           1
#define SELFTEST_Y_FAILED           2
#define SELFTEST_X_Y_FAILED         3
#define SELFTEST_Z_FAILED           4
#define SELFTEST_X_Z_FAILED         5
#define SELFTEST_Y_Z_FAILED         6
#define SELFTEST_X_Y_Z_FAILED       7
#define SELFTEST_UNSUPPORTED        8
#define SELFTEST_NO_DEVICE          9

/*! define self-test result */
#define FOC_RESULT_SUCCESS             0
#define FOC_RESULT_FAIL             0x65
#define FOC_RESULT_ERR_UNKNOWN      0x23

/*! check bit if it is 1 */
#define CHK_BIT(data, bit)              (((u32)data>>bit)&0x01)

/*!
 *
 * @brief fifo buffer structure
 *
 */
typedef struct fifo_buffer_type
{
    u32 read_pos;
    u32 read_length;
    u32 remain_length;
    u32 buffer_size;
    u8* p_buffer;
}fifo_buffer_t;

/*!
 *
 * @brief fifo obj structure
 *
 */
typedef struct bhy_fifo_type
{
    fifo_buffer_t wkup_fifo;
    fifo_buffer_t nonwkup_fifo;
    fifo_buffer_t status_fifo;
}bhy_fifo_t;

/*!
 *
 * @brief 3 axes data structure
 *
 */
typedef struct data_3_axis_type
{
    s16 x;
    s16 y;
    s16 z;
}data_3_axis_t;

/*!
 *
 * @brief quaternion data structure
 *
 */
typedef struct data_quaternion_type
{
    s16 x;
    s16 y;
    s16 z;
    s16 w;
    s16 accuracy;
}data_quaternion_t;

/*!
 *
 * @brief orientation data structure
 *
 */
typedef struct data_orientation_type
{
    s16 heading;
    s16 pitch;
    s16 roll;
}data_orientation_t;

/*!
 *
 * @brief unsigned char data structure
 *
 */
typedef struct data_scalar_u8_type
{
    u8 data_u8;
} data_scalar_u8_t;

/*!
 *
 * @brief unsigned short data structure
 *
 */
typedef struct data_scalar_u16_type
{
    u16 data_u16;
} data_scalar_u16_t;

/*!
 *
 * @brief signed short data structure
 *
 */
typedef struct data_scalar_s16_type
{
    s16 data_s16;
} data_scalar_s16_t;

/*!
 *
 * @brief unsigned long 3 bytes data structure
 *
 */
typedef struct data_scalar_u24_type
{
    u32 data_u24;
} data_scalar_u24_t;

/*!
 *
 * @brief unsigned long data structure
 *
 */
typedef struct data_scalar_u32_type
{
    u32 data_u32;
} data_scalar_u32_t;

/*!
 *
 * @brief bhy2 debug message data structure
 *
 */
typedef struct data_debug_type
{
    u8 id;
    u8 flag;
    u8 buffer[17];
} data_debug_t;

/*!
 *
 * @brief bhy2 gps data structure
 *
 */
typedef struct data_gps_type
{
    u8 id;
    u8 bytes_to_transfer;
    u16 bytes_transferred;
    u16 total_length;
    u8 protocol_type;
    u8 buffer[GPS_BUFFER_LENGTH];
} data_gps_t;

/*!
 *
 * @brief bhy2 timestamp data structure
 *
 */
typedef struct data_time_stamp_type
{
    u64 time_stamp;
    u64 time_stamp_wkup;
} data_time_stamp_t;

/*!
 *
 * @brief bhy2 meta-event data structure
 *
 */
typedef struct data_meta_event_type
{
    u8 meta_event_type;
    u8 byte1;
    u8 byte2;
} data_meta_event_t;

/*!
 *
 * @brief bhy2 virtual sensor data
 *
 */
typedef union virt_sensor_data_type
{
    data_3_axis_t         three_axis;
    data_quaternion_t     quaternion;
    data_orientation_t    orientation;
    data_scalar_u8_t      scalar_u8;
    data_scalar_u16_t     scalar_u16;
    data_scalar_s16_t     scalar_s16;
    data_scalar_u24_t     scalar_u24;
    data_scalar_u32_t     scalar_u32;
}virt_sensor_data_t;

/*!
 *
 * @brief bhy2 virtual sensor infomation structure
 *
 */
typedef struct sensor_info_type
{
    u8 present;
    u8 type;    /*1: continuous,2: on change*/
    u8  id;
    u8 accuracy;
    u16 range;
    u32 latency;
    float sample_rate;
    float scale_factor;
    //virt_sensor_data data;
} virt_sensor_info_t;

struct bhy2_hub_type;
typedef struct bhy2_hub_type bhy2_hub_t;

/*!
 *
 * @brief bhy2 frame data structure
 *
 */
typedef struct bhy_frame_type
{
    u8 frame_id;
    u8 frame_length;
    s16 (*frame_parse_func)(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);
}bhy_frame_t;

/*!
 *
 * @brief bhy2 virtual sensor configuration structure
 *
 */
typedef struct virt_sensor_conf_type
{
    u16 sensitivity;
    u16 range;
    u32 latency;
    float sample_rate;
    u8 buf[12];
} virt_sensor_conf_t;

/*!
 *
 * @brief bhy2 virtual sensor structure
 *
 */
typedef struct virt_sensor_type
{
    virt_sensor_info_t info;
    virt_sensor_data_t data;
    bhy_frame_t frame;
} virt_sensor_t;

/*!
 *
 * @brief register configuration check status
 *
 */
typedef union reg_conf_check_type
{
    struct
    {
        u8 chip_ctrl_bit:               1;
        u8 host_interface_ctrl_bit:     1;
        u8 host_interrupt_ctrl_bit:     1;
        u8 wgp1_0_bit:                  1;
        u8 wgp1_1_bit:                  1;
        u8 wgp1_2_bit:                  1;
        u8 wgp1_3_bit:                  1;
        u8 wgp2_0_bit:                  1;
        u8 wgp2_1_bit:                  1;
        u8 wgp2_2_bit:                  1;
        u8 wgp2_3_bit:                  1;
        u8 wgp3_0_bit:                  1;
        u8 wgp3_1_bit:                  1;
        u8 wgp3_2_bit:                  1;
        u8 wgp3_3_bit:                  1;
        u8 reset_request_bit:           1;
        u8 event_irq_request_bit:       1;
        u8 host_ctrl_bit:               1;
        u8 Reserved_bit:         1;
        u8 fpga_jtag_ctrl_bit:          1;
    }bit;
    u32 data;
}reg_conf_check_t;

/*!
 *
 * @brief register configuration structure
 *
 */
typedef struct reg_conf_type
{
    u8 chip_ctrl;
    u8 host_interface_ctrl;
    u8 host_interrupt_ctrl;
    u8 wgp1_0;
    u8 wgp1_1;
    u8 wgp1_2;
    u8 wgp1_3;
    u8 wgp2_0;
    u8 wgp2_1;
    u8 wgp2_2;
    u8 wgp2_3;
    u8 wgp3_0;
    u8 wgp3_1;
    u8 wgp3_2;
    u8 wgp3_3;
    u8 reset_request;
    u8 event_irq_request;
    u8 host_ctrl;
    u8 fpga_jtag_ctrl;
}reg_conf_t;

typedef union reg_conf_save_type
{
    reg_conf_t reg;
    u8 reg_buf[sizeof( reg_conf_t)];
}reg_conf_save_t;

/*!
 *
 * @brief sensor hub structure
 *
 */
struct bhy2_hub_type
{
    virt_sensor_t virt_sensor[BHY_SYS_SENSOR_ID_MAX];
    u8 virt_sen_present[32];
    u64 time_stamp;
    u64 time_stamp_wkup;
    data_meta_event_t meta_event;
    data_meta_event_t meta_event_wkup;
    float max_sample_rate;
    float min_sample_rate;
    u32 max_latency;
    u32 min_latency;
    u32 sys_time_ms;
    u32 swdog_delay_ms;
    u8 sensor_type_check;
    u8 product_id;
    u16 rom_version;
    u16 ram_version;
    struct bhy_hif_handle handle;
    bhy_fifo_t fifos;
    const u8* p_patch;
    u32 patch_size;
    reg_conf_check_t reg_conf_chk;
    reg_conf_save_t reg_conf_save;
};

/*!
 *
 * @brief soft passthrough configuration structure
 *
 */
typedef union soft_passthrough_conf_type
{
    struct
    {
        /*! byte 1 */
        u8 direction:   1;  /**< 0: read; 1: write. */
        u8 trans_type:  1;  /**< 0: single burst; 1:multi single transfers. */
        u8 delay_ctrl:  1;  /**< 0: none; 1: delay between bytes. */
        u8 master_bus:  2;  /**< 1: SIF1; 2: SIF2; 3:SIF3. */
        u8 spi_mode:    1;  /**< 0: 4 wire; 1: 3 wire. */
        u8 cpol:        1;  /**< spi clock polarity. */
        u8 cpha:        1;  /**< spi clock phase. */
        /*! byte 2 */
        u8 delay_val:   6;  /**< mutiples of 50us. */
        u8 cs_level:    1;  /**< chip select level. */
        u8 lsb_first:   1;  /**< least significant byte first. */
        /*! byte 3~4 */
        u16 trans_rate;     /**< spi clock rate. */
        /*! byte 5 */
        u8 address_shift:   4;  /**< number of bits to shift register address. */
        u8 read_bit_pol:   1;   /**< 0: active low; 1: active high. */
        u8 read_bit_pos:   3;   /**< bit number of read bit in command byte. */
        /*! byte 6 */
        union
        {
            u8 slave_address;
            u8 cs_pin;
        }func_set;
        /*! byte 7 */
        u8 trans_count;
        /*! byte 8 */
        u8 reg;
    }conf;
    u8 data[8];

}soft_passthrough_conf_t;

extern u8 sensor_table[][30];
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
s16 bhy2_register_get( bhy2_hub_t* p_bhy2_hub, u8 reg, u8* p_data);

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
s16 bhy2_register_set( bhy2_hub_t* p_bhy2_hub, u8 reg, u8 data);

/*!
 * @brief update bhy2 obj sample rate
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 *
 * @return res: operation result
 */
s16 bhy2_update_sample_rate( bhy2_hub_t* p_bhy2_hub );

/*!
 * @brief update bhy2 obj latency
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 *
 * @return res: operation result
 */
s16 bhy2_update_latency( bhy2_hub_t* p_bhy2_hub );

/*!
 * @brief dump registers' data
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 *
 * @return res: operation result
 */
s16 bhy2_dump_registers(bhy2_hub_t* p_bhy2_hub);

/*!
 * @brief init software watchdog
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 *
 * @return res: operation result
 */
s16 bhy2_softwatchdog_init(bhy2_hub_t* p_bhy2_hub);

/*!
 * @brief clean soft-watchdog value
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 *
 * @return res: operation result
 */
s16 bhy2_softwatchdog_clean(bhy2_hub_t* p_bhy2_hub);

/*!
 * @brief soft-watchdog error process
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 *
 * @return res: operation result
 */
s16 bhy2_softwatchdog_error_process(bhy2_hub_t* p_bhy2_hub);

/*!
 * @brief soft-watchdog schedule call
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 *
 * @return res: operation result
 */
s16 bhy2_softwatchdog_schedule( bhy2_hub_t* p_bhy2_hub);

/*!
 * @brief parse padding frame
 *
 *
 * @param[in] p_fifo_buffer: pointer to fifo buffer
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_frame_padding(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub);

/*!
 * @brief parse fifos
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 *
 * @return res: operation result
 */
s16 bhy2_parse_fifos(bhy2_hub_t* p_bhy2_hub);

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
s16 bhy2_virt_sensor_present_get (bhy2_hub_t* p_bhy2_hub, u8* p_param);

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
s16 bhy2_virt_sensor_cfg_get(bhy2_hub_t* p_bhy2_hub, u8 sensor_id, virt_sensor_conf_t* virt_sensor_conf);

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
s16 bhy2_virt_sensor_cfg_set(bhy2_hub_t* p_bhy2_hub, u8 sensor_id, float sample_rate, u32 latency);

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
s16 bhy2_virt_sensor_range_set(bhy2_hub_t* p_bhy2_hub, u8 sensor_id, u16 range);

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
                                s16 (*frame_parse_func)(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub));

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
s16 bhy2_fifo_ctrl_get( bhy2_hub_t* p_bhy2_hub, u32* p_fifo_ctrl);

/*!
 * @brief set wakeup fifo watermark
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] watermark: watermark to pass in
 *
 * @return res: operation result
 */
s16 bhy2_wkup_fifo_watermark_set( bhy2_hub_t* p_bhy2_hub , u32 watermark);

/*!
 * @brief get wakeup fifo watermark
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[out] watermark: pointer to get watermark
 *
 * @return res: operation result
 */
s16 bhy2_wkup_fifo_watermark_get( bhy2_hub_t* p_bhy2_hub , u32* watermark);

/*!
 * @brief set non-wakeup fifo watermark
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] watermark: watermark to pass in
 *
 * @return res: operation result
 */
s16 bhy2_nonwkup_fifo_watermark_set( bhy2_hub_t* p_bhy2_hub , u32 watermark);

/*!
 * @brief get non-wakeup fifo watermark
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[out] watermark: pointer to get watermark
 *
 * @return res: operation result
 */
s16 bhy2_nonwkup_fifo_watermark_get( bhy2_hub_t* p_bhy2_hub , u32* watermark);

/*!
 * @brief get product id
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[out] p_prod_id: pointer to get product id
 *
 * @return res: operation result
 */
s16 bhy2_product_id_get(bhy2_hub_t* p_bhy2_hub, u8* p_prod_id);

/*!
 * @brief get revision id
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[out] p_revision_id: pointer to get revision id
 *
 * @return res: operation result
 */
s16 bhy2_revision_id_get(bhy2_hub_t* p_bhy2_hub, u8* p_revision_id);

/*!
 * @brief get rom version
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[out] p_rom_version: pointer to get rom version
 *
 * @return res: operation result
 */
s16 bhy2_rom_version_get(bhy2_hub_t* p_bhy2_hub, u16* p_rom_version);

/*!
 * @brief get ram version
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[out] p_ram_version: pointer to get ram version
 *
 * @return res: operation result
 */
s16 bhy2_ram_version_get(bhy2_hub_t* p_bhy2_hub, u16* p_ram_version);

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
s16 bhy2_patch_upload_to_ram(bhy2_hub_t* p_bhy2_hub, const u8* p_patch, u32 length);

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
s16 bhy2_patch_upload_to_ram_partly(bhy2_hub_t* p_bhy2_hub, const u8* p_buf, u32 total_len, u32 cur_pos, u32 pack_len);

/*!
 * @brief boot hub from ram
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 *
 * @return res: operation result
 */
s16 bhy2_patch_boot_from_ram(bhy2_hub_t* p_bhy2_hub);

/*!
 * @brief erase qspi flash
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 *
 * @return res: operation result
 */
s16 bhy2_patch_erase_flash(bhy2_hub_t* p_bhy2_hub);

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
s16 bhy2_patch_upload_to_flash(bhy2_hub_t* p_bhy2_hub, const u8* p_patch, u32 length);

/*!
 * @brief boot from qspi flash
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 *
 * @return res: operation result
 */
s16 bhy2_patch_boot_from_flash(bhy2_hub_t* p_bhy2_hub);

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
s16 bhy2_patch_upload_to_flash_partly(bhy2_hub_t* p_bhy2_hub, const u8* p_buf, u32 total_len, u32 cur_pos, u32 pack_len);

/*!
 * @brief set host interrupt control register
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] param: param to set
 *
 * @return res: operation result
 */
s16 bhy2_host_interrupt_ctrl_set(bhy2_hub_t* p_bhy2_hub, u8 param);

/*!
 * @brief set host interface control register
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] param: param to set
 *
 * @return res: operation result
 */
s16 bhy2_host_interface_ctrl_set(bhy2_hub_t* p_bhy2_hub, u8 param);

/*!
 * @brief configure output timestamp in fifo
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] param: param to set
 *              the value should be: BHY_FIFO_FORMAT_CTRL_ENABLE_TIMPSTAMP	        (0)
 *                                   BHY_FIFO_FORMAT_CTRL_DISABLE_TIMPSTAMP	        (1 << 0)
 *                                   BHY_FIFO_FORMAT_CTRL_DISABLE_FULL_TIMPSTAMP     (1 << 1)
 *
 * @return res: operation result
 */
s16 bhy2_timestamp_output_ctrl_set(bhy2_hub_t* p_bhy2_hub, u8 param);

/*!
 * @brief configure hardware timestamp
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] param: param to set
 *
 * @return res: operation result
 */
s16 bhy2_hw_timestamp_ctrl_set (bhy2_hub_t* p_bhy2_hub, u8 param);

/*!
 * @brief get hardware timestamp configuration
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[out] p_param: pointer to get configuration
 *
 * @return res: operation result
 */
s16 bhy2_hw_timestamp_ctrl_get (bhy2_hub_t* p_bhy2_hub, u8* p_param);

/*!
 * @brief get hardware timestamp
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[out] p_timestamp: pointer to get timestamp
 *
 * @return res: operation result
 */
s16 bhy2_hw_timestamp_get (bhy2_hub_t* p_bhy2_hub, u64* p_timestamp);

/*!
 * @brief get fpga version
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[out] p_param: pointer to get fpga version
 *
 * @return res: operation result
 */
s16 bhy2_fpga_version_get (bhy2_hub_t* p_bhy2_hub, u8* p_param);

/*!
 * @brief set host control register
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] param: data to set
 *
 * @return res: operation result
 */
s16 bhy2_host_ctrl_set (bhy2_hub_t* p_bhy2_hub, u8 param);

/*!
 * @brief get host control register
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[out] p_param: pointer to register data
 *
 * @return res: operation result
 */
s16 bhy2_host_ctrl_get (bhy2_hub_t* p_bhy2_hub, u8* p_param);


/*!
 * @brief dump registers' data
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] p_conf: pointer to soft-passthrough configuration
 * @param[in] reg: register to operate(read/write)
 * @param[in] trans_num: number of data to transfer(read/write)
 * @param[in,out] p_buf: pointer to buffer to operate(read/write);
 *
 * @return res: operation result
 */
s16 bhy2_soft_passthrough_operation (bhy2_hub_t* p_bhy2_hub, soft_passthrough_conf_t* p_conf,
                                    u8 reg, u8 trans_num, u8* p_buf);

/*!
 * @brief flush specific virtual sensor data in fifos
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] sensor_id: virtual sensor id
 *
 * @return res: operation result
 */
s16 bhy2_fifo_flush(bhy2_hub_t* p_bhy2_hub, u8 sensor_id);

/*!
 * @brief reset sensor hub,
 *          after reset, the hub is in boot mode and need to upload ram patch to it again
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 *
 * @return res: operation result
 */
s16 bhy2_hub_reset( bhy2_hub_t* p_bhy2_hub );

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
s16 bhy2_do_self_test( bhy2_hub_t* p_bhy2_hub, u8 sensor_id, u8 *ret_status, s16 *offset );

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
s16 bhy2_do_foc( bhy2_hub_t* p_bhy2_hub, u8 sensor_id, u8 *ret_status, s16 *offset );

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
s16 bhy2_orientation_matrix_set( bhy2_hub_t* p_bhy2_hub, u8 sensor_id, s8 *matrix, u8 matrix_len );

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
s16 bhy2_orientation_matrix_get( bhy2_hub_t* p_bhy2_hub, u8 sensor_id, s8 *p_matrix, u8* p_matrix_len );

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
s16 bhy2_sic_matrix_set( bhy2_hub_t* p_bhy2_hub, u8 *p_buf, u16 buf_len );

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
s16 bhy2_sic_matrix_get( bhy2_hub_t* p_bhy2_hub, u8 *p_buf, u16 buf_len, u32* p_ret_len );

/*!
 * @brief get calibration profile from a physical sensor.
 * physical sensor id:
 *                      BHY_PHYS_SENSOR_ID_ACC,
 *                      BHY_PHYS_SENSOR_ID_MAG,
 *                      BHY_PHYS_SENSOR_ID_GYRO,
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] sensor_id: physical sensor id
 * @param[in] p_buf: pointer of a buffer to get calibration profile
 * @param[in] buf_len: length of buffer
 * @param[out] p_ret_len: actual length of calibration profile
 *
 * @return res: operation result
 */
s16 bhy2_calibration_profile_get( bhy2_hub_t* p_bhy2_hub, u8 sensor_id, u8 *p_buf, u16 buf_len, u32* p_ret_len );

/*!
 * @brief set calibration profile to a physical sensor.
 * physical sensor id:
 *                      BHY_PHYS_SENSOR_ID_ACC,
 *                      BHY_PHYS_SENSOR_ID_MAG,
 *                      BHY_PHYS_SENSOR_ID_GYRO,
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] sensor_id: physical sensor id
 * @param[in] p_buf: pointer of a buffer to set calibration profile
 * @param[in] buf_len: length of calibration profile
 *
 * @return res: operation result
 */
s16 bhy2_calibration_profile_set( bhy2_hub_t* p_bhy2_hub, u8 sensor_id, u8 *p_buf, u16 buf_len );

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
s16 bhy2_post_mortem_data_get( bhy2_hub_t* p_bhy2_hub, u16 *p_code, u32 *p_ret_len, u8 *p_buf, u32 buf_len);

/*!
 * @brief trigger fatal error.
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[in] error_type: error type
 * @param[in] flag: flag for error type
 *
 * @return res: operation result
 */
s16 bhy2_trigger_fatal_error( bhy2_hub_t* p_bhy2_hub, u8 error_type, u8 flag);

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
                    void *private_data);
#endif
