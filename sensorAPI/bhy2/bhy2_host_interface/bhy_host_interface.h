/*!
* @section LICENSE
* $license_gpl$
*
* @filename $filename$
* @date     $date$
* @id       $id$
*
* @brief The files for BHy host interface
*/

/*!
 * @addtogroup hostinterface
 * @{*/

#ifndef BHY_HOST_INTERFACE_H
#define BHY_HOST_INTERFACE_H

#include "bhy_host_interface_defs.h"

/*!
 * @brief Host interface handle structure
 *
 * This handle holds all the helper functions required for the api to work.
 * Pointer to this handle is required for all the functions in this api.
 */
struct bhy_hif_handle {
	int (*bus_read_func)(bhy_u8, bhy_u8 *, bhy_u32, void *);
	int (*bus_write_func)(bhy_u8, const bhy_u8 *, bhy_u32, void *);
	void (*delay_us_func)(bhy_u32, void *);
	bhy_u8 command_buf[BHY_COMMAND_PACKET_LEN];
	void *private_data;
	bhy_u32 max_burst;
};

/*!
 * @brief Firmware header structure
 */
struct bhy_fw_header {
	bhy_u8 magic[2];
	bhy_u8 flags[2];
	bhy_u8 key_flags[2];
	bhy_u8 version[2];
	bhy_u8 sha_256[32];
	bhy_u8 cert[48];
	bhy_u8 payload_len[4]; /* not include fw header */
	bhy_u8 image_crc[4];
	bhy_u8 key_offset[4];
	bhy_u8 expected_version[4];
	bhy_u8 reserved2[15];
	bhy_u8 header_crc[4];
};

/*!
 * @brief Initialize the host interface handle
 * @param[out] handle: Host interface handle that needs initialization
 * @param[in]  bus_read_func: Pointer to bus read helper function
 * @param[in]  bus_write_func: Pointer to bus write helper function
 * @param[in]  delay_us_func: Pointer to delay function in metric of
 *                   micro seconds
 * @param[in]  private_data: Pointer to client defined data
 */
void bhy_hif_init_handle(struct bhy_hif_handle *handle,
	int (*bus_read_func)(bhy_u8 reg, bhy_u8 *buf, bhy_u32 len,
	void *private_data),
	int (*bus_write_func)(bhy_u8 reg, const bhy_u8 *buf, bhy_u32 len,
	void *private_data),
	void (*delay_us_func)(bhy_u32 interval_us, void *private_data),
	void *private_data);

/*!
 * @brief Bus read helper function
 * @param[in] handle: Host interface handle
 * @param[in]  reg: Register to read
 * @param[out] buf: Buffer to hold the returned register value
 * @param[in] len: Register length
 * @return Host interface error code
 */
bhy_s8 bhy_hif_bus_read(struct bhy_hif_handle *handle,
	bhy_u8 reg, bhy_u8 *buf, bhy_u32 len);

/*!
 * @brief Bus write helper function
 * @param[in] handle: Host interface handle
 * @param[in] reg: Register to write
 * @param[in] buf: Buffer to hold the value to write
 * @param[in] len: Register length
 * @return Host interface error code
 */
bhy_s8 bhy_hif_bus_write(struct bhy_hif_handle *handle,
	bhy_u8 reg, const bhy_u8 *buf,
	bhy_u32 len);

/*!
 * @brief Delay helper function
 * @param[in] handle: Host interface handle
 * @param[in] us: interval in the metric of micro seconds
 * @return Host interface error code
 */
bhy_s8 bhy_hif_delay_us(struct bhy_hif_handle *handle, bhy_u32 us);

/*!
 * @brief Execute commands
 * @param[in] handle: Host interface handle
 * @param[in] cmd: Command code
 * @param[in] buf: Command data
 * @param[in] len: Command data length
 * @return Host interface error code
 */
bhy_s8 bhy_hif_exec_cmd(struct bhy_hif_handle *handle,
	bhy_u16 cmd, const bhy_u8 *buf, bhy_u32 len);

/*!
 * @brief Read parameter
 * @param[in] handle: Host interface handle
 * @param[in] param: Parameter number
 * @param[out] buf: Buffer to hold parameter data
 * @param[in] buf_len: Maximum buffer length
 * @param[out] ret_len: Returned buffer length
 * @return Host interface error code
 */
bhy_s8 bhy_hif_read_parameter(struct bhy_hif_handle *handle,
	bhy_u16 param, bhy_u8 *buf, bhy_u32 buf_len, bhy_u32 *ret_len);

/*!
 * @brief Write parameter
 * @param[in] handle: Host interface handle
 * @param[in] param: Parameter number
 * @param[in] buf: Buffer holds parameter data to write
 * @param[in] len: Buffer length
 * @return Host interface error code
 */
bhy_s8 bhy_hif_write_parameter(struct bhy_hif_handle *handle,
	bhy_u16 param, const bhy_u8 *buf, bhy_u32 len);

/*!
 * @brief erase external flash
 * @param[in] handle: Host interface handle
 * @return 0 on success, negative on failure
 */
bhy_s8 bhy_hif_erase_flash(struct bhy_hif_handle *handle, bhy_u8 *work_buf,
	bhy_u32 work_buf_len, bhy_u32 *ret_len);

/*!
 * @brief write to flash
 * @param[in] handle: Host interface handle
 * @param[in] buf: files need to write to flash
 * @param[in] len: file length
 * @param[in] work_buf: work buffer
 * @param[in] work_buf_len: work buffer length
 * @return 0 on success, negative on failure
 */
bhy_s8 bhy_hif_write_flash(struct bhy_hif_handle *handle,
	const bhy_u8 *buf, bhy_u32 len, bhy_u8 *work_buf,
	bhy_u32 work_buf_len, bhy_u32 *ret_len);

/*!
 * @brief write to flash part by part
 * @param[in] handle: Host interface handle
 * @param[in] buf: firmware data need to write to flash
 * @param[in] total_len: total length of firmware data
 * @param[in] cur_pos: current write position
 * @param[in] pack_len: current buffer length of firmware data
 * @return 0 on success, negative on failure
 */
bhy_s8 bhy_hif_upload_to_flash_partly(struct bhy_hif_handle *handle,
	const bhy_u8 *buf, bhy_u32 total_len, bhy_u32 cur_pos,
	bhy_u32 pack_len);
/*!
 * @brief boot ram from flash
 * @param[in] handle: Host interface handle
 * @return 0 on success, negative on failure.
 */
bhy_s8 bhy_hif_boot_from_flash(struct bhy_hif_handle *handle);

/*!
 * @brief Read fpga version
 * @param[in] handle: Host interface handle
 * @param[out] version: fpga version
 * @return Host interface error code
 */
bhy_s8 bhy_hif_read_fpga_version(struct bhy_hif_handle *handle,
	bhy_u8 *version);

/*!
 * @brief Read product ID
 * @param[in] handle: Host interface handle
 * @param[out] prod_id: Returned product ID
 * @return Host interface error code
 */
bhy_s8 bhy_hif_read_product_id(struct bhy_hif_handle *handle, bhy_u8 *prod_id);

/*!
 * @brief Read ROM version
 * @param[in] handle: Host interface handle
 * @param[out] rom_ver: Returned ROM version
 * @return Host interface error code
 */
bhy_s8 bhy_hif_read_rom_version(struct bhy_hif_handle *handle,
	bhy_u16 *rom_ver);

/*!
 * @brief Read firmware version the kernel was built with.
 * @param[in] handle: Host interface handle
 * @param[out] ram_ver: Returned RAM version
 * @return Host interface error code
 */
bhy_s8 bhy_hif_read_ram_kernel_version(struct bhy_hif_handle *handle,
	bhy_u16 *ram_ver);

/*!
 * @brief Read firmware user version
 * @param[in] handle: Host interface handle
 * @param[out] flash_ver: Returned flash version
 * @return Host interface error code
 */
bhy_s8 bhy_hif_read_ram_user_version(struct bhy_hif_handle *handle,
	bhy_u16 *user_ver);

/*!
 * @brief Read interrupt status
 * @param[in] handle: Host interface handle
 * @param[out] irq_status: Returned interrupt status
 * @return Host interface error code
 */
bhy_s8 bhy_hif_read_irq_status(struct bhy_hif_handle *handle,
	bhy_u8 *irq_status);

/*!
 * @brief Read firmware error status
 * @param[in] handle: Host interface handle
 * @param[out] fw_error: Returned firmware status
 * @return Host interface error code
 */
bhy_s8 bhy_hif_read_fw_error(struct bhy_hif_handle *handle, bhy_u8 *fw_error);


/*!
 * @brief Request soft reset
 * @param[in] handle: Host interface handle
 * @return Host interface error code
 */
bhy_s8 bhy_hif_reset(struct bhy_hif_handle *handle);

/*!
 * @brief Upload firmware to RAM
 * @param[in] handle: Host interface handle
 * @param[in] buf: Firmware data
 * @param[in] len: Firmware data length
 * @return Host interface error code
 */
bhy_s8 bhy_hif_upload_to_ram(struct bhy_hif_handle *handle,
	const bhy_u8 *buf, bhy_u32 len);

/*!
 * @brief Upload firmware to RAM part by part
 * @param[in] handle: Host interface handle
 * @param[in] buf: Point to firmware buffer
 * @param[in] total_len: Firmware data length
 * @param[in] cur_pos: Current write position of firmware
 * @param[in] pack_len: Current firmware buffer length
 * @return Host interface error code
 */
bhy_s8 bhy_hif_upload_to_ram_partly(struct bhy_hif_handle *handle,
	const bhy_u8 *buf, bhy_u32 total_len, bhy_u32 cur_pos,
	bhy_u32 pack_len);

/*!
 * @brief Boot the uploaded firmware code
 * @param[in] handle: Host interface handle
 * @return Host interface error code
 */
bhy_s8 bhy_hif_boot_program_ram(struct bhy_hif_handle *handle);

/*!
 * @brief Read wakeup FIFO
 * @param[in] handle: Host interface handle
 * @param[out] ret_len: Returned FIFO data length
 * @param[out] buf: Returned FIFO data
 * @param[in] buf_len: Maximum buffer length
 * @return Host interface error code
 */
bhy_s8 bhy_hif_read_wakeup_fifo(struct bhy_hif_handle *handle,
	bhy_u32 *ret_len, bhy_u8 *buf, bhy_u32 buf_len, bhy_u32* remain_len);

/*!
 * @brief Read non-wakeup FIFO
 * @param[in] handle: Host interface handle
 * @param[out] ret_len: Returned FIFO data length
 * @param[out] buf: Returned FIFO data
 * @param[in] buf_len: Maximum buffer length
 * @return Host interface error code
 */
bhy_s8 bhy_hif_read_nonwakeup_fifo(struct bhy_hif_handle *handle,
	bhy_u32 *ret_len, bhy_u8 *buf, bhy_u32 buf_len, bhy_u32* remain_len);

/*!
 * @brief Read data from status channel
 * @param[in] handle: Host interface handle
 * @param[out] code: Returned status code
 * @param[out] ret_len: Returned status data length
 * @param[out] buf: Returned status data
 * @param[in] buf_len: Maximum buffer length
 * @return Host interface error code
 */
bhy_s8 bhy_hif_read_status(struct bhy_hif_handle *handle,
	bhy_u16 *code, bhy_u32 *ret_len, bhy_u8 *buf, bhy_u32 buf_len);

/*!
 * @brief Read async data from status channel
 * @param[in] handle: Host interface handle
 * @param[out] ret_len: Returned status data length
 * @param[out] buf: Returned status data
 * @param[in] buf_len: Maximum buffer length
 * @return Host interface error code
 */
bhy_s8 bhy_hif_read_status_async(struct bhy_hif_handle *handle,
	bhy_u32 *ret_len, bhy_u8 *buf, bhy_u32 buf_len, bhy_u32* remain_len);

/*!
 * @brief Read virtual sensor presence information
 * @param[in] handle: Host interface handle
 * @param[out] map: bitmap for sensor presence information (8 bytes/64-bit)
 * @return Host interface error code
 */
bhy_s8 bhy_hif_read_sensor_bitmap(struct bhy_hif_handle *handle, bhy_u8 *map);

/*!
 * @brief Configure sensor interface
 * @param[in] handle: Host interface handle
 * @param[in] sensor_id: Sensor ID for the sensor that needed to configure
 * @param[in] sample_rate: Desired sample rate in HZ.
 *                  This is actually float value
 * @param[in] latency: Desired report latency
 * @return Host interface error code
 */
bhy_s8 bhy_hif_exec_sensor_conf_cmd(struct bhy_hif_handle *handle,
	bhy_u8 sensor_id, bhy_u32 sample_rate, bhy_u32 latency);

/*!
 * @brief Flush operation for selected sensor
 * @param[in] handle: Host interface handle
 * @param[in] sensor_id: Sensor ID for the sensor that needed to flush
 * @return Host interface error code
 */
bhy_s8 bhy_hif_write_fifo_flush(struct bhy_hif_handle *handle,
	bhy_u8 sensor_id);

/*!
 * @brief self test operation for selected sensor
 * @param[in] handle: Host interface handle
 * @param[in] sensor_id: Sensor ID for the sensor that needed to do self test
 * @param[out] ret_status: Returned completion status
 * @param[out] offset: Returned self-test offset the the 3 axis if applicable
 * @return Host interface error code
 */
bhy_s8 bhy_hif_do_self_test(struct bhy_hif_handle *handle,
	bhy_u8 sensor_id, bhy_u8 *ret_status, bhy_s16 *offset);

/*!
 * @brief fast offset calibration operation for selected sensor
 * @param[in] handle: Host interface handle
 * @param[in] sensor_id: Sensor ID that needed to do fast offset calibration
 * @param[out] ret_status: Returned completion status
 * @param[out] offset: Returned FOC offset for 3 axises
 * @return Host interface error code
 */
bhy_s8 bhy_hif_do_foc(struct bhy_hif_handle *handle, bhy_u8 sensor_id,
	bhy_u8 *ret_status, bhy_s16 *offset);

/*!
 * @brief read by sensor control API
 * @param[in] handle: Host interface handle
 * @param[in] sensor_id: Sensor ID that needed to do fast offset calibration
 * @param[in] cmd: cmd for foc, fast start or OIS
 * @param[out] ret_data: Returned FOC offset for 3 axises
 * @param[out] data_len: data length
 * @return Host interface error code
 */
bhy_s8 bhy_hif_read_sensor_ctrl(struct bhy_hif_handle *handle, bhy_u8 sensor_id,
	bhy_u8 cmd, bhy_u8 *ret_data, bhy_u32 *data_len);

/*!
 * @brief Write by sensor control API
 * @param[in] handle: Host interface handle
 * @param[in] sensor_id: Sensor ID that needed to do fast offset calibration
 * @param[in] cmd: cmd for foc, fast start or OIS
 * @param[out] buf: data needs to be written
 * @param[out] buf_len: buffer length
 * @return Host interface error code
 */
bhy_s8 bhy_hif_write_sensor_ctrl(struct bhy_hif_handle *handle,
	bhy_u8 sensor_id, bhy_u8 cmd, bhy_u8 *buf, bhy_u32 buf_len);

/*!
* @brief Read bsx state inferface.
*           At least provide 68 bytes for one block reading in this api.
* @param[in] handle: Host interface handle.
* @param[in] param_num: parameter number.
* @param[in] buf: buffer to store bsx state data.
* @param[in] buf_len: buffer length.
* @param[out] ret_len: return total bsx state data length.
* @return Host interface error code.
*/
bhy_s8 bhy_hif_read_bsx_state(struct bhy_hif_handle *handle,
	bhy_u16 param_num, bhy_u8 *buf, bhy_u32 buf_len, bhy_u32 *ret_len);

/*!
* @brief Write bsx state inferface.
*           At least provide 68 bytes for one block reading in this api.
* @param[in] handle: Host interface handle.
* @param[in] param_num: parameter number.
* @param[in] buf: buffer to store bsx state data.
* @param[in] buf_len: buffer length.
* @return Host interface error code.
*/
bhy_s8 bhy_hif_write_bsx_state(struct bhy_hif_handle *handle,
	bhy_u16 param_num, const bhy_u8 *buf, bhy_u32 buf_len);

/*!
 * @brief Request current hw timestamp to be updated in host timestamp register
 * @param[in] handle: Host interface handle
 * @return Host interface error code
 */
bhy_s8 bhy_hif_request_hw_timestamp(struct bhy_hif_handle *handle);

/*!
 * @brief Read the hw timestamp from host timestamp register
 * @param[in] handle: Host interface handle
 * @param[out] ts: Returned hw timestamp
 * @return Host interface error code
 */
bhy_s8 bhy_hif_read_hw_timestamp(struct bhy_hif_handle *handle, bhy_u64 *ts);

/*!
 * @brief Combination of timestamp request and read
 * @param[in] handle: Host interface handle
 * @param[out] ts: Returned hw timestamp
 * @return Host interface error code
 */
bhy_s8 bhy_hif_req_and_read_hw_timestamp(struct bhy_hif_handle *handle,
	bhy_u64 *ts);

/*!
 * @brief execute soft pass-through command
 * @param[in] handle: Host interface handle
 * @param[in] buf: soft pass-through command
 * @param[in] buf_len: command lenght
 * @param[out] reg_data: status for soft pass-through
 * @param[out] reg_len: status length
 * @param[in] reg_buf_len: status buffer length
 * @return Host interface error code
 */
bhy_s8 bhy_hif_exec_soft_passthrough(struct bhy_hif_handle *handle,
	bhy_u8 *buf, bhy_u32 buf_len, bhy_u8 *reg_data,
	bhy_u32 reg_buf_len, bhy_u32 *reg_len);

/*!
 * @brief Read post mortem data
 * @param[in] handle: Host interface handle
 * @param[out] code: status code
 * @param[out] ret_len: post mortem data length
 * @param[out] buf: post mortem data
 * @param[in] buf_len: Maximum buffer length
 * @return 0 on success, negative on failure
 */
bhy_s8 bhy_hif_read_post_mortem(struct bhy_hif_handle *handle,
	bhy_u16 *code, bhy_u32 *ret_len, bhy_u8 *buf, bhy_u32 buf_len);

/*!
 * @brief Read sensor information structure for a sensor
 * @param[in] handle: Host interface handle
 * @param[in] sensor_id: sensor id to query its information
 * @param[out] info: Returned sensor info structure
 * @return Host interface error code
 */
bhy_s8 bhy_hif_read_sensor_info(struct bhy_hif_handle *handle,
	bhy_u8 sensor_id, union bhy_sensor_info *info);

/*!
 * @brief Read sensor information structure for a sensor
 * @param[in] handle: Host interface handle
 * @param[in] sensor_id: physical sensor id to query its information
 * @param[out] info: Returned physical sensor info structure
 * @return Host interface error code
 */
bhy_s8 bhy_hif_read_phys_sensor_info(struct bhy_hif_handle *handle,
	bhy_u8 sensor_id, union bhy_phys_sensor_info *info);

/*!
 * @brief Write orientation matrix for physical sensor
 * @param[in] handle: Host interface handle
 * @param[in] sensor_id: Physical sensor id to query its information
 * @param[in] buf: Orientation matrix
 * @param[in] len: Orientation matrix length
 * @return Host interface error code
 */
bhy_s8 bhy_hif_write_orientation_matrix(struct bhy_hif_handle *handle,
	bhy_u8 sensor_id, bhy_u8 *buf, bhy_u32 len);

/*!
* @brief Control BSX logging
* @param[in] handle: Host interface handle
* @param[in] modes: BSX logging modes (bits)
* @return Host interface error code
*/
bhy_s8 bhy_hif_set_bsx_logging_mode(struct bhy_hif_handle *handle,
	bhy_u8 modes);

/*!
* @brief set the maximum number of bytes of burst reading
* @param[in] handle: Host interface handle
* @param[in] max_burst: number of bytes of burst reading
*/
void bhy_hif_set_max_burst_read_bytes(struct bhy_hif_handle *handle,
	bhy_u32 max_burst);

/*!
* @brief get the maximum number of bytes of burst reading
* @param[in] handle: Host interface handle
* @return the maximum number of bytes of burst reading
*/
bhy_u32 bhy_hif_get_max_burst_read_bytes(struct bhy_hif_handle *handle);

/*!
* @brief Inject sensor data to firmware
* @param[in] handle: Host interface handle
* @param[in] buf: Mode to inject data
* @param[in] buf_len: Mode length
* @param[out] work_buf: returned buffer
* @param[in] work_buf_len:  Max buffer length
* @param[out] ret_len:  Returned buffer length
* @return Host interface error code
*/
bhy_s8 bhy_hif_set_inject_data_mode(struct bhy_hif_handle *handle, bhy_u8 *buf,
	bhy_u8 buf_len, bhy_u8 *work_buf, bhy_u32 work_buf_len, bhy_u32 *ret_len);

/*!
* @brief Inject data to firmware
* @param[in] handle: Host interface handle
* @param[in] buf: Sensor data need to be injected
* @param[in] buf_len: Sensor data length
* @return Host interface error code
*/
bhy_s8 bhy_hif_inject_data(struct bhy_hif_handle *handle, bhy_u8 *buf,
	bhy_u32 buf_len);
#endif /*~ BHY_HOST_INTERFACE_H */

/** @}*/

