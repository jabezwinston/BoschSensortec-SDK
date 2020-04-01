/*!
* @section LICENSE
* $license_gpl$
*
* @filename $filename$
* @date     $date$
* @id       $id$
*
* @brief
* The implementation file for BHy host interface
*/

/*!@addtogroup hostinterface
* @{*/

#ifdef __KERNEL__
#include <linux/string.h>
#include <linux/slab.h>
#else
#include <stdlib.h>
#include <string.h>
#endif /*~ __KERNEL__ */
#include <stdio.h> 
#include "bhy_host_interface.h"

void bhy_hif_init_handle(struct bhy_hif_handle *handle,
	int (*bus_read_func)(bhy_u8, bhy_u8 *, bhy_u32, void *),
	int (*bus_write_func)(bhy_u8, const bhy_u8 *, bhy_u32, void *),
	void (*delay_us_func)(bhy_u32, void *),
	void *private_data)
{
	handle->bus_read_func = bus_read_func;
	handle->bus_write_func = bus_write_func;
	handle->delay_us_func = delay_us_func;
	handle->private_data = private_data;
	bhy_hif_set_max_burst_read_bytes(handle, BHY2_DEFAULT_MAX_READ_BURST);
}

bhy_s8 bhy_hif_bus_read(struct bhy_hif_handle *handle,
	bhy_u8 reg, bhy_u8 *buf, bhy_u32 len)
{
	if (!handle)
		return BHY_HIF_E_HANDLE;
	if (handle->bus_read_func(reg, buf, len, handle->private_data) < 0)
		return BHY_HIF_E_IO;
	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_bus_write(struct bhy_hif_handle *handle,
	bhy_u8 reg, const bhy_u8 *buf, bhy_u32 len)
{
	if (!handle)
		return BHY_HIF_E_HANDLE;
	if (handle->bus_write_func(reg, buf, len, handle->private_data) < 0)
		return BHY_HIF_E_IO;
	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_delay_us(struct bhy_hif_handle *handle, bhy_u32 us)
{
	if (!handle)
		return BHY_HIF_E_HANDLE;
	handle->delay_us_func(us, handle->private_data);
	return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief Execute commands
 * @param[in] handle: Host interface handle
 * @param[in] cmd: Command code
 * @param[in] buf: Command data
 * @param[in] len: Command data length
 * @param[in] prebuf: prebuffer at the front of command data
 * @param[in] prelen: prebuffer length
 * @param[in] cmd_len: Command data length after prebuffer
 *
 * If a command need several frames to upload to sensor,
 * this command only be used in the first frame.
 *
 * |----------------------------------------|---------------|
 * |<--               frame one          -->|<--frame two-->|
 * |----------------------------------------|---------------|
 * |                 |<--prelen-->|<--len-->|
 * |-----------------|------------|---------|---------------|
 * | cmd | total len |   prebuf   |  data 1 |   data 2      |
 * |------------------------------|---------|---------------|
 *                                |        cmd data         |
 *                                |---------|---------------|
 *                                |<--     cmd_len       -->|
 *                                |-------------------------|
 *
 * @return Host interface error code
 */
static bhy_s8 bhy_hif_exec_cmd_internal(struct bhy_hif_handle *handle,
	bhy_u16 cmd, const bhy_u8 *buf, bhy_u32 len,
	const bhy_u8 *prebuf, bhy_u32 prelen, bhy_u32 cmd_len)
{
	bhy_s8 ret = BHY_HIF_E_SUCCESS;
	bhy_u32 remain, trans_len, copy_len, pos, total_len, temp_total_len;

	if (!handle)
		return BHY_HIF_E_HANDLE;
	if (len > 0 && !buf)
		return BHY_HIF_E_INVAL;
	if (prelen > 0 && !prebuf)
		return BHY_HIF_E_INVAL;

	total_len = prelen + len;
	if (cmd_len)
		temp_total_len = prelen + cmd_len;
	else
		temp_total_len = total_len;
	/* align 4 bytes */
	if (temp_total_len % 4)
		temp_total_len = 4 * (temp_total_len / 4 + 1);

	handle->command_buf[0] = cmd & 0xFF;
	handle->command_buf[1] = (cmd >> 8) & 0xFF;
	if (cmd == BHY_CMD_UPLOAD_TO_PROGRAM_RAM) { /* Length in word */
		handle->command_buf[2] = (temp_total_len / 4) & 0xFF;
		handle->command_buf[3] =
			((temp_total_len / 4) >> 8) & 0xFF;
	} else { /* Length in byte */
		handle->command_buf[2] = temp_total_len & 0xFF;
		handle->command_buf[3] = (temp_total_len >> 8) & 0xFF;
	}

	pos = 4;
	remain = total_len;
	while (remain + pos > 0) {
		if (remain + pos > BHY_COMMAND_PACKET_LEN) {
			trans_len = BHY_COMMAND_PACKET_LEN;
			copy_len = BHY_COMMAND_PACKET_LEN - pos;
		} else {
			trans_len = remain + pos;
			copy_len = remain;
			if ((trans_len & 0x03) != 0) /* Not multiple of 4 */
				trans_len = ((trans_len >> 2) + 1) << 2;
		}

		if (copy_len > 0) {
			if (remain >= len + copy_len)
				memcpy(handle->command_buf + pos,
					prebuf + total_len - remain, copy_len);
			else if (remain > len) {
				memcpy(handle->command_buf + pos,
					prebuf + total_len - remain,
					remain - len);
				memcpy(handle->command_buf + pos + remain - len,
					buf, copy_len - (remain - len));
			} else
				memcpy(handle->command_buf + pos,
					buf + len - remain, copy_len);
		}
		if (trans_len - pos - copy_len > 0)
			memset(handle->command_buf + pos + copy_len,
				0, trans_len - pos - copy_len);
		ret = bhy_hif_bus_write(handle, BHY_REG_CHAN_CMD,
			handle->command_buf, trans_len);
		if (ret < 0)
			return ret;
		pos = 0;
		remain -= copy_len;
	}

	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_exec_cmd(struct bhy_hif_handle *handle,
	bhy_u16 cmd, const bhy_u8 *buf, bhy_u32 len)
{
	return bhy_hif_exec_cmd_internal(handle, cmd, buf, len, NULL, 0, 0);
}

static bhy_s8 bhy_wait_status_ready(struct bhy_hif_handle *handle)
{
	bhy_u16 retry;
	bhy_u8 irq_status;
	bhy_s8 ret = BHY_HIF_E_TIMEOUT;

	/* wait status ready */
	for (retry = 0; retry < BHY_QUERY_PARAM_STATUS_READY_MAX_RETRY;
		++retry) {
		ret = bhy_hif_read_irq_status(handle, &irq_status);
		if (ret < 0)
			return ret;
		if (irq_status & BHY_IST_MASK_STATUS) {
			ret = BHY_HIF_E_SUCCESS;
			break;
		}
		ret = bhy_hif_delay_us(handle, 10000); /* 10ms */
		if (ret < 0)
			return ret;
	}
	return ret;
}

bhy_s8 bhy_hif_read_parameter(struct bhy_hif_handle *handle,
	bhy_u16 param, bhy_u8 *buf, bhy_u32 buf_len, bhy_u32 *ret_len)
{
	bhy_u16 code = 0;
	bhy_u8 old_status, _buf;
	bhy_s8 ret;

	if (!handle)
		return BHY_HIF_E_HANDLE;

	*ret_len = 0;
	ret = bhy_hif_bus_read(handle, BHY_REG_HOST_INTERFACE_CTRL, &_buf, 1);
	if (ret < 0)
		return ret;
	old_status = _buf;
	_buf &= ~BHY_IFCTL_ASYNC_STATUS_CHANNEL;
	ret = bhy_hif_bus_write(handle, BHY_REG_HOST_INTERFACE_CTRL, &_buf, 1);
	if (ret < 0)
		return ret;

	ret = bhy_hif_exec_cmd(handle, param | BHY_PARAM_READ_MASK, buf, 0);
	if (ret < 0)
		return ret;

	/* wait status ready */
	(void)bhy_wait_status_ready(handle);

	ret = bhy_hif_read_status(handle, &code, ret_len, buf, buf_len);
	if (ret < 0)
		return ret;

	_buf = old_status;
	ret = bhy_hif_bus_write(handle, BHY_REG_HOST_INTERFACE_CTRL, &_buf, 1);
	if (ret < 0)
		return ret;

	if (code != param)
		return BHY_HIF_E_TIMEOUT;

	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_write_parameter(struct bhy_hif_handle *handle,
	bhy_u16 param, const bhy_u8 *buf, bhy_u32 len)
{
	if (!handle)
		return BHY_HIF_E_HANDLE;
	return bhy_hif_exec_cmd(handle, param, buf, len);
}

bhy_s8 bhy_hif_read_fpga_version(struct bhy_hif_handle *handle, bhy_u8 *version)
{
	bhy_u8 buf;
	bhy_s8 ret;

	if (!handle)
		return BHY_HIF_E_HANDLE;
	ret = bhy_hif_bus_read(handle, BHY_REG_EV_TIME_REQ, &buf, sizeof(buf));
	if (ret < 0)
		return BHY_HIF_E_IO;
	*version = buf;
	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_read_product_id(struct bhy_hif_handle *handle, bhy_u8 *prod_id)
{
	bhy_u8 buf;
	bhy_s8 ret;

	if (!handle)
		return BHY_HIF_E_HANDLE;
	ret = bhy_hif_bus_read(handle, BHY_REG_PRODUCT_ID, &buf, sizeof(buf));
	if (ret < 0)
		return BHY_HIF_E_IO;
	*prod_id = buf;
	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_read_rom_version(struct bhy_hif_handle *handle, bhy_u16 *rom_ver)
{
	bhy_u8 buf[2];
	bhy_s8 ret;

	if (!handle)
		return BHY_HIF_E_HANDLE;
	ret = bhy_hif_bus_read(handle, BHY_REG_ROM_VERSION_0, buf, sizeof(buf));
	if (ret < 0)
		return BHY_HIF_E_IO;
	*rom_ver = BHY_LE2U16(buf);
	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_read_ram_kernel_version(struct bhy_hif_handle *handle,
	bhy_u16 *kernel_ver)
{
	bhy_u8 buf[2];
	bhy_s8 ret;

	if (!handle)
		return BHY_HIF_E_HANDLE;
	ret = bhy_hif_bus_read(handle, BHY_REG_RAM_VERSION_0, buf, sizeof(buf));
	if (ret < 0)
		return BHY_HIF_E_IO;
	*kernel_ver = BHY_LE2U16(buf);
	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_read_ram_user_version(struct bhy_hif_handle *handle,
	bhy_u16 *user_ver)
{
	bhy_u8 buf[2];
	bhy_s8 ret;

	if (!handle)
		return BHY_HIF_E_HANDLE;
	ret = bhy_hif_bus_read(handle, BHY_REG_FLASH_VERSION_0,
		buf, sizeof(buf));
	if (ret < 0)
		return BHY_HIF_E_IO;
	*user_ver = BHY_LE2U16(buf);
	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_read_irq_status(struct bhy_hif_handle *handle,
	bhy_u8 *irq_status)
{
	bhy_u8 buf;
	bhy_s8 ret;

	if (!handle)
		return BHY_HIF_E_HANDLE;
	ret = bhy_hif_bus_read(handle, BHY_REG_INT_STATUS, &buf, sizeof(buf));
	if (ret < 0)
		return BHY_HIF_E_IO;
	*irq_status = buf;
	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_read_fw_error(struct bhy_hif_handle *handle, bhy_u8 *fw_error)
{
	bhy_u8 buf;
	bhy_s8 ret;

	if (!handle)
		return BHY_HIF_E_HANDLE;
	ret = bhy_hif_bus_read(handle, BHY_REG_ERROR, &buf, 1);
	if (ret < 0)
		return BHY_HIF_E_IO;
	*fw_error = buf;
	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_reset(struct bhy_hif_handle *handle)
{
	bhy_u8 buf;
	bhy_s8 ret;
        bhy_u8 retry = 0;
  
	if (!handle)
		return BHY_HIF_E_HANDLE;

	buf = 1;
	ret = bhy_hif_bus_write(handle, BHY_REG_RESET_REQ, &buf, sizeof(buf));
	if (ret < 0)
		return BHY_HIF_E_IO;
	ret = bhy_hif_delay_us(handle, 100000); /* 100 ms */
	if (ret < 0)
		return ret;
	ret = bhy_hif_bus_read(handle, BHY_REG_BOOT_STATUS, &buf, sizeof(buf));
        if(ret < 0)
             return BHY_HIF_E_IO;
   
        while(!(buf & BHY_BST_HOST_INTERFACE_READY))
        { 
          //reset hub again
          //printf("reset hub retry = %d\r\n",retry);
          retry++; 
          if(retry > 3)
          {
            printf("SW reset hub three times, but still failed. You can try hardware reset.\r\n");
            return BHY_HIF_E_TIMEOUT;
          }
          ret = bhy_hif_bus_write(handle, BHY_REG_RESET_REQ, &buf, sizeof(buf));         
	  if (ret < 0)
		return BHY_HIF_E_IO;
	  ret = bhy_hif_delay_us(handle,100000); /*100ms*/
          if(ret < 0)
              return ret;
          ret = bhy_hif_bus_read(handle, BHY_REG_BOOT_STATUS, &buf, sizeof(buf)); 
	} 
	return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief Check boot status when use ram firmware
 * @param[in] handle: Host interface handle
 * @return Host interface error code
 */
static bhy_s8 bhy_hif_check_boot_status_ram(struct bhy_hif_handle *handle)
{
	bhy_s8 ret;
	bhy_u16 i;
	bhy_u8 bs;

	for (i = 0; i < BHY_BST_CHECK_RETRY; ++i) { /* total 5s */
		ret = bhy_hif_delay_us(handle, 50000); /* 50ms */
		if (ret < 0)
			return ret;
		ret = bhy_hif_bus_read(handle, BHY_REG_BOOT_STATUS,
			&bs, sizeof(bs));
		if (ret < 0)
			return ret;
		if ((bs & BHY_BST_HOST_INTERFACE_READY) &&
			(bs & BHY_BST_HOST_FW_VERIFY_DONE) &&
			(!(bs & BHY_BST_HOST_FW_VERIFY_ERROR)))
			break;
	}
	if (i == BHY_BST_CHECK_RETRY)
		return BHY_HIF_E_TIMEOUT;

	return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief Check boot status when use flash firmware
 * @param[in] handle: Host interface handle
 * @return Host interface error code
 */
static bhy_s8 bhy_hif_check_boot_status_flash(struct bhy_hif_handle *handle)
{
	bhy_s8 ret;
	bhy_u16 i;
	bhy_u8 bs;

	for (i = 0; i < BHY_BST_CHECK_RETRY; ++i) { /* total 5s */
		ret = bhy_hif_delay_us(handle, 50000); /* 50ms */
		if (ret < 0)
			return ret;
		ret = bhy_hif_bus_read(handle, BHY_REG_BOOT_STATUS,
			&bs, sizeof(bs));
		if (ret < 0)
			return ret;
		if ((bs & BHY_BST_HOST_INTERFACE_READY) &&
			(bs & BHY_BST_FLASH_VERIFY_DONE) &&
			(!(bs & BHY_BST_FLASH_VERIFY_ERROR)))
			break;
	}
	if (i == BHY_BST_CHECK_RETRY)
		return BHY_HIF_E_TIMEOUT;

	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_upload_to_ram(struct bhy_hif_handle *handle,
	const bhy_u8 *buf, bhy_u32 len)
{
	bhy_s8 ret;
	struct bhy_fw_header *header;
	bhy_u16 magic;

	if (!handle)
		return BHY_HIF_E_HANDLE;

	/* check firmware header */
	header = (struct bhy_fw_header *)buf;
	magic = BHY_LE2U16(header->magic);
	if (magic != BHY_FW_MAGIC)
		return BHY_HIF_E_MAGIC;

	/* upload program ram */
	ret = bhy_hif_exec_cmd(handle, BHY_CMD_UPLOAD_TO_PROGRAM_RAM, buf, len);
	if (ret < 0)
		return ret;
	ret = bhy_hif_check_boot_status_ram(handle);
	if (ret < 0)
		return ret;

	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_upload_to_ram_partly(struct bhy_hif_handle *handle,
	const bhy_u8 *buf, bhy_u32 total_len, bhy_u32 cur_pos,
	bhy_u32 pack_len)
{
	bhy_s8 ret;
	struct bhy_fw_header *header;
	bhy_u16 magic;

	if (!handle)
		return BHY_HIF_E_HANDLE;

	if (cur_pos == 0) {
		/* check firmware header */
		header = (struct bhy_fw_header *)buf;
		magic = BHY_LE2U16(header->magic);
		if (magic != BHY_FW_MAGIC)
			return BHY_HIF_E_MAGIC;
		/* the first time to upload ram firmware to ram */
		ret = bhy_hif_exec_cmd_internal(handle,
			BHY_CMD_UPLOAD_TO_PROGRAM_RAM,
			buf, pack_len, NULL, 0, total_len);
	} else {
		/* the other times to upload ram firmware to ram */
		ret = bhy_hif_bus_write(handle, BHY_REG_CHAN_CMD,
			buf, pack_len);
	}

	if (ret < 0)
		return ret;

	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_boot_program_ram(struct bhy_hif_handle *handle)
{
	bhy_s8 ret;

	if (!handle)
		return BHY_HIF_E_HANDLE;
	ret = bhy_hif_exec_cmd(handle, BHY_CMD_BOOT_PROGRAM_RAM, NULL, 0);
	if (ret < 0)
		return ret;
	ret = bhy_hif_check_boot_status_ram(handle);
	if (ret < 0)
		return ret;
	return BHY_HIF_E_SUCCESS;
}

void bhy_hif_set_max_burst_read_bytes(struct bhy_hif_handle *handle,
	bhy_u32 max_burst)
{
	handle->max_burst = max_burst;
}

bhy_u32 bhy_hif_get_max_burst_read_bytes(struct bhy_hif_handle *handle)
{
	return handle->max_burst;
}

static bhy_s8 bhy_hif_read_fifo(struct bhy_hif_handle *handle,
	bhy_u8 reg, bhy_u8 *buf, bhy_u32 buf_len, bhy_u32 *ret_len, bhy_u32* remain_len)
{
	bhy_s8 ret = BHY_HIF_E_SUCCESS;
	bhy_u8 _buf[2];
	bhy_u32 read_len;
	bhy_u32 max_burst;

	max_burst = bhy_hif_get_max_burst_read_bytes(handle);

	if (!handle)
		return BHY_HIF_E_HANDLE;
	if(*remain_len == 0)
	{
		ret = bhy_hif_bus_read(handle, reg, _buf, sizeof(_buf));
		*remain_len = BHY_LE2U16(_buf);
		if (*remain_len == 0)
			return BHY_HIF_E_SUCCESS;
		if (ret < 0)
			return BHY_HIF_E_IO;
	}
	if (buf_len < *remain_len)
		*ret_len = buf_len;
	else
		*ret_len = *remain_len;

	read_len = *ret_len;
	while (read_len > max_burst) {
		ret = bhy_hif_bus_read(handle, reg, buf, max_burst);
		if (ret < 0)
			return BHY_HIF_E_IO;
		read_len -= max_burst;
		buf += max_burst;
	}

	ret = bhy_hif_bus_read(handle, reg, buf, read_len);
	if (ret < 0)
		return BHY_HIF_E_IO;

	*remain_len -= read_len;

	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_read_wakeup_fifo(struct bhy_hif_handle *handle,
	bhy_u32 *ret_len, bhy_u8 *buf, bhy_u32 buf_len, bhy_u32* remain_len)
{
	return bhy_hif_read_fifo(handle, BHY_REG_CHAN_FIFO_W, buf, buf_len,
		ret_len, remain_len);
}

bhy_s8 bhy_hif_read_nonwakeup_fifo(struct bhy_hif_handle *handle,
	bhy_u32 *ret_len, bhy_u8 *buf, bhy_u32 buf_len, bhy_u32* remain_len)
{
	return bhy_hif_read_fifo(handle, BHY_REG_CHAN_FIFO_NW, buf, buf_len,
		ret_len, remain_len);
}

bhy_s8 bhy_hif_read_status(struct bhy_hif_handle *handle,
	bhy_u16 *code, bhy_u32 *ret_len, bhy_u8 *buf, bhy_u32 buf_len)
{
	bhy_s8 ret;
	bhy_u8 _buf[4];

	if (!handle)
		return BHY_HIF_E_HANDLE;
	ret = bhy_hif_bus_read(handle, BHY_REG_CHAN_STATUS,
		_buf, sizeof(_buf));
	if (ret < 0)
		return BHY_HIF_E_IO;
	*code = BHY_LE2U16(&_buf[0]);
	*ret_len = BHY_LE2U16(&_buf[2]);
	if (*ret_len == 0)
		return BHY_HIF_E_SUCCESS;
	if (buf_len < *ret_len)
		return BHY_HIF_E_BUF;
	ret = bhy_hif_bus_read(handle, BHY_REG_CHAN_STATUS, buf, *ret_len);
	if (ret < 0)
		return BHY_HIF_E_IO;
	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_read_status_async(struct bhy_hif_handle *handle,
	bhy_u32 *ret_len, bhy_u8 *buf, bhy_u32 buf_len, bhy_u32* remain_len)
{
	return bhy_hif_read_fifo(handle, BHY_REG_CHAN_STATUS,
		buf, buf_len, ret_len, remain_len);
}

bhy_s8 bhy_hif_exec_sensor_conf_cmd(struct bhy_hif_handle *handle,
	bhy_u8 sensor_id, bhy_u32 sample_rate, bhy_u32 latency)
{
	bhy_u8 _buf[8];

	/*sample rate in 32bits, latency in 24bits*/
	_buf[0] = sensor_id;
	_buf[1] = sample_rate & 0xFF;
	_buf[2] = (sample_rate >> 8) & 0xFF;
	_buf[3] = (sample_rate >> 16) & 0xFF;
	_buf[4] = (sample_rate >> 24) & 0xFF;
	_buf[5] = latency & 0xFF;
	_buf[6] = (latency >> 8) & 0xFF;
	_buf[7] = (latency >> 16) & 0xFF;

	return bhy_hif_exec_cmd(handle, BHY_CMD_CONFIG_SENSOR,
		_buf, sizeof(_buf));
}

bhy_s8 bhy_hif_write_fifo_flush(struct bhy_hif_handle *handle, bhy_u8 sensor_id)
{
	bhy_u8 _buf[4];
	_buf[0] = sensor_id & 0xFF;
	_buf[1] = _buf[2] = _buf[3] = 0;
	return bhy_hif_exec_cmd(handle, BHY_CMD_FIFO_FLUSH, _buf, 4);
}

bhy_s8 bhy_hif_exec_soft_passthrough(struct bhy_hif_handle *handle,
	bhy_u8 *buf, bhy_u32 buf_len, bhy_u8 *reg_data,
	bhy_u32 reg_buf_len, bhy_u32 *reg_len)
{
	bhy_u16 code = 0;
	bhy_u8 old_status, _buf;
	bhy_s8 ret = BHY_HIF_E_SUCCESS;

	if (!handle)
		return BHY_HIF_E_HANDLE;

	/* set synchronous status */
	ret = bhy_hif_bus_read(handle, BHY_REG_HOST_INTERFACE_CTRL, &_buf, 1);
	if (ret < 0)
		return ret;
	old_status = _buf;
	_buf &= ~BHY_IFCTL_ASYNC_STATUS_CHANNEL;
	ret = bhy_hif_bus_write(handle, BHY_REG_HOST_INTERFACE_CTRL, &_buf, 1);
	if (ret < 0)
		return ret;

	/* do sw passthrough */
	ret = bhy_hif_exec_cmd(handle, BHY_CMD_SW_PASSTHROUGH, buf, buf_len);

	if (ret < 0)
		return ret;
	/* wait status ready */
	(void)bhy_wait_status_ready(handle);
	/* read status for result */
	ret = bhy_hif_read_status(handle, &code, reg_len, reg_data,
		reg_buf_len);
	if (ret < 0)
		return ret;

	/* set asynchronous status */
	_buf = old_status;
	ret = bhy_hif_bus_write(handle, BHY_REG_HOST_INTERFACE_CTRL, &_buf, 1);
	if (ret < 0)
		return ret;

	if (code != BHY_STATUS_SW_PASS_THRU_RES)
		return BHY_HIF_E_TIMEOUT;

	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_read_post_mortem(struct bhy_hif_handle *handle,
	bhy_u16 *code, bhy_u32 *ret_len, bhy_u8 *buf, bhy_u32 buf_len)
{
	bhy_u8 old_status, _buf;
	bhy_s8 ret;

	if (!handle)
		return BHY_HIF_E_HANDLE;

	ret = bhy_hif_bus_read(handle, BHY_REG_HOST_INTERFACE_CTRL, &_buf, 1);
	if (ret < 0)
		return ret;
	old_status = _buf;
	_buf &= ~BHY_IFCTL_ASYNC_STATUS_CHANNEL;
	ret = bhy_hif_bus_write(handle, BHY_REG_HOST_INTERFACE_CTRL, &_buf, 1);
	if (ret < 0)
		return ret;

	ret = bhy_hif_exec_cmd(handle, BHY_CMD_REQ_POST_MORTEM_DATA, NULL, 0);
	if (ret < 0)
		return ret;

	/* wait status ready */
	(void)bhy_wait_status_ready(handle);
	ret = bhy_hif_read_status(handle, code, ret_len, buf,
		buf_len);
	if (ret < 0)
		return ret;

	_buf = old_status;
	ret = bhy_hif_bus_write(handle, BHY_REG_HOST_INTERFACE_CTRL, &_buf, 1);
	if (ret < 0)
		return ret;

	if (*code != BHY_STATUS_CRASH_DUMP)
		return BHY_HIF_E_TIMEOUT;

	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_do_self_test(struct bhy_hif_handle *handle, bhy_u8 sensor_id,
	bhy_u8 *ret_status, bhy_s16 *offset)
{
	bhy_u16 code = 0;
	bhy_u8 old_status, _buf, req[4], ret_data[16];
	bhy_u32 ret_len = 0;
	bhy_s8 ret;

	if (!handle)
		return BHY_HIF_E_HANDLE;

	ret = bhy_hif_bus_read(handle, BHY_REG_HOST_INTERFACE_CTRL, &_buf, 1);
	if (ret < 0)
		return ret;
	old_status = _buf;
	_buf &= ~BHY_IFCTL_ASYNC_STATUS_CHANNEL;
	ret = bhy_hif_bus_write(handle, BHY_REG_HOST_INTERFACE_CTRL, &_buf, 1);
	if (ret < 0)
		return ret;

	req[0] = sensor_id & 0xFF;
	req[1] = req[2] = req[3] = 0;
	ret = bhy_hif_exec_cmd(handle, BHY_CMD_REQ_SELF_TEST, req, sizeof(req));
	if (ret < 0)
		return ret;

	/* wait status ready */
	(void)bhy_wait_status_ready(handle);
	ret = bhy_hif_read_status(handle, &code, &ret_len, ret_data,
		sizeof(ret_data));
	if (ret < 0)
		return ret;

	_buf = old_status;
	ret = bhy_hif_bus_write(handle, BHY_REG_HOST_INTERFACE_CTRL, &_buf, 1);
	if (ret < 0)
		return ret;

	if (code != BHY_STATUS_SELF_TEST_RES)
		return BHY_HIF_E_TIMEOUT;
	if (ret_data[0] != sensor_id)
		return BHY_HIF_E_INVAL;

	*ret_status = (bhy_s8)ret_data[1];
	offset[0] = BHY_LE2S16(&ret_data[2]);
	offset[1] = BHY_LE2S16(&ret_data[4]);
	offset[2] = BHY_LE2S16(&ret_data[6]);

	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_do_foc(struct bhy_hif_handle *handle, bhy_u8 sensor_id,
	bhy_u8 *ret_status, bhy_s16 *offset)
{
	bhy_u16 code = 0;
	bhy_u8 old_status, _buf, req[4], ret_data[16];
	bhy_u32 ret_len = 0;
	bhy_s8 ret;

	if (!handle)
		return BHY_HIF_E_HANDLE;

	ret = bhy_hif_bus_read(handle, BHY_REG_HOST_INTERFACE_CTRL, &_buf, 1);
	if (ret < 0)
		return ret;
	old_status = _buf;
	_buf &= ~BHY_IFCTL_ASYNC_STATUS_CHANNEL;
	ret = bhy_hif_bus_write(handle, BHY_REG_HOST_INTERFACE_CTRL, &_buf, 1);
	if (ret < 0)
		return ret;

	req[0] = sensor_id & 0xFF;
	req[1] = req[2] = req[3] = 0;
	ret = bhy_hif_exec_cmd(handle, BHY_CMD_REQ_FOC, req,
		sizeof(req));
	if (ret < 0)
		return ret;

	/* wait status ready */
	(void)bhy_wait_status_ready(handle);
	ret = bhy_hif_read_status(handle, &code, &ret_len, ret_data,
		sizeof(ret_data));
	if (ret < 0)
		return ret;

	_buf = old_status;
	ret = bhy_hif_bus_write(handle, BHY_REG_HOST_INTERFACE_CTRL, &_buf, 1);
	if (ret < 0)
		return ret;

	if (code != BHY_STATUS_FOC_RES)
		return BHY_HIF_E_TIMEOUT;
	if (ret_data[0] != sensor_id)
		return BHY_HIF_E_INVAL;

	*ret_status = (bhy_s8)ret_data[1];
	offset[0] = BHY_LE2S16(&ret_data[2]);
	offset[1] = BHY_LE2S16(&ret_data[4]);
	offset[2] = BHY_LE2S16(&ret_data[6]);

	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_read_sensor_ctrl(struct bhy_hif_handle *handle, bhy_u8 sensor_id,
	bhy_u8 cmd, bhy_u8 *ret_data, bhy_u32 *data_len)
{
	bhy_u16 code = 0;
	bhy_u8 old_status = 0;
	bhy_u8 _buf[4] = {0x0};
	bhy_u8 buf = 0;
	bhy_u32 len = 0;
	bhy_u16 param_num = 0;
	bhy_s8 ret;

	if (!handle)
		return BHY_HIF_E_HANDLE;

	ret = bhy_hif_bus_read(handle, BHY_REG_HOST_INTERFACE_CTRL, &buf, 1);
	if (ret < 0)
		return ret;
	old_status = buf;
	buf &= ~BHY_IFCTL_ASYNC_STATUS_CHANNEL;
	ret = bhy_hif_bus_write(handle, BHY_REG_HOST_INTERFACE_CTRL, &buf, 1);
	if (ret < 0)
		return ret;

	/* set parameter to request data */
	param_num = BHY_PARAM_SET_SENSOR_CTRL | sensor_id;
	_buf[0] = cmd | BHY_PARAM_SENSOR_CTRL_READ;
	len = sizeof(_buf);
	ret = bhy_hif_write_parameter(handle, param_num, _buf, len);
	if (ret < 0)
		return ret;

	/* get parameter to read data actually */
	param_num = BHY_PARAM_GET_SENSOR_CTRL | sensor_id;
	len = 0;
	ret = bhy_hif_write_parameter(handle, param_num, NULL, len);
	if (ret < 0)
		return ret;

	/* wait status ready */
	(void)bhy_wait_status_ready(handle);
	ret = bhy_hif_read_status(handle, &code, data_len, ret_data,
		sizeof(ret_data));
	if (ret < 0)
		return ret;

	buf = old_status;
	ret = bhy_hif_bus_write(handle, BHY_REG_HOST_INTERFACE_CTRL, &buf, 1);
	if (ret < 0)
		return ret;

	if (code != (BHY_PARAM_SET_SENSOR_CTRL | sensor_id))
		return BHY_HIF_E_INVAL;
	if (ret_data[0] != cmd)
		return BHY_HIF_E_INVAL;

	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_write_sensor_ctrl(struct bhy_hif_handle *handle,
	bhy_u8 sensor_id, bhy_u8 cmd, bhy_u8 *buf, bhy_u32 buf_len)
{
	bhy_u16 param_num = 0;
	bhy_u8 command;
	bhy_s8 ret;

	/* set parameter to request data */
	param_num = BHY_PARAM_SET_SENSOR_CTRL | sensor_id;
	command = cmd & (~BHY_PARAM_SENSOR_CTRL_READ);
	ret = bhy_hif_exec_cmd_internal(handle, param_num,
			buf, buf_len, &command, 1, 0);
	if (ret < 0)
		return ret;

	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_read_bsx_state(struct bhy_hif_handle *handle,
	bhy_u16 param_num, bhy_u8 *buf, bhy_u32 buf_len, bhy_u32 *ret_len)
{
	bhy_u8 complete_flag = 0;
	bhy_u8 section_num = 0;
	bhy_u16 state_len = 0;
	bhy_u32 read_len;
	bhy_s8 ret;
	bhy_u8 tmp_bsx_state_buf[BSX_STATE_STRUCT_LEN];

	while (complete_flag == 0) {
		ret = bhy_hif_read_parameter(handle, param_num,
				tmp_bsx_state_buf, BSX_STATE_STRUCT_LEN,
				&read_len);
		if (ret < 0)
			return ret;
		if (read_len != BSX_STATE_STRUCT_LEN)
			return BHY_HIF_E_INVAL;

		complete_flag = tmp_bsx_state_buf[0]
			& BSX_STATE_TRANSFER_COMPLETE;
		section_num = tmp_bsx_state_buf[0] & 0x7F;
		state_len = BHY_LE2U16(&tmp_bsx_state_buf[2]);
		if (!state_len)
			return BHY_HIF_E_INVAL;
		if (buf_len < state_len)
			return BHY_HIF_E_BUF;

		memcpy(&buf[section_num * BSX_STATE_BLOCK_LEN],
				&tmp_bsx_state_buf[4],
				tmp_bsx_state_buf[1]);
		/* get whole state data length */
		*ret_len = state_len;
	}
	return ret;
}

bhy_s8 bhy_hif_write_bsx_state(struct bhy_hif_handle *handle,
	bhy_u16 param_num, const bhy_u8 *buf, bhy_u32 buf_len)
{
	bhy_u16 pos;
	bhy_u16 num = 0;
	bhy_s8 ret;
	bhy_u8 tmp_bsx_state_buf[BSX_STATE_STRUCT_LEN];

	for (pos = 0; pos < buf_len; pos += BSX_STATE_BLOCK_LEN, ++num) {
		tmp_bsx_state_buf[0] = num & 0x7F;

		if ((buf_len - pos) <= BSX_STATE_BLOCK_LEN) {
			tmp_bsx_state_buf[0] |= 0x80;
			tmp_bsx_state_buf[1] = (buf_len - pos);
		} else
			tmp_bsx_state_buf[1] = BSX_STATE_BLOCK_LEN;

		tmp_bsx_state_buf[2] = buf_len & 0xFF;
		tmp_bsx_state_buf[3] = (buf_len >> 8) & 0xFF;

		memcpy(&tmp_bsx_state_buf[4], &buf[pos],
			tmp_bsx_state_buf[1]);
		ret = bhy_hif_write_parameter(handle, param_num,
			tmp_bsx_state_buf, BSX_STATE_STRUCT_LEN);
		if (ret < 0)
			return ret;
	}
	return 0;
}

bhy_s8 bhy_hif_erase_flash(struct bhy_hif_handle *handle, bhy_u8 *work_buf,
	bhy_u32 work_buf_len, bhy_u32 *ret_len)
{
	bhy_u8 buf[8];
	bhy_u8 irq_status = 0;
	bhy_u16 retry;
	bhy_u16 code = 0;
	bhy_u16 param_num;
	bhy_s8 ret;

	buf[0] = buf[1] = buf[2] = buf[3] = 0xff;
	buf[4] = buf[5] = buf[6] = buf[7] = 0xff;

	/* erase flash */
	param_num = BHY_CMD_ERASE_FLASH;
	ret = bhy_hif_write_parameter(handle, param_num, buf, 8);
	if (ret < 0)
		return ret;

	/* wait for status */
	for (retry = 0; retry < BHY_QUERY_FLASH_MAX_RETRY; ++retry) {
		ret = bhy_hif_read_irq_status(handle, &irq_status);
		if (ret < 0)
			return ret;
		if (irq_status & BHY_IST_MASK_STATUS)
			break;
		/* need at least wait about 20s for erase complete */
		ret = bhy_hif_delay_us(handle, 50000); /* 5 ms */
		if (ret < 0)
			return ret;
	}

	/* read status */
	ret = bhy_hif_read_status(handle, &code, ret_len, work_buf,
		work_buf_len);
	if (ret < 0)
		return ret;
	/* status code */
	if (code != BHY_STATUS_FLASH_ERASE_COMPLETE)
		return BHY_HIF_E_TIMEOUT;

	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_write_flash(struct bhy_hif_handle *handle,
	const bhy_u8 *buf, bhy_u32 len, bhy_u8 *work_buf, bhy_u32 work_buf_len,
	bhy_u32 *ret_len)
{
	struct bhy_fw_header *header;
	bhy_u16 magic;
	bhy_u32 sector_addr = 0x1f84; /* sector start addr in 4bytes */
	bhy_u8 addr_buf[4];
	static bhy_u32 remain, trans_len, pos;
	bhy_u16 code;
	bhy_s8 ret;

	/* check firmware header */
	header = (struct bhy_fw_header *)buf;
	magic = BHY_LE2U16(header->magic);
	if (magic != BHY_FW_MAGIC)
		return BHY_HIF_E_MAGIC;

	pos = 0;
	remain = len;
	while (remain > 0) {
		if (remain > BHY_FLASH_FW_LEN_UNIT)
			trans_len = BHY_FLASH_FW_LEN_UNIT;
		else
			trans_len = remain;

		/* set sector address */
		addr_buf[0] = sector_addr & 0xFF;
		addr_buf[1] = (sector_addr >> 8) & 0xFF;
		addr_buf[2] = (sector_addr >> 16) & 0xFF;
		addr_buf[3] = (sector_addr >> 24) & 0xFF;
		/* transfer flash firmware data */
		ret = bhy_hif_exec_cmd_internal(handle, BHY_CMD_WRITE_FLASH,
				buf + pos, trans_len, addr_buf,
				sizeof(addr_buf), 0);
		if (ret < 0)
			return ret;

		/* wait status ready */
		(void)bhy_wait_status_ready(handle);
		/* read status */
		ret = bhy_hif_read_status(handle, &code, ret_len,
			work_buf, work_buf_len);
		if (ret < 0)
			return ret;
		/* status code */
		if (code != BHY_STATUS_FLASH_WRITE_COMPLETE)
			return BHY_HIF_E_TIMEOUT;

		pos += trans_len;
		remain -= trans_len;
		sector_addr += BHY_FLASH_FW_LEN_UNIT;
	}

	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_upload_to_flash_partly(struct bhy_hif_handle *handle,
	const bhy_u8 *buf, bhy_u32 total_len, bhy_u32 cur_pos,
	bhy_u32 pack_len)
{
	bhy_s8 ret;
	struct bhy_fw_header *header;
	bhy_u16 magic;
	bhy_u32 sector_addr = 0x1f84; /* sector start addr in 4bytes */
	bhy_u8 addr_buf[4];

	if (!handle)
		return BHY_HIF_E_HANDLE;

	if (cur_pos == 0) {
		/* check firmware header */
		header = (struct bhy_fw_header *)buf;
		magic = BHY_LE2U16(header->magic);
		/* set sector address */
		addr_buf[0] = sector_addr & 0xFF;
		addr_buf[1] = (sector_addr >> 8) & 0xFF;
		addr_buf[2] = (sector_addr >> 16) & 0xFF;
		addr_buf[3] = (sector_addr >> 24) & 0xFF;
		/* transfer flash firmware data */
		if (magic != BHY_FW_MAGIC)
			return BHY_HIF_E_MAGIC;
		/* the first time to upload ram firmware to ram */
		ret = bhy_hif_exec_cmd_internal(handle, BHY_CMD_WRITE_FLASH,
			buf, pack_len, addr_buf, sizeof(addr_buf), total_len);
	} else {
		/* the other times to upload ram firmware to ram */
		ret = bhy_hif_bus_write(handle, BHY_REG_CHAN_CMD,
			buf, pack_len);
	}

	if (ret < 0)
		return ret;

	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_boot_from_flash(struct bhy_hif_handle *handle)
{
	bhy_s8 ret;

	ret = bhy_hif_exec_cmd(handle, BHY_CMD_BOOT_FLASH, NULL, 0);
	if (ret < 0)
		return BHY_HIF_E_IO;

	ret = bhy_hif_delay_us(handle, 2000 * 1000);
	if (ret < 0)
		return BHY_HIF_E_HANDLE;

	ret = bhy_hif_check_boot_status_flash(handle);
	if (ret < 0)
		return ret;

	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_request_hw_timestamp(struct bhy_hif_handle *handle)
{
	bhy_u8 _buf[1];
	_buf[0] = 1;

	return bhy_hif_bus_write(handle, BHY_REG_EV_TIME_REQ, _buf, 1);
}

bhy_s8 bhy_hif_read_hw_timestamp(struct bhy_hif_handle *handle, bhy_u64 *ts)
{
	bhy_s8 ret;
	bhy_u8 _buf[5];
	ret = bhy_hif_bus_read(handle, BHY_REG_HOST_IRQ_TIMESTAMP_0, _buf, 5);
	if (ret < 0)
		return ret;
	*ts = BHY_LE2U40(_buf);
	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_req_and_read_hw_timestamp(struct bhy_hif_handle *handle,
	bhy_u64 *ts)
{
	bhy_s8 ret;
	bhy_u8 retry;
	bhy_u8 _buf[5];
	bhy_u64 ts_old;

	if (!handle)
		return BHY_HIF_E_HANDLE;
	ret = bhy_hif_bus_read(handle, BHY_REG_HOST_IRQ_TIMESTAMP_0, _buf, 5);
	if (ret < 0)
		return ret;
	ts_old = BHY_LE2U40(_buf);
	_buf[0] = 1;
	ret = bhy_hif_bus_write(handle, BHY_REG_EV_TIME_REQ, _buf, 1);
	if (ret < 0)
		return ret;
	for (retry = 0; retry < 5; ++retry) {
		ret = bhy_hif_bus_read(handle,
			BHY_REG_HOST_IRQ_TIMESTAMP_0, _buf, 5);
		*ts = BHY_LE2U40(_buf);
		if (*ts != ts_old)
			return BHY_HIF_E_SUCCESS;
		ret = bhy_hif_delay_us(handle, 10); /* 10 us */
		if (ret < 0)
			return ret;
	}

	return BHY_HIF_E_TIMEOUT;
}

bhy_s8 bhy_hif_read_sensor_info(struct bhy_hif_handle *handle,
	bhy_u8 sensor_id, union bhy_sensor_info *info)
{
	bhy_s8 ret;
	bhy_u32 len;

	if (!handle)
		return BHY_HIF_E_HANDLE;
	ret = bhy_hif_read_parameter(handle,
		BHY_PARAM_SENSOR_INFO_0 + sensor_id,
		info->bytes, sizeof(*info), &len);
	if (ret < 0)
		return ret;
	if (len != sizeof(*info))
		return BHY_HIF_E_INVAL;

	info->info.max_range.u16_val = BHY_LE2U16(info->info.max_range.bytes);
	info->info.resolution.u16_val =
		BHY_LE2U16(info->info.resolution.bytes);
	info->info.max_rate.u32_val = BHY_LE2U32(info->info.max_rate.bytes);
	info->info.fifo_reserved.u32_val =
		BHY_LE2U32(info->info.fifo_reserved.bytes);
	info->info.fifo_max.u32_val = BHY_LE2U32(info->info.fifo_max.bytes);
	info->info.min_rate.u32_val = BHY_LE2U32(info->info.min_rate.bytes);

	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_read_phys_sensor_info(struct bhy_hif_handle *handle,
	bhy_u8 sensor_id, union bhy_phys_sensor_info *info)
{
	bhy_s8 ret;
	bhy_u32 len;

	if (!handle)
		return BHY_HIF_E_HANDLE;
	ret = bhy_hif_read_parameter(handle,
		BHY_PARAM_SYS_PHYS_SENSOR_INFO_0 + sensor_id,
		info->bytes, sizeof(*info), &len);
	if (ret < 0)
		return ret;
	if (len != sizeof(*info))
		return BHY_HIF_E_INVAL;

	info->info.curr_range.u16_val = BHY_LE2U16(info->info.curr_range.bytes);
	info->info.curr_rate.u32_val = BHY_LE2U32(info->info.curr_rate.bytes);

	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_write_orientation_matrix(struct bhy_hif_handle *handle,
	bhy_u8 sensor_id, bhy_u8 *buf, bhy_u32 len)
{
	bhy_s8 ret;

	if (!handle)
		return BHY_HIF_E_HANDLE;
	ret = bhy_hif_write_parameter(handle,
		BHY_PARAM_SYS_PHYS_SENSOR_INFO_0 + sensor_id, buf, len);
	if (ret < 0)
		return ret;

	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_set_bsx_logging_mode(struct bhy_hif_handle *handle, bhy_u8 modes)
{
	bhy_u8 _buf[4];

	if (!handle)
		return BHY_HIF_E_HANDLE;
	_buf[0] = modes & 0xFF;
	_buf[1] = _buf[2] = _buf[3] = 0;
	return bhy_hif_exec_cmd(handle, BHY_CMD_BSX_LOGGING_CTRL, _buf,
		sizeof(_buf));
}

bhy_s8 bhy_hif_set_inject_data_mode(struct bhy_hif_handle *handle,
	bhy_u8 *buf, bhy_u8 buf_len, bhy_u8 *work_buf,
	bhy_u32 work_buf_len, bhy_u32 *ret_len)
{
	bhy_u16 code = 0;
	bhy_u8 old_status, _buf;
	bhy_s8 ret = BHY_HIF_E_SUCCESS;

	if (!handle)
		return BHY_HIF_E_HANDLE;

	ret = bhy_hif_bus_read(handle, BHY_REG_HOST_INTERFACE_CTRL, &_buf, 1);
	if (ret < 0)
		return ret;
	old_status = _buf;
	_buf &= ~BHY_IFCTL_ASYNC_STATUS_CHANNEL;
	ret = bhy_hif_bus_write(handle, BHY_REG_HOST_INTERFACE_CTRL, &_buf, 1);
	if (ret < 0)
		return ret;

	/* Set injection mode */
	ret = bhy_hif_exec_cmd(handle, BHY_CMD_SET_INJECT_MODE, buf, buf_len);
	if (ret < 0)
		return ret;

	/* wait status ready */
	(void)bhy_wait_status_ready(handle);
	ret = bhy_hif_read_status(handle, &code, ret_len,
		work_buf, work_buf_len);
	if (ret < 0)
		return ret;

	_buf = old_status;
	ret = bhy_hif_bus_write(handle, BHY_REG_HOST_INTERFACE_CTRL, &_buf, 1);
	if (ret < 0)
		return ret;

	if (code != BHY_STATUS_INJECT_SENSOR_CONF_REQ)
		return BHY_HIF_E_TIMEOUT;

	return BHY_HIF_E_SUCCESS;
}

bhy_s8 bhy_hif_inject_data(struct bhy_hif_handle *handle, bhy_u8 *buf,
		bhy_u32 buf_len)
{
	bhy_s8 ret = BHY_HIF_E_SUCCESS;

	if (!handle)
		return BHY_HIF_E_HANDLE;

	/* inject data to hub */
	ret = bhy_hif_exec_cmd(handle, BHY_CMD_INJECT_DATA, buf, buf_len);
	if (ret < 0)
		return ret;

	return BHY_HIF_E_SUCCESS;
}

/** @}*/

