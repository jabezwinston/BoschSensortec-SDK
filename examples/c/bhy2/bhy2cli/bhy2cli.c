/**
 * Copyright (C) 2019 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bhy2cli.c
 * @date    April-22-2019
 * @brief   BHy2 command line interface via COINES
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include "hal.h"
#include "bhy_host_interface.h"
#include "bhy2_api.h"
#include "bhy2_cus.h"
#include "cli_defines.h"

#define HUB_STATUS_BIT0_BOOT_MODE           (1 << 0)
#define HUB_STATUS_BIT1_FW_IN_FLASH         (1 << 1)
#define HUB_STATUS_BIT2_FW_IN_RAM           (1 << 2)
#define HUB_STATUS_BIT3_BOOT_FROM_FLASH     (1 << 3)
#define HUB_STATUS_BIT4_BOOT_FROM_RAM       (1 << 4)

#define BHY2_ASSERT(expr)	bhy2_check(__LINE__, #expr, expr)
#define VAL_ASSERT(expr)	if(!(expr)) ERROR("Assert failed at line %d.\r\n", __LINE__)
#define INFO(format, ...)	printf(format, ##__VA_ARGS__)
#define ERROR(format, ...)	printf("[Error]"format, ##__VA_ARGS__)
#define TST_PRINT(format, ...)       printf("[Info]"format, ##__VA_ARGS__)


bhy2_hub_t bhy2_hub_obj;
volatile int need_to_stop = 0;
uint8_t hub_status = 0;

/*! linked list contains information about the payload of a custom virt. sensor provided by the user input  */
typedef struct custom_driver_parameters{
	uint16_t    sensor_type;
	char 	    *sensor_name;
	char		*fifo_type;
	uint16_t    sensor_payload;
	char        *output_formats;
	struct      custom_driver_parameters* next_driver_parameters;
} custom_driver_parameters_t;

/*! contains payloads of custom virtual sensors and the information whether a parsing function for interpreting fifo data of this driver has been registered */
typedef struct custom_driver_information{
	uint16_t    sensor_payload;
	uint8_t     is_registered;
} custom_driver_information_t;

/*! global array that contains the payloads of present custom virtual sensors, derived by a parameter read. */
custom_driver_information_t custom_driver_information[32];

/*! global pointer that indicates start of custom driver parameters (linked list) used to parse sensor output.*/
custom_driver_parameters_t * custom_driver_parameters_head_node;

/*!
 * @brief check and interpret return value of supplied function
 *
 *
 * @param[in] 	line: line where function was called
 * 			  	func: pointer to called function
 *			    val: return value of called function
 *
 * @param[out] none.
 *
 */
void bhy2_check(int line, const char *func, int val)
{
    if (val != BHY_HIF_E_SUCCESS)
    {
        ERROR("BHI260 API failed at line %d. The function %s returned error code %d\r\n", line, func,
              val);
        switch (val)
        {
            case BHY_HIF_E_HANDLE:
                ERROR("(Handler error)\r\n");
                break;
            case BHY_HIF_E_INVAL:
                ERROR("(Invalid value)\r\n");
                break;
            case BHY_HIF_E_IO:
                ERROR("(Input/Output error)\r\n");
                break;
            case BHY_HIF_E_MAGIC:
                ERROR("(Invalid firmware)\r\n");
                break;
            case BHY_HIF_E_CRC:
                ERROR("(CRC check failed)\r\n");
                break;
            case BHY_HIF_E_TIMEOUT:
                ERROR("(Timeout occurred)\r\n");
                break;
            case BHY_HIF_E_BUF:
                ERROR("(Invalid buffer size)\r\n");
                break;
            case BHY_HIF_E_INITIALIZED:
                ERROR("(Initialization error)\r\n");
                break;
            default:
                ERROR("(Unknown error code)\r\n");
                break;
        }

        exit(1);
    }
}

/*!
 * @brief get information about virtual sensors present in the firmware and store it in the hub object
 *
 *
 * @param[in] none.
 * @param[out] none.
 *
 */

void bhy2_get_present_custom_sensors()
{
    u32 tmp_ret_len = 0;

    bhy_hif_read_parameter( &bhy2_hub_obj.handle, BHY_PARAM_SYS_VIRT_SENSOR_PRESENT, bhy2_hub_obj.virt_sen_present, sizeof(bhy2_hub_obj.virt_sen_present), &tmp_ret_len);

	/* see datasheet for response interpretation of BHY_PARAM_SYS_VIRT_SENSOR_PRESENT */
	for(u16 i = 0; i < sizeof(bhy2_hub_obj.virt_sen_present); i++)
    {
		for(u8 j = 0; j < 8; j++)
		{
			bhy2_hub_obj.virt_sensor[i * 8 + j].info.present = ((bhy2_hub_obj.virt_sen_present[i] >> j) & 0x01);
		}
	}
}


/*!
 * @brief parse fifo data of custom virt. driver based on the total payload (without data interpretation), print as hex.
 *
 *
 * @param[in] p_fifo_buffer: point to fifo buffer , p_bhy2_hub: point to bhy2 hub object
 * @param[out] none.
 *
 * @return res: operation result
 */
 int16_t bhy2_parse_custom_sensor_default(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
	uint8_t this_sensor_type = 0;
	uint8_t byte_counter = 0;
	uint8_t this_sensor_payload;
	u8 tmp_data_u8 = 0 ;
	s32 ts32_read_pos = p_fifo_buffer->read_pos;

	this_sensor_type = p_fifo_buffer->p_buffer[(ts32_read_pos)];
    this_sensor_payload = custom_driver_information[this_sensor_type - SENSOR_TYPE_CUSTOMER_VISIBLE_START].sensor_payload;


	if((p_fifo_buffer->read_pos + this_sensor_payload) > p_fifo_buffer->read_length)
	{
		return BHY_HIF_E_BUF;
	}


	/* print sensor name */
	TST_PRINT("sensor: %i Data: ", this_sensor_type);

	for(byte_counter = 0 ; byte_counter < this_sensor_payload - 1; byte_counter ++)
	{
		/* output data in 1 byte chunks */
		tmp_data_u8 = p_fifo_buffer->p_buffer[(ts32_read_pos + byte_counter + 1)];
		INFO("00x%02x ", tmp_data_u8);
	}
	INFO("\r\n");


	p_fifo_buffer->read_pos += this_sensor_payload;

    return BHY_HIF_E_SUCCESS;
}

/*!
 * @brief parse fifo data of custom virt. driver depending on user input
 * uses linked list (custom_driver_parameters) to parse the output of a fifo when an unregistered sensor is activated
 *
 *
 * @param[in] p_fifo_buffer: point to fifo buffer , p_bhy2_hub: point to bhy2 hub object
 * @param[out] none.
 *
 * @return res: operation result
 */

 int16_t bhy2_parse_custom_sensor(fifo_buffer_t* p_fifo_buffer, bhy2_hub_t* p_bhy2_hub)
{
	uint8_t byte_counter = 0;
	uint8_t output_formats_counter = 0;
	char* strtok_ptr = NULL;
	char* parameter_delimiter = ":";
	char* fifo_type_nw = "nw";
	char tmp_output_formats[BHY2CLI_MAX_STRING_LENGTH];
	uint8_t this_sensor_type = 0;


	float tmp_time_stamp = 0;

	u8 tmp_data_u8 = 0 ;
	u16 tmp_data_u16 = 0;
	u32 tmp_data_u32 = 0;
	s8 tmp_data_s8 = 0;
	s16 tmp_data_s16 = 0 ;
	s32 tmp_data_s32 = 0 ;
	u8 tmp_data_c = 0;

	s32 ts32_read_pos = p_fifo_buffer->read_pos;

	/* get sensor_type to access correct parsing information from global linked list  */
	this_sensor_type = p_fifo_buffer->p_buffer[(ts32_read_pos)];

	/* start with head node of linked list and search for the node with the current sensor_type */
	custom_driver_parameters_t  * current_custom_driver_parameters = custom_driver_parameters_head_node;
	while(current_custom_driver_parameters!=NULL)
    {
		current_custom_driver_parameters = current_custom_driver_parameters->next_driver_parameters;

        if(current_custom_driver_parameters->sensor_type == this_sensor_type)
        {
			break;
		}
    }

	/* fetch output_formats string from linked list */
	strcpy(tmp_output_formats,current_custom_driver_parameters->output_formats);
	strtok_ptr = strtok(tmp_output_formats, parameter_delimiter);


	if(strcmp(current_custom_driver_parameters->fifo_type, fifo_type_nw) == 0)
	{
		tmp_time_stamp = (float)p_bhy2_hub->time_stamp / 64000; /*1 LSB = 1/64000 sec */

	}
	else
	{
		tmp_time_stamp = (float)p_bhy2_hub->time_stamp_wkup / 64000; /*1 LSB = 1/64000 sec */
	}


	if((p_fifo_buffer->read_pos + current_custom_driver_parameters->sensor_payload + 1) > p_fifo_buffer->read_length)
    {
        return BHY_HIF_E_BUF;
    }

	/* print sensor name and timestamp */
	TST_PRINT("sensor: %s timestamp: %13.6fs Data: ", current_custom_driver_parameters->sensor_name, tmp_time_stamp);

	/* parse output_formats and output data depending on the individual format of an output */
    while (strtok_ptr != NULL)
    {

		if (strcmp(strtok_ptr, "u8") == 0)
		{
			tmp_data_u8 = p_fifo_buffer->p_buffer[(ts32_read_pos + byte_counter + 1)];
			byte_counter += 1;

			INFO("%u ", tmp_data_u8);
		}
		else if(strcmp(strtok_ptr, "u16") == 0)
		{
			tmp_data_u16 = p_fifo_buffer->p_buffer[(ts32_read_pos + byte_counter + 1)]
													| ((u16)p_fifo_buffer->p_buffer[(ts32_read_pos + byte_counter + 2)] << 8);
			byte_counter += 2;

			INFO("%u ", tmp_data_u16);
		}
		else if(strcmp(strtok_ptr, "u32") == 0)
		{
			tmp_data_u32 = p_fifo_buffer->p_buffer[(ts32_read_pos + byte_counter + 1)]
													| ((u32)p_fifo_buffer->p_buffer[(ts32_read_pos + byte_counter + 2)] << 8)
													| ((u32)p_fifo_buffer->p_buffer[(ts32_read_pos + byte_counter + 3)] << 16)
													| ((u32)p_fifo_buffer->p_buffer[(ts32_read_pos + byte_counter + 4)] << 24);
			byte_counter += 4;

			INFO("%u ", tmp_data_u32);
		}
		else if(strcmp(strtok_ptr, "s8") == 0)
		{
			tmp_data_s8 = p_fifo_buffer->p_buffer[(ts32_read_pos + byte_counter + 1)];
			byte_counter += 1;

			INFO("%d ", tmp_data_s8);
		}
		else if(strcmp(strtok_ptr, "s16") == 0)
		{
			tmp_data_s16 = p_fifo_buffer->p_buffer[(ts32_read_pos + byte_counter + 1)]
													| ((s16)p_fifo_buffer->p_buffer[(ts32_read_pos + byte_counter + 2)] << 8);
			byte_counter += 2;

			INFO("%d ", tmp_data_s16);
		}
		else if(strcmp(strtok_ptr, "s32") == 0)
		{
			tmp_data_s32 = p_fifo_buffer->p_buffer[(ts32_read_pos + byte_counter + 1)]
													| ((s32)p_fifo_buffer->p_buffer[(ts32_read_pos + byte_counter + 2)] << 8)
													| ((s32)p_fifo_buffer->p_buffer[(ts32_read_pos + byte_counter + 3)] << 16)
													| ((s32)p_fifo_buffer->p_buffer[(ts32_read_pos + byte_counter + 4)] << 24);
			byte_counter += 4;

			INFO("%d ", tmp_data_s32);
		}
		else if(strcmp(strtok_ptr, "c") == 0)
		{
			tmp_data_c = p_fifo_buffer->p_buffer[(ts32_read_pos + byte_counter + 1)];
			byte_counter += 1;

			INFO("%c ", tmp_data_c);
		}
		else if(strcmp(strtok_ptr, "f") == 0)
		{
			/* float values have to be read as unsigned and then interpreted as float */
			tmp_data_u32 = p_fifo_buffer->p_buffer[(ts32_read_pos + byte_counter + 1)]
													| ((u32)p_fifo_buffer->p_buffer[(ts32_read_pos + byte_counter + 2)] << 8)
													| ((u32)p_fifo_buffer->p_buffer[(ts32_read_pos + byte_counter + 3)] << 16)
													| ((u32)p_fifo_buffer->p_buffer[(ts32_read_pos + byte_counter + 4)] << 24);

			byte_counter += 4;

			/* the binary data has to be interpreted as a float */
			INFO("%6.4f ", *((float*)&tmp_data_u32));
		}

        strtok_ptr = strtok(NULL, parameter_delimiter);
        ++output_formats_counter;
    }

	INFO("\r\n");

	if(byte_counter != current_custom_driver_parameters->sensor_payload)
	{
		ERROR("Provided Output format sizes don't add up to total sensor payload! \r\n");
		exit(1);
	}

	p_fifo_buffer->read_pos += current_custom_driver_parameters->sensor_payload + 1;

    return BHY_HIF_E_SUCCESS;

}

/*!
 * @brief parse user input for registering payload of custom virt. driver
 * fills a global struct (custom_driver_parameters) with information about the expected payload of a new sensor
 *
 *
 * @param[in] payload: point to string containing information about the payload of a custom virt. driver
 * @param[out] none.
 */
void cli_cmd_do_add_driver(const char *payload)
{
	char* p_start = NULL;
    char* p_end = NULL;
    char* str_sensor_type;
	char* str_sensor_payload;
	char* fifo_type;
	char* output_formats;
	char* sensor_name;
	uint8_t sensor_type;
	uint8_t sensor_payload;
	uint8_t len_of_output_formats;

	/* used for getting sensor payload from param read */
	uint8_t tmp_buf[1024] = { 0 };
	uint32_t ret_len = 0;

	/* get global head node of linked list */
	custom_driver_parameters_t  * current_custom_driver_parameters = custom_driver_parameters_head_node;

	/* find last item of linked list */
	while(current_custom_driver_parameters->next_driver_parameters != NULL)
	{
		current_custom_driver_parameters = current_custom_driver_parameters->next_driver_parameters;
	}

	/* allocate memory for the current linked list node to be added */
	current_custom_driver_parameters->next_driver_parameters = malloc(sizeof(custom_driver_parameters_t));

	/* set end of linked list to next node */
	current_custom_driver_parameters->next_driver_parameters->next_driver_parameters = NULL;

	/* allocate memory for str_sensor_type */
	str_sensor_type = (char *)malloc(sizeof(char) * BHY2CLI_MAX_STRING_LENGTH);
	if(str_sensor_type == NULL)
	{
		ERROR("No free space available for malloc of str_sensor_type\r\n");
	}

	/* allocate memory for str_sensor_payload */
	str_sensor_payload = (char *)malloc(sizeof(char) * BHY2CLI_MAX_STRING_LENGTH);
	if(str_sensor_payload == NULL)
	{
		ERROR("No free space available for malloc of str_sensor_payload\r\n");
	}

	/* allocate memory for sensor_name */
	sensor_name = (char *)malloc(sizeof(char) * BHY2CLI_MAX_STRING_LENGTH);
	if(sensor_name == NULL)
	{
		ERROR("No free space available for malloc of sensor_name\r\n");
	}

	/* allocate memory for output_formats */
	output_formats = (char *)malloc(sizeof(char) * BHY2CLI_MAX_STRING_LENGTH);
	if(output_formats == NULL)
	{
		ERROR("No free space available for malloc of output_formats\r\n");
	}

	/* allocate memory for fifo_type (only "nw" or "wk" allowed) */
	fifo_type = (char *)malloc(sizeof(char) * 3);
	if(fifo_type == NULL)
	{
		ERROR("No free space available for malloc of fifo_type\r\n");
	}

    p_start = (char*)payload;
    p_end = strchr(p_start, ':');


	len_of_output_formats = strlen(p_start);
    if (p_end == NULL)
    {
        ERROR("Add Sensor format error.\r\n");
        exit(1);
    }

	/* parse sensor type */

	/* check length of string */
	if ((p_end - p_start)> 32)
	{
		ERROR("Too many characters for sensor type!\r\n");
		exit(1);
	}
    strncpy(str_sensor_type, p_start, p_end - p_start);
    str_sensor_type[p_end - p_start] = '\0';

	/* convert string to int */
    sensor_type = strtol(str_sensor_type, NULL, 10);
	INFO("Sensor Type: %d \r\n",sensor_type);

	/* parse sensor name */
    p_start = p_end + 1;
    p_end = strchr(p_start, ':');

	/* check length of string */
	if ((p_end - p_start)> 32)
	{
		ERROR("Too many characters for sensor name!\r\n");
		exit(1);
	}
	strncpy(sensor_name, p_start, p_end - p_start);
	sensor_name[p_end - p_start] = '\0';
	INFO("Sensor Name: %s \r\n",sensor_name);

	/* parse fifo type */
	p_start = p_end + 1;
    p_end = strchr(p_start, ':');

	strncpy(fifo_type,p_start, p_end - p_start);
	fifo_type[p_end - p_start] = '\0';

	if(strcmp(fifo_type,"nw") == 0)
	{
		INFO("FIFO Type: Non-Wake up\r\n");
	}
	else
	{
		if(strcmp(fifo_type,"wk") == 0)
		{
			INFO("FIFO Type: Wake up\r\n");
		}
		else
		{
			ERROR("Invalid FIFO type provided. Valid options are: \"wk\", \"nw\"\r\n");
			exit(1);
		}
	}

	/* parse sensor payload */
	p_start = p_end + 1;
    p_end = strchr(p_start, ':');

	strncpy(str_sensor_payload, p_start, p_end - p_start);
	str_sensor_payload[p_end - p_start] = '\0';
	sensor_payload = strtol(str_sensor_payload, NULL, 10);
	INFO("Sensor Payload: %d \r\n",sensor_payload);

	/* parse output formats string, final parsing of each output is done in the parsing callback function*/
	p_start = p_end + 1;
	len_of_output_formats = strlen(p_start);
	p_end = p_start + len_of_output_formats;
	strncpy(output_formats, p_start, p_end - p_start);
	output_formats[p_end - p_start] = '\0';

	/* param id for reading sensor payload, adding sensor id specifies virtual sensor */
	bhy_hif_read_parameter(&bhy2_hub_obj.handle, BHY_PARAM_SENSOR_INFO_0 + sensor_type, tmp_buf, sizeof(tmp_buf), &ret_len);

	/* check if supplied payload matches the return value of reading the sensor info parameter */
	if (tmp_buf[20] != sensor_payload + 1)
	{
		ERROR("Provided total payload size of sensor %d doesn't match the actual payload size!\r\n",sensor_type);
		exit(1);
	}


	/* store parsed data in linked list */
	current_custom_driver_parameters->next_driver_parameters->sensor_type = sensor_type;
	current_custom_driver_parameters->next_driver_parameters->sensor_name = sensor_name;
	current_custom_driver_parameters->next_driver_parameters->fifo_type = fifo_type;
	current_custom_driver_parameters->next_driver_parameters->sensor_payload = sensor_payload;
	current_custom_driver_parameters->next_driver_parameters->output_formats = output_formats;

	/*register the custom sensor callback function*/
	bhy2_virt_sensor_callback_register(&bhy2_hub_obj, sensor_type, bhy2_parse_custom_sensor);

	/* flag this virt sensor as already registered */
	custom_driver_information[sensor_type - BHY_SENSOR_ID_CUSTOMER_BEGIN].is_registered = 1;

    INFO("Adding custom driver payload successful.\r\n");
}

/*!
 * @brief install virtual sensor process callbacks
 *
 *
 * @param[in] p_bhy2_hub: pointer to bhy2 obj
 * @param[out] none.
 *
 * @return res: operation result
 */
int16_t bhy2_install_callbacks(bhy2_hub_t* p_bhy2_hub)
{

    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SYS_SENSOR_ID_TS_SMALL_DELTA, bhy2_parse_frame_timestamp_small_delta);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SYS_SENSOR_ID_TS_LARGE_DELTA, bhy2_parse_frame_timestamp_large_delta);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SYS_SENSOR_ID_TS_FULL, bhy2_parse_frame_full_timestamp);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SYS_SENSOR_ID_META_EVENT, bhy2_parse_frame_meta_event);

    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SYS_SENSOR_ID_TS_SMALL_DELTA_WU, bhy2_parse_frame_timestamp_small_delta_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SYS_SENSOR_ID_TS_LARGE_DELTA_WU, bhy2_parse_frame_timestamp_large_delta_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SYS_SENSOR_ID_TS_FULL_WU, bhy2_parse_frame_full_timestamp_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SYS_SENSOR_ID_META_EVENT_WU, bhy2_parse_frame_meta_event_wkup);

    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_ACC, bhy2_parse_frame_acc_corrected);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_ACC_WU, bhy2_parse_frame_acc_corrected_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_ACC_PASS, bhy2_parse_frame_acc_passthrough);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_ACC_RAW, bhy2_parse_frame_acc_raw);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_ACC_RAW_WU, bhy2_parse_frame_acc_raw_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_ACC_BIAS, bhy2_parse_frame_acc_offset);

    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_MAG, bhy2_parse_frame_mag_corrected);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_MAG_WU, bhy2_parse_frame_mag_corrected_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_MAG_PASS, bhy2_parse_frame_mag_passthrough);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_MAG_RAW, bhy2_parse_frame_mag_raw);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_MAG_RAW_WU, bhy2_parse_frame_mag_raw_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_MAG_BIAS, bhy2_parse_frame_mag_offset);

    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_GYRO, bhy2_parse_frame_gyro_corrected);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_GYRO_WU, bhy2_parse_frame_gyro_corrected_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_GYRO_PASS, bhy2_parse_frame_gyro_passthrough);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_GYRO_RAW, bhy2_parse_frame_gyro_raw);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_GYRO_RAW_WU, bhy2_parse_frame_gyro_raw_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_GYRO_BIAS, bhy2_parse_frame_gyro_offset);

    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_ORI, bhy2_parse_frame_orientation);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_ORI_WU, bhy2_parse_frame_orientation_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_TEMP, bhy2_parse_frame_temperature);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_TEMP_WU, bhy2_parse_frame_temperature_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_GRA, bhy2_parse_frame_gravity);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_GRA_WU, bhy2_parse_frame_gravity_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_LACC, bhy2_parse_frame_linear_acc);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_LACC_WU, bhy2_parse_frame_linear_acc_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_RV, bhy2_parse_frame_rotation);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_RV_WU, bhy2_parse_frame_rotation_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_GAMERV, bhy2_parse_frame_game_rotation);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_GAMERV_WU, bhy2_parse_frame_game_rotation_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_SIG_WU, bhy2_parse_frame_sig_motion_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_SIG_HW_WU, bhy2_parse_frame_sig_motion_hw_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_STD, bhy2_parse_frame_step_detect);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_STD_WU, bhy2_parse_frame_step_detect_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_STD_HW, bhy2_parse_frame_step_detect_hw);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_STD_HW_WU, bhy2_parse_frame_step_detect_hw_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_STC, bhy2_parse_frame_step_counter);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_STC_WU, bhy2_parse_frame_step_counter_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_STC_HW, bhy2_parse_frame_step_counter_hw);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_STC_HW_WU, bhy2_parse_frame_step_counter_hw_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_GEORV, bhy2_parse_frame_geo_rotation);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_GEORV_WU, bhy2_parse_frame_geo_rotation_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_TILT_WU, bhy2_parse_frame_tilt_detect_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_WAKE_WU, bhy2_parse_frame_wake_gesture_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_GLANCE_WU, bhy2_parse_frame_glance_gesture_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_PICKUP_WU, bhy2_parse_frame_pickup_gesture_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_AR_WU, bhy2_parse_frame_activity_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_GPS, bhy2_parse_frame_gps);

    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_LIGHT, bhy2_parse_frame_light);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_LIGHT_WU, bhy2_parse_frame_light_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_BARO, bhy2_parse_frame_barometer);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_BARO_WU, bhy2_parse_frame_barometer_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_PROX, bhy2_parse_frame_proximity);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_PROX_WU, bhy2_parse_frame_proximity_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_HUM, bhy2_parse_frame_humidity);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_HUM_WU, bhy2_parse_frame_humidity_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_ATEMP, bhy2_parse_frame_padding);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_ATEMP_WU, bhy2_parse_frame_padding);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_HEART, bhy2_parse_frame_padding);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_HEART_WU, bhy2_parse_frame_padding);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_ANY_MOTION, bhy2_parse_frame_any_motion_hw);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_ANY_MOTION_WU, bhy2_parse_frame_any_motion_hw_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_GAS, bhy2_parse_frame_gas);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_GAS_WU, bhy2_parse_frame_gas_wkup);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_EXCAMERA, bhy2_parse_frame_padding);
    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SENSOR_ID_AR_WU, bhy2_parse_frame_activity_wkup);

    bhy2_virt_sensor_callback_register(&bhy2_hub_obj, BHY_SYS_SENSOR_ID_DEBUG_MSG, bhy2_parse_frame_debug_message);

    return 0;
}

/*!
 * @brief print out bhy2cli usages
 *
 *
 * @param[in] none.
 * @param[out] none.
 *
 * @return none.
 */
void print_usage(void)
{

    INFO("Usage:\r\n");
    INFO("bhy2cli [<options>]\r\n");
    INFO("Options:\r\n");
    INFO("  -h                                                    = Print this usage message.\r\n");
	INFO("  -i                                                    = Show device information: Device ID, ROM version, RAM version, Power state, list of available sensors,\r\n");
	INFO("                                                          content of Boot Status register, content of Error value register.\r\n");
    INFO("  -b <firmware path>                                    = Reset, upload specified firmware to RAM and boot from RAM [equivalent to using -n -u -g successively].\r\n");
    INFO("  -n                                                    = Reset sensor hub.\r\n");
    INFO("  -a <sensor_type>:<sensor_name>:<fifo_type>:<total_output_payload_in_bytes>:<output_format_0>:<output_format_1>...\r\n");
    INFO("                                                        = Register the expected payload of a new custom virtual sensor,\r\n");
    INFO("                                                          including information about the FIFO (fifo_type), which contains data produced by this sensor.\r\n");
    INFO("                                                            -Valid fifo_types: wk: Wake up, nw: Non-Wake up\r\n");
    INFO("                                                            -Valid output_formats: u8: Unsigned 8 Bit, u16: Unsigned 16 Bit, u32: Unsigned 32 Bit, s8: Signed 8 Bit,\r\n");
	INFO("                                                                                   s16: Signed 16 Bit, s32: Signed 32 Bit, f: Float, c: Char \r\n");
    INFO("                                                            -e.g.: -a 160:\"Lean Orientation\":nw:2:c:c \r\n");
    INFO("                                                            -Note that the corresponding virtual sensor has to be enabled in the same function call (trailing -c option),\r\n");
    INFO("                                                             since the registration of the sensor is temporary. \r\n");
    INFO("  -r <adr>[:<len>]                                      = Read from register address <adr> for length <len> bytes.\r\n");
    INFO("                                                            -If input <len> is not provided, the default read length is 1 byte.\r\n");
    INFO("                                                            -When reading registers with auto-increment, the provided register as well as the following registers will be read.\r\n");
    INFO("                                                            -e.g -r 0x08:3 will read the data of registers 0x08, 0x09 and 0x0a.\r\n");
    INFO("                                                          max. 53 bytes can be read at once.\r\n");
    INFO("  -w <adr>=<val1>[,<val2>]...                           = Write to register address <adr> with comma separated values <val>.\r\n");
    INFO("                                                            -If more values provided <val>, the additional\r\n");
    INFO("                                                             values will be written to the following addresses.\r\n");
    INFO("                                                            -When writing to registers with auto-increment, the provided register as well as the following registers will be written.\r\n");
	INFO("                                                            -e.g -w 0x08=0x02,0x03,0x04 will write the provided data to registers 0x08, 0x09 and 0x0a.\r\n");
    INFO("                                                          max. 46 bytes can be written at once.\r\n");
	INFO("  -s <param_id>                                         = Display read_param response of parameter <param_id>.\r\n");
	INFO("  -t <param_id>=<val1>[,<val2>]...                      = Write data to parameter <param_id> with the bytes to be written, <val1>[,<val2>]... .\r\n");
	INFO("                                                            -e.g. 0x103=5,6 will write 0x05 to the first byte and 0x06 to the second byte of the parameter \"Fifo Control\"\r\n");
    INFO("  -u <firmware path>                                    = Upload firmware to RAM.\r\n");
    INFO("  -f <firmware path>                                    = Upload firmware to external-flash.\r\n");
    INFO("  -g <medium>                                           = Boot from the specified <medium>: \"f\" for FLASH, \"r\" for RAM.\r\n");
    INFO("  -e                                                    = Erase external-flash.\r\n");
    INFO("  -c <sensor_type>:<frequency>[:<latency>]              = Activate sensor <sensor_type> at specified sample rate <frequency>,\r\n");
    INFO("                                                        = latency <latency>, duration time <time>, sample counts <count>.\r\n");
    INFO("                                                            - At least <frequency> is a must input parameter.\r\n");
    INFO("                                                            - <latency> is optional.\r\n");
    INFO("                                                            - User can active more than one sensor at the same time by repeating -c option.\r\n");
    INFO("                                                            - id: sensor id.\r\n");
    INFO("                                                            - frequency(Hz): sensor ODR.\r\n");
    INFO("                                                            - latency(ms): sensor data outputs with a latency.\r\n");
}



/*!
 * @brief process command: get list of available sensor drivers
 *
 *
 * @param[in] none.
 * @param[out] none.
 *
 */
void cli_cmd_do_show_information()
{
    uint16_t ram_version = 0;
    uint16_t rom_version = 0;
    uint8_t product_id = 0;
    uint8_t host_status = 0;
	uint8_t val = 0;
	char* power_state = NULL;

	/* get product_id */
	BHY2_ASSERT(bhy2_product_id_get(&bhy2_hub_obj, &product_id));

    /* get RAM version */
    BHY2_ASSERT(bhy2_ram_version_get(&bhy2_hub_obj, &ram_version));

	/* get ROM version */
    BHY2_ASSERT(bhy2_rom_version_get(&bhy2_hub_obj, &rom_version));


	BHY2_ASSERT(bhy_hif_bus_read(&(&bhy2_hub_obj)->handle, BHY_REG_HOST_STATUS, &host_status, 1));

	if((host_status && 0x01) == 0x00)
	{
		power_state = "active";
	}
	else
	{
		power_state = "sleeping";
	}


    INFO("Device         : 71%02x\r\nRAM version    : %04u\r\nROM version    : %04u\r\nPower state    : %s\r\n\r\n",product_id, ram_version, rom_version,power_state);

	if(ram_version)
	{
		/* get presented virtual sensor */
		BHY2_ASSERT(bhy2_virt_sensor_present_get(&bhy2_hub_obj, 0));
	}
	/* read boot status */
	BHY2_ASSERT(bhy_hif_bus_read(&(&bhy2_hub_obj)->handle, BHY_REG_BOOT_STATUS, &val, 1));

	/* show content of boot status register */
	INFO("\n0x25: Boot Status:      0x%02x \r\n", val);
	INFO("---------------------------------\r\n");

	for(int i = 0 ; i < 16 ; i++)
	{
		if((boot_status_fields[i].mask & val) == boot_status_fields[i].value )
		{
			INFO("Bit[%d]: %d = %s \r\n",i/2, boot_status_fields[i].mask && boot_status_fields[i].value , boot_status_fields[i].output_text);
		}
	}
	/* read error value */
	BHY2_ASSERT(bhy_hif_bus_read(&(&bhy2_hub_obj)->handle, BHY_REG_ERROR, &val, 1));

	for(int i = 0; i < sizeof(error_values)/sizeof(error_values[0]); i++)
	{
		if(val == error_values[i].value)
		{
			INFO("\nError value: 0x%02x  = %s  | Error Category: %s\r\n", val, error_values[i].output_text,error_values[i].error_category);
		}
	}
}

/*!
 * @brief process command: Reset Sensor Hub
 *
 *
 * @param[in] none.
 * @param[out] none.
 *
 */
void cli_cmd_do_reset_hub()
{
    uint8_t data = 0;

    BHY2_ASSERT(bhy2_hub_reset(&bhy2_hub_obj));

    bhy2_register_set(&bhy2_hub_obj, BHY_REG_CHIP_CTRL, 0x01);
    bhy2_register_get(&bhy2_hub_obj, BHY_REG_CHIP_CTRL, &data);
    VAL_ASSERT(data == 0x01);

    /* Get trigger mode on int pin */
    bhy2_register_get(&bhy2_hub_obj, BHY_REG_HOST_INTERRUPT_CTRL, &data);
    VAL_ASSERT(data == 0x00);

    /* Get status channel config */
    bhy2_register_get(&bhy2_hub_obj, BHY_REG_HOST_INTERFACE_CTRL, &data);
    VAL_ASSERT(data == 0x00);

    /* Set trigger mode on int pin */
    bhy2_host_interrupt_ctrl_set(&bhy2_hub_obj, /*BHY_ICTL_EDGE|*/BHY_ICTL_DISABLE_STATUS);
    bhy2_register_get(&bhy2_hub_obj, BHY_REG_HOST_INTERRUPT_CTRL, &data);
    VAL_ASSERT(data == 0x04);

    /* Config status channel */
    bhy2_host_interface_ctrl_set(&bhy2_hub_obj, BHY_IFCTL_ASYNC_STATUS_CHANNEL);
    bhy2_register_get(&bhy2_hub_obj, BHY_REG_HOST_INTERFACE_CTRL, &data);
    VAL_ASSERT(data == 0x80);

    hub_status |= HUB_STATUS_BIT0_BOOT_MODE;
    INFO("Reset successful.\r\n");

}

/*!
 * @brief process command: upload firmware to RAM
 *
 *
 * @param[in] filepath: point to a firmware file
 * @param[out] none.
 *
 */
void cli_cmd_do_upload_to_ram(const char *filepath)
{
    FILE *fp;
    uint8_t *firmware_image;
    long len;
    size_t ret;
    int16_t ret_val = 0;
    uint8_t data = 0;

    bhy2_register_get(&bhy2_hub_obj, BHY_REG_BOOT_STATUS, &data);
    if ((data & BHY_BST_HOST_INTERFACE_READY) == BHY_BST_HOST_INTERFACE_READY)
    {
        if (((data & BHY_BST_HOST_FW_VERIFY_DONE) == BHY_BST_HOST_FW_VERIFY_DONE)
            || ((data & BHY_BST_FLASH_VERIFY_DONE) == BHY_BST_FLASH_VERIFY_DONE))
        {
            ERROR("Please reset the BHI260/BHA260 before uploading firmware.\r\n");
            exit(1);
        }

        /* open & read RAM firmware -s*/
        fp = fopen(filepath, "rb");
        if (!fp)
        {
			ERROR("Cannot open file: %s\r\n", filepath);
            exit(1);
        }
        fseek(fp, 0, SEEK_END);
        len = ftell(fp);
        rewind(fp);
        firmware_image = (uint8_t *)malloc(len);
        ret = fread(firmware_image, sizeof(uint8_t), len, fp);
        fclose(fp);
        if (ret != (size_t)len)
        {
            free(firmware_image);
            ERROR("Reading file failed.\r\n");
            exit(1);
        }
        /* open & read RAM firmware -e*/
        INFO("Uploading firmware to RAM.\r\n");
        /* upload RAM ptach to RAM */
        ret_val = bhy2_patch_upload_to_ram(&bhy2_hub_obj, firmware_image, len);
        free(firmware_image);
        if (ret_val < 0)
        {
            ERROR("Firmware upload failed. Download status: %02d.\r\n", ret_val);
            exit(1);
        }
    }
    else
    {
        ERROR("Host interface is not ready.\r\n");
        exit(1);
    }

    hub_status |= HUB_STATUS_BIT2_FW_IN_RAM;
    INFO("Uploading firmware to RAM successful.\r\n");
}

/*!
 * @brief process command: boot device from firmware stored in RAM
 *
 *
 * @param[in] payload: point to payload
 * @param[out] none.
 *
 */
void cli_cmd_do_boot_from_ram()
{
    int16_t ret_val = 0;
    uint16_t tu16_data = 0;
    uint8_t data = 0;
    uint8_t error_val = 0;


    bhy2_register_get(&bhy2_hub_obj, BHY_REG_BOOT_STATUS, &data);
    if ((data & BHY_BST_HOST_INTERFACE_READY) == BHY_BST_HOST_INTERFACE_READY)
    {
        if ((data & BHY_BST_HOST_FW_VERIFY_DONE) == BHY_BST_HOST_FW_VERIFY_DONE)
        {
            /* get RAM version */
            bhy2_ram_version_get(&bhy2_hub_obj, &tu16_data);
            if (tu16_data == 0)
            {

                /* if success, boot hub from RAM */
                ret_val = bhy2_patch_boot_from_ram(&bhy2_hub_obj);
                if (ret_val < 0)
                {
					ERROR("Boot from RAM status: %02d.\r\n", ret_val);

					/* read error value */
					BHY2_ASSERT(bhy_hif_bus_read(&(&bhy2_hub_obj)->handle, BHY_REG_ERROR, &error_val, 1));

					for(int i = 0; i < sizeof(error_values)/sizeof(error_values[0]); i++)
					{
						if(error_val == error_values[i].value)
						{
							INFO("\nError register content: 0x%02x  = %s  | Error Category: %s\r\n", error_val, error_values[i].output_text,error_values[i].error_category);
						}
					}

                    exit(1);
                }
            }
            /* get RAM version */
            bhy2_ram_version_get(&bhy2_hub_obj, &tu16_data);
            if (tu16_data)
            {
                /* get presented virtual sensor, can be uncommented if there should be a list of existing sensors after boot */
                /* bhy2_virt_sensor_present_get(&bhy2_hub_obj, 0); */

                for (uint16_t i = 0; i < 10; i++)
                {
                    /* parse & execute bhi160 fifo data */
                    bhy2_parse_fifos(&bhy2_hub_obj);
                    delay_ms(10, NULL);
                }
            }
            else
            {
                ERROR("Reading RAM version failed, booting RAM failed.\r\n");
                exit(1);
            }
        }
        else
        {
            ERROR("Upload firmware to RAM before boot.\r\n");
            exit(1);
        }

    }
    else
    {
        ERROR("Host interface is not ready.\r\n");
        exit(1);
    }
    hub_status &= (~HUB_STATUS_BIT0_BOOT_MODE);
    hub_status &= (~HUB_STATUS_BIT2_FW_IN_RAM);
    hub_status |= HUB_STATUS_BIT4_BOOT_FROM_RAM;
    INFO("Booting from RAM successful.\r\n");
}

/*!
 * @brief process command: upload firmware to flash
 *
 *
 * @param[in] none.
 * @param[out] none.
 *
 */
void cli_cmd_do_erase_flash()
{
    int16_t ret_val = 0;
    uint8_t data = 0;

    bhy2_register_get(&bhy2_hub_obj, BHY_REG_BOOT_STATUS, &data);
    if ((data & BHY_BST_HOST_INTERFACE_READY) == BHY_BST_HOST_INTERFACE_READY)
    {
        if (((data & BHY_BST_HOST_FW_VERIFY_DONE) == BHY_BST_HOST_FW_VERIFY_DONE)
            || ((data & BHY_BST_FLASH_VERIFY_DONE) == BHY_BST_FLASH_VERIFY_DONE))
        {
            ERROR("Reset the BHI260/BHA260 before erasing external flash.\r\n");
            exit(1);
        }
    }
    INFO("Erasing flash.\r\n");
    ret_val = bhy2_patch_erase_flash(&bhy2_hub_obj);
    if (ret_val < 0)
    {
        ERROR("Erasing flash failed, status: %02d.\r\n", ret_val);
        exit(1);
    }

    hub_status &= (~HUB_STATUS_BIT1_FW_IN_FLASH);
    INFO("Erasing flash successful.\r\n");
}

/*!
 * @brief process command: upload firmware to flash
 *
 *
 * @param[in] filepath: point to a firmware file
 * @param[out] none.
 *
 * @return res: operation result
 */
void cli_cmd_do_upload_to_flash(const char *filepath)
{
    FILE *fp;
    uint8_t *firmware_image;
    long len;
    size_t ret;
    int16_t ret_val = 0;
    uint8_t data = 0;

    bhy2_register_get(&bhy2_hub_obj, BHY_REG_BOOT_STATUS, &data);
    if ((data & BHY_BST_HOST_INTERFACE_READY) == BHY_BST_HOST_INTERFACE_READY)
    {
        if (((data & BHY_BST_HOST_FW_VERIFY_DONE) == BHY_BST_HOST_FW_VERIFY_DONE)
            || ((data & BHY_BST_FLASH_VERIFY_DONE) == BHY_BST_FLASH_VERIFY_DONE))
        {
            ERROR("Reset the BHI260/BHA260 before uploading firmware to external flash.\r\n");
            exit(1);
        }

        /* open & read FLASH firmware -s*/
        fp = fopen(filepath, "rb");
        if (!fp)
        {
            ERROR("Cannot open file: %s.\r\n", filepath);
            exit(1);
        }
        fseek(fp, 0, SEEK_END);
        len = ftell(fp);
        rewind(fp);
        firmware_image = (uint8_t *)malloc(len);
        ret = fread(firmware_image, sizeof(uint8_t), len, fp);
        fclose(fp);
        if (ret != (size_t)len)
        {
            free(firmware_image);
            ERROR("Reading file failed.\r\n");
            exit(1);
        }
        /* open & read FLASH firmware -e*/
        INFO("Uploading firmware to flash.\r\n");
        /* upload FLASH patch to FLASH */
        ret_val = bhy2_patch_upload_to_flash(&bhy2_hub_obj, firmware_image, len);
        free(firmware_image);
        if (ret_val < 0)
        {
            ERROR("Firmware upload failed. Firmware download status: %02d.\r\n", ret_val);
            exit(1);
        }
    }
    else
    {
        ERROR("Host interface is not ready.\r\n");
        exit(1);
    }

    hub_status |= HUB_STATUS_BIT1_FW_IN_FLASH;
    INFO("Uploading firmware to flash successful.\r\n");
}

/*!
 * @brief process command: upload firmware to RAM
 *
 *
 * @param[in] filepath: point to a firmware file
 * @param[out] none.
 *
 */
void cli_cmd_do_boot_from_flash()
{
    int16_t ret_val = 0;
    uint16_t tu16_data = 0;
    uint8_t data = 0;
	uint8_t error_val = 0;

    bhy2_register_get(&bhy2_hub_obj, BHY_REG_BOOT_STATUS, &data);
    if ((data & BHY_BST_HOST_INTERFACE_READY) == BHY_BST_HOST_INTERFACE_READY)
    {
        if (((data & BHY_BST_FLASH_DETECTED) == BHY_BST_FLASH_DETECTED))
        {
            /* get RAM version */
            bhy2_ram_version_get(&bhy2_hub_obj, &tu16_data);
            if (tu16_data == 0)
            {
                /* if no RAM firmware is running, boot from FLASH */
                ret_val = bhy2_patch_boot_from_flash(&bhy2_hub_obj);
                if (ret_val < 0)
                {
                    ERROR("Booting from flash status: %02d.\r\n", ret_val);

					/* read error value */
					BHY2_ASSERT(bhy_hif_bus_read(&(&bhy2_hub_obj)->handle, BHY_REG_ERROR, &error_val, 1));

					for(int i = 0; i < sizeof(error_values)/sizeof(error_values[0]); i++)
					{
						if(error_val == error_values[i].value)
						{
							INFO("\nError register content: 0x%02x  = %s  | Error Category: %s\r\n", error_val, error_values[i].output_text,error_values[i].error_category);
						}
					}
                    exit(1);
                }
                /* check boot status -s*/
                bhy2_register_get(&bhy2_hub_obj, BHY_REG_BOOT_STATUS, &data);
                INFO("Boot status: 0x%02X.\r\n", data);
                if ((data & BHY_BST_HOST_INTERFACE_READY) == BHY_BST_HOST_INTERFACE_READY)
                {
                    INFO("Host interface is ready.\r\n");
                    if ((data & BHY_BST_FLASH_DETECTED) == BHY_BST_FLASH_DETECTED)
                    {
                        INFO("External flash is detected.\r\n");
                        if ((data & BHY_BST_FLASH_VERIFY_DONE) == BHY_BST_FLASH_VERIFY_DONE)
                        {
                            if ((data & BHY_BST_FLASH_VERIFY_ERROR) == BHY_BST_FLASH_VERIFY_ERROR)
                            {
                                INFO("Verifying external flash failed.\r\n");
                            }
                            else
                            {
                                INFO("Booting from external flash passed.\r\n");
                            }
                        }
                        else    /* no patch in flash */
                        {
                            INFO("No firmware in external flash, need to write upload firmware to external flash.\r\n");
                        }
                    }
                    else    /* no flash, need to upload RAM patch */
                    {
                        INFO("No external flash detected, need to upload firmware to RAM.\r\n");

                    }
                }
                else
                {
                    /* hub is not ready, need reset hub */
                    INFO("host interface is not ready, need reset hub.\r\n");

                    bhy2_hub_reset(&bhy2_hub_obj);
                }
                /* check boot status -e*/
            }
            /* get RAM version */
            bhy2_ram_version_get(&bhy2_hub_obj, &tu16_data);
            if (tu16_data)
            {
                /* get presented virtual sensor, can be uncommented if there should be a list of existing sensors after boot */
                // bhy2_virt_sensor_present_get(&bhy2_hub_obj, 0/* &data*/);

                for (uint16_t i = 0; i < 10; i++)
                {
                    /* parse & execute bhi160 fifo data */
                    bhy2_parse_fifos(&bhy2_hub_obj);
                    delay_ms(10, NULL);
                }
            }
            else    /* no patch in flash */
            {
                ERROR("Reading RAM version failed, booting from flash failed.\r\n");
                exit(1);
            }
        }
        else    /* no flash, need to upload RAM patch */
        {
            ERROR("Can't detect external flash.\r\n");
            exit(1);
        }
    }
    else
    {
        /* hub is not ready, need reset hub */
        ERROR("Host interface is not ready.\r\n");
        exit(1);
    }
    hub_status &= (~HUB_STATUS_BIT0_BOOT_MODE);
    hub_status |= HUB_STATUS_BIT3_BOOT_FROM_FLASH;
    INFO("Booting from flash successful.\r\n");
}

/*!
 * @brief process command: activate a virtual sensor
 *
 *
 * @param[in] sensor_parameters: point to sensor params
 * @param[out] none.
 *
 */
void cli_cmd_do_activate(char *sensor_parameters)
{
    char sen_id_str[8], sen_odr_str[8], sen_latency_str[8];
    uint8_t sen_id;
    int32_t sen_latency = 0;
    float sen_odr;
    char *p_start = NULL, *p_end = NULL;

	uint8_t tmp_buf[1024] = { 0 };
	uint32_t ret_len = 0;

    /* parse ID */
    p_start = sensor_parameters;
    p_end = strchr(p_start, ':');
    if (p_end == NULL)
    {
        ERROR("Sensor ID / Sample rate format error.\r\n");
        exit(1);
    }

    strncpy(sen_id_str, p_start, p_end - p_start);
    sen_id_str[p_end - p_start] = '\0';
    sen_id = atoi(sen_id_str);

    /* parse ODR */
    p_start = p_end + 1;
    p_end = strchr(p_start, ':');

    if (p_end == NULL)
    {
        p_end = p_start + strlen(p_start);
    }

    strncpy(sen_odr_str, p_start, p_end - p_start);
    sen_odr_str[p_end - p_start] = '\0';
    sen_odr = atof(sen_odr_str);

    if (sen_odr < 0)
    {
        sen_odr = 0.0f;
    }

   /*  parse Latency */
    if (strlen(p_end))
    {
        p_start = p_end + 1;
        p_end = strchr(p_start, ':');

        if (p_end == NULL)
        {
            p_end = p_start + strlen(p_start);
        }

        strncpy(sen_latency_str, p_start, p_end - p_start);
        sen_latency_str[p_end - p_start] = '\0';
        sen_latency = atoi(sen_latency_str);
    }

	/* if the payload of this sensor is not yet registered and within the custom virtual sensor id range, register the default parsing function */
	if(bhy2_hub_obj.virt_sensor[sen_id].info.present == 1)
	{
		if(custom_driver_information[sen_id - BHY_SENSOR_ID_CUSTOMER_BEGIN].is_registered != 1)
		{
			/* if sensor id is within the customer reserved range */
			if((sen_id >= BHY_SENSOR_ID_CUSTOMER_BEGIN) && (sen_id <= BHY_SENSOR_ID_CUSTOMER_END))
			{
				/* param id for reading sensor payload, adding sensor id specifies virtual sensor */
				bhy_hif_read_parameter(&bhy2_hub_obj.handle, BHY_PARAM_SENSOR_INFO_0 + sen_id, tmp_buf, sizeof(tmp_buf), &ret_len);

				if(tmp_buf[0])
				{
					/* pick the payload information from the parameter response */
					custom_driver_information[sen_id - BHY_SENSOR_ID_CUSTOMER_BEGIN].sensor_payload = tmp_buf[20];
				}
				bhy2_virt_sensor_callback_register(&bhy2_hub_obj, sen_id, bhy2_parse_custom_sensor_default);
				INFO("No output interpretation has been provided for this sensor. FIFO data will be printed as hex values. For registering the payload interpretation, use the -a option.\r\n\r\n");
			}

		}
	}
	else
	{
		ERROR("The requested sensor is not present in the loaded firmware!\r\n");
		exit(1);
	}


    INFO("Sensor ID: %d, sample rate: %f Hz, latency: %d ms.\r\n", sen_id, sen_odr, sen_latency);

    /* flush sensor fifo */
    BHY2_ASSERT(bhy2_fifo_flush(&bhy2_hub_obj, sen_id));

    /* enable sensor & set odr */
    BHY2_ASSERT(bhy2_virt_sensor_cfg_set(&bhy2_hub_obj, sen_id, sen_odr, sen_latency));

	/* sensor data will be parsed at the end of the tool call */

}

/*!
 * @brief process command: read value from an address
 *
 *
 * @param[in] payload: point to command params
 * @param[out] none.
 *
 */

void cli_cmd_do_read_from_register_address(const char* payload)
{
    char* p_start = NULL;
    char* p_end = NULL;
    char str_reg[8] = { 0 };
    uint8_t reg;
    uint8_t val[1024];
    uint32_t len = 1;
    int16_t i = 0;
	int16_t j = 0;

    p_start = (char*)payload;
    p_end = strchr(p_start, ':');
    if (p_end == NULL)
    {
        p_end = p_start + strlen(p_start);
    }

	/* parse register address */
    strncpy(str_reg, p_start, p_end - p_start);
    str_reg[p_end - p_start] = '\0';
    reg = strtol(str_reg, NULL, 0);

	/* parse read length, rest of the string */
    p_start = p_end + 1;
    len = strtol(p_start, NULL, 0);

	/* default read length is 1 */
    if (len < 1)
    {
        len = 1;
    }

	/* execution of bus read function */
    BHY2_ASSERT(bhy_hif_bus_read(&(&bhy2_hub_obj)->handle, reg, &val[0], len));

	/* print register data to console */
	INFO("\r\n");

	/* registers from 4 are auto increment, reading more than 1 byte will lead to reading the subsequent register addresses */
	if(reg <= 3)
	{
		INFO("Reading from register address 0x%02x:\r\n\r\n",reg);
		INFO("\nByte hex      dec | Data\r\n");
		INFO("-------------------------------------------\r\n");
		for (i = 0; i < len; i++)
		{	if(j == 0)
			{
				INFO("0x%06x %8d |", i, i);
			}
			INFO(" %02x", val[i]);
			++j;
			if(j >= 8)
			{
				INFO("\r\n");
				j = 0;
			}
		}
		INFO("\r\n");
	}

	else
	{
		INFO("Register address: Data\r\n");
		INFO("----------------------\r\n");
		for (i = 0; i < len; i++)
		{
			INFO("0x%02x            : %02x \r\n",reg+i, val[i]);
		}
	}
	INFO("\r\n");
    INFO("Reading address successful.\r\n");
}


/*!
 * @brief process command: write data to an address
 *
 *
 * @param[in] payload: point to command params
 * @param[out] none.
 *
 */

void cli_cmd_do_write_to_register_address(const char *payload)
{
    char* p_start = NULL;
    char* p_end = NULL;
    char str_reg[8] = { 0 };
    uint8_t reg;
    uint8_t val[1024];
    char *strtok_ptr;
    char *byte_delimiter = ",";
    uint16_t len = 0;

	/* parse register address */
    p_start = (char*)payload;
    p_end = strchr(p_start, '=');
    if (p_end == NULL)
    {
        ERROR("Write address format error.\r\n");
        exit(1);
    }
    strncpy(str_reg, p_start, p_end - p_start);
    str_reg[p_end - p_start] = '\0';
    reg = strtol(str_reg, NULL, 0);

	/* parse values to be written */
    p_start = p_end + 1;
    strtok_ptr = strtok(p_start, byte_delimiter);

    while (strtok_ptr != NULL)
    {
        val[len] = strtol(strtok_ptr, NULL, 0);
        strtok_ptr = strtok(NULL, byte_delimiter);
        ++len;
    }


	/* execution of bus write function */
    BHY2_ASSERT(bhy_hif_bus_write(&(&bhy2_hub_obj)->handle, reg, &val[0], len));

    INFO("Writing address successful.\r\n");
}

/*!
 * @brief process command: issue Host command to read a parameter
 *
 *
 * @param[in] payload: point to command params
 * @param[out] none.
 *
 */
void cli_cmd_do_read_param(const char *payload)
{
    char str_param_id[8] = { 0 };
    uint8_t tmp_buf[1024] = { 0 };
    uint16_t param_id = 0;
    uint32_t ret_len = 0;
    uint16_t i = 0;
    uint16_t j = 0;

    strncpy(str_param_id, payload, strlen(payload));
    str_param_id[strlen(payload)] = '\0';
    param_id = strtol(str_param_id, NULL, 0);

    BHY2_ASSERT(bhy_hif_read_parameter(&bhy2_hub_obj.handle, param_id, tmp_buf, sizeof(tmp_buf), &ret_len));
	INFO("\nByte hex      dec | Data\r\n");
	INFO("-------------------------------------------\r\n");
    for (i = 0; i < ret_len; i++)
    {
		if(j == 0)
		{
			INFO("0x%06x %8d |", i, i);
		}
        INFO("%02x ", tmp_buf[i]);
        j++;
        if (j >= 8)
        {
            INFO("\r\n");
            j = 0;
        }
    }
    INFO("\r\n\r\n");

    INFO("Reading paramter I/O successful.\r\n");
}

/*!
 * @brief process command: issue host command to write a parameter
 *
 *
 * @param[in] payload: point to command params
 * @param[out] none.
 */
void cli_cmd_do_write_param(const char *payload)
{
    char *p_start = NULL, *p_end = NULL;
    char str_param_id[8] = { 0 };
    char str_data[8] = { 0 };
    uint8_t data_buf[1024] = { 0 };
    uint16_t param_id = 0;
    uint8_t val;
    uint16_t buf_size = 0;
    uint8_t break_out = 0;

    p_start = (char*)payload;
    p_end = strchr(p_start, '=');
    if (p_end == NULL)
    {
        ERROR("Write parameter I/O format error.\r\n");
        exit(1);
    }
    strncpy(str_param_id, p_start, p_end - p_start);
    str_param_id[p_end - p_start] = '\0';
    param_id = strtol(str_param_id, NULL, 0);

    /* parse write data -s*/
    do
    {
        p_start = p_end + 1;
        p_end = strchr(p_start, ',');
        if (p_end == NULL)
        {
            p_end = p_start + strlen(p_start);
            break_out++;
        }
        strncpy(str_data, p_start, p_end - p_start);
        str_data[p_end - p_start] = '\0';
        val = strtol(str_data,NULL,0);
        data_buf[buf_size] = val;
        buf_size++;
    }
    while (!break_out);
    /* parse write data -e*/
    /* make sure write buffer size is always mutiples of 4 -s*/
    if (buf_size % 4)
    {
        buf_size = (buf_size / 4 + 1) * 4;
    }
    /* make sure write buffer size is always mutiples of 4 -e*/
    BHY2_ASSERT(bhy_hif_write_parameter(&bhy2_hub_obj.handle, param_id, data_buf, buf_size));

    INFO("Writing parameter successful.\r\n");
}

/*!
 * @brief process command: get pin states
 *
 *
 * @param[in] payload: point to command params
 * @param[out] none.
 *
 * @return res: operation result
 */
void cli_cmd_do_get_pin_state(const char *payload)
{
    get_pin_state(payload);
}

/*!
 * @brief signal interrupt handler
 *
 *
 * @param[in] sig_num: signal
 * @param[out] none.
 *
 * @return res: operation result
 */
void sigint_handler(int sig_num)
{
    signal(SIGINT, NULL);
    need_to_stop = 1;
    INFO("Exiting...\r\n");
}

/*!
 * @brief main entrance
 *
 *
 * @param[in] argc: params' count
 * @param[in] *argv[]: point to params
 * @param[out] none.
 *
 */
int main(int argc, char *argv[])
{
	/* used for going through the command line arguments */
	int i;

	/* used to skip parsing the first command if help command was detected */
	int for_start = 1;

	uint8_t sen_en_check = 0;

	/* used to keep track of sensors activated via -c command */
	uint8_t number_of_activated_sensors = 0;

	/* used to enable sensor parsing */
	int sensor_parsing_en = 0;

	uint8_t expected_data;

	/* disable printf buffering for stdout */
	setbuf(stdout, NULL);

	/* allocate memory for the linked list that holds custom driver payload information */
	custom_driver_parameters_head_node = malloc(sizeof(custom_driver_parameters_t));

	/* define end of linked list initially */
	custom_driver_parameters_head_node->next_driver_parameters = NULL;

    INFO("\nCopyright (c) 2019 Bosch Sensortec GmbH\r\n\r\n");

	/* print build date */
	INFO("Build date: " __DATE__ "\r\n\r\n");

    if (argc <= 1)
    {
        ERROR("No argument supplied!\nPlease refer to the following usage message:\r\n\r\n");
        print_usage();
		exit(1);
    }
	/* parse -h before starting board communication -> help can be displayed without a board attached */
	if (argv[1][1] == 'h')
	{
		print_usage();
		/* if help is recognized, don't parse first option of command anymore */
		for_start ++;
	}

	start_board_communication();

	BHY2_ASSERT(bhy2_hub_init(&bhy2_hub_obj, spi_read, spi_write, delay_us, NULL));

    bhy2_register_set(&bhy2_hub_obj, BHY_REG_CHIP_CTRL, 0x01);


    /* Config status channel */
    bhy2_host_interface_ctrl_set(&bhy2_hub_obj, BHY_IFCTL_ASYNC_STATUS_CHANNEL);
    bhy2_register_get(&bhy2_hub_obj, BHY_REG_HOST_INTERFACE_CTRL, &expected_data);
    VAL_ASSERT(expected_data == 0x80);

    /* Install virtual sensor callbacks */
    bhy2_install_callbacks(&bhy2_hub_obj);

    /* Check hub state */
    BHY2_ASSERT(bhy2_product_id_get(&bhy2_hub_obj, &expected_data));

    signal(SIGINT, sigint_handler);

	/* fill the global array which contains information about virtual sensors present in the firmware */
	bhy2_get_present_custom_sensors();

	/* go through all commandline arguments and take action acording to supplied commands */
    for (i = for_start; i < argc; ++i)
    {
        if (argv[i][0] != '-' && argv[i][0] != '/')
        {
            ERROR("Invalid command: %s\r\n", argv[i]);
            exit(1);
        }

        switch (argv[i][1])
        {
			case 'h':
			print_usage();
			break;
            case 'b':
                cli_cmd_do_reset_hub(argv[i]);
                ++i;
				if (i >= argc)
                {
                    ERROR("Firmware path expected.\r\n");
                    exit(1);
                }
                cli_cmd_do_upload_to_ram(argv[i]);
                cli_cmd_do_boot_from_ram();
                break;
            case 'n':
                cli_cmd_do_reset_hub(argv[i]);
                break;
			case 'a':
				++i;
				if (i >= argc)
                {
                    ERROR("Sensor Type and Output Format details expected.\r\n");
                    exit(1);
                }
				cli_cmd_do_add_driver(argv[i]);
				break;
            case 'g':
                ++i;
                if ((argv[i][0]) == 'r')
                {
                    cli_cmd_do_boot_from_ram();

                }
                else if ((argv[i][0]) == 'f')
                {
                    cli_cmd_do_boot_from_flash();

                }
                else
                {
                    ERROR("Invalid boot medium: %s\r\n", argv[i]);
                    exit(1);
                }
                break;
            case 'c':
                ++i;
				if (i >= argc)
                {
                    ERROR("Sensor Type and ODR expected.\r\n");
                    exit(1);
                }
                cli_cmd_do_activate(argv[i]);
				sensor_parsing_en = 1;
				number_of_activated_sensors ++;
                break;
            case 'e':
                cli_cmd_do_erase_flash();
                break;
            case 'r':
                ++i;
                if (i >= argc)
                {
                    ERROR("Register and length expected.\r\n");
                    exit(1);
                }
                cli_cmd_do_read_from_register_address(argv[i]);
                break;
            case 'u':
                ++i;
                cli_cmd_do_upload_to_ram(argv[i]);

                break;
            case 'f':
                ++i;
                if (i >= argc)
                {
                    ERROR("Firmware path expected.\r\n");
                    exit(1);
                }
                cli_cmd_do_upload_to_flash(argv[i]);
                break;
            case 'w':
                ++i;
                if (i >= argc)
                {
                    ERROR("Register and data expected.\r\n");
                    exit(1);
                }
                cli_cmd_do_write_to_register_address(argv[i]);
                break;
            case 'i':
                cli_cmd_do_show_information(argv[i]);
                break;
            case 'p':
                cli_cmd_do_get_pin_state(argv[i]);
                break;
            case 's':
                ++i;
                if (i >= argc)
                {
                    ERROR("Parameter ID expected.\r\n");
                    exit(1);
                }
                cli_cmd_do_read_param(argv[i]);
                break;
            case 't':
                ++i;
                if (i >= argc)
                {
                    ERROR("Parameter ID and data expected.\r\n");
                    exit(1);
                }
                cli_cmd_do_write_param(argv[i]);
                break;
            default:
                ERROR("Unrecognized command %s!\r\n", argv[i]);
                break;
        }
    }

	/* if -c option was used, sensor data will be streamed AFTER parsing and exectuing all commands */
	if(sensor_parsing_en == 1)
	{
		while (!need_to_stop)
		{
			if (get_intr_pin_status())
			{
				BHY2_ASSERT(bhy2_parse_fifos(&bhy2_hub_obj));
			}
			sen_en_check = 0;
			for (uint16_t i = 0; i < BHY_SYS_SENSOR_ID_MAX; i++)
			{

				if (bhy2_hub_obj.virt_sensor[i].info.id)
				{
					sen_en_check = 1;
				}

			}
			if (sen_en_check == 0)
			{
				need_to_stop = 1;
			}
		}
		for (uint16_t i = 0; i < BHY_SYS_SENSOR_ID_MAX; i++)
		{
			if (bhy2_hub_obj.virt_sensor[i].info.id)
			{
				/* deactivate sensor */
				BHY2_ASSERT(bhy2_virt_sensor_cfg_set(&bhy2_hub_obj, i, 0, 0));
			}
		}
		/* parse remaining fifo data */
		for (uint16_t i = 0; i < number_of_activated_sensors * 10 ; i++)
		{
			/* parse & execute bhi160 fifo data */
			bhy2_parse_fifos(&bhy2_hub_obj);
			delay_ms(2, NULL);
		}
	}

    end_board_communication();

    exit(0);
}

