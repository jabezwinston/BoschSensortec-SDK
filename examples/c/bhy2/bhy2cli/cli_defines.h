/**
 * Copyright (C) 2019 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    cli_defines.h
 * @date    Feb-05-2019
 * @brief   Register defines for bhy2cli
 *
 */
 
 #define BHY2CLI_MAX_STRING_LENGTH 32
 
 typedef struct enum_entry_t {
    const char*     output_text;
    unsigned int    mask;
    unsigned int    value;
} enum_entry_t;

typedef struct enum_register_value_interpretation_t {
	const char* output_text;
	const char* error_category;
	unsigned int value;
} enum_register_value_interpretation_t;

/* used to print boot status with cli tool (-i option) */
static enum_entry_t boot_status_fields[] = {
    { "Flash Detected", 0x01, 0x01 },
    { "Flash Not Detected", 0x01, 0x00 },

    { "Flash Verify Done", 0x02, 0x02 },
    { "Flash Verify Not Done", 0x02, 0x00 },

    { "Flash Verify Error", 0x04, 0x04 },
    { "Flash Verify OK", 0x04, 0x00 },

    { "Flash Not Installed", 0x08, 0x08 },
    { "Flash Installed", 0x08, 0x00 },

    { "Host Interface Ready", 0x10, 0x10 },
    { "Host Interface Not Ready", 0x10, 0x00 },

    { "Firmware Verify Done", 0x20, 0x20 },
    { "Firmware Verify Not Done", 0x20, 0x00 },

    { "Firmware Verify Error", 0x40, 0x40 },
    { "Firmware Verify OK", 0x40, 0x00 },

    { "Firmware Idle", 0x80, 0x80 },
    { "Firmware Running", 0x80, 0x00 },

    { 0, 0, 0 }
};

/* used to print error register content + interpretation with cli tool (-i option) */
static enum_register_value_interpretation_t error_values[] =
{
	{ "No Error","-",0x00},
	{ "Firmware Expected Version Mismatch","Fatal",0x10},
	{ "Firmware Upload Failed: Bad Header CRC","Fatal",0x11},
	{ "Firmware Upload Failed: SHA Hash Mismatch","Fatal",0x12},
	{ "Firmware Upload Failed: Bad Image CRC","Fatal",0x13},
	{ "Firmware Upload Failed: ECDSA Signature Verification Failed","",0x14},
	{ "Firmware Upload Failed: Bad Public Key CRC","Fatal",0x15},
	{ "Firmware Upload Failed: Signed Firmware Required","Fatal",0x16},
	{ "Firmware Upload Failed: FW Header Missing","Fatal",0x17},
	{ "Unexpected Watchdog Reset","Fatal",0x19},
	{ "ROM Version Mismatch","Fatal",0x1a},
	{ "Fatal Firmware Error","Fatal",0x1b},
	{ "Chained Firmware Error: Next Payload Not Found","Fatal",0x1c},
	{ "Chained Firmware Error: Payload Not Valid","",0x1d},
	{ "Chained Firmware Error: Payload Entries Invalid","Fatal",0x1e},
	{ "Bootloader Error: OTP CRC Invalid","Fatal",0x1f},
	{ "Firmware Init Failed","Hardware",0x20},
	{ "Sensor Init Failed: Unexpected Device ID","Hardware",0x21},
	{ "Sensor Init Failed: No Response from Device","Programming",0x22},
	{ "Sensor Init Failed: Unknown","Programming",0x23},
	{ "Sensor Error: No Valid Data","Programming",0x24},
	{ "Slow Sample Rate","Temporary",0x25},
	{ "Data Overflow (saturated sensor data)","Fatal",0x26},
	{ "Stack Overflow","Fatal",0x27},
	{ "Insufficient Free RAM","Fatal",0x28},
	{ "Sensor Init Failed: Driver Parsing Error","Fatal",0x29},
	{ "Too Many RAM Banks Required","Programming",0x2a},
	{ "Invalid Event Specified","Programming",0x2b},
	{ "More than 32 On Change","Programming",0x2c},
	{ "Firmware Too Large","Fatal",0x2d},
	{ "Invalid RAM Banks","Fatal",0x2f},
	{ "Math Error","Fatal",0x30},
	{ "Memory Error","Fatal",0x40},
	{ "SWI3 Error","Fatal",0x41},
	{ "SWI4 Error","Fatal",0x42},
	{ "Illegal Instruction Error","Fatal",0x43},
	{ "Unhandled Interrupt Error / Exception / Postmortem Available","Fatal",0x44},
	{ "Invalid Memory Access","Fatal",0x45},
	{ "Algorithm Error: BSX Init","Programming",0x50},
	{ "Algorithm Error: BSX Do Step","Programming",0x51},
	{ "Algorithm Error: Update Sub","Programming",0x52},
	{ "Algorithm Error: Get Sub","Programming",0x53},
	{ "Algorithm Error: Get Phys","Programming",0x54},
	{ "Algorithm Error: Unsupported Phys Rate","Programming",0x55},
	{ "Algorithm Error: Cannot find BSX Driver","Programming",0x56},
	{ "Sensor Self-Test Failure","Hardware",0x60},
	{ "Sensor Self-Test X Axis Failure","Hardware",0x61},
	{ "Sensor Self-Test Y Axis Failure","Hardware",0x62},
	{ "Sensor Self-Test Z Axis Failure","Hardware",0x64},
	{ "FOC Failure","Hardware",0x65},
	{ "Sensor Busy","Hardware",0x66},
	{ "Self-Test or FOC Test Unsupported","Programming",0x6f},
	{ "No Host Interrupt Set","Fatal",0x72},
	{ "Event ID Passed to Host Interface Has No Known Size","Programming",0x73},
	{ "Host Download Channel Underflow (Host Read Too Fast)","Temporary",0x75},
	{ "Host Upload Channel Overflow (Host Wrote Too Fast)","Temporary",0x76},
	{ "Host Download Channel Empty","Temporary",0x77},
	{ "DMA Error","Hardware",0x78},
	{ "Corrupted Input Block Chain","Programming",0x79},
	{ "Corrupted Output Block Chain","Programming",0x7a},
	{ "Buffer Block Manager Error","Programming",0x7b},
	{ "Input Channel Not Word Aligned","Temporary",0x7c},
	{ "Too Many Flush Events","Temporary",0x7d},
	{ "Unknown Host Channel Error","Hardware",0x7e},
	{ "Decimation Too Large","Programming",0x81},
	{ "Master SPI/I2C Queue Overflow","Fatal",0x90},
	{ "SPI/I2C Callback Error","Fatal",0x91},
	{ "Timer Scheduling Error","Fatal",0xa0},
	{ "Invalid GPIO for Host IRQ","Fatal",0xb0},
	{ "Error Sending Initialized Meta Events","Fatal",0xb1},
	{ "Command Error","Temporary",0xc0},
	{ "Command Too Long","Temporary",0xc1},
	{ "Command Buffer Overflow","Temporary",0xc2},
	{ "User Mode Error: Sys Call Invalid","Fatal",0xd0},
	{ "User Mode Error: Trap Invalid","Fatal",0xd1},
	{ "Firmware Upload Failed: Firmware header corrupt","Fatal",0xe1},
	{ "Sensor Data Injection: Invalid input stream","Programming",0xe2},	
};	

	
	
	
	
	