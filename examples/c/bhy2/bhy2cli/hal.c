/**
 * Copyright (C) 2019 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    hal.c
 * @date    April-22-2019
 * @brief   Hardware abstraction layer for the bhy2 command line interface
 *
 */

#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>

#include "hal.h"
#include "coines.h"
#include "comm_intf.h"

#ifndef UNUSED
#define UNUSED(x) (void)(x)
#endif

#define BHA260_CS_PIN COINES_SHUTTLE_PIN_7


#define BHA260_SHUTTLE_ID 0x139
#define BHI260_SHUTTLE_ID 0x119

#define COINES_ZEUS_BHI_INT         0x00 /**<  int pin to BHI2 hub on ZEUS board */
#define COINES_ZEUS_BHI_RST         0x01 /**<  reset pin to BHI2 hub on ZEUS board*/
#define COINES_ZEUS_LED             0x02 /**<  LED control pin on ZEUS board*/
#define COINES_ZEUS_KEY             0x03 /**<  KEY control pin on ZEUS board*/
#define COINES_ZEUS_GPS             0x04 /**<  GPS control pin on ZEUS board*/

#define VAL_ASSERT(expr)	if(!(expr)) ERROR("Assert failed at line %d.\r\n", __LINE__)
#define INFO(format, ...)	printf(format, ##__VA_ARGS__)
#define INFO_TEXT(format, ...)	printf(format, ##__VA_ARGS__)
#define ERROR(format, ...)	printf("[Error]"format, ##__VA_ARGS__)
#define COINES_ASSERT(expr) coines_check(__LINE__, #expr, expr)

struct coines_board_info board_info;
/*! Variable to hold the board type*/
extern coines_board_t coines_board;

static void coines_check(int line, const char *func, int val)
{
    if (val != COINES_SUCCESS)
    {
        ERROR("COINES API failed at line %d. The function %s returned error code %d\r\n", line, func,
              val);

        switch (val)
		{
			case COINES_SUCCESS:
				break;

			case COINES_E_COMM_IO_ERROR:
				printf("Unable to initialize libusb \r\n"
					   "1.Check that your system has a usb controller \r\n"
					   "2.USB port unresponsive \r\n");
				break;

			case COINES_E_DEVICE_NOT_FOUND:
				printf("Unable to find the device \r\n"
					   "1.Check that device is connected \r\n"
					   "2.Check the device is powered on \r\n");
				break;

			case COINES_E_UNABLE_OPEN_DEVICE:
				printf("Unable to open the device. \r\n"
					   "1.USB Access denied or insufficient permissions \r\n"
					   "2.LibUSB driver not installed. Please install bst_app20_libusb_driver.exe from C:\\COINES\\driver [Windows]\r\n");
				break;

			case COINES_E_MEMORY_ALLOCATION:
				printf("Unable to allocate required memory. \r\n");
				break;

			default:
				printf("General error. Please check exit code for more details. \r\n");
				break;
		}

        exit(1);
    }
}



/*!
 * @brief initialize app board
 *
 *
 * @param[in] none.
 * @param[out] none.
 *
 * @return none.
 */
static void init_app_board_communication()
{
	/* setup SPI connection to hub*/

	COINES_ASSERT(coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_5_MHZ, COINES_SPI_MODE0));
	coines_delay_msec(10);
	COINES_ASSERT(coines_set_shuttleboard_vdd_vddio_config(3300, 3300));
	coines_delay_msec(10);

	/* CS pin is made low for selecting SPI protocol */
	COINES_ASSERT(coines_set_pin_config(COINES_SHUTTLE_PIN_7, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW));
	coines_delay_msec(10);
	COINES_ASSERT(coines_set_pin_config(COINES_SHUTTLE_PIN_7, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH));

	/* config INT pin */
	coines_set_pin_config(COINES_SHUTTLE_PIN_21, COINES_PIN_DIRECTION_IN, COINES_PIN_VALUE_LOW);
	coines_delay_msec(10);

}

/*!
 * @brief initialize WRD board
 *
 *
 * @param[in] none.
 * @param[out] none.
 *
 * @return none.
 */
static void init_wrd_board_communication()
{
    /* zeus board inits spi interface by itself, no need host control */
}

/**
 * @brief Start the communication interface
 */
void start_board_communication()
{

	/* setup communication to appboard*/
    COINES_ASSERT(comm_intf_open(COINES_COMM_INTF_USB, &coines_board));
    COINES_ASSERT(coines_get_board_info(&board_info));

    switch(board_info.board)
    {
        case 1: /* 1 --> app board */
        case 2: /* 2 --> dev board */
        case 3: /* 3 --> app board 2.0 */
            if (BHI260_SHUTTLE_ID == board_info.shuttle_id)
            {
                INFO("Found BHI260 shuttle on APP 2.0 board.\r\n");
            }
            else if (BHA260_SHUTTLE_ID == board_info.shuttle_id)
            {
                INFO("Found BHA260 shuttle on APP 2.0 board.\r\n");
            }
            else
            {
                ERROR("Expected the BHI260/BHA260 Shuttle ID 0x%04X/0x%04X but found 0x%04X instead, exiting.\r\n",
                      BHI260_SHUTTLE_ID,
                      BHA260_SHUTTLE_ID, board_info.shuttle_id);
                exit(0);
            }
            init_app_board_communication();
            break;
        case 4: /* 4 --> BNO usb stick */
            INFO("Found BNO usb stick.\r\n");
            break;
        case 5: /* 5 --> Nuwa board */
            INFO("Found APP 3.0 board.\r\n");
            break;
        case 6: /* 6 --> Zeus board */
            INFO("Found WRD board Successfully!\n\r");
            init_wrd_board_communication();
            break;
        default:
            ERROR("No board connected %d!\n\r", board_info.board);
    	    exit(1);
            break;
    }
}

/**
 * @brief End the communication with the sensor
 */
void end_board_communication(void)
{
    COINES_ASSERT(coines_close_comm_intf(COINES_COMM_INTF_USB));
}

/*!
 * @brief Read registers over SPI
 * @param[in] reg: register addr.
 * @param[out] buf: data from register.
 * @param[in] len: data length.
 * @param[in] private_data: private data.
 * @return 0 on success, negative on failure.
 */
int32_t spi_read(uint8_t reg, uint8_t *buf, uint32_t len, void *private_data)
{
    UNUSED(private_data);
    return coines_read_spi(BHA260_CS_PIN, reg, buf, len);
}

/*!
 * @brief Write registers over SPI
 * @param[in] reg: register addr.
 * @param[in] buf: data written into register.
 * @param[in] len: data length.
 * @param[in] private_data: private data.
 * @return 0 on success, negative on failure.
 */
int32_t spi_write(uint8_t reg, const uint8_t *buf, uint32_t len, void *private_data)
{

    UNUSED(private_data);
    return coines_write_spi(BHA260_CS_PIN, reg, (uint8_t*)buf, len);
}


/*!
 * @brief Delay function in microsecond.
 * @param[in] private_data: private data.
 */
void delay_us(uint32_t us, void *private_data)
{
    UNUSED(private_data);
    struct timespec req, rem;
    req.tv_sec = 0;
    req.tv_nsec = us * 1000;
    nanosleep(&req, &rem);

}

/*!
 * @brief Delay function in millisecond.
 * @param[in] private_data: private data.
 */
void delay_ms(uint32_t ms, void *private_data)
{
    delay_us(ms * 1000, private_data);
}

/*!
 * @brief get hardware platform systick.
 * @param[in] private_data: private data.
 * @return tmp_systick: return platform systick, unit is millisecond.
 */
uint32_t get_systick_ms(void *private_data)
{
    UNUSED(private_data);

    clock_t currentTime = clock();

    return (uint32_t)(currentTime / (CLOCKS_PER_SEC / 1000));
}

/**
 * @brief Get the status of the interrupt pin
 */
bool get_intr_pin_status(void)
{
    enum coines_pin_direction pin_direction;
    enum coines_pin_value pin_value;

    switch(board_info.board)
    {
        case 1: /* 1 --> app board */
        case 2: /* 2 --> dev board */
        case 3: /* 3 --> app board 2.0 */
            coines_get_pin_config(COINES_SHUTTLE_PIN_21,
                                  (enum coines_pin_direction *)&pin_direction,
                                  (enum coines_pin_value *)&pin_value);
            break;
        case 4: /* 4 --> BNO usb stick */
            break;
        case 5: /* 5 --> Nuwa board */
            break;
        case 6: /* 6 --> Zeus board */
            coines_get_pin_config(COINES_ZEUS_BHI_INT,
                                  (enum coines_pin_direction *)&pin_direction,
                                  (enum coines_pin_value *)&pin_value);
            break;
        default:
            ERROR("No board connected %d!\n\r", board_info.board);
            exit(1);
            break;
    }


    return pin_value;
}

/**
 * @brief Get the status of the pins connected on the board
 */
void get_pin_state(const char *payload)
{
    enum coines_pin_direction pin_direction;
    enum coines_pin_value pin_value;

    switch(board_info.board)
    {
        case 1: /* 1 --> app board */
        case 2: /* 2 --> dev board */
        case 3: /* 3 --> app board 2.0 */
            COINES_ASSERT(coines_get_pin_config(COINES_SHUTTLE_PIN_21, &pin_direction, &pin_value));

            INFO("Pin: BHI_INT, Dir: %s, Val: 0x%02X.\r\n", (pin_direction ? "OUT" : "IN "), pin_value);

            COINES_ASSERT(coines_get_pin_config(COINES_SHUTTLE_PIN_7, &pin_direction, &pin_value));

            INFO("Pin: BHI_CS, Dir: %s, Val: 0x%02X.\r\n", (pin_direction ? "OUT" : "IN "), pin_value);
            break;
        case 4: /* 4 --> BNO usb stick */
            break;
        case 5: /* 5 --> Nuwa board */
            break;
        case 6: /* 6 --> Zeus board */
            COINES_ASSERT(coines_get_pin_config(COINES_ZEUS_BHI_INT, &pin_direction, &pin_value));

            INFO("Pin: BHI_INT, Dir: %s, Val: 0x%02X.\r\n", (pin_direction ? "OUT" : "IN "), pin_value);
            break;
        default:
            ERROR("No board connected %d!\n\r", board_info.board);
            exit(1);
            break;
    }


}
