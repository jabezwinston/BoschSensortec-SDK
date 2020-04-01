/**
 * Copyright (C) 2020 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bmi08x_read_from_fifo.c
 * @brief   Sample code to read BMI08x sensor data from FIFO using COINES library
 *
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
/*********************************************************************/
/* own header files */
/*********************************************************************/
#include "coines.h"
#include "bmi08x.h"

/*********************************************************************/
/* local macro definitions */
/*********************************************************************/
/*! i2c interface communication, 1 - Enable; 0- Disable */
#define BMI08x_INTERFACE_I2C             1
/*! spi interface communication, 1 - Enable; 0- Disable */
#define BMI08x_INTERFACE_SPI             0

#if (!((BMI08x_INTERFACE_I2C==1) && (BMI08x_INTERFACE_SPI==0)) && \
	(!((BMI08x_INTERFACE_I2C==0) && (BMI08x_INTERFACE_SPI==1))))
#error "Invalid value given for the macros BMI08x_INTERFACE_I2C / BMI08x_INTERFACE_SPI"
#endif

/*! bmi085 shuttle id */
#define BMI085_SHUTTLE_ID         0x46
/*! bmi088 shuttle id */
#define BMI088_SHUTTLE_ID         0x66

/*! bmi08x Accel Device address */
#define BMI08x_ACCEL_DEV_ADDR BMI08X_ACCEL_I2C_ADDR_PRIMARY
/*! bmi08x Gyro Device address */
#define BMI08x_GYRO_DEV_ADDR BMI08X_GYRO_I2C_ADDR_PRIMARY

/*********************************************************************/
/* global variables */
/*********************************************************************/

/*! @brief This structure containing relevant bmi08x info */
struct bmi08x_dev bmi08xdev;
/*! @brief variable to hold the bmi08x accel data */
struct bmi08x_sensor_data bmi08x_accel;
/*! @brief variable to hold the bmi08x gyro data */
struct bmi08x_sensor_data bmi08x_gyro;

/*********************************************************************/
/* static function declarations */
/*********************************************************************/

/*!
 *  @brief This API is used to parse the sensor data from
 *  the FIFO data and store it in the instance of the structure
 *  bmi08x_sensor_data.
 */
static void unpack_sensor_data(struct bmi08x_sensor_data *sens_data, uint8_t *buffer)
{
    uint16_t data_lsb;
    uint16_t data_msb;
    uint16_t start_idx = 0;

    /* Gyro raw x data */
    data_lsb = buffer[start_idx++];
    data_msb = buffer[start_idx++];
    sens_data->x = (int16_t)((data_msb << 8) | data_lsb);

    /* Gyro raw y data */
    data_lsb = buffer[start_idx++];
    data_msb = buffer[start_idx++];
    sens_data->y = (int16_t)((data_msb << 8) | data_lsb);

    /* Gyro raw z data */
    data_lsb = buffer[start_idx++];
    data_msb = buffer[start_idx++];
    sens_data->z = (int16_t)((data_msb << 8) | data_lsb);
}

/*!
 *  @brief This internal API is used to initialize the sensor interface depending
 *   on selection either SPI or I2C.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void init_sensor_interface(void)
{
    /* Switch VDD for sensor off */
    coines_set_shuttleboard_vdd_vddio_config(0, 0);
    coines_delay_msec(10);
#if BMI08x_INTERFACE_I2C==1
    /* set the sensor interface as I2C with 400kHz speed*/
    coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_FAST_MODE);
    coines_delay_msec(10);
    /* PS pin is made high for selecting I2C protocol*/
    coines_set_pin_config(COINES_SHUTTLE_PIN_9, COINES_PIN_DIRECTION_OUT,
                          COINES_PIN_VALUE_HIGH);
#endif
#if BMI08x_INTERFACE_SPI==1
    /* CS pin is made high for selecting SPI protocol*/
    coines_set_pin_config(COINES_SHUTTLE_PIN_8, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
    /* CS pin is made high for selecting SPI protocol*/
    coines_set_pin_config(COINES_SHUTTLE_PIN_14, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
    /* PS pin is made low for selecting SPI protocol*/
    coines_set_pin_config(COINES_SHUTTLE_PIN_9, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
    coines_delay_msec(10);
    coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_5_MHZ, COINES_SPI_MODE3);
#endif
    coines_delay_msec(10);
    /* Switch VDD for sensor on */
    coines_set_shuttleboard_vdd_vddio_config(3300, 3300);
}

/*!
 *  @brief This internal API is used to initializes the bmi08x sensor
 *  settings like power mode and OSRS settings.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void init_bmi08x(void)
{
    if (bmi08a_init(&bmi08xdev) == BMI08X_OK
        && bmi08g_init(&bmi08xdev) == BMI08X_OK)
    {
        printf("BMI08x initialization success !\n");
        printf("Accel chip ID - 0x%x\n", bmi08xdev.accel_chip_id);
        printf("Gyro chip ID - 0x%x\n", bmi08xdev.gyro_chip_id);
    }
    else
    {
        printf("BMI08x initialization failure !\n");
        exit(COINES_E_FAILURE);
    }
    bmi08xdev.accel_cfg.odr = BMI08X_ACCEL_ODR_1600_HZ;
#if BMI08X_FEATURE_BMI085 == 1
    bmi08xdev.accel_cfg.range = BMI085_ACCEL_RANGE_16G;
#elif BMI08X_FEATURE_BMI088 == 1
    bmi08xdev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
#endif
    bmi08xdev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE; //user_accel_power_modes[user_bmi088_accel_low_power];
    bmi08xdev.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL; /* Bandwidth and OSR are same */

    bmi08a_set_power_mode(&bmi08xdev);
    coines_delay_msec(10);
    bmi08a_set_meas_conf(&bmi08xdev);
    coines_delay_msec(10);

    bmi08xdev.gyro_cfg.odr = BMI08X_GYRO_BW_230_ODR_2000_HZ;
    bmi08xdev.gyro_cfg.range = BMI08X_GYRO_RANGE_250_DPS;
    bmi08xdev.gyro_cfg.bw = BMI08X_GYRO_BW_230_ODR_2000_HZ;
    bmi08xdev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;

    bmi08g_set_power_mode(&bmi08xdev);
    coines_delay_msec(10);
    bmi08g_set_meas_conf(&bmi08xdev);
    coines_delay_msec(10);
}

/*!
 *  @brief This internal API is used to set the sensor driver interface to
 *  read/write the data.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void init_bmi08x_sensor_driver_interface(void)
{
#if BMI08x_INTERFACE_I2C==1
    /* I2C setup */
    /* link read/write/delay function of host system to appropriate
     * bmi08x function call prototypes */
    bmi08xdev.write = coines_write_i2c;
    bmi08xdev.read = coines_read_i2c;
    bmi08xdev.delay_ms = coines_delay_msec;
    /* set correct i2c address */
    bmi08xdev.accel_id = (unsigned char) BMI08x_ACCEL_DEV_ADDR;
    bmi08xdev.gyro_id = (unsigned char) BMI08x_GYRO_DEV_ADDR;
    bmi08xdev.intf = BMI08X_I2C_INTF;
#endif
#if BMI08x_INTERFACE_SPI==1
    /* SPI setup */
    /* link read/write/delay function of host system to appropriate
     *  bmi08x function call prototypes */
    bmi08xdev.write = coines_write_spi;
    bmi08xdev.read = coines_read_spi;
    bmi08xdev.delay_ms = coines_delay_msec;
    bmi08xdev.intf = BMI08X_SPI_INTF;
    bmi08xdev.accel_id = COINES_SHUTTLE_PIN_8;
    bmi08xdev.gyro_id = COINES_SHUTTLE_PIN_14;
#endif
}

/*!
 *  @brief Main Function where the execution getting started to test the code.
 *
 *  @param[in] argc
 *  @param[in] argv
 *
 *  @return status
 *
 */
int main(int argc, char *argv[])

{
    struct coines_board_info board_info;
    int16_t rslt;
    /* Max number of bytes that can be read over COINES usb connection: 53 
       (This is due to required backward compatibility and will be enhanced soon.) */
    uint16_t buf_len = 53;
	uint8_t buffer[buf_len];

    init_bmi08x_sensor_driver_interface();

    rslt = coines_open_comm_intf(COINES_COMM_INTF_USB);

    if (rslt < 0)
    {
        printf("\n Unable to connect with Application Board ! \n"
               " 1. Check if the board is connected and powered on. \n"
               " 2. Check if Application Board USB driver is installed. \n"
               " 3. Check if board is in use by another application. (Insufficient permissions to access USB) \n");
        exit(rslt);
    }
    rslt = coines_get_board_info(&board_info);

    if (rslt == COINES_SUCCESS)
    {
        if ((board_info.shuttle_id != BMI085_SHUTTLE_ID)
            && (board_info.shuttle_id != BMI088_SHUTTLE_ID))
        {

            printf("! Warning invalid sensor shuttle \n ,"
                   "This application will not support this sensor \n");
            exit(COINES_E_FAILURE);
        }
    }

    /* *********************************************************************************************/    
    /* Configure APP2.0 pin, which is connected to INT3 pin, as input pin */
    coines_set_pin_config(COINES_SHUTTLE_PIN_22, COINES_PIN_DIRECTION_IN, COINES_PIN_VALUE_LOW);
    
    init_sensor_interface();
    /* after sensor init introduce 200 msec sleep */
    coines_delay_msec(200);
	init_bmi08x();
    
    /* *********************************************************************************************/
    printf("################################################################\n");
    printf("1. Configure gyro FIFO to stop-at-full, INT3 as FIFO interrrupt\n");
    printf("   pin, set watermark interrupt at 10 frames.\n");

    /* Gyro config: 100Hz ODR, no oversampling */ 
    buffer[0] |= 0x87;
    bmi08g_set_regs(0x10, &buffer[0], 1, &bmi08xdev);

    /* Configure INT3 as output pin, level low: nothing to do, already default (reg. 0x16) */
    /* Set INT3 as FIFO interrrupt pin */
    buffer[0] = 0x04; // Set bit #2
    bmi08g_set_regs(0x18, &buffer[0], 1, &bmi08xdev);
        
    /* Enable FIFO data interrupt */
    bmi08g_get_regs(0x15, &buffer[0], 1, &bmi08xdev);
    buffer[0] |= 0x40; // Set bit #6
    bmi08g_set_regs(0x15, &buffer[0], 1, &bmi08xdev);
	
    /* Enable watermark interrrupt */
    buffer[0] = 0x88; 
    bmi08g_set_regs(0x1e, &buffer[0], 1, &bmi08xdev);
    
    /* Set watermark to 10 frames */
    uint8_t watermark = 10;
    bmi08g_set_regs(0x3d, &watermark, 1, &bmi08xdev);
    
    /* Switch on FIFO mode (stop-at-full) */
    buffer[0] = 0x40;
    bmi08g_set_regs(0x3e, &buffer[0], 1, &bmi08xdev);

    /* Wait for FIFO to be filled */
    enum coines_pin_value pin = COINES_PIN_VALUE_LOW;
    printf("FIFO frame counter (reg 0x0e & 0x7F): ");
    while(pin == COINES_PIN_VALUE_LOW)
    {
        bmi08g_get_regs(0x0e, buffer, 1, &bmi08xdev);
        printf("%d ... ", buffer[0] & 0x7F);
        coines_get_pin_config(COINES_SHUTTLE_PIN_22, (enum coines_pin_direction *) &buffer[0], (enum coines_pin_value *) &pin);
    }
    printf("%d frames.\n", watermark);
    
    /* read 60 bytes of aligned sensor data. */
    bmi08g_get_regs(0x3f, buffer, watermark * 6, &bmi08xdev);
    for(int i = 0; i < watermark; i ++)
    {
        unpack_sensor_data(&bmi08x_gyro, &buffer[i * 6]);
        printf("frame: %03d gx:%d gy:%d gz:%d\n", i, bmi08x_gyro.x, bmi08x_gyro.y, bmi08x_gyro.z);
    }
    

    /* *********************************************************************************************/
    printf("\n################################################################\n");
    printf("2. Configure accel FIFO to stop-at-full, read accel data from FIFO\n");    
        
    /* Switch on accelerometer */
    bmi08xdev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
    bmi08a_set_power_mode(&bmi08xdev);
    coines_delay_msec(50);
    
    /* ODR = 1.6kHz, no oversampling */
    bmi08xdev.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL; 
    bmi08xdev.accel_cfg.odr = BMI08X_ACCEL_ODR_400_HZ;
    bmi08a_set_meas_conf(&bmi08xdev);
    coines_delay_msec(100);
    
    /* Desired FIFO mode is stop-at-full: set bit #0 to 1 in 0x48 
       Bit #1 must always be one! */
    buffer[0] = 0x01 | 0x02;
    rslt = bmi08a_set_regs(0x48, &buffer[0], 1, &bmi08xdev);

    /* Downsampling factor 2**4 = 16: write 4 into bit #4-6 of reg. 0x45 
       Bit #7 must always be one! */
    buffer[0] = (4 << 4) | 0x80;
    rslt = bmi08a_set_regs(0x45, &buffer[0], 1, &bmi08xdev);
    
    /* Set water mark to 42 bytes (aka 6 frames, each 7 bytes: 1 byte header + 6 bytes accel data) */ 
    uint16_t wml = 42;
    buffer[0] = (uint8_t) wml & 0xff;
    buffer[1] = (uint8_t) (wml >> 8) & 0xff;
    rslt = bmi08a_set_regs(0x46, &buffer[0], 2, &bmi08xdev);
    
    /* Enable the actual FIFO functionality: write 0x50 to 0x49 
       Bit #4 must always be one! */
    buffer[0] = 0x10 | 0x40;
    rslt = bmi08a_set_regs(0x49, &buffer[0], 1, &bmi08xdev);
    coines_delay_msec(50);

    int fifo_fill_level = 0;
    printf("FIFO fill level: ");
    while(fifo_fill_level < wml)
    {
        rslt = bmi08a_get_regs(0x24, buffer, 2, &bmi08xdev);
        fifo_fill_level = buffer[0] + 256 * buffer[1];
        printf("%d ... ", fifo_fill_level);
    }
    printf("bytes.\n");
    
    bmi08a_get_regs(0x26, buffer, fifo_fill_level, &bmi08xdev);
    /* This is a super-simple FIFO parsing loop, hoping it will only find valid accel data packets */
    for(int i = 0; i < fifo_fill_level;)
    {
        /* Header of acceleration sensor data frame: 100001xxb, where x is INT1/INT2 tag, ignored here */
        if(buffer[i] == (0x84 & 0x8c))
        {
            unpack_sensor_data(&bmi08x_accel, &buffer[i + 1]);
            printf("frame: %03d ax:%d ay:%d az:%d\n", i/6, bmi08x_accel.x, bmi08x_accel.y, bmi08x_accel.z);
            i += 7;
        }
        else
            i++;
    }

    coines_close_comm_intf(COINES_COMM_INTF_USB);
    return 0;
}
