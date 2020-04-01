/**
 * Copyright (c) 2020 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bmi270_fifo_watermark_headerless_mode.c
 *
 */

/*!
 * @ingroup bmi2xyGroupExample
 * @defgroup bmi2xyGroupExample FIFO
 * @brief example to showcase FIFO watermark level for accel and gyro data in headerless mode
 * \include fifo_watermark_headerless_mode.c
 */

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include "bmi2.h"
#include "bmi270.h"
#include "bmi2_error_codes.h"
#include "bmi2xy_hal_interface.h"

/******************************************************************************/
/*!                  Macros                                                   */

/*! I2C interface communication, 1 - Enable; 0- Disable */
#define BMI270_INTERFACE_I2C             UINT8_C(0)

/*! SPI interface communication, 1 - Enable; 0- Disable */
#define BMI270_INTERFACE_SPI             UINT8_C(1)

#if (!((BMI270_INTERFACE_I2C==1) && (BMI270_INTERFACE_SPI==0)) && \
    (!((BMI270_INTERFACE_I2C==0) && (BMI270_INTERFACE_SPI==1))))
#error "Invalid value given for the macros BMI270_INTERFACE_I2C / BMI270_INTERFACE_SPI"
#endif

/*! Buffer size allocated to store raw FIFO data */
#define BMI270_FIFO_RAW_DATA_BUFFER_SIZE UINT16_C(2048)

/*! Length of data to be read from FIFO */
#define BMI270_FIFO_RAW_DATA_USER_LENGTH UINT16_C(600)

/*! Number of accel frames to be extracted from FIFO */
#define BMI270_FIFO_ACCEL_FRAME_COUNT    UINT8_C(50)

/*! Number of gyro frames to be extracted from FIFO */
#define BMI270_FIFO_GYRO_FRAME_COUNT     UINT8_C(50)

/*! Setting a watermark level in FIFO */
#define BMI270_FIFO_WATERMARK_LEVEL      UINT16_C(600)

/******************************************************************************/
/*!           Static Function Declaration                                     */

/*!
 *  @brief This internal API is used to set configurations for accel and gyro.
 *  @param[in] dev       : Structure instance of bmi2_dev.
 *  @return Status of execution.
 */
static int8_t set_accel_gyro_config(struct bmi2_dev *dev);

/******************************************************************************/
/*!                 Functions                                                 */

/* This function starts the execution of program. */
int main(void)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Variable to index bytes. */
    uint16_t index = 0;

    uint16_t fifo_length = 0;

    uint16_t accel_frame_length = BMI270_FIFO_ACCEL_FRAME_COUNT;

    uint16_t gyro_frame_length = BMI270_FIFO_GYRO_FRAME_COUNT;

    /* Number of bytes of FIFO data. */
    uint8_t fifo_data[BMI270_FIFO_RAW_DATA_BUFFER_SIZE] = { 0 };

    /* Sensor initialization configuration. */
    struct bmi2_dev dev;

    /* Array of accelerometer frames -> Total bytes =
     * 50 * (6 bytes(2 for each axis)) = 300 bytes */
    struct bmi2_sens_axes_data fifo_accel_data[BMI270_FIFO_ACCEL_FRAME_COUNT] = { 0 };

    /* Array of gyro frames -> Total bytes =
     * 50 * (6 bytes(2 for each axis)) = 300 bytes */
    struct bmi2_sens_axes_data fifo_gyro_data[BMI270_FIFO_GYRO_FRAME_COUNT] = { 0 };

    /* Initialize FIFO frame structure. */
    struct bmi2_fifo_frame fifoframe = { 0 };

    /* Accel and gyro sensor are listed in array. */
    uint8_t sensor_sel[2] = { BMI2_ACCEL, BMI2_GYRO };

    /* Variable to get fifo water-mark interrupt status. */
    uint16_t int_status = 0;

    uint16_t watermark = 0;

    /* Bus configuration : I2C */
    #if BMI270_INTERFACE_I2C == 1

    /* To initialize the hal function */
    bmi2xy_hal_i2c_init();
    dev.dev_id = BMI2_I2C_PRIM_ADDR;
    dev.intf = BMI2_I2C_INTERFACE;
    dev.read = bmi2xy_hal_i2c_bus_read;
    dev.write = bmi2xy_hal_i2c_bus_write;

    /* Bus configuration : SPI */
    #elif BMI270_INTERFACE_SPI == 1

    /* To initialize the hal function */
    bmi2xy_hal_spi_init();
    dev.dev_id = SPI_CS;
    dev.intf = BMI2_SPI_INTERFACE;
    dev.read = bmi2xy_hal_spi_bus_read;
    dev.write = bmi2xy_hal_spi_bus_write;
    #endif

    dev.read_write_len = 32;
    dev.delay_us = bmi2xy_hal_delay_usec;

    /* Assign to NULL to load the default config file. */
    dev.config_file_ptr = NULL;

    /* Initialize bmi270. */
    rslt = bmi270_init(&dev);
    print_rslt(rslt);

    /* Enable accel and gyro sensor. */
    rslt = bmi2_sensor_enable(sensor_sel, 2, &dev);
    print_rslt(rslt);

    /* Configuration settings for accel and gyro. */
    rslt = set_accel_gyro_config(&dev);
    print_rslt(rslt);

    /* Before setting FIFO, disable the advance power save mode. */
    rslt = bmi2_set_adv_power_save(BMI2_DISABLE, &dev);
    print_rslt(rslt);

    /* Initially disable all configurations in fifo. */
    rslt = bmi2_set_fifo_config(BMI2_FIFO_ALL_EN, BMI2_DISABLE, &dev);
    print_rslt(rslt);

    /* Update FIFO structure. */
    /* Mapping the buffer to store the fifo data. */
    fifoframe.data = fifo_data;

    /* Length of FIFO frame. */
    fifoframe.length = BMI270_FIFO_RAW_DATA_USER_LENGTH;

    /* Set FIFO configuration by enabling accel, gyro.
     * NOTE 1: The header mode is enabled by default.
     * NOTE 2: By default the FIFO operating mode is FIFO mode. */
    rslt = bmi2_set_fifo_config(BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN, BMI2_ENABLE, &dev);
    print_rslt(rslt);

    printf("FIFO is configured in headerless mode\n");

    /* To enable headerless mode, disable the header. */
    if (rslt == BMI2_OK)
    {
        rslt = bmi2_set_fifo_config(BMI2_FIFO_HEADER_EN, BMI2_DISABLE, &dev);
    }

    /* FIFO water-mark interrupt is enabled. */
    fifoframe.data_int_map = BMI2_FWM_INT;

    /* Map water-mark interrupt to the required interrupt pin. */
    rslt = bmi2_map_data_int(fifoframe.data_int_map, BMI2_INT1, &dev);
    print_rslt(rslt);

    /* Set water-mark level. */
    fifoframe.wm_lvl = BMI270_FIFO_WATERMARK_LEVEL;

    fifoframe.length = BMI270_FIFO_RAW_DATA_USER_LENGTH;

    /* Set the water-mark level if water-mark interrupt is mapped. */
    rslt = bmi2_set_fifo_wm(fifoframe.wm_lvl, &dev);
    print_rslt(rslt);

    rslt = bmi2_get_fifo_wm(&watermark, &dev);
    print_rslt(rslt);
    printf("\nFIFO watermark level is %d\n", watermark);
    while (1)
    {
        /* Read FIFO data on interrupt. */
        rslt = bmi2_get_int_status(&int_status, &dev);

        /* To check the status of FIFO watermark interrupt. */
        if ((rslt == BMI2_OK) && (int_status & BMI2_FWM_INT_STATUS_MASK))
        {
            printf("\nWatermark interrupt occurred\n");

            rslt = bmi2_get_fifo_length(&fifo_length, &dev);
            print_rslt(rslt);
            printf("\nFIFO data bytes available : %d \n", fifo_length);
            printf("\nFIFO data bytes requested : %d \n", fifoframe.length);

            /* Read FIFO data. */
            rslt = bmi2_read_fifo_data(&fifoframe, &dev);
            print_rslt(rslt);

            if (rslt == BMI2_OK)
            {
                printf("\nFIFO accel frames requested : %d \n", accel_frame_length);

                /* Parse the FIFO data to extract accelerometer data from the FIFO buffer. */
                rslt = bmi2_extract_accel(fifo_accel_data, &accel_frame_length, &fifoframe, &dev);

                /* Evaluating if the FIFO is empty before extracting the accel frames */
                if (rslt == BMI2_W_FIFO_EMPTY)
                {
                    printf("\nFIFO accel frames extracted : %d \n", accel_frame_length);

                    printf("\nFIFO gyro frames requested : %d \n", gyro_frame_length);

                    /* Parse the FIFO data to extract gyro data from the FIFO buffer. */
                    rslt = bmi2_extract_gyro(fifo_gyro_data, &gyro_frame_length, &fifoframe, &dev);

                    /* Evaluating if the FIFO is empty before extracting the gyro frames */
                    if (rslt == BMI2_W_FIFO_EMPTY)
                    {
                        printf("\nFIFO gyro frames extracted : %d \n", gyro_frame_length);

                        printf("\nExracted accel frames\n");

                        /* Print the parsed accelerometer data from the FIFO buffer. */
                        for (index = 0; index < accel_frame_length; index++)
                        {
                            printf("ACCEL[%d] X : %d\t Y : %d\t Z : %d\n",
                                   index,
                                   fifo_accel_data[index].x,
                                   fifo_accel_data[index].y,
                                   fifo_accel_data[index].z);
                        }

                        printf("\nExtracted gyro frames\n");

                        /* Print the parsed gyro data from the FIFO buffer. */
                        for (index = 0; index < gyro_frame_length; index++)
                        {
                            printf("GYRO[%d] X : %d\t Y : %d\t Z : %d\n",
                                   index,
                                   fifo_gyro_data[index].x,
                                   fifo_gyro_data[index].y,
                                   fifo_gyro_data[index].z);
                        }
                    }
                }
            }

            break;
        }
    }

    return rslt;
}

/*!
 * @brief This internal API is used to set configurations for accel.
 */
static int8_t set_accel_gyro_config(struct bmi2_dev *dev)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define accel and gyro configurations. */
    struct bmi2_sens_config config[2];

    /* Configure the type of feature. */
    config[0].type = BMI2_ACCEL;
    config[1].type = BMI2_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi2_get_sensor_config(config, 2, dev);
    print_rslt(rslt);

    if (rslt == BMI2_OK)
    {
        /* NOTE: The user can change the following configuration parameter according to their requirement. */
        /* Accel configuration settings. */
        /* Output Data Rate. By default ODR is set as 100Hz for accel. */
        config[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;

        /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
        config[0].cfg.acc.range = BMI2_ACC_RANGE_2G;

        /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
         * if it is set to 2, then 2^(bandwidth parameter) samples
         * are averaged, resulting in 4 averaged samples
         * Note1 : For more information, refer the datasheet.
         * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
         * this has an adverse effect on the power consumed.
         */
        config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;

        /* Enable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)/ Performance optimized mode(BMI2_PERF_OPT_MODE)
         * For more info refer datasheet.
         */
        config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

        /* Gyro configuration settings. */
        /* Output Data Rate. Default ODR is 200Hz, setting to 100Hz. */
        config[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;

        /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps. */
        config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;

        /* Gyroscope Bandwidth parameters. By default the gyro bandwidth is in normal mode. */
        config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;

        /* Enable/Disable the noise performance mode for precision yaw rate sensing
         * There are two modes
         *  0 -> Ultra low power mode(Default)
         *  1 -> High performance mode/ Power optimized mode(BMI2_POWER_OPT_MODE)
         */
        config[1].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;

        /* Enable/Disable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)/ Performance optimized mode(BMI2_PERF_OPT_MODE)
         */
        config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

        /* Set new configurations. */
        rslt = bmi2_set_sensor_config(config, 2, dev);
        print_rslt(rslt);
    }

    return rslt;
}
