/*!
* @section LICENSE
* $license_gpl$
*
* @filename $filename$
* @date     $date$
* @id       $id$
*
* @brief The files for BHy host interface definitions
*/

/*!
 * @defgroup hostinterface BHy host interface definition
 * @{*/

#ifndef BHY_HOST_INTERFACE_DEFS_H
#define BHY_HOST_INTERFACE_DEFS_H

/*! Data type definitions */
#ifdef __KERNEL__
#include <linux/types.h>
#define bhy_u8 u8
#define bhy_s8 s8
#define bhy_u16 u16
#define bhy_s16 s16
#define bhy_u32 u32
#define bhy_s32 s32
#define bhy_u64 u64
#define bhy_s64 s64
#define bhy_float u32
#else
#include <stdint.h>
#define bhy_u8 uint8_t
#define bhy_s8 int8_t
#define bhy_u16 uint16_t
#define bhy_s16 int16_t
#define bhy_u32 uint32_t
#define bhy_s32 int32_t
#define bhy_u64 uint64_t
#define bhy_s64 int64_t
#define bhy_float float
#endif /*~ __KERNEL__ */

#define PRODUCT_ID 0x89

/*! Host interface error codes */
#define BHY_HIF_E_SUCCESS        0
#define BHY_HIF_E_HANDLE         -1
#define BHY_HIF_E_INVAL          -2
#define BHY_HIF_E_IO             -3
#define BHY_HIF_E_MAGIC          -4
#define BHY_HIF_E_CRC            -5
#define BHY_HIF_E_TIMEOUT        -6
#define BHY_HIF_E_BUF            -7
#define BHY_HIF_E_INITIALIZED    -8
#define BHY_HIF_E_BUS            -9
#define BHY_HIF_E_NOMEM          -10

#define BHY2_DEFAULT_MAX_READ_BURST 65536

/*! Register map */
#define BHY_REG_CHAN_0                0x0
#define BHY_REG_CHAN_CMD              BHY_REG_CHAN_0
#define BHY_REG_CHAN_1                0x1
#define BHY_REG_CHAN_FIFO_W           BHY_REG_CHAN_1
#define BHY_REG_CHAN_2                0x2
#define BHY_REG_CHAN_FIFO_NW          BHY_REG_CHAN_2
#define BHY_REG_CHAN_3                0x3
#define BHY_REG_CHAN_STATUS           BHY_REG_CHAN_3
#define BHY_REG_CHIP_CTRL             0x05
#define BHY_REG_HOST_INTERFACE_CTRL   0x06
#define BHY_REG_HOST_INTERRUPT_CTRL   0x07
#define BHY_REG_RESET_REQ             0x14
#define BHY_REG_EV_TIME_REQ           0x15
#define BHY_REG_HOST_CTRL             0x16
#define BHY_REG_HOST_STATUS           0x17
#define BHY_REG_CRC_0                 0x18 /* Totally 4 */
#define BHY_REG_PRODUCT_ID            0x1C
#define BHY_REG_REVISION_ID           0x1D
#define BHY_REG_ROM_VERSION_0         0x1E /* Totally 2 */
#define BHY_REG_RAM_VERSION_0         0x20 /* Totally 2 */
#define BHY_REG_FLASH_VERSION_0       0x22 /* Totally 2 */
#define BHY_REG_FEATURE_STATUS        0x24
#define BHY_REG_BOOT_STATUS           0x25
#define BHY_REG_HOST_IRQ_TIMESTAMP_0  0x26 /* Totally 5 */
#define BHY_REG_INT_STATUS            0x2D
#define BHY_REG_ERROR                 0x2E
#define BHY_REG_INT_STATE             0x2F
#define BHY_REG_DEBUG_VALUE           0x30
#define BHY_REG_DEBUG_STATE           0x31

/*! Command packets */
#define BHY_CMD_REQ_POST_MORTEM_DATA   0x1
#define BHY_CMD_UPLOAD_TO_PROGRAM_RAM  0x2
#define BHY_CMD_BOOT_PROGRAM_RAM       0x3
#define BHY_CMD_ERASE_FLASH            0x4
#define BHY_CMD_WRITE_FLASH            0x5
#define BHY_CMD_BOOT_FLASH             0x6
#define BHY_CMD_SET_INJECT_MODE        0x7
#define BHY_CMD_INJECT_DATA            0x8
#define BHY_CMD_FIFO_FLUSH             0x9
#define BHY_CMD_SW_PASSTHROUGH         0xA
#define BHY_CMD_REQ_SELF_TEST          0xB
#define BHY_CMD_REQ_FOC                0xC
#define BHY_CMD_CONFIG_SENSOR          0xD
#define BHY_CMD_CHANGE_RANGE           0xE
#define BHY_CMD_TRIG_FATAL_ERROR       0x13
#define BHY_CMD_BSX_LOGGING_CTRL       0x14
#define BHY_CMD_FIFO_FORMAT_CTRL       0x15
/*! Command packet length,
should be mutiple of 4bytes,
at least 4 bytes */
#define BHY_COMMAND_PACKET_LEN    44

/*! Firmware header related fields */
#define BHY_FW_MAGIC              0x662B
#define BHY_KEY_LOCATION_OTP      0x0
#define BHY_KEY_LOCATION_ROM      0x1
#define BHY_KEY_LOCATION_CHAIN    0x2
#define BHY_KEY_TYPE_ECDSA        0x0
#define BHY_KEY_NO_AUTHENTICATION 0xF
#define BHY_KEY_MAX_LOCATION      0xF

/*! Host control */
#define BHY_HOCTL_SPI_3_WIRE    (1 << 0)
#define BHY_HOCTL_I2C_WDOG_EN   (1 << 1)
#define BHY_HOCTL_50MS_TIMEOUT  (1 << 2)

/*! Boot status */
#define BHY_BST_FLASH_DETECTED       (1 << 0)
#define BHY_BST_FLASH_VERIFY_DONE    (1 << 1)
#define BHY_BST_FLASH_VERIFY_ERROR   (1 << 2)
#define BHY_BST_NO_FLASH             (1 << 3)
#define BHY_BST_HOST_INTERFACE_READY (1 << 4)
#define BHY_BST_HOST_FW_VERIFY_DONE  (1 << 5)
#define BHY_BST_HOST_FW_VERIFY_ERROR (1 << 6)
#define BHY_BST_HOST_FW_IDLE         (1 << 7)
#define BHY_BST_CHECK_RETRY          100

/*! IRQ status masks */
#define BHY_IST_MASK_ASSERTED           0x1
#define BHY_IST_MASK_FIFO_W             0x6
#define BHY_IST_MASK_FIFO_NW            0x18
#define BHY_IST_MASK_STATUS             0x20
#define BHY_IST_MASK_DEBUG              0x40
#define BHY_IST_MASK_RESET_FAULT        0x80
#define BHY_IST_FIFO_W_DRDY             0x2
#define BHY_IST_FIFO_W_LTCY             0x4
#define BHY_IST_FIFO_W_WM               0x6
#define BHY_IST_FIFO_NW_DRDY            0x8
#define BHY_IST_FIFO_NW_LTCY            0x10
#define BHY_IST_FIFO_NW_WM              0x18
#define BHY_IS_IRQ_FIFO_W(status)       ((status) & BHY_IST_MASK_FIFO_W)
#define BHY_IS_IRQ_FIFO_NW(status)      ((status) & BHY_IST_MASK_FIFO_NW)
#define BHY_IS_IRQ_STATUS(status)       ((status) & BHY_IST_MASK_STATUS)
#define BHY_IS_IRQ_ASYNC_STATUS(status) ((status) & BHY_IST_MASK_DEBUG)
#define BHY_IS_IRQ_RESET(status)        ((status) & BHY_IST_MASK_RESET_FAULT)
#define BHY_IST_MASK_FIFO        (BHY_IST_MASK_FIFO_W | BHY_IST_MASK_FIFO_NW)
#define BHY_IS_IRQ_FIFO(status)         ((status) & BHY_IST_MASK_FIFO)

/*! Host interface control bits */
#define BHY_IFCTL_ABORT_TRANSFER_CHANNEL_0  (1 << 0)
#define BHY_IFCTL_ABORT_TRANSFER_CHANNEL_1  (1 << 1)
#define BHY_IFCTL_ABORT_TRANSFER_CHANNEL_2  (1 << 2)
#define BHY_IFCTL_ABORT_TRANSFER_CHANNEL_3  (1 << 3)
#define BHY_IFCTL_AP_SUSPENDED              (1 << 4)
#define BHY_IFCTL_NED_COORDINATES           (1 << 5)
#define BHY_IFCTL_IRQ_TIMESTAMP_CONTROL     (1 << 6)
#define BHY_IFCTL_ASYNC_STATUS_CHANNEL      (1 << 7)

/*! IRQ control bits */
#define BHY_ICTL_DISABLE_FIFO_W             (1 << 0)
#define BHY_ICTL_DISABLE_FIFO_NW            (1 << 1)
#define BHY_ICTL_DISABLE_STATUS             (1 << 2)
#define BHY_ICTL_DISABLE_DEBUG              (1 << 3)
#define BHY_ICTL_DISABLE_FAULT              (1 << 4)
#define BHY_ICTL_ACTIVE_LOW                 (1 << 5)
#define BHY_ICTL_EDGE                       (1 << 6)
#define BHY_ICTL_OPEN_DRAIN                 (1 << 7)

/*! BSX logging control modes */
#define BHY_BSX_LOG_DO_STEPS_MAIN_THREAD    (1 << 0)
#define BHY_BSX_LOG_UPDATE_SUBSCRIPTION     (1 << 1)
#define BHY_BSX_LOG_DO_STEPS_CALIB_THREAD   (1 << 2)
#define BHY_BSX_LOG_GET_OUTPUT_SIGNAL_MIN   (1 << 3)
#define BHY_BSX_LOG_GET_OUTPUT_SIGNAL_ALL   (1 << 4)

/*! FIFO format control bits */
#define BHY_FIFO_FORMAT_CTRL_ENABLE_TIMPSTAMP       (0)
#define BHY_FIFO_FORMAT_CTRL_DISABLE_TIMPSTAMP      (1 << 0)
#define BHY_FIFO_FORMAT_CTRL_DISABLE_FULL_TIMPSTAMP (1 << 1)

/* sensor information bits */
#define BHY_SENSOR_INFO_BUS                         (1 << 4)

/*! System parameters */
#define BHY_PARAM_READ_MASK               0x1000
#define BHY_PARAM_FIFO_CTRL               0x103
#define BHY_PARAM_SYS_VIRT_SENSOR_PRESENT 0x11F
#define BHY_PARAM_SYS_PHYS_SENSOR_PRESENT 0x120
#define BHY_PARAM_SYS_PHYS_SENSOR_INFO_0  0x120

/*! Physical sensor information parameters */
#define BHY_PARAM_PHYSICAL_SENSOR_BASE    0x120

/*! Algorithm parameters */
#define BHY_PARAM_CALIB_STATE_BASE        0x200
#define BHY_PARAM_SIC                     0x27D
#define BHY_PARAM_BSX_VERSION             0x27E
#define BHY_PARAM_SET_SENSOR_CTRL         0x0E00
#define BHY_PARAM_GET_SENSOR_CTRL         0x1E00
#define BHY_PARAM_SENSOR_CTRL_FOC         0x1
#define BHY_PARAM_SENSOR_CTRL_OIS         0x2
#define BHY_PARAM_SENSOR_CTRL_FST         0x3
#define BHY_PARAM_SENSOR_CTRL_READ        0x80

/*! Sensor parameters */
#define BHY_PARAM_SENSOR_INFO_0           0x300
#define BHY_PARAM_SENSOR_CONF_0           0x500

#define MAX_BSX_STATE_LEN                 1024
#define BSX_STATE_TRANSFER_COMPLETE       0x80 /* 1 << 7 */
#define BSX_STATE_BLOCK_LEN               64
#define BSX_STATE_STRUCT_LEN              68

#define BHY_QUERY_PARAM_STATUS_READY_MAX_RETRY 1000
#define BHY_QUERY_FLASH_MAX_RETRY              1000

/*! Meta event definitions */
#define BHY_META_EVENT_FLUSH_COMPLETE           1
#define BHY_META_EVENT_SAMPLE_RATE_CHANGED      2
#define BHY_META_EVENT_POWER_MODE_CHANGED       3
#define BHY_META_EVENT_ALGORITHM_EVENTS         5
#define BHY_META_EVENT_SENSOR_STATUS            6
#define BHY_META_EVENT_BSX_DO_STEPS_MAIN        7
#define BHY_META_EVENT_BSX_DO_STEPS_CALIB       8
#define BHY_META_EVENT_BSX_GET_OUTPUT_SIGNAL    9
#define BHY_META_EVENT_RESERVED1                10
#define BHY_META_EVENT_SENSOR_ERROR             11
#define BHY_META_EVENT_FIFO_OVERFLOW            12
#define BHY_META_EVENT_DYNAMIC_RANGE_CHANGED    13
#define BHY_META_EVENT_FIFO_WATERMARK           14
#define BHY_META_EVENT_RESERVED2                15
#define BHY_META_EVENT_INITIALIZED              16
#define BHY_META_TRANSFER_CAUSE                 17
#define BHY_META_EVENT_SENSOR_FRAMEWORK         18
#define BHY_META_EVENT_RESET                    19
#define BHY_META_EVENT_SPACER                   20

/* Hub reset */
#define BHY_POST_MORTEM_MAX_LEN         9000
#define BHY_META_INITIALIZED_MAX_RETRY  500
#define BHY_REG_MAP_MAX_LEN             256

/* Inject data */
#define BHY_INJECT_DATA_MAX_LEN         2048

/* software passthrough */
#define BHY_SW_PASSTHRU_READ            0
#define BHY_SW_PASSTHRU_WRITE           1
#define BHY_SW_PASSTHRU_SINGLE_BURST    0
#define BHY_SW_PASSTHRU_NO_DELAY        0
#define BHY_SW_PASSTHRU_SPI_MODE        0

/* Hub flash */
#define BHY_FLASH_FW_LEN_UNIT           (65528)
#define BHY_FLASH_SECTOR_UNIT           (4 * 1024)
#define BHY_FLASH_SECTOR_ADDR_LEN        4

/* Hub error state */
#define BHY_HUB_ERR_SHA_MISMATCH         0x12
#define BHY_HUB_ERR_HW_FAIL              0x18
#define BHY_HUB_ERR_UNEXPECTED_WD_RESET  0x19
#define BHY_HUB_ERR_ROM_MISCMATCH        0x1A
#define BHY_HUB_ERR_FW_FATAL_ERR         0x1B
#define BHY_HUB_ERR_SENSOR_ERR           0x24
#define BHY_HUB_ERR_DATA_OVERFLOW        0x26
#define BHY_HUB_ERR_STACK_OVERFLOW       0x27
#define BHY_HUB_ERR_SENSOR_INIT_FAIL     0x29
#define BHY_HUB_ERR_FOC_FAIL             0x65
#define BHY_HUB_ERR_SENSOR_BUSY          0x66
#define BHY_HUB_ERR_NO_HOST_IRQ          0x72
#define BHY_HUB_ERR_CMD_ERR              0xC0
#define BHY_HUB_ERR_CMD_BUF_OVERFLOW     0xC2
#define BHY_HUB_ERR_WD_TIMEOUT           0xF0

/*! Sensor ID definitions */
#define BHY_SYS_SENSOR_ID_MAX            256
#define BHY_SENSOR_ID_MAX                200

/* dummy register address in bmi160 */
#define BMI160_REG_DUMMY_REG             0x7F
#define BMI160_REG_CHIP_ID               0x00
#define BMI160_REG_ACC_CONF              0x40
#define BMI160_REG_ACC_RANGE             0x41

/* BSX sensor IDs from bsx_virtual_sensor_identifier.h */
#define  BSX_VIRTUAL_SENSOR_ID_INVALID            (0)
#define BSX_OUTPUT_ID_ACCELERATION_PASSTHROUGH    (2)
#define BSX_CUSTOM_ID_ACCELERATION_PASSTHROUGH    (4)
#define BSX_OUTPUT_ID_ACCELERATION_RAW            (6)
#define BSX_OUTPUT_ID_ACCELERATION_CORRECTED      (8)
#define BSX_OUTPUT_ID_ACCELERATION_OFFSET         (10)
#define BSX_WAKEUP_ID_ACCELERATION_OFFSET         (182)
#define BSX_WAKEUP_ID_ACCELERATION_CORRECTED      (12)
#define BSX_WAKEUP_ID_ACCELERATION_RAW            (14)
#define BSX_CUSTOM_ID_ACCELERATION_CORRECTED      (16)
#define BSX_CUSTOM_ID_ACCELERATION_RAW            (18)
#define BSX_OUTPUT_ID_ANGULARRATE_PASSTHROUGH     (20)
#define BSX_CUSTOM_ID_ANGULARRATE_PASSTHROUGH     (22)
#define BSX_OUTPUT_ID_ANGULARRATE_RAW             (24)
#define BSX_OUTPUT_ID_ANGULARRATE_CORRECTED       (26)
#define BSX_OUTPUT_ID_ANGULARRATE_OFFSET          (28)
#define BSX_WAKEUP_ID_ANGULARRATE_OFFSET          (184)
#define BSX_WAKEUP_ID_ANGULARRATE_CORRECTED       (30)
#define BSX_WAKEUP_ID_ANGULARRATE_RAW             (32)
#define BSX_CUSTOM_ID_ANGULARRATE_CORRECTED       (34)
#define BSX_CUSTOM_ID_ANGULARRATE_RAW             (36)
#define BSX_OUTPUT_ID_MAGNETICFIELD_PASSTHROUGH   (38)
#define BSX_CUSTOM_ID_MAGNETICFIELD_PASSTHROUGH   (40)
#define BSX_OUTPUT_ID_MAGNETICFIELD_RAW           (42)
#define BSX_OUTPUT_ID_MAGNETICFIELD_CORRECTED     (44)
#define BSX_OUTPUT_ID_MAGNETICFIELD_OFFSET        (46)
#define BSX_WAKEUP_ID_MAGNETICFIELD_OFFSET        (186)
#define BSX_WAKEUP_ID_MAGNETICFIELD_CORRECTED     (48)
#define BSX_WAKEUP_ID_MAGNETICFIELD_RAW           (50)
#define BSX_CUSTOM_ID_MAGNETICFIELD_CORRECTED     (52)
#define BSX_CUSTOM_ID_MAGNETICFIELD_RAW           (54)
#define BSX_OUTPUT_ID_GRAVITY                     (56)
#define BSX_WAKEUP_ID_GRAVITY                     (58)
#define BSX_CUSTOM_ID_GRAVITY                     (60)
#define BSX_OUTPUT_ID_LINEARACCELERATION          (62)
#define BSX_WAKEUP_ID_LINEARACCELERATION          (64)
#define BSX_CUSTOM_ID_LINEARACCELERATION          (66)
/** @brief NDOF or e-Compass */
#define BSX_OUTPUT_ID_ROTATION                    (68)
/** @brief NDOF or e-Compass */
#define BSX_WAKEUP_ID_ROTATION                    (70)
#define BSX_CUSTOM_ID_ROTATION                    (72)
/** @brief no magnetic field */
#define BSX_OUTPUT_ID_ROTATION_GAME               (74)
/** @brief no magnetic field */
#define BSX_WAKEUP_ID_ROTATION_GAME               (76)
#define BSX_CUSTOM_ID_ROTATION_GAME               (78)
/** @brief no gyroscope, only e-Compass */
#define BSX_OUTPUT_ID_ROTATION_GEOMAGNETIC        (80)
/** @brief no gyroscope, only e-Compass */
#define BSX_WAKEUP_ID_ROTATION_GEOMAGNETIC        (82)
#define BSX_CUSTOM_ID_ROTATION_GEOMAGNETIC        (84)
/** @brief deprecated: NDOF or e-Compass computed from OUTPUT_ROTATION */
#define BSX_OUTPUT_ID_ORIENTATION                 (86)
/** @brief deprecated: NDOF or e-Compass computed from WAKEUP_ROTATION */
#define BSX_WAKEUP_ID_ORIENTATION                 (88)
#define BSX_CUSTOM_ID_ORIENTATION                 (90)
#define BSX_OUTPUT_ID_FLIP_STATUS                 (92)
#define BSX_CUSTOM_ID_FLIP_STATUS                 (94)
#define BSX_OUTPUT_ID_TILT_STATUS                 (96)
#define BSX_CUSTOM_ID_TILT_STATUS                 (98)
#define BSX_OUTPUT_ID_STEPDETECTOR                (100)
#define BSX_WAKEUP_ID_STEPDETECTOR                (188)
#define BSX_CUSTOM_ID_STEPDETECTOR                (102)
#define BSX_OUTPUT_ID_STEPCOUNTER                 (104)
#define BSX_WAKEUP_ID_STEPCOUNTER                 (106)
#define BSX_CUSTOM_ID_STEPCOUNTER                 (108)
#define BSX_OUTPUT_ID_SIGNIFICANTMOTION_STATUS    (110)
#define BSX_CUSTOM_ID_SIGNIFICANTMOTION_STATUS    (112)
#define BSX_OUTPUT_ID_WAKE_STATUS                 (114)
#define BSX_CUSTOM_ID_WAKE_STATUS                 (116)
#define BSX_OUTPUT_ID_GLANCE_STATUS               (118)
#define BSX_CUSTOM_ID_GLANCE_STATUS               (120)
#define BSX_OUTPUT_ID_PICKUP_STATUS               (122)
#define BSX_CUSTOM_ID_PICKUP_STATUS               (124)
#define BSX_OUTPUT_ID_ACTIVITY                    (126)
#define BSX_CUSTOM_ID_ACTIVITY                    (128)
#define BSX_OUTPUT_ID_PROPAGATION                 (130)
#define BSX_OUTPUT_ID_POSITION_STEPS              (132)
#define BSX_OUTPUT_ID_WRIST_TILT_STATUS           (134)
#define BSX_CUSTOM_ID_WRIST_TILT_STATUS           (136)
#define BSX_OUTPUT_ID_DEVICE_ORIENTATION          (138)
#define BSX_WAKEUP_ID_DEVICE_ORIENTATION          (140)
#define BSX_CUSTOM_ID_DEVICE_ORIENTATION          (142)
#define BSX_OUTPUT_ID_POSE_6DOF                   (144)
#define BSX_WAKEUP_ID_POSE_6DOF                   (146)
#define BSX_CUSTOM_ID_POSE_6DOF                   (148)
#define BSX_OUTPUT_ID_STATIONARY_DETECT           (150)
#define BSX_CUSTOM_ID_STATIONARY_DETECT           (152)
#define BSX_OUTPUT_ID_MOTION_DETECT               (154)
#define BSX_CUSTOM_ID_MOTION_DETECT               (156)
#define BSX_OUTPUT_ID_STANDBY_STATUS              (158)
#define BSX_OUTPUT_ID_ACCELERATION_STATUS         (160)
#define BSX_OUTPUT_ID_ACCELERATION_DYNAMIC        (162)
#define BSX_OUTPUT_ID_ANGULARRATE_STATUS          (164)
#define BSX_OUTPUT_ID_MAGNETICFIELD_STATUS        (166)
/** @brief dedicated debug output for the angular rate from M4G */
#define BSX_OUTPUT_ID_ANGULARRATE_M4G             (168)
/** @brief dedicated debug output for the angular rate from M4G */
#define BSX_WAKEUP_ID_ANGULARRATE_M4G             (170)

#define BHYID(bsx_id)                             ((bsx_id) >> 1)
#define BSX_SENSOR_ID_MAX                          0x7F

/* Non-bsx sensor IDs from SensorAPI.h inside firmware */
#define NONBSX_SENSOR_ID_MAX                     0x9F
#define NONBSX_SENSOR_ID_TEMPERATURE             0x80
#define NONBSX_SENSOR_ID_PRESSURE                0x81
#define NONBSX_SENSOR_ID_HUMIDITY                0x82
#define NONBSX_SENSOR_ID_GAS                     0x83
#define NONBSX_SENSOR_ID_WAKE_TEMPERATURE        0x84
#define NONBSX_SENSOR_ID_WAKE_PRESSURE           0x85
#define NONBSX_SENSOR_ID_WAKE_HUMIDITY           0x86
#define NONBSX_SENSOR_ID_WAKE_GAS                0x87
#define NONBSX_SENSOR_ID_STEP_COUNTER            0x88
#define NONBSX_SENSOR_ID_STEP_DETECTOR           0x89
#define NONBSX_SENSOR_ID_SIGNIFICANT_MOTION      0x8A
#define NONBSX_SENSOR_ID_WAKE_STEP_COUNTER       0x8B
#define NONBSX_SENSOR_ID_WAKE_STEP_DETECTOR      0x8C
#define NONBSX_SENSOR_ID_WAKE_SIGNIFICANT_MOTION 0x8D
#define NONBSX_SENSOR_ID_ANY_MOTION              0x8E
#define NONBSX_SENSOR_ID_WAKE_ANY_MOTION         0x8F
#define NONBSX_SENSOR_ID_EXCAMERA                0x90
#define NONBSX_SENSOR_ID_GPS                     0x91
#define NONBSX_SENSOR_ID_LIGHT                   0x92
#define NONBSX_SENSOR_ID_WAKE_LIGHT              0x94
#define NONBSX_SENSOR_ID_PROXIMITY               0x93
#define NONBSX_SENSOR_ID_WAKE_PROXIMITY          0x95

/* This is from SensorAPI.h (SDK code) */
#define SENSOR_TYPE_CUSTOMER_VISIBLE_START  (0xA0)
#define SENSOR_TYPE_CUSTOMER_VISIBLE_END    (0xBF)

#define BHY_SENSOR_ID_TBD           (BHY_SENSOR_ID_MAX - 1)
#define BHY_SENSOR_ID_ACC           BHYID(BSX_OUTPUT_ID_ACCELERATION_CORRECTED)
#define BHY_SENSOR_ID_ACC_WU        BHYID(BSX_WAKEUP_ID_ACCELERATION_CORRECTED)
#define BHY_SENSOR_ID_ACC_PASS    BHYID(BSX_OUTPUT_ID_ACCELERATION_PASSTHROUGH)
#define BHY_SENSOR_ID_MAG           BHYID(BSX_OUTPUT_ID_MAGNETICFIELD_CORRECTED)
#define BHY_SENSOR_ID_MAG_WU        BHYID(BSX_WAKEUP_ID_MAGNETICFIELD_CORRECTED)
#define BHY_SENSOR_ID_MAG_PASS    BHYID(BSX_OUTPUT_ID_MAGNETICFIELD_PASSTHROUGH)
#define BHY_SENSOR_ID_ORI           BHYID(BSX_OUTPUT_ID_ORIENTATION)
#define BHY_SENSOR_ID_ORI_WU        BHYID(BSX_WAKEUP_ID_ORIENTATION)
#define BHY_SENSOR_ID_GYRO          BHYID(BSX_OUTPUT_ID_ANGULARRATE_CORRECTED)
#define BHY_SENSOR_ID_GYRO_WU       BHYID(BSX_WAKEUP_ID_ANGULARRATE_CORRECTED)
#define BHY_SENSOR_ID_GYRO_PASS     BHYID(BSX_OUTPUT_ID_ANGULARRATE_PASSTHROUGH)
#define BHY_SENSOR_ID_LIGHT         NONBSX_SENSOR_ID_LIGHT
#define BHY_SENSOR_ID_LIGHT_WU      NONBSX_SENSOR_ID_WAKE_LIGHT
#define BHY_SENSOR_ID_BARO          NONBSX_SENSOR_ID_PRESSURE
#define BHY_SENSOR_ID_BARO_WU       NONBSX_SENSOR_ID_WAKE_PRESSURE
#define BHY_SENSOR_ID_TEMP          NONBSX_SENSOR_ID_TEMPERATURE
#define BHY_SENSOR_ID_TEMP_WU       NONBSX_SENSOR_ID_WAKE_TEMPERATURE
#define BHY_SENSOR_ID_PROX          NONBSX_SENSOR_ID_PROXIMITY
#define BHY_SENSOR_ID_PROX_WU       NONBSX_SENSOR_ID_WAKE_PROXIMITY
#define BHY_SENSOR_ID_GRA           BHYID(BSX_OUTPUT_ID_GRAVITY)
#define BHY_SENSOR_ID_GRA_WU        BHYID(BSX_WAKEUP_ID_GRAVITY)
#define BHY_SENSOR_ID_LACC          BHYID(BSX_OUTPUT_ID_LINEARACCELERATION)
#define BHY_SENSOR_ID_LACC_WU       BHYID(BSX_WAKEUP_ID_LINEARACCELERATION)
#define BHY_SENSOR_ID_RV            BHYID(BSX_OUTPUT_ID_ROTATION)
#define BHY_SENSOR_ID_RV_WU         BHYID(BSX_WAKEUP_ID_ROTATION)
#define BHY_SENSOR_ID_RV_CUS        BHYID(BSX_CUSTOM_ID_ROTATION)
#define BHY_SENSOR_ID_HUM           NONBSX_SENSOR_ID_HUMIDITY
#define BHY_SENSOR_ID_HUM_WU        NONBSX_SENSOR_ID_WAKE_HUMIDITY
#define BHY_SENSOR_ID_ATEMP         BHY_SENSOR_ID_TBD
#define BHY_SENSOR_ID_ATEMP_WU      BHY_SENSOR_ID_TBD
#define BHY_SENSOR_ID_MAGU          BHYID(BSX_OUTPUT_ID_MAGNETICFIELD_RAW)
#define BHY_SENSOR_ID_MAGU_WU       BHYID(BSX_WAKEUP_ID_MAGNETICFIELD_RAW)
#define BHY_SENSOR_ID_GAMERV        BHYID(BSX_OUTPUT_ID_ROTATION_GAME)
#define BHY_SENSOR_ID_GAMERV_WU     BHYID(BSX_WAKEUP_ID_ROTATION_GAME)
#define BHY_SENSOR_ID_GYROU         BHYID(BSX_OUTPUT_ID_ANGULARRATE_RAW)
#define BHY_SENSOR_ID_GYROU_WU      BHYID(BSX_WAKEUP_ID_ANGULARRATE_RAW)
#define BHY_SENSOR_ID_SIG    BHYID(BSX_OUTPUT_ID_SIGNIFICANTMOTION_STATUS)
/* only has wakeup type */
#define BHY_SENSOR_ID_SIG_WU        BHY_SENSOR_ID_SIG
#define BHY_SENSOR_ID_SIG_HW        BHY_SENSOR_ID_TBD
/* only has wakeup type */
#define BHY_SENSOR_ID_SIG_HW_WU     NONBSX_SENSOR_ID_WAKE_SIGNIFICANT_MOTION
#define BHY_SENSOR_ID_STD           BHYID(BSX_OUTPUT_ID_STEPDETECTOR)
#define BHY_SENSOR_ID_STD_WU        BHYID(BSX_WAKEUP_ID_STEPDETECTOR)
#define BHY_SENSOR_ID_STD_HW        NONBSX_SENSOR_ID_STEP_DETECTOR
#define BHY_SENSOR_ID_STD_HW_WU     NONBSX_SENSOR_ID_WAKE_STEP_DETECTOR
#define BHY_SENSOR_ID_STC           BHYID(BSX_OUTPUT_ID_STEPCOUNTER)
#define BHY_SENSOR_ID_STC_WU        BHYID(BSX_WAKEUP_ID_STEPCOUNTER)
#define BHY_SENSOR_ID_STC_HW        NONBSX_SENSOR_ID_STEP_COUNTER
#define BHY_SENSOR_ID_STC_HW_WU     NONBSX_SENSOR_ID_WAKE_STEP_COUNTER
#define BHY_SENSOR_ID_GEORV         BHYID(BSX_OUTPUT_ID_ROTATION_GEOMAGNETIC)
#define BHY_SENSOR_ID_GEORV_WU      BHYID(BSX_WAKEUP_ID_ROTATION_GEOMAGNETIC)
#define BHY_SENSOR_ID_HEART         BHY_SENSOR_ID_TBD
#define BHY_SENSOR_ID_HEART_WU      BHY_SENSOR_ID_TBD
#define BHY_SENSOR_ID_TILT          BHYID(BSX_OUTPUT_ID_TILT_STATUS)
/* only has wakeup type */
#define BHY_SENSOR_ID_TILT_WU       BHY_SENSOR_ID_TILT
#define BHY_SENSOR_ID_WRIST_TILT_GUESTURE BHYID(BSX_OUTPUT_ID_WRIST_TILT_STATUS)
#define BHY_SENSOR_ID_WAKE          BHYID(BSX_OUTPUT_ID_WAKE_STATUS)
/* only has wakeup type */
#define BHY_SENSOR_ID_WAKE_WU       BHY_SENSOR_ID_WAKE
#define BHY_SENSOR_ID_GLANCE        BHYID(BSX_OUTPUT_ID_GLANCE_STATUS)
/* only has wakeup type */
#define BHY_SENSOR_ID_GLANCE_WU     BHY_SENSOR_ID_GLANCE
#define BHY_SENSOR_ID_PICKUP        BHYID(BSX_OUTPUT_ID_PICKUP_STATUS)
/* only has wakeup type */
#define BHY_SENSOR_ID_PICKUP_WU     BHY_SENSOR_ID_PICKUP
#define BHY_SENSOR_ID_DEVICE_ORI    BHYID(BSX_OUTPUT_ID_DEVICE_ORIENTATION)
#define BHY_SENSOR_ID_DEVICE_ORI_WU BHYID(BSX_WAKEUP_ID_DEVICE_ORIENTATION)
#define BHY_SENSOR_ID_STATIONARY_DET BHYID(BSX_OUTPUT_ID_STATIONARY_DETECT)
#define BHY_SENSOR_ID_MOTION_DET    BHYID(BSX_OUTPUT_ID_MOTION_DETECT)
#define BHY_SENSOR_ID_AR            BHYID(BSX_OUTPUT_ID_ACTIVITY)
/* only has wakeup type */
#define BHY_SENSOR_ID_AR_WU         BHY_SENSOR_ID_AR
#define BHY_SENSOR_ID_ACC_RAW       BHYID(BSX_OUTPUT_ID_ACCELERATION_RAW)
#define BHY_SENSOR_ID_ACC_RAW_WU    BHYID(BSX_WAKEUP_ID_ACCELERATION_RAW)
#define BHY_SENSOR_ID_MAG_RAW       BHYID(BSX_OUTPUT_ID_MAGNETICFIELD_RAW)
#define BHY_SENSOR_ID_MAG_RAW_WU    BHYID(BSX_WAKEUP_ID_MAGNETICFIELD_RAW)
#define BHY_SENSOR_ID_GYRO_RAW      BHYID(BSX_OUTPUT_ID_ANGULARRATE_RAW)
#define BHY_SENSOR_ID_GYRO_RAW_WU   BHYID(BSX_WAKEUP_ID_ANGULARRATE_RAW)
#define BHY_SENSOR_ID_ACC_BIAS      BHYID(BSX_OUTPUT_ID_ACCELERATION_OFFSET)
#define BHY_SENSOR_ID_MAG_BIAS      BHYID(BSX_OUTPUT_ID_MAGNETICFIELD_OFFSET)
#define BHY_SENSOR_ID_GYRO_BIAS     BHYID(BSX_OUTPUT_ID_ANGULARRATE_OFFSET)
#define BHY_SENSOR_ID_ANY_MOTION    NONBSX_SENSOR_ID_ANY_MOTION
#define BHY_SENSOR_ID_ANY_MOTION_WU NONBSX_SENSOR_ID_WAKE_ANY_MOTION
#define BHY_SENSOR_ID_GAS           NONBSX_SENSOR_ID_GAS
#define BHY_SENSOR_ID_GAS_WU        NONBSX_SENSOR_ID_WAKE_GAS
#define BHY_SENSOR_ID_GPS           NONBSX_SENSOR_ID_GPS
#define BHY_SENSOR_ID_EXCAMERA      NONBSX_SENSOR_ID_EXCAMERA

/* Customer sensor ID */
#define BHY_SENSOR_ID_CUSTOMER_BEGIN SENSOR_TYPE_CUSTOMER_VISIBLE_START
#define BHY_SENSOR_ID_CUSTOMER_END SENSOR_TYPE_CUSTOMER_VISIBLE_END
#define BHY_SENSOR_ID_CUSTOMER_0 BHY_SENSOR_ID_CUSTOMER_BEGIN
#define BHY_SENSOR_ID_CUSTOMER_1 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 1)
#define BHY_SENSOR_ID_CUSTOMER_2 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 2)
#define BHY_SENSOR_ID_CUSTOMER_3 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 3)
#define BHY_SENSOR_ID_CUSTOMER_4 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 4)
#define BHY_SENSOR_ID_CUSTOMER_5 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 5)
#define BHY_SENSOR_ID_CUSTOMER_6 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 6)
#define BHY_SENSOR_ID_CUSTOMER_7 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 7)
#define BHY_SENSOR_ID_CUSTOMER_8 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 8)
#define BHY_SENSOR_ID_CUSTOMER_9 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 9)
#define BHY_SENSOR_ID_CUSTOMER_10 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 10)
#define BHY_SENSOR_ID_CUSTOMER_11 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 11)
#define BHY_SENSOR_ID_CUSTOMER_12 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 12)
#define BHY_SENSOR_ID_CUSTOMER_13 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 13)
#define BHY_SENSOR_ID_CUSTOMER_14 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 14)
#define BHY_SENSOR_ID_CUSTOMER_15 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 15)
#define BHY_SENSOR_ID_CUSTOMER_16 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 16)
#define BHY_SENSOR_ID_CUSTOMER_17 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 17)
#define BHY_SENSOR_ID_CUSTOMER_18 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 18)
#define BHY_SENSOR_ID_CUSTOMER_19 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 19)
#define BHY_SENSOR_ID_CUSTOMER_20 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 20)
#define BHY_SENSOR_ID_CUSTOMER_21 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 21)
#define BHY_SENSOR_ID_CUSTOMER_22 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 22)
#define BHY_SENSOR_ID_CUSTOMER_23 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 23)
#define BHY_SENSOR_ID_CUSTOMER_24 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 24)
#define BHY_SENSOR_ID_CUSTOMER_25 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 25)
#define BHY_SENSOR_ID_CUSTOMER_26 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 26)
#define BHY_SENSOR_ID_CUSTOMER_27 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 27)
#define BHY_SENSOR_ID_CUSTOMER_28 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 28)
#define BHY_SENSOR_ID_CUSTOMER_29 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 29)
#define BHY_SENSOR_ID_CUSTOMER_30 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 30)
#define BHY_SENSOR_ID_CUSTOMER_31 (BHY_SENSOR_ID_CUSTOMER_BEGIN + 31)

/*! System data IDs */
#define BHY_SYS_SENSOR_ID_TS_SMALL_DELTA      251
#define BHY_SYS_SENSOR_ID_TS_LARGE_DELTA      252
#define BHY_SYS_SENSOR_ID_TS_FULL             253
#define BHY_SYS_SENSOR_ID_META_EVENT          254
#define BHY_SYS_SENSOR_ID_TS_SMALL_DELTA_WU   245
#define BHY_SYS_SENSOR_ID_TS_LARGE_DELTA_WU   246
#define BHY_SYS_SENSOR_ID_TS_FULL_WU          247
#define BHY_SYS_SENSOR_ID_META_EVENT_WU       248
#define BHY_SYS_SENSOR_ID_FILLER              255
#define BHY_SYS_SENSOR_ID_DEBUG_MSG           250
#define BHY_SYS_SENSOR_ID_BSX_LOG_UPDATE_SUB  243
#define BHY_SYS_SENSOR_ID_BSX_LOG_DOSTEP      244
#define BHY_IS_SYS_SENSOR_ID(SENSOR_ID)       ((SENSOR_ID) >= 0xE0)

/*! Status code definitions */
#define BHY_STATUS_INITIALIZED              0x1
#define BHY_STATUS_DEBUG_OUTPUT             0x2
#define BHY_STATUS_CRASH_DUMP               0x3
#define BHY_STATUS_INJECT_SENSOR_CONF_REQ   0x4
#define BHY_STATUS_SW_PASS_THRU_RES         0x5
#define BHY_STATUS_SELF_TEST_RES            0x6
#define BHY_STATUS_FOC_RES                  0x7
#define BHY_STATUS_SYSTEM_ERROR             0x8
#define BHY_STATUS_SENSOR_ERROR             0x9
#define BHY_STATUS_FLASH_ERASE_COMPLETE     0xA
#define BHY_STATUS_FLASH_WRITE_COMPLETE     0xB
#define BHY_STATUS_FLASH_CONTINUE_UPLOAD    0xC
#define BHY_STATUS_HOST_EV_TIMESTAMP        0xD
#define BHY_STATUS_DUT_TEST_RES             0xE
#define BHY_STATUS_CMD_ERR                  0xF

#define BHY_IS_STATUS_GET_PARAM_OUTPUT(status)\
	((status) >= 0x100 && (status) <= 0xFFF)

/*! Activity bits */
#define BHY_STILL_ACTIVITY_ENDED                (1<<0)
#define BHY_WALKING_ACTIVITY_ENDED              (1<<1)
#define BHY_RUNNING_ACTIVITY_ENDED              (1<<2)
#define BHY_ON_BICYCLE_ACTIVITY_ENDED           (1<<3)
#define BHY_IN_VEHICLE_ACTIVITY_ENDED           (1<<4)
#define BHY_TILTING_ACTIVITY_ENDED              (1<<5)

#define BHY_STILL_ACTIVITY_STARTED              (1<<8)
#define BHY_WALKING_ACTIVITY_STARTED            (1<<9)
#define BHY_RUNNING_ACTIVITY_STARTED            (1<<10)
#define BHY_ON_BICYCLE_ACTIVITY_STARTED         (1<<11)
#define BHY_IN_VEHICLE_ACTIVITY_STARTED         (1<<12)
#define BHY_TILTING_ACTIVITY_STARTED            (1<<13)

#define BHY_LE2U16(x) ((bhy_u16)((x)[0] | (x)[1] << 8))
#define BHY_LE2S16(x) ((bhy_s16)BHY_LE2U16(x))
#define BHY_LE2U24(x) ((bhy_u32)((x)[0] | (x)[1] << 8 | (x)[2] << 16))
#define BHY_LE2U32(x) \
((bhy_u32)((x)[0] | (x)[1] << 8 | (x)[2] << 16 | (x)[3] << 24))
#define BHY_LE2S32(x) ((bhy_s32)BHY_LE2U32(x))
#define BHY_LE2U40(x) (BHY_LE2U32(x) | (bhy_u64)(x)[4] << 32)
#define BHY_LE2U64(x) (BHY_LE2U32(x) | (bhy_u64)BHY_LE2U32(&(x)[4]) << 32)

#define BHY_DRIVER_ID_BMA455                 0x2D
#define BHY_DRIVER_ID_BMI160_ACC             0x30
#define BHY_DRIVER_ID_BMI160_ACC_NEW_DRIVER  0x20
#define BHY_DRIVER_ID_AK09915                0x60
#define BHY_DRIVER_ID_BMM150                 0x0B
#define BHY_DRIVER_ID_BMI160_GYRO            0x31
#define BHY_DRIVER_ID_BMI160_GYRO_NEW_DRIVER 0x21
#define BHY_DRIVER_ID_BMG250                 0x32
#define BHY_DRIVER_ID_BMI160_ACC_FIFO_MODE   0x18
#define BHY_DRIVER_ID_BMI160_GYRO_FIFO_MODE  0x19
#define BHY_DRIVER_ID_AK09915_ON_BMI160      0x17
#define BHY_DRIVER_ID_BMM150_ON_BMI160       0x16

/*! Physical sensor ID definitions */

/* BSX input ID defs from bsx_physical_sensor_identifier.h */
#define BSX_PHYSICAL_SENSOR_ID_INVALID      0
#define BSX_INPUT_ID_ACCELERATION           1
#define BSX_INPUT_ID_ANGULARRATE            3
#define BSX_INPUT_ID_MAGNETICFIELD          5
#define BSX_INPUT_ID_TEMPERATURE_GYROSCOPE  7
#define BSX_INPUT_ID_ANYMOTION              9
#define BSX_INPUT_ID_PRESSURE               11
#define BSX_INPUT_ID_POSITION               13
#define BSX_INPUT_ID_HUMIDITY               15
#define BSX_INPUT_ID_TEMPERATURE            17
#define BSX_INPUT_ID_GASRESISTOR            19

#define BHY_PHYS_SENSOR_ID_MAX     64
#define BHY_PHYS_SENSOR_ID_ACC     BSX_INPUT_ID_ACCELERATION
#define BHY_PHYS_SENSOR_ID_MAG     BSX_INPUT_ID_MAGNETICFIELD
#define BHY_PHYS_SENSOR_ID_GYRO    BSX_INPUT_ID_ANGULARRATE

/*! Debug message flag definition */
#define BHY_DEBUG_MSG_FORMAT_BINARY  0x40
#define BHY_DEBUG_MSG_LENGTH_MASK    0x3F

#ifdef __GNUC__
union bhy_u16_conv {
	bhy_u16 u16_val;
	bhy_u8 bytes[2];
};

union bhy_u32_conv {
	bhy_u32 u32_val;
	bhy_u8 bytes[4];
};

union bhy_float_conv {
	bhy_float f_val;
	bhy_u32 u32_val;
	bhy_u8 bytes[4];
};

union bhy_sensor_info {
	struct {
		bhy_u8 sensor_type;
		bhy_u8 driver_id;
		bhy_u8 driver_version;
		bhy_u8 power;
		union bhy_u16_conv max_range;
		union bhy_u16_conv resolution;
		union bhy_float_conv max_rate;
		union bhy_u32_conv fifo_reserved;
		union bhy_u32_conv fifo_max;
		bhy_u8 event_size;
		union bhy_float_conv min_rate;
		bhy_u8 reserved[3];
	} __attribute__((__packed__)) info;
	bhy_u8 bytes[28];
};

union bhy_phys_sensor_info {
	struct {
		bhy_u8 sensor_type;
		bhy_u8 driver_id;
		bhy_u8 driver_version;
		bhy_u8 power_current;
		union bhy_u16_conv curr_range;
		bhy_u8 flags;
		bhy_u8 slave_address;
		bhy_u8 gpio_assignment;
		union bhy_float_conv curr_rate;
		bhy_u8 num_axis;
		bhy_u8 orientation_matrix[5];
		bhy_u8 reserved[1];
	} __attribute__((__packed__)) info;
	bhy_u8 bytes[20];
};

#else

__packed union bhy_u16_conv {
	bhy_u16 u16_val;
	bhy_u8 bytes[2];
};

__packed union bhy_u32_conv {
	bhy_u32 u32_val;
	bhy_u8 bytes[4];
};

__packed union bhy_float_conv {
	bhy_float f_val;
	bhy_u32 u32_val;
	bhy_u8 bytes[4];
};

union bhy_sensor_info {
	__packed struct {
		bhy_u8 sensor_type;
		bhy_u8 driver_id;
		bhy_u8 driver_version;
		bhy_u8 power;
		union bhy_u16_conv max_range;
		union bhy_u16_conv resolution;
		union bhy_float_conv max_rate;
		union bhy_u32_conv fifo_reserved;
		union bhy_u32_conv fifo_max;
		bhy_u8 event_size;
		union bhy_float_conv min_rate;
		bhy_u8 reserved[3];
	} info;
	bhy_u8 bytes[28];
};

union bhy_phys_sensor_info {
	__packed struct {
		bhy_u8 sensor_type;
		bhy_u8 driver_id;
		bhy_u8 driver_version;
		bhy_u8 power_current;
		union bhy_u16_conv curr_range;
		bhy_u8 flags;
		bhy_u8 slave_address;
		bhy_u8 gpio_assignment;
		union bhy_float_conv curr_rate;
		bhy_u8 num_axis;
		bhy_u8 orientation_matrix[5];
		bhy_u8 reserved[1];
	} info;
	bhy_u8 bytes[20];
};

#endif
#endif /*~ BHY_HOST_INTERFACE_DEFS_H */

/** @}*/
