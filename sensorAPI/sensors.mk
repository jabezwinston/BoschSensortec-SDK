#BME280
ifeq ($(SENSOR),BME280)
C_SRCS += $(COINES_INSTALL_PATH)/sensorAPI/bme280/bme280.c

INCLUDEPATHS += $(COINES_INSTALL_PATH)/sensorAPI/bme280
endif

###############################################################################

#BMI08x
ifneq (,$(findstring BMI08,$(SENSOR)))
C_SRCS += \
$(COINES_INSTALL_PATH)/sensorAPI/bmi08x/bmi08a.c \
$(COINES_INSTALL_PATH)/sensorAPI/bmi08x/bmi08g.c \

    ifeq ($(SENSOR),BMI085)
    C_SRCS += $(COINES_INSTALL_PATH)/sensorAPI/bmi08x/bmi085.c
    CFLAGS += -DBMI08X_ENABLE_BMI085=1 -DBMI08X_ENABLE_BMI088=0
    endif

    ifeq ($(SENSOR),BMI088)
    C_SRCS += $(COINES_INSTALL_PATH)/sensorAPI/bmi08x/bmi088.c
    CFLAGS += -DBMI08X_ENABLE_BMI085=0 -DBMI08X_ENABLE_BMI088=1
    endif

INCLUDEPATHS += $(COINES_INSTALL_PATH)/sensorAPI/bmi08x
endif

###############################################################################

#BHY
ifeq ($(SENSOR),$(filter $(SENSOR),BHI160 BHI160B BHA250))
C_SRCS += \
$(COINES_INSTALL_PATH)/sensorAPI/bhy/src/BHy_support.c \
$(COINES_INSTALL_PATH)/sensorAPI/bhy/src/bhy_uc_driver.c  \
$(COINES_INSTALL_PATH)/sensorAPI/bhy/src/bhy.c  \
$(COINES_INSTALL_PATH)/sensorAPI/bhy/AppBoard_usb_driver/AppBoard_usb_driver.c 

INCLUDEPATHS += \
$(COINES_INSTALL_PATH)/sensorAPI/bhy/include \
$(COINES_INSTALL_PATH)/sensorAPI/bhy/AppBoard_usb_driver
endif

###############################################################################

#BMA400
ifeq ($(SENSOR),BMA400)
C_SRCS += \
$(COINES_INSTALL_PATH)/sensorAPI/bma400/bma400.c

INCLUDEPATHS += \
$(COINES_INSTALL_PATH)/sensorAPI/bma400
endif

###############################################################################

#BMP3
ifeq ($(SENSOR),BMP3)
C_SRCS += \
$(COINES_INSTALL_PATH)/sensorAPI/bmp3/bmp3.c

INCLUDEPATHS += \
$(COINES_INSTALL_PATH)/sensorAPI/bmp3
endif

###############################################################################

#BMI160
ifeq ($(SENSOR),BMI160)
C_SRCS += \
$(COINES_INSTALL_PATH)/sensorAPI/bmi160/bmi160.c

INCLUDEPATHS += \
$(COINES_INSTALL_PATH)/sensorAPI/bmi160
endif

###############################################################################

#BMA423
ifeq ($(SENSOR),BMA423)
C_SRCS += \
$(COINES_INSTALL_PATH)/sensorAPI/bma423/bma4.c \
$(COINES_INSTALL_PATH)/sensorAPI/bma423/bma423.c

INCLUDEPATHS += \
$(COINES_INSTALL_PATH)/sensorAPI/bma423
endif

###############################################################################

#BMA456
ifeq ($(SENSOR),BMA456)
C_SRCS += \
$(COINES_INSTALL_PATH)/sensorAPI/bma456/bma4.c \
$(COINES_INSTALL_PATH)/sensorAPI/bma456/bma456.c

INCLUDEPATHS += \
$(COINES_INSTALL_PATH)/sensorAPI/bma456
endif

###############################################################################

#BME680
ifeq ($(SENSOR),BME680)
C_SRCS += \
$(COINES_INSTALL_PATH)/sensorAPI/bme680/bme680.c

INCLUDEPATHS += \
$(COINES_INSTALL_PATH)/sensorAPI/bme680

endif

###############################################################################

#BHY2
ifeq ($(SENSOR),$(filter $(SENSOR),BHA260 BHI260))
C_SRCS += \
$(COINES_INSTALL_PATH)/sensorAPI/bhy2/bhy2_api/bhy2_api.c \
$(COINES_INSTALL_PATH)/sensorAPI/bhy2/bhy2_host_interface/bhy_host_interface.c \
$(COINES_INSTALL_PATH)/sensorAPI/bhy2/bhy2_hal/bhy2_hal.c \
$(COINES_INSTALL_PATH)/sensorAPI/bhy2/bhy2_cus/bhy2_cus.c \


INCLUDEPATHS += \
$(COINES_INSTALL_PATH)/sensorAPI/bhy2/bhy2_api \
$(COINES_INSTALL_PATH)/sensorAPI/bhy2/bhy2_hal \
$(COINES_INSTALL_PATH)/sensorAPI/bhy2/bhy2_cus \
$(COINES_INSTALL_PATH)/sensorAPI/bhy2/bhy2_host_interface \
$(COINES_INSTALL_PATH)/sensorAPI/bhy2/bhy2_firmware \

endif


###############################################################################

#BMI270
ifeq ($(SENSOR),BMI270)
C_SRCS += \
$(COINES_INSTALL_PATH)/sensorAPI/bmi270/bmi2.c \
$(COINES_INSTALL_PATH)/sensorAPI/bmi270/bmi270.c \

INCLUDEPATHS += \
$(COINES_INSTALL_PATH)/sensorAPI/bmi270 \
$(COINES_INSTALL_PATH)/sensorAPI/bmi270/support

endif

###############################################################################
