#CCE_Board_Definitions:BME280

COINES_INSTALL_PATH ?= ../../../..

EXAMPLE_FILE ?= bme280_stream_sensor_data_pc.c

SHUTTLE_BOARD ?= BME280

TARGET = PC

ifneq ($(TARGET),$(filter $(TARGET),PC))
    $(info $(EXAMPLE_FILE) is meant to run only on PC Target !)
    $(info Supported 'TARGET's : PC)
    $(error Exit)
endif

include $(COINES_INSTALL_PATH)/examples/examples.mk
