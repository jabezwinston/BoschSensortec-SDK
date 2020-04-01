#CCE_Board_Definitions:BMI085;BMI088

COINES_INSTALL_PATH ?= ../../../..

EXAMPLE_FILE ?= bmi08x_read_synchronized_data_mcu.c

SHUTTLE_BOARD ?= BMI085

TARGET ?= MCU_APP20

ifneq ($(TARGET),$(filter $(TARGET),MCU_APP20))
    $(info $(EXAMPLE_FILE) is meant to run on MCU_APP20 Target !)
    $(info Supported 'TARGET's : MCU_APP20)
    $(error Exit)
endif

include $(COINES_INSTALL_PATH)/examples/examples.mk
