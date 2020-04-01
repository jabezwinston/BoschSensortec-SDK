#CCE_Board_Definitions:BMI160

COINES_INSTALL_PATH ?= ../../../..

EXAMPLE_FILE ?= bmi160_polling_streaming_pc.c

SHUTTLE_BOARD ?= BMI160

TARGET=PC

ifneq ($(TARGET),$(filter $(TARGET),PC))
    $(info $(EXAMPLE_FILE) is meant to run only on PC Target !)
    $(info Supported 'TARGET's : PC)
    $(error Exit)
endif

include $(COINES_INSTALL_PATH)/examples/examples.mk
