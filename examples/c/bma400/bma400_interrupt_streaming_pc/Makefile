#CCE_Board_Definitions:BMA400

COINES_INSTALL_PATH ?= ../../../..

EXAMPLE_FILE ?= bma400_interrupt_streaming_pc.c

TARGET = PC

SHUTTLE_BOARD = BMA400
ifneq ($(TARGET),$(filter $(TARGET),PC))
    $(info $(EXAMPLE_FILE) is meant to run only on PC Target !)
    $(info Supported 'TARGET's : PC)
    $(error Exit)
endif

include $(COINES_INSTALL_PATH)/examples/examples.mk
