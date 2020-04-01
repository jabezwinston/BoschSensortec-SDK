#CCE_Board_Definitions:BMA423

COINES_INSTALL_PATH ?= ../../../..

EXAMPLE_FILE = bma423_calibration_pc.c

TARGET = PC

SHUTTLE_BOARD = BMA423

ifneq ($(TARGET),$(filter $(TARGET),PC))
    $(info $(EXAMPLE_FILE) is meant to run only on PC Target !)
    $(info Supported 'TARGET's : PC)
    $(error Exit)
endif

include $(COINES_INSTALL_PATH)/examples/examples.mk
