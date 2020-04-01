#CCE_Board_Definitions:BMA456

COINES_INSTALL_PATH ?= ../../../..

EXAMPLE_FILE = bma456_comm_perf_test_pc.c

TARGET = PC

SHUTTLE_BOARD = BMA456

ifneq ($(TARGET),$(filter $(TARGET),PC))
    $(info $(EXAMPLE_FILE) is meant to run only on PC Target !)
    $(info Supported 'TARGET's : PC)
    $(error Exit)
endif

include $(COINES_INSTALL_PATH)/examples/examples.mk
