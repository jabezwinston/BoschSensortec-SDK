#CCE_Board_Definitions:BHA260;BHI260

COINES_INSTALL_PATH ?= ../../../..

EXAMPLE_FILE ?= bhy2_bme_env.c

TARGET = PC

SHUTTLE_BOARD ?= BHI260

CFLAGS += -DBST_APPBOARD_VIA_USB -D$(SHUTTLE_BOARD) 

ifneq ($(TARGET),$(filter $(TARGET),PC))
    $(info $(EXAMPLE_FILE) is meant to run only on PC Target !)
    $(info Supported 'TARGET's : PC)
    $(error Exit)
endif

include $(COINES_INSTALL_PATH)/examples/examples.mk
