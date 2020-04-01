#CCE_Board_Definitions:BHA260;BHI260

COINES_INSTALL_PATH ?= ../../../..

EXAMPLE_FILE ?= bhy2cli.c

TARGET = PC

SHUTTLE_BOARD ?= BHI260

C_SRCS += hal.c 

CFLAGS += -DBST_APPBOARD_VIA_USB -D$(SHUTTLE_BOARD) 

INCLUDEPATHS += $(COINES_INSTALL_PATH)/coinesAPI/pc/
INCLUDEPATHS += $(COINES_INSTALL_PATH)/coinesAPI/pc/comm_intf

ifneq ($(TARGET),$(filter $(TARGET),PC))
    $(info $(EXAMPLE_FILE) is meant to run only on PC Target !)
    $(info Supported 'TARGET's : PC)
    $(error Exit)
endif

include $(COINES_INSTALL_PATH)/examples/examples.mk
