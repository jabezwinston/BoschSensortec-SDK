#CCE_Board_Definitions:BME680
COINES_INSTALL_PATH ?= ../../../..

EXAMPLE_FILE ?= bsec_iot_ulp_plus_example.c

SHUTTLE_BOARD ?= BME680

TARGET ?= PC

BSEC_DIR = $(COINES_INSTALL_PATH)/examples/c/bme680/_BSEC

C_SRCS += \
$(BSEC_DIR)/bsec_integration.c \
$(BSEC_DIR)/config/generic_33v_3s_4d/bsec_serialized_configurations_iaq.c

INCLUDEPATHS += \
$(BSEC_DIR) \
$(BSEC_DIR)/normal/inc \
$(BSEC_DIR)/config/generic_33v_3s_4d

include $(BSEC_DIR)/bsec.mk

include $(COINES_INSTALL_PATH)/examples/examples.mk
