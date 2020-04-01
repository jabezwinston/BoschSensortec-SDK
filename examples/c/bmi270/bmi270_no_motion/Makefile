#CCE_Board_Definitions:BMI270

COINES_INSTALL_PATH ?= ../../../..

EXAMPLE_FILE ?= bmi270_no_motion.c

C_SRCS += \
$(COINES_INSTALL_PATH)/examples/c/bmi270/_support/bmi2_error_codes.c \
$(COINES_INSTALL_PATH)/examples/c/bmi270/_support/bmi2xy_hal_interface.c

INCLUDEPATHS += $(COINES_INSTALL_PATH)/examples/c/bmi270/_support

SHUTTLE_BOARD ?= BMI270

include $(COINES_INSTALL_PATH)/examples/examples.mk
