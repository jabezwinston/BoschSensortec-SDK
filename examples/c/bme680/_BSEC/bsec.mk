ifeq ($(TARGET),PC)
    GCC_ARCH = $(shell gcc -dumpmachine)

    ifeq ($(OS),Windows_NT)
        ifneq (,$(findstring x86_64,$(GCC_ARCH)))
            EXTRA_LIBS += $(BSEC_DIR)/normal/bin/windows_x64/BSECLibrary64.dll
            POSTBUILD_CMD = $(CP) $(BSEC_DIR)/normal/bin/windows_x64/BSECLibrary64.dll .
        else
            EXTRA_LIBS += $(BSEC_DIR)/normal/bin/windows_x86/BSECLibrary32.dll
            POSTBUILD_CMD = $(CP) $(BSEC_DIR)/normal/bin/windows_x86/BSECLibrary32.dll .
        endif

    else ifneq (,$(findstring arm-linux-gnueabihf,$(GCC_ARCH)))
            LIBPATHS += $(BSEC_DIR)/normal/bin/linux_armhf
            LIBS += algobsec
    else
            $(error BSEC : Not supported in this PLATFORM (or) TARGET !!)
    endif

endif

ifeq ($(TARGET),MCU_APP20)
    LIBPATHS += $(BSEC_DIR)/normal/bin/app20_cortex-m4
    LIBS += algobsec
endif

ifeq ($(TARGET),MCU_APP30)
    LIBPATHS += $(BSEC_DIR)/normal/bin/app30_cortex-m4f
    LIBS += algobsec
endif