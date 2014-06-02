# set fx3dir to the directory that this makefile is included in
FX3DIR:=$(dir $(lastword $(MAKEFILE_LIST)))

DEPS = fx3_terminals.h


BUILDDIR ?= build
GENDIR ?= gen
FX3FWROOT = $(CYFX3SDK)/firmware
FX3PFWROOT = $(CYFX3SDK)/firmware/u3p_firmware
CYCONFOPT=fx3_release
CYDEVICE=CYUSB3011

include $(CYFX3SDK)/firmware/common/fx3_build_config.mak
include config.mk

Include += -I$(GENDIR) -I../../../Microchip/M24XX/fx3 -I../../../Xilinx/Spartan/fx3 -I$(FX3DIR)

all:fx3_terminals.h compile

$(GENDIR)/vidpid.h: config.mk
	@mkdir -p $(GENDIR)
	@echo "#define VID $(VID)" > $@
	@echo "#define PID $(PID)" >> $@
	@echo "#define USB3_POWER $(USB3_POWER)" >> $@
	@echo "#define USB2_POWER $(USB2_POWER)" >> $@
	@echo "#define FIRMWARE_VERSION $(FIRMWARE_VERSION)" >> $@

fx3_terminals.h: ../terminals.py
	di --header $@ $<

SOURCE_ABS := $(abspath $(SOURCE))
C_OBJECT := $(addprefix $(BUILDDIR), $(SOURCE_ABS:%.c=%.o))
SOURCE_ASM_ABS := $(abspath $(SOURCE_ASM))
A_OBJECT:= $(addprefix $(BUILDDIR), $(SOURCE_ASM_ABS:%.S=%.o))


EXES = $(MODULE).$(EXEEXT)

$(MODULE).$(EXEEXT): $(A_OBJECT) $(C_OBJECT)
	$(LINK)

$(C_OBJECT) : $(BUILDDIR)/%.o : /%.c
	@mkdir -p $(dir $@)
	$(COMPILE)

$(A_OBJECT) : $(BUILDDIR)/%.o : /%.S
	@mkdir -p $(dir $@)
	$(ASSEMBLE)

ELF2IMG         = $(CYFX3SDK)/util/elf2img/elf2img
GEN_IMAGE       = $(ELF2IMG)  -i $< -o $@ -imgtype 0xB0 -i2cconf 0x1E

$(MODULE).img: $(MODULE).$(EXEEXT)
	$(GEN_IMAGE)

compile: $(GENDIR)/vidpid.h $(C_OBJECT) $(A_OBJECT) $(EXES) $(MODULE).img

clean:
	rm -f ./$(MODULE).img
	rm -f $(MODULE).$(EXEEXT)
	rm -f $(MODULE).map
	rm -f $(A_OBJECT) $(C_OBJECT)
	rm -f $(GENDIR)/vidpid.h

