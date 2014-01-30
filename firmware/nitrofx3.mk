# set fx3dir to the directory that this makefile is included in
FX3DIR:=$(dir $(lastword $(MAKEFILE_LIST)))

DEPS = fx3_terminals.h

include config.mk

BUILDDIR ?= build
GENDIR ?= gen


SOURCE_ABS := $(abspath $(SOURCE) $(GENDIR)/cyfxtx.c )
C_OBJECT := $(addprefix $(BUILDDIR), $(SOURCE_ABS:%.c=%.o))
SOURCE_ASM_ABS := $(abspath $(SOURCE_ASM))
A_OBJECT:= $(addprefix $(BUILDDIR), $(SOURCE_ASM_ABS:%.S=%.o))
CYFXTX_C := $(abspath $(GENDIR)/cyfxtx.c)

EXES = $(MODULE).$(EXEEXT)


# the common include path
Include	+= -I$(CYFX3SDK)/firmware/u3p_firmware/inc -I. -I$(FX3DIR) -I$(GENDIR) -I../../../Microchip/M24XX/fx3 -I../../../Xilinx/Spartan/fx3

# the common compiler options
CCFLAGS += -DTX_ENABLE_EVENT_TRACE -DCYU3P_FX3=1 \
		   -D__CYU3P_TX__=1 $(Include)

# the common linker options
LDFLAGS	= --entry CyU3PFirmwareEntry $(LDLIBS)

# the common libraries
# NOTE: This order is important for GNU linker. Do not change
LDLIBS = $(CYFX3SDK)/firmware/u3p_firmware/lib/fx3_release/cyfxapi.a \
	 $(CYFX3SDK)/firmware/u3p_firmware/lib/fx3_release/cyu3lpp.a \
	 $(CYFX3SDK)/firmware/u3p_firmware/lib/fx3_release/cyu3threadx.a \

#	 lib/cyfx3_boot.a

CC	= arm-none-eabi-gcc
AS	= arm-none-eabi-gcc
LD	= arm-none-eabi-ld
AR	= arm-none-eabi-ar

# Arguments
ASMFLAGS = -Wall -c -mcpu=arm926ej-s -mthumb-interwork -Os -lgcc	

CCFLAGS	 += -Wall -mcpu=arm926ej-s -mthumb-interwork -Os -lgcc

LDFLAGS	 += -T $(GENDIR)/fx3.ld -d --no-wchar-size-warning -Map $(MODULE).map

#The ARM toolchain location and the version are taken from environment variables
LDLIBS  += \
	"$$ARMGCC_INSTALL_PATH"/arm-none-eabi/lib/libc.a	\
	"$$ARMGCC_INSTALL_PATH"/lib/gcc/arm-none-eabi/$(ARMGCC_VERSION)/libgcc.a

EXEEXT		= elf

# Command Shortcuts
COMPILE		= $(CC) $(CCFLAGS) -c -o $@ $< 
ASSEMBLE	= $(AS) $(ASMFLAGS) -o $@ $<
LINK		= $(LD) $(A_OBJECT) $(C_OBJECT) $(LDFLAGS) -o $@
BDLIB		= $(AR) -r $@ $+
ELF2IMG         = $(CYFX3SDK)/util/elf2img/elf2img
GEN_IMAGE       = $(ELF2IMG)  -i $< -o $@ -imgtype 0xB0 -i2cconf 0x1E

all:fx3_terminals.h compile

$(CYFXTX_C): $(CYFX3SDK)/firmware/common/cyfxtx.c config.mk
	mkdir -p $(GENDIR)
	cat $< | sed -e 's/0x40038000/0x4002E000/' \
			     -e 's/0x40078000/0x40040000/' > $@

$(GENDIR)/fx3.ld: $(CYFX3SDK)/firmware/common/fx3.ld config.mk
	mkdir -p $(GENDIR)
	cat $< | sed -e 's/0x2D000/0x23000/' \
			     -e 's/0x40030000/0x40026000/' > $@

ELF2IMG:
	# not this needs fixed
	make -C $(CYFX3SDK)/util/elf2img

$(MODULE).img: $(MODULE).$(EXEEXT)
	$(GEN_IMAGE)

$(MODULE).$(EXEEXT): $(A_OBJECT) $(C_OBJECT) $(GENDIR)/fx3.ld
	$(LINK)

$(C_OBJECT) : $(BUILDDIR)/%.o : /%.c
	@mkdir -p $(dir $@)
	$(COMPILE)

$(A_OBJECT) : $(BUILDDIR)/%.o : /%.S
	@mkdir -p $(dir $@)
	$(ASSEMBLE)

clean:
	rm -f ./$(MODULE).img
	rm -f $(MODULE).$(EXEEXT)
	rm -f $(MODULE).map
	rm -f $(A_OBJECT) $(C_OBJECT)
	rm -f $(GENDIR)/vidpid.h
	rm -f $(GENDIR)/cyfxtx.c
	rm -f $(GENDIR)/fx3.ld

compile: $(GENDIR)/vidpid.h $(C_OBJECT) $(A_OBJECT) $(EXES) $(MODULE).img

$(GENDIR)/vidpid.h: config.mk
	@mkdir -p $(GENDIR)
	@echo "#define VID $(VID)" > $@
	@echo "#define PID $(PID)" >> $@
	@echo "#define USB3_POWER $(USB3_POWER)" >> $@
	@echo "#define USB2_POWER $(USB2_POWER)" >> $@

fx3_terminals.h: ../terminals.py
	di --header $@ $<

