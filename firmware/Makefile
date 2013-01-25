MODULE = main

# the required assembly files
SOURCE_ASM = cyfx_gcc_startup.S
SOURCE  = cyfxtx.c
SOURCE += dscr.c 
SOURCE += vc_handler.c
SOURCE += main.c 

C_OBJECT=$(SOURCE:%.c=./%.o)
A_OBJECT=$(SOURCE_ASM:%.S=./%.o)
EXES = $(MODULE).$(EXEEXT)

# the common include path
Include	=-Iu3p_firmware/inc -I.

# the common compiler options
CCFLAGS	= -g -O0		\
		-DTX_ENABLE_EVENT_TRACE -DCYU3P_FX3=1	\
		-D__CYU3P_TX__=1 $(Include)

# the common linker options
LDFLAGS	= --entry CyU3PFirmwareEntry $(LDLIBS)


# the common libraries
# NOTE: This order is important for GNU linker. Do not change
LDLIBS = u3p_firmware/lib/fx3_release/cyfxapi.a \
	 u3p_firmware/lib/fx3_release/cyu3lpp.a \
	 u3p_firmware/lib/fx3_release/cyu3threadx.a \

#	 lib/cyfx3_boot.a

CC	= arm-none-eabi-gcc
AS	= arm-none-eabi-gcc
LD	= arm-none-eabi-ld
AR	= arm-none-eabi-ar

# Arguments
ASMFLAGS = -Wall -c -mcpu=arm926ej-s -mthumb-interwork -O0 -lgcc	

CCFLAGS	 += -Wall -mcpu=arm926ej-s -mthumb-interwork -O0 -lgcc

LDFLAGS	 += -T fx3.ld  -d --no-wchar-size-warning -Map $(MODULE).map

#The ARM toolchain location and the version are taken from environment variables
LDLIBS  += \
	"$$ARMGCC_INSTALL_PATH"/arm-none-eabi/lib/libc.a	\
	"$$ARMGCC_INSTALL_PATH"/lib/gcc/arm-none-eabi/$(ARMGCC_VERSION)/libgcc.a

EXEEXT		= elf

# Command Shortcuts
COMPILE		= $(CC) $(CCFLAGS) -c -o $@ $< 
ASSEMBLE	= $(AS) $(ASMFLAGS) -o $@ $<
LINK		= $(LD) $+ $(LDFLAGS) -o $@
BDLIB		= $(AR) -r $@ $+
ELF2IMG         = ../tools/elf2img/elf2img
GEN_IMAGE       = $(ELF2IMG)  -i $< -o $@ -imgtype 0xB0

all:compile

ELF2IMG:
	make -C ../tools/elf2img

$(MODULE).img: $(MODULE).$(EXEEXT)
	$(GEN_IMAGE)

$(MODULE).$(EXEEXT): $(A_OBJECT) $(C_OBJECT)
	$(LINK)

$(C_OBJECT) : %.o : %.c main.h
	$(COMPILE)

$(A_OBJECT) : %.o : %.S
	$(ASSEMBLE)



clean:
	rm -f ./$(MODULE).img
	rm -f ./$(MODULE).$(EXEEXT)
	rm -f ./$(MODULE).map
	rm -f ./*.o

compile: $(C_OBJECT) $(A_OBJECT) $(EXES) $(MODULE).img
