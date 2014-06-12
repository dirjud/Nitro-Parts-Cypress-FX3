

MODULE = main

VID=0x1fe1
PID=0x00f0
USB3_POWER=0x32 # 400ma (8mA units)
USB2_POWER=0xC8 # 400ma (2mA units)
FIRMWARE_VERSION=1 # firmware should set this to whatever is appropriate

# the required assembly files
SOURCE += $(FX3DIR)dscr.c 
SOURCE += $(FX3DIR)error_handler.c
SOURCE += $(FX3DIR)cpu_handler.c
SOURCE += $(FX3DIR)slfifo_handler.c
SOURCE += $(FX3DIR)rdwr.c
SOURCE += $(FX3DIR)handlers.c
SOURCE += $(FX3DIR)../../../Microchip/M24XX/fx3/m24xx.c
SOURCE += $(FX3DIR)main.c 
SOURCE += $(FX3DIR)fx3_term.c
# only needed if you want firmware_di
#SOURCE += $(FX3DIR)di.c

# add any custom debugging or cflags
#CCFLAGS += -Dxxx 

# enable if you want to have di get/set functionality inside the fx3
# requires adding a source file with di_main
# CCFLAGS += -DFIRMWARE_DI

# customize build directory
#BUILDDIR = build
