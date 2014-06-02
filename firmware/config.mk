

MODULE = main

VID=0x1fe1
PID=0x00f0
USB3_POWER=0x32 # 400ma (8mA units)
USB2_POWER=0xC8 # 400ma (2mA units)
FIRMWARE_VERSION=1

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

# add any custom debugging or cflags
#CCFLAGS := -Dxxx 

# customize build directory
#BUILDDIR = build
