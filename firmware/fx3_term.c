#include "log.h"
#include "fx3_term.h"
#include "fx3_terminals.h"

#include "rdwr.h"
#include "vidpid.h"

extern rdwr_cmd_t gRdwrCmd;

extern const uint8_t CyFxUSB30DeviceDscr[];


fx3_gpif_config_t fx3_gpif_config = {
    .gpif_clk_div=8,
    .gpif_clk_halfdiv=CyFalse,
    .gpif_clk_src= CY_U3P_SYS_CLK,
    .gpif_drive_strength = CY_U3P_DS_QUARTER_STRENGTH,
    .enable = CyTrue,
};

uint16_t fx3_read(CyU3PDmaBuffer_t* pBuf) {
    uint16_t ret;
    switch (gRdwrCmd.header.reg_addr) {
       case FX3_VERSION:
         ret = FIRMWARE_VERSION;
         break;
       case FX3_USBVER:
         ret= CyFxUSB30DeviceDscr[12]|CyFxUSB30DeviceDscr[13]<<8;
         break;
       case FX3_USB3:
        ret=gRdwrCmd.ep_buffer_size == 1024 ? 1 : 0;
        break;
       case FX3_GPIF_CLK_DIV:
        ret=fx3_gpif_config.gpif_clk_div; 
        break;
       case FX3_GPIF_CLK_HALFDIV:
        ret=fx3_gpif_config.gpif_clk_halfdiv;
        break;
       case FX3_GPIF_CLK_SRC:
        ret=fx3_gpif_config.gpif_clk_src;
        break;
       case FX3_GPIF_DRIVE_STRENGTH:
        ret=fx3_gpif_config.gpif_drive_strength;
        break;
       case FX3_GPIF_ENABLE:
        ret=fx3_gpif_config.enable;
        break;
       default:
           return 1;
    }
    CyU3PMemCopy ( pBuf->buffer, (uint8_t*)&ret, 2 );
    return 0;

}

extern void slfifo_init();

uint16_t fx3_write(CyU3PDmaBuffer_t* pBuf) {
    uint16_t val = pBuf->buffer[0] | (pBuf->buffer[1]<<8);
    switch ( gRdwrCmd.header.reg_addr) {
        case FX3_GPIF_CLK_DIV:
            if (val<2 ||val>1024) return 1; 
            fx3_gpif_config.gpif_clk_div = val; 
            break;
        case FX3_GPIF_CLK_HALFDIV:
            fx3_gpif_config.gpif_clk_halfdiv = val>0?CyTrue:CyFalse;
            break;
        case FX3_GPIF_CLK_SRC:
            if (val>3) return 1;
            fx3_gpif_config.gpif_clk_src=val;
            break;
        case FX3_GPIF_CLK_TRIGGER:
            CyU3PPibDeInit();
            slfifo_init();
            break;
        case FX3_GPIF_DRIVE_STRENGTH:
            fx3_gpif_config.gpif_drive_strength=val;
            CyU3PSetPportDriveStrength(val);
            break;
        case FX3_GPIF_ENABLE:
            fx3_gpif_config.enable=val;
            break;
        default:
            return 1;
    }
    
    return 0;
}
