#include "log.h"
#include "fx3_term.h"
#include "fx3_terminals.h"

#include "rdwr.h"
#include "vidpid.h"

extern rdwr_cmd_t gRdwrCmd;

extern const uint8_t CyFxUSB30DeviceDscr[];


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
       default:
           return 1;
    }
    CyU3PMemCopy ( pBuf->buffer, (uint8_t*)&ret, 2 );
    return 0;

}


uint16_t fx3_write(CyU3PDmaBuffer_t* pBuf) {
    return 1;
}
