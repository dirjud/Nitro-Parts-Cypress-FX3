#include "fx3_term.h"
#include "fx3_terminals.h"

#include "rdwr.h"

extern rdwr_cmd_t gRdwrCmd;

#warning "This terminal isn't completely implemented"

uint16_t fx3_read(CyU3PDmaBuffer_t* pBuf) {

    switch (gRdwrCmd.header.reg_addr) {
       case FX3_USB3:
       {
        uint16_t isUsb3 = gRdwrCmd.ep_buffer_size == 1024 ? 1 : 0;
        CyU3PMemCopy ( pBuf->buffer, (uint8_t*)&isUsb3, 2 );
        return 0;
       }
    }

    return 1;
}


