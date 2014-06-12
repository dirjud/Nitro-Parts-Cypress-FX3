
#include "main.h" 
#include "vendor_commands.h"
#include "rdwr.h"
#include "di.h"

#include <cyu3error.h>
#include <cyu3os.h>

#include "log.h"

#ifndef FIRMWARE_DI
#error "Add -DFIRMWARE_DI to CCFLAGS to use firmware di features"
#endif

#ifndef DEBUG_DI
#undef log_debug
#define log_debug(...) do {} while(0)
#endif

io_handler_t firmware_di_handler = {
    HANDLER_TYPE_FDI,
    0, // term addr
    0, // boot
    0, // init 
    0, 0, 0, 0, 0, 0 };


CyU3PDmaChannel glChHandleFDI_CPUtoP;  /* DMA MANUAL_IN channel handle.  */
CyU3PDmaChannel glChHandleFDI_PtoCPU;  /* DMA MANUAL_OUT channel handle.  */
extern CyU3PDmaChannel glChHandleCPUtoP; // from slfifo for resetting
CyBool_t gFdiHandlerActive = CyFalse;

extern uint16_t slfifo_setup_cputop();
    
uint16_t fdi_setup() {
   CyU3PDmaChannelConfig_t dmaCfg;
   CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
 
   if (gFdiHandlerActive) return 0; 

   /* Create a DMA MANUAL channel for P2CPUtransfer. */
   dmaCfg.size           = 1024;
   dmaCfg.count          = 2;
   dmaCfg.dmaMode        = CY_U3P_DMA_MODE_BYTE;
   dmaCfg.prodSckId      = CY_FX_PRODUCER_PPORT_SOCKET; 
   dmaCfg.consSckId      = CY_U3P_CPU_SOCKET_CONS; 
   dmaCfg.notification   = 0;
   dmaCfg.cb             = 0;
   dmaCfg.prodHeader     = 0;
   dmaCfg.prodFooter     = 0;
   dmaCfg.consHeader     = 0;
   dmaCfg.prodAvailCount = 0;
 
   status |= CyU3PDmaChannelCreate (&glChHandleFDI_PtoCPU, CY_U3P_DMA_TYPE_MANUAL_IN, &dmaCfg);
   if (status != CY_U3P_SUCCESS) {
     log_error("CyU3PDmaChannelCreate3 failed, Error code = %d\n", status);
     return status;
   }

   /* Create a DMA MANUAL channel for CPU2P transfer. */
   dmaCfg.prodSckId      = CY_U3P_CPU_SOCKET_PROD;
   dmaCfg.consSckId      = CY_FX_CONSUMER_PPORT_SOCKET;
   dmaCfg.notification   = 0;
   dmaCfg.cb             = 0;
   status |= CyU3PDmaChannelCreate (&glChHandleFDI_CPUtoP, CY_U3P_DMA_TYPE_MANUAL_OUT, &dmaCfg);
   if (status != CY_U3P_SUCCESS) {
     log_error("CyU3PDmaChannelCreate3 failed, Error code = %d\n", status);
     return status;
   }

   status = slfifo_setup_cputop();
   if (status != CY_U3P_SUCCESS) return status;

   // NOTE do we really need to watch/log errors on all these?
   // I just haven't seen any of them every fail.
   CyU3PDmaChannelReset(&glChHandleFDI_CPUtoP);
   CyU3PDmaChannelReset(&glChHandleFDI_PtoCPU);
   CyU3PDmaChannelReset(&glChHandleCPUtoP);
   CyU3PDmaChannelSetXfer(&glChHandleFDI_CPUtoP, 0);
   CyU3PDmaChannelSetXfer(&glChHandleFDI_PtoCPU, 0);
   CyU3PDmaChannelSetXfer(&glChHandleCPUtoP, 0);

   gFdiHandlerActive = CyTrue;
   log_info ( "FDI setup: %d\n", status );
   return status;
}

void fdi_teardown() {
  CyU3PDmaChannelDestroy (&glChHandleFDI_CPUtoP);
  CyU3PDmaChannelDestroy (&glChHandleFDI_PtoCPU);
  CyU3PDmaChannelDestroy (&glChHandleCPUtoP);
  gFdiHandlerActive = CyFalse;
}


uint8_t *gDIbuf; // buf for reading/writing di results.

uint16_t fdi_handler_dmacb() {
   CyU3PDmaBuffer_t dmaBuf;
   uint16_t ret=0;
   uint32_t max_tx = gRdwrCmd.header.transfer_length - gRdwrCmd.transfered_so_far;

   if (gRdwrCmd.header.command == COMMAND_READ) {
        log_debug ( "R" );
        ret = CyU3PDmaChannelGetBuffer ( &glChHandleFDI_PtoCPU, &dmaBuf, 500); 
        if (ret) {
         log_warn ( "No read Buffer\n" );
         return ret;
        }
        
        if ( gRdwrCmd.transfered_so_far < gRdwrCmd.header.transfer_length ) {
            uint32_t tx = dmaBuf.count > max_tx ? max_tx : dmaBuf.count;
            CyU3PMemCopy ( gDIbuf+gRdwrCmd.transfered_so_far, dmaBuf.buffer, tx );
            gRdwrCmd.transfered_so_far += tx; // the difference would be for an odd count transfer (or a bug)
            log_debug ( "Rcv %d bytes\n", tx );
        } else {
            // the ack
            if ( dmaBuf.count == 8 ) {
                log_debug ( "Rcv ack\n" );
                gRdwrCmd.done = 1; 
                // NOTE copy ack status and checksum
            } else {
                log_error ( "Unexpected packet size for ack: %d", dmaBuf.count );
                ret=1;
            }
        }
        CyU3PDmaChannelDiscardBuffer( &glChHandleFDI_PtoCPU );
        return ret;
   } else {
      log_debug ( "W" );
      if ( gRdwrCmd.transfered_so_far < gRdwrCmd.header.transfer_length ) {
         uint16_t tx = max_tx > 1024 ? 1024 : max_tx;
         ret = CyU3PDmaChannelGetBuffer ( &glChHandleFDI_CPUtoP, &dmaBuf, 500 ); 
         if (ret) {
          log_warn ( "No write buffer\n" );
          return ret;
         }
         CyU3PMemCopy ( dmaBuf.buffer, gDIbuf + gRdwrCmd.transfered_so_far, tx );
         dmaBuf.count = tx;
         ret = CyU3PDmaChannelCommitBuffer( &glChHandleFDI_CPUtoP, dmaBuf.count, 0 );
         if (ret) {
          log_warn ( "Failed to send buffer\n" );
          return ret;
         }
         log_debug ( "Send %d bytes\n", tx );

         gRdwrCmd.transfered_so_far += tx;
         if (gRdwrCmd.transfered_so_far >= gRdwrCmd.header.transfer_length) {
            // switches to reading the ack
            gRdwrCmd.header.command = COMMAND_READ; 
         }
      } else {
          log_error ( "Logic error" );
          ret = 1;
      }

      return ret;
   }

   return 1; // nothing to do
}

    

uint16_t di_get ( uint16_t term, uint32_t addr, uint32_t *val ) {
    return di_read ( term, addr, (uint8_t*)val, 4 );
}
uint16_t di_set ( uint16_t term, uint32_t addr, uint32_t val ) {
    return di_write ( term, addr, (uint8_t*)&val, 4 );
}

rdwr_data_header_t di_header;

CyU3PReturnStatus_t di_setup() {
 CyU3PMemCopy ( (void*)&gRdwrCmd.header, (void*)&di_header, sizeof ( rdwr_data_header_t ) );
 return 0;
}


uint16_t do_trans( uint8_t cmd, uint16_t term, uint32_t addr, uint8_t* buf, uint32_t len) {
   uint16_t ret;
   uint32_t t0, time_out;
   di_header.command = cmd;
   di_header.term_addr = term;
   di_header.reg_addr = addr;
   di_header.transfer_length = len;

   gDIbuf = buf;
   
   // do that before we lock the mutex in case the main loop decides to lock the mutex for a rdwr trans.
   ret=start_rdwr( term, (uint16_t)len, di_setup, CyTrue );   
   if (ret) return ret;
   t0 = CyU3PGetTime();
   time_out = t0+2000;

   log_debug ( "Wait for rdwr..t0=%d to=%d\n", t0, time_out );
   while ( !gRdwrCmd.done ) {
    if (time_out < CyU3PGetTime()) break;
    CyU3PThreadSleep(1);
   }
   log_debug ( ".. t=%d\n", CyU3PGetTime() );
   RDWR_DONE(CyFalse);
   return gRdwrCmd.done ? 0 : 1;
}

uint16_t di_read ( uint16_t term, uint32_t addr, uint8_t *buf, uint32_t len ) {
   return do_trans ( COMMAND_READ, term, addr, buf, len ); 
}
uint16_t di_write ( uint16_t term, uint32_t addr, uint8_t *buf, uint32_t len ) {
   return do_trans ( COMMAND_WRITE, term, addr, buf, len );
}

