#include "cpu_handler.h"
#include <cyu3system.h>
#include <cyu3dma.h>
#include <cyu3usb.h>
#include "rdwr.h"
#include "log.h"
#include "error_handler.h"
#include "main.h"

#ifndef DEBUG_CPU_HANDLER
#undef log_debug
#define log_debug(...) do {} while (0)
#endif


CyU3PDmaChannel glChHandleBulkSink; /* DMA MANUAL_IN channel handle.  */
CyU3PDmaChannel glChHandleBulkSrc;  /* DMA MANUAL_OUT channel handle. */
CyBool_t gCpuHandlerActive = CyFalse;
extern rdwr_cmd_t gRdwrCmd;
ack_pkt_t gAckPkt;

void cpu_handler_read(CyU3PDmaBuffer_t *buf_p) {

  log_debug("C %d\n", buf_p->size);
  buf_p->count = (gRdwrCmd.transfered_so_far + buf_p->size > gRdwrCmd.header.transfer_length) ? gRdwrCmd.header.transfer_length - gRdwrCmd.transfered_so_far : buf_p->size;

  // Call the read handler if the read handler function exists and if
  // the status is still OK. Otherwise, try continuing the data
  // transfer with bogus data.
  if(gRdwrCmd.handler->read_handler && gAckPkt.status == 0) {
    gAckPkt.status |= gRdwrCmd.handler->read_handler(buf_p);
  }

  gRdwrCmd.transfered_so_far += buf_p->count;  
  gAckPkt.status |= CyU3PDmaChannelCommitBuffer(&glChHandleBulkSrc, buf_p->count, 0);
  if (gAckPkt.status) {
    log_error ( "gAckPck.status %d\n" );
  }
  log_debug("R %d/%d\n", gRdwrCmd.transfered_so_far, gRdwrCmd.header.transfer_length);
}

void cpu_handler_write(CyU3PDmaBuffer_t *buf_p) {
  if(gRdwrCmd.handler->write_handler && gAckPkt.status == 0) {
    gAckPkt.status |= gRdwrCmd.handler->write_handler(buf_p);
  }
  gAckPkt.status |= CyU3PDmaChannelDiscardBuffer(&glChHandleBulkSink);
  gRdwrCmd.transfered_so_far += buf_p->count;
  log_debug("WRITE %d/%d\n", gRdwrCmd.transfered_so_far, gRdwrCmd.header.transfer_length);
}

/* commits the ack packet when any cpu handler is done */
void cpu_handler_commit_ack() {
  CyU3PReturnStatus_t status;
  CyU3PDmaBuffer_t buf_p;

  status = CyU3PDmaChannelGetBuffer (&glChHandleBulkSrc, &buf_p, 500 ); //CYU3P_NO_WAIT);
  if (status == CY_U3P_SUCCESS) {
    CyU3PMemCopy(buf_p.buffer, (uint8_t *) (&gAckPkt), sizeof(gAckPkt));
    CyU3PDmaChannelCommitBuffer (&glChHandleBulkSrc, sizeof(gAckPkt),0);
  }
  log_debug("ACK %d\n", gAckPkt.status);
}

/* Called at the start of any newly received cpu handler. */
void cpu_handler_cmd_start() {
  gAckPkt.id       = ACK_PKT_ID;
  gAckPkt.checksum = 0;
  gAckPkt.status   = 0;
  gAckPkt.reserved = 0;


  // Call this handlers init function, if it exists
  if(gRdwrCmd.handler->init_handler) {
    gAckPkt.status |= gRdwrCmd.handler->init_handler();
  }

}

uint16_t cpu_handler_readcb() {
     // a read
     CyU3PDmaBuffer_t dmaBuf_p;
     uint16_t ret = CyU3PDmaChannelGetBuffer (&glChHandleBulkSrc, &dmaBuf_p, 500 ); //CYU3P_WAIT_FOREVER);
     if (ret != CY_U3P_SUCCESS) {
         log_debug ( "didn't get a read buffer: %d\n", ret );
         return ret;
     }
     cpu_handler_read(&dmaBuf_p);
     return 0;
}

uint16_t cpu_handler_writecb() {
     CyU3PDmaBuffer_t dmaBuf_p;
     uint16_t ret = CyU3PDmaChannelGetBuffer (&glChHandleBulkSink, &dmaBuf_p, 500 ); //CYU3P_WAIT_FOREVER);
     if (ret != CY_U3P_SUCCESS) {
         // no buffer to write currently
         CyU3PDmaState_t stat;
         log_debug ( "didn't get write buffer: %d\n", ret );
         CyU3PDmaChannelGetStatus(&glChHandleBulkSink, &stat, 0, 0);
         log_debug ( "chstat %d\n", stat );
         return ret;
     }
     
     cpu_handler_write(&dmaBuf_p);
     return 0;
}

uint16_t cpu_handler_dmacb() {

    CyU3PReturnStatus_t ret;
    log_debug ( "DMA cb %d/%d done %d\n", gRdwrCmd.transfered_so_far, gRdwrCmd.header.transfer_length, gRdwrCmd.done );

    if (gRdwrCmd.header.command & bmSETWRITE) {
        // a write
        // wait for a buffer on the producer socket
        ret=cpu_handler_writecb();
        if (ret) return ret;
        
    } else {
        ret = cpu_handler_readcb();
        if (ret) return ret;
    }
    
    if (gRdwrCmd.transfered_so_far >= gRdwrCmd.header.transfer_length) {
        cpu_handler_commit_ack();
        gRdwrCmd.done = 1; // note setting done to 1 even if the buffer didn't work
    }

    return 0;
}
 
// NOTE tried only reseting only if new dma channel created 
// no effect
// so reset always seems more robust

// NOTE with not always resetting 1MB reads didn't recover after 1-3 tries like gets do
//
// NOTE flush/reset ep in between dma reset/setXfer seems to have made it behave 
// like it does when it first connects but all the time
// so did the reset toggle
// flush by itself didn't break it

uint16_t cpu_handler_reset_write() {

 /* reset our bulk channels */
  CyU3PReturnStatus_t apiRetStatus = CyU3PDmaChannelReset(&glChHandleBulkSink);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    log_error("Channel Reset Failed, Error Code = %d\n",apiRetStatus);
  }

  // cpu term at least seems ok with or
  // without flushing so leaving for now.
  CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);

  /* Set DMA Channel transfer size to infinite */
  apiRetStatus = CyU3PDmaChannelSetXfer (&glChHandleBulkSink, 0);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    log_error("CyU3PDmaChannelSetXfer failed, Error code = %d\n", apiRetStatus);
  }
  return apiRetStatus;
}

uint16_t cpu_handler_reset_read() {
  CyU3PReturnStatus_t apiRetStatus = CyU3PDmaChannelReset(&glChHandleBulkSrc);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    log_error("Channel Reset Failed, Error Code = %d\n",apiRetStatus);
  }
  CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);
  apiRetStatus = CyU3PDmaChannelSetXfer (&glChHandleBulkSrc, 0);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    log_error("CyU3PDmaChannelSetXfer failed, Error code = %d\n", apiRetStatus);
  }
  return apiRetStatus;
}

/* This function sets up the DMA channels to pipe data to and from the
 * CPU so that cpu handlers can deals with it. */
CyU3PReturnStatus_t cpu_handler_setup(void) {
  CyU3PDmaChannelConfig_t dmaCfg;
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

//  if (gCpuHandlerActive) {
//    log_debug ( "Cpu handler already active.\n" );
//    return CY_U3P_SUCCESS; // not an error if we're already set up
//  }
  log_debug ( "cpu_handler setup\n" );
  

  if (!gCpuHandlerActive) {

      /* Create a DMA MANUAL_IN (USB OUT transfer) channel for the producer socket. */
      CyU3PMemSet ((uint8_t *)&dmaCfg, 0, sizeof (dmaCfg));

      /* The buffer size will be same as packet size for the full speed,
       * high speed and super speed non-burst modes.  For super speed
       * burst mode of operation, the buffers will be 1024 * burst length
       * so that a full burst can be completed.  This will mean that a
       * buffer will be available only after it has been filled or when a
       * short packet is received. */
      dmaCfg.size      = gRdwrCmd.ep_buffer_size * CY_FX_DMA_SIZE_MULTIPLIER;
      dmaCfg.count     = CY_FX_EP_BUF_COUNT;
      dmaCfg.prodSckId = CY_FX_EP_PRODUCER_SOCKET;
      dmaCfg.consSckId = CY_U3P_CPU_SOCKET_CONS;
      dmaCfg.dmaMode   = CY_U3P_DMA_MODE_BYTE;
      dmaCfg.notification = 0; //0xFFFF; //CY_U3P_DMA_CB_PROD_EVENT | CY_U3P_DMA_CB_CONS_EVENT;
      dmaCfg.cb = 0; //cpu_handler_callback;
      dmaCfg.prodHeader = 0;
      dmaCfg.prodFooter = 0;
      dmaCfg.consHeader = 0;
      dmaCfg.prodAvailCount = 0;

      apiRetStatus = CyU3PDmaChannelCreate (&glChHandleBulkSink, CY_U3P_DMA_TYPE_MANUAL_IN, &dmaCfg);
      if (apiRetStatus != CY_U3P_SUCCESS) {
        log_error("CyU3PDmaChannelCreate MANUAL_IN failed, Error code = %d\n", apiRetStatus);
        error_handler(apiRetStatus);
      }

      /* Create a DMA MANUAL_OUT (USB IN transfer) channel for the consumer socket. */
      dmaCfg.prodSckId = CY_U3P_CPU_SOCKET_PROD;
      dmaCfg.consSckId = CY_FX_EP_CONSUMER_SOCKET;
      dmaCfg.size  = gRdwrCmd.ep_buffer_size * CY_FX_EP_BURST_LENGTH * CY_FX_DMA_SIZE_MULTIPLIER; 
      apiRetStatus = CyU3PDmaChannelCreate (&glChHandleBulkSrc, CY_U3P_DMA_TYPE_MANUAL_OUT, &dmaCfg);
      if (apiRetStatus != CY_U3P_SUCCESS) {
        log_error("CyU3PDmaChannelCreate failed, Error code = %d\n", apiRetStatus);
        error_handler(apiRetStatus);
      }
  }

  apiRetStatus = cpu_handler_reset_read();
  apiRetStatus |= cpu_handler_reset_write();
  
// NOTE this made it behave like it did on connect but all the time
//
//  CyU3PUsbStall (CY_FX_EP_PRODUCER, CyFalse, CyTrue ); // clear data toggle
//  CyU3PUsbStall (CY_FX_EP_CONSUMER, CyFalse, CyTrue );

//  log_debug ( "CPU Sleepy..." );
//  CyU3PThreadSleep(20);
  
  gCpuHandlerActive = CyTrue;

  return apiRetStatus;
}

/* This function tears down the DMA channels setup for CPU type handlers. */
void cpu_handler_teardown(void) {
  /* Destroy the channels */
  CyU3PDmaChannelDestroy (&glChHandleBulkSink);
  CyU3PDmaChannelDestroy (&glChHandleBulkSrc);
  gCpuHandlerActive = CyFalse;
}
