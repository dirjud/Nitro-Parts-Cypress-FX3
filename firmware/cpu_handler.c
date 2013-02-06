#include "cpu_handler.h"
#include <cyu3system.h>
#include <cyu3error.h>
#include <cyu3dma.h>
#include "rdwr.h"
#include "log.h"
#include "error_handler.h"
#include "main.h"

CyU3PDmaChannel glChHandleBulkSink; /* DMA MANUAL_IN channel handle.  */
CyU3PDmaChannel glChHandleBulkSrc;  /* DMA MANUAL_OUT channel handle. */
uint32_t gTransferedSoFar;  // number of bytes handled/transfered already

extern rdwr_cmd_t gRdwrCmd;
ack_pkt_t gAckPkt;

void cpu_handler_read() {
  CyU3PDmaBuffer_t buf_p;
  gAckPkt.status |= CyU3PDmaChannelGetBuffer(&glChHandleBulkSrc, &buf_p, CYU3P_NO_WAIT);

  log_debug("C %d\n", buf_p.size);
  buf_p.count = (gTransferedSoFar + buf_p.size > gRdwrCmd.header.transfer_length) ? gRdwrCmd.header.transfer_length - gTransferedSoFar : buf_p.size;

  // Call the read handler if the read handler function exists and if
  // the status is still OK. Otherwise, try continuing the data
  // transfer with bogus data.
  if(gRdwrCmd.handler->read_handler && gAckPkt.status == 0) {
    gAckPkt.status |= gRdwrCmd.handler->read_handler(&buf_p);
  }
  gAckPkt.status |= CyU3PDmaChannelCommitBuffer(&glChHandleBulkSrc, buf_p.count, 0);
  gTransferedSoFar += buf_p.count;
  log_debug("R %d/%d\n", gTransferedSoFar, gRdwrCmd.header.transfer_length);
}

void cpu_handler_write(CyU3PDmaBuffer_t *buf_p) {
  if(gRdwrCmd.handler->write_handler && gAckPkt.status == 0) {
    gAckPkt.status |= gRdwrCmd.handler->write_handler(buf_p);
  }
  gAckPkt.status |= CyU3PDmaChannelDiscardBuffer(&glChHandleBulkSink);
  gTransferedSoFar += buf_p->count;
  log_debug("WRITE %d/%d\n", gTransferedSoFar, gRdwrCmd.header.transfer_length);
}

/* commits the ack packet when any cpu handler is done */
void cpu_handler_commit_ack() {
  CyU3PReturnStatus_t status;
  CyU3PDmaBuffer_t buf_p;

  status = CyU3PDmaChannelGetBuffer (&glChHandleBulkSrc, &buf_p, CYU3P_NO_WAIT);
  if (status == CY_U3P_SUCCESS) {
    CyU3PMemCopy(buf_p.buffer, (uint8_t *) (&gAckPkt), sizeof(gAckPkt));
    CyU3PDmaChannelCommitBuffer (&glChHandleBulkSrc, sizeof(gAckPkt),0);
    gRdwrCmd.done = 1;
  }
  log_debug("ACK %d\n", gAckPkt.status);
}

/* Called at the start of any newly received cpu handler. */
void cpu_handler_cmd_start() {
  gTransferedSoFar = 0;
  gAckPkt.id       = ACK_PKT_ID;
  gAckPkt.checksum = 0;
  gAckPkt.status   = 0;
  gAckPkt.reserved = 0;

  // Call this handlers init function, if it exists
  if(gRdwrCmd.handler->init_handler) {
    gAckPkt.status |= gRdwrCmd.handler->init_handler();
  }

  // If this is a read or get command, kick of the first packet read.
  // Additional packets reads will get initiated in the event callback.
  if(gRdwrCmd.header.command == COMMAND_READ ||
     gRdwrCmd.header.command == COMMAND_GET) {
    cpu_handler_read();
  }
}

void cpu_handler_cmd_end() {
  if(gRdwrCmd.handler->uninit_handler) {
    gRdwrCmd.handler->uninit_handler();
  }
}




/* Callback funtion for the DMA event notification. */
void cpu_handler_callback(CyU3PDmaChannel   *chHandle, CyU3PDmaCbType_t  type, CyU3PDmaCBInput_t *input) {

  if (type == CY_U3P_DMA_CB_PROD_EVENT) {
    // We received some write data, so call the write handler.
    cpu_handler_write(&(input->buffer_p));

    // Check if we should commit ack packet yet.
    if(gTransferedSoFar >= gRdwrCmd.header.transfer_length) {
      cpu_handler_commit_ack();
    }

  } else if (type == CY_U3P_DMA_CB_CONS_EVENT) {
    // Check if we should commit ack packet yet.
    if(gRdwrCmd.done) {
      // do nothing if we are done
    } else if(gTransferedSoFar >= gRdwrCmd.header.transfer_length) {
      cpu_handler_commit_ack();
    } else { // otherwise, we initiate another read.
      cpu_handler_read();
    }
  }
}

/* This function sets up the DMA channels to pipe data to and from the
 * CPU so that cpu handlers can deals with it. */
void cpu_handler_setup(void) {
  CyU3PDmaChannelConfig_t dmaCfg;
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

  /* Create a DMA MANUAL_IN channel for the producer socket. */
  CyU3PMemSet ((uint8_t *)&dmaCfg, 0, sizeof (dmaCfg));

  /* The buffer size will be same as packet size for the full speed,
   * high speed and super speed non-burst modes.  For super speed
   * burst mode of operation, the buffers will be 1024 * burst length
   * so that a full burst can be completed.  This will mean that a
   * buffer will be available only after it has been filled or when a
   * short packet is received. */
  dmaCfg.size  = gRdwrCmd.ep_buffer_size;
  /* Multiply the buffer size with the multiplier for performance
   * improvement. */
  dmaCfg.size *= CY_FX_DMA_SIZE_MULTIPLIER;
  dmaCfg.count     = 4;
  dmaCfg.prodSckId = CY_FX_EP_PRODUCER_SOCKET;
  dmaCfg.consSckId = CY_U3P_CPU_SOCKET_CONS;
  dmaCfg.dmaMode   = CY_U3P_DMA_MODE_BYTE;
  dmaCfg.notification = CY_U3P_DMA_CB_PROD_EVENT | CY_U3P_DMA_CB_CONS_EVENT;
  dmaCfg.cb = cpu_handler_callback;
  dmaCfg.prodHeader = 0;
  dmaCfg.prodFooter = 0;
  dmaCfg.consHeader = 0;
  dmaCfg.prodAvailCount = 0;

  apiRetStatus = CyU3PDmaChannelCreate (&glChHandleBulkSink, CY_U3P_DMA_TYPE_MANUAL_IN, &dmaCfg);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    log_error("CyU3PDmaChannelCreate failed, Error code = %d\n", apiRetStatus);
    error_handler(apiRetStatus);
  }

  /* Create a DMA MANUAL_OUT channel for the consumer socket. */
  dmaCfg.prodSckId = CY_U3P_CPU_SOCKET_PROD;
  dmaCfg.consSckId = CY_FX_EP_CONSUMER_SOCKET;
  apiRetStatus = CyU3PDmaChannelCreate (&glChHandleBulkSrc, CY_U3P_DMA_TYPE_MANUAL_OUT, &dmaCfg);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    log_error("CyU3PDmaChannelCreate failed, Error code = %d\n", apiRetStatus);
    error_handler(apiRetStatus);
  }

  /* Set DMA Channel transfer size to infinite */
  apiRetStatus = CyU3PDmaChannelSetXfer (&glChHandleBulkSink, 0);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    log_error("CyU3PDmaChannelSetXfer failed, Error code = %d\n", apiRetStatus);
  }

  apiRetStatus = CyU3PDmaChannelSetXfer (&glChHandleBulkSrc, 0);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    log_error("CyU3PDmaChannelSetXfer failed, Error code = %d\n", apiRetStatus);
  }
}

/* This function tears down the DMA channels setup for CPU type handlers. */
void cpu_handler_teardown(void) {
  /* Destroy the channels */
  CyU3PDmaChannelDestroy (&glChHandleBulkSink);
  CyU3PDmaChannelDestroy (&glChHandleBulkSrc);
}
