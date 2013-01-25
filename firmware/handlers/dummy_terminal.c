/* This file impliments a dummy fx3 handler that receives and sends
   dummy data using the DMA MANUAL_IN and DMA MANUAL_OUT mode.  The
   data source and sink is achieved with the help of a DMA MANUAL IN
   channel and a DMA MANUAL OUT channel. A DMA MANUAL IN channel is
   created between the producer USB bulk endpoint and the CPU. A DMA
   MANUAL OUT channel is created between the CPU and the consumer USB
   bulk endpoint. Data is received in the IN channel DMA buffer from
   the host through the producer endpoint. CPU is signalled of the
   data reception using DMA callbacks. The CPU discards this
   buffer. This leads to the sink mechanism. A constant patern data is
   loaded onto the OUT Channel DMA buffer whenever the buffer is
   available. CPU issues commit of the DMA data transfer to the
   consumer endpoint which then gets transferred to the host.  This
   leads to a constant source mechanism.

   The DMA buffer size is defined based on the USB speed. 64 for full
   speed, 512 for high speed and 1024 for super
   speed. CY_FX_BULKSRCSINK_DMA_BUF_COUNT in the header file defines
   the number of DMA buffers.
 */

#include <cyu3system.h>
#include <cyu3usb.h>
#include <cyu3dma.h>
#include <cyu3error.h>
#include "log.h"
#include "main.h"
#include "handlers.h"
#include "vc_handler.h"

#define CY_FX_BULKSRCSINK_DMA_BUF_COUNT      (4) /* Bulk channel buffer count */
#define CY_FX_BULKSRCSINK_DMA_TX_SIZE        (0) /* DMA transfr size infinite */
#define CY_FX_BULKSRCSINK_PATTERN            (0xAA) /* 8-bit pattern to be loaded to the source buffers. */
#define CY_FX_EP_PRODUCER_SOCKET        CY_U3P_UIB_SOCKET_PROD_1    /* Socket 1 is producer */
#define CY_FX_EP_CONSUMER_SOCKET        CY_U3P_UIB_SOCKET_CONS_1    /* Socket 1 is consumer */

/* Burst mode definitions: Only for super speed operation. The maximum
 * burst mode supported is limited by the USB hosts available. The
 * maximum value for this is 16 and the minimum (no-burst) is 1. */

/* Burst length in 1 KB packets. Only applicable to USB 3.0. */
#define CY_FX_EP_BURST_LENGTH          (8)
/* Multiplication factor used when allocating DMA buffers to reduce
   DMA callback frequency. */
#define CY_FX_DMA_SIZE_MULTIPLIER      (2)

CyU3PDmaChannel glChHandleBulkSink; /* DMA MANUAL_IN channel handle.  */
CyU3PDmaChannel glChHandleBulkSrc;  /* DMA MANUAL_OUT channel handle. */

extern rdwr_cmd_t gRdwrCmd;

/* Callback funtion for the DMA event notification. */
void dummyTermCallback (
        CyU3PDmaChannel   *chHandle, /* Handle to the DMA channel. */
        CyU3PDmaCbType_t  type,      /* Callback type.             */
        CyU3PDmaCBInput_t *input) {   /* Callback status.           */

  //  CyU3PDmaBuffer_t buf_p;
  CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
  CyU3PDmaBuffer_t buf_p = input->buffer_p;
  ack_pkt_t *ack_pkt;


  //log_debug("Entering dummyTermCallback type=0x%x\n", type);

  if (type == CY_U3P_DMA_CB_PROD_EVENT) {
    /* This is a produce event notification to the CPU. This
     * notification is received upon reception of every buffer. We
     * have to discard the buffer as soon as it is received to
     * implement the data sink. */
    gRdwrCmd.transfered_so_far += buf_p->count;
    status = CyU3PDmaChannelDiscardBuffer(chHandle);
    if (status != CY_U3P_SUCCESS) {
      log_error("CyU3PDmaChannelDiscardBuffer failed, Error=%d\n", status);
    }

    // Check if we should commit ack packet yet.
    if(gRdwrCmd.transfered_so_far >= gRdwrCmd.header.transfer_length) {
      status = CyU3PDmaChannelGetBuffer (&glChHandleBulkSrc, &buf_p, CYU3P_NO_WAIT);
      if (status == CY_U3P_SUCCESS) {
	ack_pkt = (ack_pkt_t *) buf_p.buffer;
	ack_pkt->id       = ACK_PKT_ID;
	ack_pkt->checksum = 0;
	ack_pkt->status   = 0;
	ack_pkt->reserved = 0;
	status = CyU3PDmaChannelCommitBuffer (&glChHandleBulkSrc,sizeof(*ack_pkt),0);
	gRdwrCmd.done = 1;
      }
    }
  } else if (type == CY_U3P_DMA_CB_CONS_EVENT) {
    log_debug("TX\n");
//    /* This is a consume event notification to the CPU. This
//     * notification is received when a buffer is sent out from the
//     * device. We have to commit a new buffer as soon as a buffer is
//     * available to implement the data source. The data is preloaded
//     * into the buffer at that start. So just commit the buffer. */
//    status = CyU3PDmaChannelGetBuffer (chHandle, &buf_p, CYU3P_NO_WAIT);
//    if (status == CY_U3P_SUCCESS) {
//      /* Commit the full buffer with default status. */
//      status = CyU3PDmaChannelCommitBuffer (chHandle, buf_p.size, 0);
//      if (status != CY_U3P_SUCCESS) {
//	log_error("CyU3PDmaChannelCommitBuffer failed, Error = %d\n", status);
//      }
//    } else {
//      log_error("CyU3PDmaChannelGetBuffer failed, Error = %d\n", status);
//    }
//    glDMATxCount++;
//    log_debug("DUMMY FX3 TERM sent buffer %d.\n", glDMATxCount);
  } else {
    log_debug(" Unhandled type.\n");
  }
}

/* This function starts the application. This is called when a
 * SET_CONF event is received from the USB host. The endpoints are
 * configured and the DMA pipe is setup in this function. */
void dummy_term_init_handler(void) {
  //uint16_t index = 0;
  //  CyU3PDmaBuffer_t buf_p;
  CyU3PDmaChannelConfig_t dmaCfg;
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
  
  //log_debug("Entering dummy_term_init_handler\n");

  /* Flush the endpoint memory */
  CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);
  CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);
  
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
  dmaCfg.count = CY_FX_BULKSRCSINK_DMA_BUF_COUNT;
  dmaCfg.prodSckId = CY_FX_EP_PRODUCER_SOCKET;
  dmaCfg.consSckId = CY_U3P_CPU_SOCKET_CONS;
  dmaCfg.dmaMode = CY_U3P_DMA_MODE_BYTE;
  dmaCfg.notification = CY_U3P_DMA_CB_PROD_EVENT | CY_U3P_DMA_CB_CONS_EVENT;
  dmaCfg.cb = dummyTermCallback;
  dmaCfg.prodHeader = 0;
  dmaCfg.prodFooter = 0;
  dmaCfg.consHeader = 0;
  dmaCfg.prodAvailCount = 0;

  apiRetStatus = CyU3PDmaChannelCreate (&glChHandleBulkSink, CY_U3P_DMA_TYPE_MANUAL_IN, &dmaCfg);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    log_error("CyU3PDmaChannelCreate failed, Error code = %d\n", apiRetStatus);
    CyFxAppErrorHandler(apiRetStatus);
  }

  /* Create a DMA MANUAL_OUT channel for the consumer socket. */
  dmaCfg.prodSckId = CY_U3P_CPU_SOCKET_PROD;
  dmaCfg.consSckId = CY_FX_EP_CONSUMER_SOCKET;
  apiRetStatus = CyU3PDmaChannelCreate (&glChHandleBulkSrc, CY_U3P_DMA_TYPE_MANUAL_OUT, &dmaCfg);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    log_error("CyU3PDmaChannelCreate failed, Error code = %d\n", apiRetStatus);
    CyFxAppErrorHandler(apiRetStatus);
  }

  /* Set DMA Channel transfer size */
  apiRetStatus = CyU3PDmaChannelSetXfer (&glChHandleBulkSink, CY_FX_BULKSRCSINK_DMA_TX_SIZE);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    log_error("CyU3PDmaChannelSetXfer failed, Error code = %d\n", apiRetStatus);
    CyFxAppErrorHandler(apiRetStatus);
  }

  apiRetStatus = CyU3PDmaChannelSetXfer (&glChHandleBulkSrc, CY_FX_BULKSRCSINK_DMA_TX_SIZE);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    log_error("CyU3PDmaChannelSetXfer failed, Error code = %d\n", apiRetStatus);
    CyFxAppErrorHandler(apiRetStatus);
  }

//  /* Now preload all buffers in the MANUAL_OUT pipe with the required data. */
//  for (index = 0; index < CY_FX_BULKSRCSINK_DMA_BUF_COUNT; index++) {
//    apiRetStatus = CyU3PDmaChannelGetBuffer (&glChHandleBulkSrc, &buf_p, CYU3P_NO_WAIT);
//    if (apiRetStatus != CY_U3P_SUCCESS) {
//      log_error("CyU3PDmaChannelGetBuffer failed, Error = %d\n", apiRetStatus);
//      CyFxAppErrorHandler(apiRetStatus);
//    }
//    CyU3PMemSet (buf_p.buffer, CY_FX_BULKSRCSINK_PATTERN, buf_p.size);
//    apiRetStatus = CyU3PDmaChannelCommitBuffer (&glChHandleBulkSrc, buf_p.size, 0);
//    if (apiRetStatus != CY_U3P_SUCCESS) {
//      log_error("CyU3PDmaChannelCommitBuffer failed, Error code = %d\n", apiRetStatus);
//      CyFxAppErrorHandler(apiRetStatus);
//    }
//  }
}

/* This function destroys the dummy terminal DMA channels. */
void dummy_term_uninit_handler(void) {
  //log_debug("Entering dummy_term_uninit_handler\n");

  /* Destroy the channels */
  CyU3PDmaChannelDestroy (&glChHandleBulkSink);
  CyU3PDmaChannelDestroy (&glChHandleBulkSrc);

  /* Flush the endpoint memory */
  CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);
  CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);
}

void dummy_term_write_handler() {
  /* Flush the endpoint memory */
  CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);
  CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);
  gRdwrCmd.transfered_so_far = 0;
}


void configureDummyTerminalHandler(io_handler_t *handler) {
  handler->boot_handler   = NULL;
  handler->init_handler   = dummy_term_init_handler;
  handler->read_handler   = NULL;
  handler->write_handler  = dummy_term_write_handler;
  handler->status_handler = NULL;
  handler->chksum_handler = NULL;
  handler->uninit_handler = dummy_term_uninit_handler;
}
