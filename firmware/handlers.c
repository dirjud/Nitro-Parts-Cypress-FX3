#include "cyu3error.h"
#include "cyu3usb.h"
#include "vc_handler.h"
#include "vendor_commands.h"
#include "handlers.h"
#include "log.h"

typedef struct {
  rdwr_data_header_t header;   // current command
  io_handler_t *handler;       // current io_handler
  uint16_t ep_buffer_size;     // usb end point buffer size
  uint32_t transfered_so_far;  // number of bytes handled/transfered already
  uint8_t done;                // has this command been handled?
} rdwr_cmd_t;

  
rdwr_cmd_t gRdwrCmd;
io_handlers_t gIoHandlers;
io_handler_t dummy_term_handler;
void configureDummyTerminalHandler(io_handler_t *handler);
void initHandlers() {
  gIoHandlers.head = NULL;
  gIoHandlers.tail = NULL;
  gIoHandlers.default_handler = NULL;


  configureDummyTerminalHandler(&dummy_term_handler);
  dummy_term_handler.term_addr = 5;
  registerHandler(&dummy_term_handler);
}

void registerHandler(io_handler_t *handler) {
  if(gIoHandlers.tail) {
    gIoHandlers.tail->next = handler;
  } else {
    // list is empty, so add this as the only element in the list
    gIoHandlers.head = handler;
  }
  handler->next = NULL;
  gIoHandlers.tail = handler;
}

void registerDefaultHandler(io_handler_t *handler) {
  gIoHandlers.default_handler = handler;
}

void stopEventHandler() {
  if (gRdwrCmd.handler && gRdwrCmd.handler->uninit_handler) {
    gRdwrCmd.handler->uninit_handler();
  }
  gRdwrCmd.handler = NULL;
}


/******************************************************************************/
CyU3PReturnStatus_t handleRDWR(bReqType, wLength) {
  CyU3PReturnStatus_t status;
  io_handler_t *prev_handler = gRdwrCmd.handler;

  //log_debug("Entering handleRDWR\n");
  if (bReqType != 0x40 || wLength != sizeof(rdwr_data_header_t)) {
    log_error("Bad ReqType or length=%d (%d)\n", wLength, sizeof(rdwr_data_header_t));
    return CY_U3P_ERROR_BAD_ARGUMENT;
  }

  // Fetch the rdwr command
  status = CyU3PUsbGetEP0Data(wLength, (uint8_t *) &(gRdwrCmd.header), 0);
  if(status != CY_U3P_SUCCESS){
    log_error("Error get EP0 Data\n", status);
    return status;
  }

  //log_debug("Received RDWR Command:\n");
  //log_debug("  command   : 0x%x\n", gRdwrCmd.header.command);
  //log_debug("  term_addr : 0x%x\n", gRdwrCmd.header.term_addr);
  //log_debug("  reg_addr  : 0x%x\n", gRdwrCmd.header.reg_addr);
  //log_debug("  len       : 0x%x\n", gRdwrCmd.header.transfer_length);

  gRdwrCmd.done = 0;

  // Select the appropriate handler
  for(gRdwrCmd.handler = gIoHandlers.head; gRdwrCmd.handler; gRdwrCmd.handler = gRdwrCmd.handler->next) {
    if(gRdwrCmd.handler->term_addr == gRdwrCmd.header.term_addr) {
      //log_debug("Found RDWR Handler for term=0x%x\n", gRdwrCmd.header.term_addr);
      break;
    }
  }

  // Select the default handler if no terminal was found matching
  if(gRdwrCmd.handler == NULL) {
    gRdwrCmd.handler = gIoHandlers.default_handler;
  }

  // if the handler has changed, then call the uninit() function on the 
  // previous handler and the init() functionon the new handler.
  if(prev_handler != gRdwrCmd.handler) {
    if (prev_handler && prev_handler->uninit_handler) {
      prev_handler->uninit_handler();
    }
    if (gRdwrCmd.handler && gRdwrCmd.handler->init_handler) {
      gRdwrCmd.handler->init_handler();
    }
  }

  if (gRdwrCmd.header.command & bmSETWRITE) {
    if (gRdwrCmd.handler && gRdwrCmd.handler->write_handler) {
      //log_debug("WH\n");
      gRdwrCmd.handler->write_handler();
    }
  }

  return CY_U3P_SUCCESS;
}


CyBool_t handle_vendor_cmd(uint8_t  bRequest, uint8_t bReqType,
			   uint8_t  bType, uint8_t bTarget,
			   uint16_t wValue, uint16_t wIndex, 
			   uint16_t wLength) {

  //log_debug("Entering handle_vendor_cmd\n");
  CyBool_t isHandled = CyTrue;
  CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

  
  switch (bRequest) {
  case VC_HI_RDWR:
    status = handleRDWR(bReqType, wLength);
    break;

//  case VC_RDWR_RAM:
//    status = handleRamRdwr(bRequest, bReqType, bType, bTarget, wValue, wIndex, wLength);
//    break;
    
  default:
    isHandled = CyFalse;
    break;
  }

  if ((isHandled != CyTrue) || (status != CY_U3P_SUCCESS)) {
    /* This is an unhandled setup command. Stall the EP. */
    //log_debug("VC stalled\n");
    CyU3PUsbStall (0, CyTrue, CyFalse);
  } else {
    //log_debug("VC Acked\n");
    CyU3PUsbAckSetup ();
  }

  return CyTrue;
}


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
