#include "cyu3error.h"
#include "cyu3usb.h"
#include "vendor_commands.h"
#include "rdwr.h"
#include "log.h"
#include "cpu_handler.h"
#include "main.h"

rdwr_cmd_t gRdwrCmd;

void rdwr_teardown() {
  if(gRdwrCmd.handler) {
    switch(gRdwrCmd.handler->type) {
    case HANDLER_TYPE_CPU:
      cpu_handler_teardown();
      break;
    }
  }
}

/******************************************************************************/
CyU3PReturnStatus_t handle_rdwr(bReqType, wLength) {
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

  gRdwrCmd.done    = 0;
  gRdwrCmd.handler = NULL;

  // Select the appropriate handler. Any hanlder specified with term_addr of
  // 0 is considered the wild card handler and will prevent any handlers
  // following if from being accessed.
  int i = 0;
  while(io_handlers[i].type != HANDLER_TYPE_TERMINATOR) {
    if(io_handlers[i].term_addr == gRdwrCmd.header.term_addr ||
       io_handlers[i].term_addr == 0) { 
      gRdwrCmd.handler = &(io_handlers[i]);
      //log_debug("Found handler %d\n", i);
      break;
    }
    i++;
  }
  
  // if there is a previous handler, call the previous handler types
  // uninit function
  if(prev_handler) {
    switch(gRdwrCmd.handler->type) {
    case HANDLER_TYPE_CPU:
      cpu_handler_cmd_end();
      break;
    }
  }

  // If the handler type is changing, then we call the appropriate 
  // DMA channel tear down and setup functions.
  if((gRdwrCmd.handler && (prev_handler == NULL || prev_handler->type != gRdwrCmd.handler->type)) ||
     (prev_handler && (gRdwrCmd.handler == NULL || prev_handler->type != gRdwrCmd.handler->type))) {
    // first tear down previous handlers DMA channels
    if(prev_handler) {
      switch(prev_handler->type) {
      case HANDLER_TYPE_CPU:
	cpu_handler_teardown();
	break;

      default:
	// do nothing by default
	break;
      }
    }

    // now setup the new hanlder types DMA channels
    if(gRdwrCmd.handler) {
      switch(gRdwrCmd.handler->type) {
      case HANDLER_TYPE_CPU:
	cpu_handler_setup();
	break;
	
      default:
	// do nothing by default
	break;
      }
    }
  }

  /* Flush the endpoint memory */
  CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);
  CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);

  // call the new handlers init function, if it exists
  if (gRdwrCmd.handler) {
    switch(gRdwrCmd.handler->type) {
    case HANDLER_TYPE_CPU:
      cpu_handler_cmd_start();
      break;
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
    status = handle_rdwr(bReqType, wLength);
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
//    CyU3PUsbAckSetup ();
  }

  return CyTrue;
}
