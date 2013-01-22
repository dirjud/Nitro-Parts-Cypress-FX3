#include "cyu3error.h"
#include "cyu3usb.h"
#include "vc_handler.h"
#include "vendor_commands.h"
#include "handlers.h"
#include "log.h"

typedef struct {
  rdwr_data_header_t header;  // current command
  io_handler_t *handler;   // current io_handler
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


// #include "cyfx3device.h"
// #include "cyfx3usb.h"
// void memCopy (uint8_t *d, uint8_t *s, int32_t cnt) {
//   int32_t i;
//   for (i = 0; i < cnt; i++) {
//     *d++ = *s++;
//   }
// }
// 
// /* This function validates the addresses being written to/read from 
//    Return Value:
//     0 - Address is valid
//    -1 - Address is not valid
// */
// int ramCheckAddress (uint32_t address, uint32_t len) {
//   if (address & 3) { /* expect long word boundary */
//     return -1; 
//   }
//   len += address;
//   if ((address >= CY_FX3_BOOT_SYSMEM_BASE1) && (len <= CY_FX3_BOOT_SYSMEM_END)){
//     return 0;
//   }
//   if (len <= CY_FX3_BOOT_ITCM_END) {
//     return 0;
//   }
//   return -1;
// }
// 
// uint8_t tmp[4096];
// 
// /******************************************************************************/
// #define USB_DATA_BUF_SIZE (1024*4)
// #define USB_SETUP_DIR               (0x80) /* 0x80 = To Host */
// CyU3PReturnStatus_t handleRamRdwr(uint8_t  bRequest, uint8_t bReqType,
// 				  uint8_t  bType, uint8_t bTarget,
// 				  uint16_t wValue, uint16_t wIndex, 
// 				  uint16_t wLength) {
//   
//   uint32_t address = (wIndex << 16) | wValue;
// 
//   CyU3PDebugPrint(0, "Entering handleRamRdwr\n");
//   CyU3PDebugPrint(0, "This function does not work yet\n");
//   return CY_U3P_ERROR_BAD_ARGUMENT;
// 
//   if (wLength > USB_DATA_BUF_SIZE) {
//     CyU3PDebugPrint(0, "Error: wLength too large\n");
//     return CY_U3P_ERROR_BAD_ARGUMENT;
//   }
// 
//   if (address == 0xE600) {
//     /* Note: This is a command issued by the CyControl Application
//      * to detect the legacy products.  As we are an FX3 device we
//      * stall the endpoint to indicate that this is not a legacy
//      * device.
//      */
//     CyU3PDebugPrint(0, "Error: FX2 Addr Provided\n");
//     return CY_U3P_ERROR_BAD_ARGUMENT;
//   }
// 
//   // check if this is a signal to reboot
//   if (wLength == 0) {	
//     CyU3PDebugPrint(0, "Reboot requested\n"); 
// //    /* Mask the USB Interrupts and Disconnect the USB Phy. */
// //    CyFx3BootUsbConnect (CyFalse, CyTrue);
// //    /* Transfer to Program Entry */    
// //    CyFx3BootJumpToProgramEntry (address);
//     return CY_U3P_SUCCESS;
//   }
// 
//   // check address is within appropriate ranges
//   if(ramCheckAddress (address, wLength) < 0) {
//     int2str("Error: Bad Addr Provided", address);
//     return  CY_U3P_ERROR_BAD_ARGUMENT;
//   }
// 
//   if (bReqType & USB_SETUP_DIR) {   
//     CyU3PDebugPrint(0, "Reading RAM data\n"); 
//     return CyU3PUsbSendEP0Data(wLength, (uint8_t *) address);
//   } else {
//     CyU3PDebugPrint(0, "Writing RAM data\n"); 
//     CyU3PReturnStatus_t status = CyU3PUsbGetEP0Data(wLength, tmp, 0); //(uint8_t *) address, 0);
//     memCopy((uint8_t *) address, tmp, wLength);
//     int2str("STATUS = ", status);
//     return status;
//   }
// }


/******************************************************************************/
CyU3PReturnStatus_t handleRDWR(bReqType, wLength) {
  CyU3PReturnStatus_t status;
  io_handler_t *prev_handler = gRdwrCmd.handler;

  log_debug("Entering handleRDWR\n");
  if (bReqType != 0x40 || wLength != sizeof(rdwr_data_header_t)) {
    log_debug("Bad ReqType or length=%d (%d)\n", wLength, sizeof(rdwr_data_header_t));
    return CY_U3P_ERROR_BAD_ARGUMENT;
  }

  // Fetch the rdwr command
  status = CyU3PUsbGetEP0Data(wLength, (uint8_t *) &(gRdwrCmd.header), 0);
  if(status != CY_U3P_SUCCESS){
    log_debug("Error get EP0 Data\n", status);
    return status;
  }

  log_debug("Received RDWR Command:\n");
  log_debug("  command   : 0x%x\n", gRdwrCmd.header.command);
  log_debug("  term_addr : 0x%x\n", gRdwrCmd.header.term_addr);
  log_debug("  reg_addr  : 0x%x\n", gRdwrCmd.header.reg_addr);
  log_debug("  len       : 0x%x\n", gRdwrCmd.header.transfer_length);

  // Select the appropriate handler
  for(gRdwrCmd.handler = gIoHandlers.head; gRdwrCmd.handler; gRdwrCmd.handler = gRdwrCmd.handler->next) {
    if(gRdwrCmd.handler->term_addr == gRdwrCmd.header.term_addr) {
      log_debug("Found RDWR Handler for term=0x%x\n", gRdwrCmd.header.term_addr);
      break;
    }
  }

  // Select the default handler if no terminal was found matching
  if(gRdwrCmd.handler == NULL) {
    gRdwrCmd.handler = gIoHandlers.default_handler;
    log_debug("Using default RDWR Handler\n");
  }

  // if the handler has changed, then call the uninit() function on the 
  // previous handler and the init() functionon the new handler.
  if(prev_handler != gRdwrCmd.handler) {
    log_debug("Switching handlers\n");
    if (prev_handler && prev_handler->uninit_handler) {
      prev_handler->uninit_handler();
    }
    if (gRdwrCmd.handler && gRdwrCmd.handler->init_handler) {
      gRdwrCmd.handler->init_handler();
    }
  }
  return CY_U3P_SUCCESS;
}


CyBool_t handle_vendor_cmd(uint8_t  bRequest, uint8_t bReqType,
			   uint8_t  bType, uint8_t bTarget,
			   uint16_t wValue, uint16_t wIndex, 
			   uint16_t wLength) {

  log_debug("Entering handle_vendor_cmd\n");
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
    log_debug("VC stalled\n");
    CyU3PUsbStall (0, CyTrue, CyFalse);
  } else {
    log_debug("VC Acked\n");
    CyU3PUsbAckSetup ();
  }

  return CyTrue;
}
