#include "cyu3error.h"
#include "cyu3usb.h"
#include "vendor_commands.h"
#include "rdwr.h"
#include "log.h"
#include "cpu_handler.h"
#include "slfifo_handler.h"
#include "main.h"
#include "fx3_terminals.h"
#include <cyu3i2c.h>
#include <m24xx.h>

#ifndef DEBUG_RDWR
#undef log_debug
#define log_debug(...) do {} while (0)
#endif

rdwr_cmd_t gRdwrCmd;
uint8_t gSerialNum[16];

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
  uint16_t debug_datasize, orig_ep_buffer_size;

  //log_debug("Entering handleRDWR\n");
  if (bReqType != 0x40 || wLength != sizeof(rdwr_data_header_t)) {
    log_error("Bad ReqType or length=%d (%d)\n", wLength, sizeof(rdwr_data_header_t));
    return CY_U3P_ERROR_BAD_ARGUMENT;
  }

  orig_ep_buffer_size=gRdwrCmd.ep_buffer_size;
  log_debug ( "gRdwrCmd buffer: %d\n", gRdwrCmd.ep_buffer_size );
  // Fetch the rdwr command
  
  status = CyU3PUsbGetEP0Data(wLength, (uint8_t *) &(gRdwrCmd.header), &debug_datasize);
  if (!gRdwrCmd.ep_buffer_size) {
    log_error ( "gRdwrCmd buffer: %d wLength %d data read: %d\n" , gRdwrCmd.ep_buffer_size, wLength, debug_datasize );
    gRdwrCmd.ep_buffer_size=orig_ep_buffer_size;
  }

  if(status != CY_U3P_SUCCESS){
    log_error("Error get EP0 Data\n", status);
    log_error("Flush status = %d\n", CyU3PUsbFlushEp(0));
    return status;
  }

  gRdwrCmd.done    = 0;
  gRdwrCmd.handler = NULL;

  // Select the appropriate handler. Any handler specified with term_addr of
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
    case HANDLER_TYPE_SLAVE_FIFO:
      slfifo_cmd_end();
      break;
    }
  }

  // first tear down previous handlers DMA channels
  if(prev_handler && prev_handler != gRdwrCmd.handler) {
    switch(prev_handler->type) {
    case HANDLER_TYPE_CPU:
      cpu_handler_teardown();
      break;
      
    case HANDLER_TYPE_SLAVE_FIFO:
      slfifo_teardown();
      break;
      
    default:
      // do nothing by default
      break;
    }
  }

  // now setup the new handler types DMA channels
  if(gRdwrCmd.handler) {
    switch(gRdwrCmd.handler->type) {
    case HANDLER_TYPE_CPU:
      status=cpu_handler_setup();
      break;
      
    case HANDLER_TYPE_SLAVE_FIFO:
      status=slfifo_setup();
      break;

    default:
      // do nothing by default
      break;
    }
  }
  if (status) {
    log_error ( "gRdWrCmd.handler failed to setup. %d\n", status );
    return status; 
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
    case HANDLER_TYPE_SLAVE_FIFO:
      slfifo_cmd_start();
      break;
    }
  }

  return CY_U3P_SUCCESS;
}

/******************************************************************************/
CyBool_t handle_serial_num(uint8_t bReqType, uint16_t wLength) {
  CyU3PI2cPreamble_t preamble;
  uint32_t reg_addr = FX3_PROM_SERIALNUM0_0;
  uint8_t dev_addr = 0x50;
  uint8_t size = 17; // size of prom
  uint16_t status;

  if (wLength != 16) {
    log_error("Bad length=%d \n", wLength);
    return CY_U3P_ERROR_BAD_ARGUMENT;
  }

  switch(bReqType) {
  case 0xC0:
    preamble.length    = 4;
    preamble.buffer[0] = m24xx_get_dev_addr(dev_addr, reg_addr, size, 0);
    preamble.buffer[1] = (uint8_t)(reg_addr >> 8);
    preamble.buffer[2] = (uint8_t)(reg_addr & 0xFF);
    preamble.buffer[3] = m24xx_get_dev_addr(dev_addr, reg_addr, size, 1);
    preamble.ctrlMask  = 0x0004;
    status = CyU3PI2cReceiveBytes (&preamble, gSerialNum, 16, 1);
    if(status) {
      log_error("Error reading serial num from prom (%d)\n", status);
      return CyFalse;
    }
    status = CyU3PUsbSendEP0Data(16, gSerialNum);
    if(status) {
      log_error("Error Sending serial num to EP0 (%d)\n", status);
      return CyFalse;
    }
    return CyTrue;

  case 0x40:
    status = CyU3PUsbGetEP0Data(wLength, gSerialNum, 0);
    if(status) {
      log_error("Error getting serial num from EP0 (%d)\n", status);
      return status;
    }

    preamble.length    = 3;
    preamble.buffer[0] = m24xx_get_dev_addr(dev_addr, reg_addr, size, 0);
    preamble.buffer[1] = (uint8_t)(reg_addr >> 8);
    preamble.buffer[2] = (uint8_t)(reg_addr & 0xFF);
    preamble.ctrlMask  = 0x0000;
    
    status = CyU3PI2cTransmitBytes(&preamble, gSerialNum, 16, 1);
    if(status) {
      log_error("Error writing serial num to I2C (%d)\n", status);
      return CyFalse;
    }
    
    /* Wait for the write to complete. */
    preamble.length = 1;
    status = CyU3PI2cWaitForAck(&preamble, 200);
    if(status) {
      log_error("Error waiting for i2c ACK after writing serial num (%d)\n", status);
      return CyFalse;
    }
    
    /* An additional delay seems to be required after receiving an ACK. */
    CyU3PThreadSleep (1);
    return CyTrue;


  default:
    log_error("Bad ReqType=%d \n", bReqType);
    return CY_U3P_ERROR_BAD_ARGUMENT;
  }
}


/******************************************************************************/
CyBool_t handle_vendor_cmd(uint8_t  bRequest, uint8_t bReqType,
			   uint8_t  bType, uint8_t bTarget,
			   uint16_t wValue, uint16_t wIndex, 
			   uint16_t wLength) {

  //log_debug("Entering handle_vendor_cmd\n");
  CyBool_t isHandled = CyTrue;
  CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

  //  log_debug("VC%x\n", bRequest);
  switch (bRequest) {
  case VC_HI_RDWR:
    log_debug("Call handle_rdwr\n");
    status = handle_rdwr(bReqType, wLength);
    break;

  case VC_SERIAL:
    status = handle_serial_num(bReqType, wLength);
    break;

//  case VC_RDWR_RAM:
//    status = handleRamRdwr(bRequest, bReqType, bType, bTarget, wValue, wIndex, wLength);
//    break;

  case VC_RENUM:
    CyU3PDeviceReset(CyFalse); // cold boot from prom
    break; // for readability but the above function actually doesn't return.
    
  default:
    isHandled = CyFalse;
    break;
  }

  if ((isHandled != CyTrue) || (status != CY_U3P_SUCCESS)) {
    /* This is an unhandled setup command. Stall the EP. */
    log_debug("VC stalled\n" ); // (cmd: %d)\n", bRequest);
    CyU3PUsbStall (0, CyTrue, CyFalse);
  } else {
    log_debug("VC Acked\n");
    CyU3PUsbAckSetup ();
  }

  log_debug ( "handle_vendor_cmd exit\n");
  return CyTrue;
}
