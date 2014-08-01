#include "slfifo_handler.h"
#include <cyu3system.h>
#include <cyu3dma.h>
#include <cyu3gpio.h>
#include <cyu3usb.h>
#include "cyu3pib.h"
#include "rdwr.h"

#include "error_handler.h"
#include "main.h"
/* This file should be included only once as it contains
 * structure definitions. Including it in multiple places
 * can result in linker error. */
#include "cyfxgpif2config.h"
//#include "cyfxgpif_syncsfclock.h"

#include "log.h"
#ifndef DEBUG_SLFIFO_HANDLER
#undef log_debug
#define log_debug(...) do {} while(0)
#endif



CyU3PDmaChannel glChHandleCPUtoP;  /* DMA MANUAL_IN channel handle.  */
CyU3PDmaChannel glChHandleUtoP;  /* DMA MANUAL_IN channel handle.  */
CyU3PDmaChannel glChHandlePtoU;  /* DMA MANUAL_OUT channel handle. */

extern rdwr_cmd_t gRdwrCmd;

CyBool_t gSlFifoActive = CyFalse;
CyBool_t gSlFifoAutoMode = CyFalse;
#ifdef  FIRMWARE_DI
extern CyBool_t gFdiHandlerActive;
#endif

void slfifo_cmd_start() {
  CyU3PReturnStatus_t apiRetStatus;
  CyU3PDmaBuffer_t buf_p;

  if(!gSlFifoActive
  #ifdef FIRMWARE_DI
    && !gFdiHandlerActive
  #endif
  ) {
   log_error ( "slfifo handler not active.\n" );
   return;
  }
  log_debug ( "slfifo cmd start\n" );

  // raise FLAGC to tell the FPGA a new command is coming
  CyU3PGpioSetValue (23, CyTrue);

  apiRetStatus = CyU3PGpifSMSwitch(0xFFFF, RESET, 0xFFFF, ALPHA_RESET, 0);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    log_error("GpifSMSwitch failed, Error Code = %d\n",apiRetStatus);
    return;
  }

  //CyU3PThreadSleep(50);

  // inject a command packet out to the FPGA
  CyU3PDmaChannelGetBuffer(&glChHandleCPUtoP, &buf_p, CYU3P_NO_WAIT);

  slfifo_cmd_t *slfifo_cmd = (slfifo_cmd_t *) (buf_p.buffer);
  slfifo_cmd->cmd             = (gRdwrCmd.header.command & bmSETWRITE) ? 0xC302 : 0xC301;
  slfifo_cmd->buffer_length   =  
  #ifdef FIRMWARE_DI
  // ep_buffer_size is always 1024 for fdi
  // in addition ep_buffer_size is not set correctly 
  // when external power is supplied not via usb cable.
    (gRdwrCmd.handler && 
     gRdwrCmd.handler->type == HANDLER_TYPE_FDI) ? 1024 :
  #endif
    gRdwrCmd.ep_buffer_size;
  slfifo_cmd->term_addr       = gRdwrCmd.header.term_addr;
  slfifo_cmd->reserved        = 0;
  slfifo_cmd->reg_addr        = gRdwrCmd.header.reg_addr; 
  slfifo_cmd->transfer_length = gRdwrCmd.header.transfer_length;
  buf_p.count = sizeof(*slfifo_cmd);

  // send the command
  apiRetStatus = CyU3PDmaChannelCommitBuffer(&glChHandleCPUtoP, buf_p.count, 0);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    log_error ( "SLFIFO commit fpga buffer fail %d\n", apiRetStatus );
    return;
  }
  log_debug ( "SLFIFO commit fpga buffer cmd %d term: %d addr %d len %d\n", 
    gRdwrCmd.header.command, 
    gRdwrCmd.header.term_addr,
    gRdwrCmd.header.reg_addr,
    gRdwrCmd.header.transfer_length );

  // drop FLAGC to tell FPGA the new command is ready
  CyU3PGpioSetValue (23, CyFalse);  /* Set the GPIO 23 to high */
}

/*
 *  This callback only used for non-mult of 4 reads 
 **/
void gpif2usb_cb(CyU3PDmaChannel   *chHandle, /* Handle to the DMA channel. */
		 CyU3PDmaCbType_t  type,      /* Callback type.             */
		 CyU3PDmaCBInput_t *input)    /* Callback status.           */{

  CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    
  if (type == CY_U3P_DMA_CB_PROD_EVENT) {
    uint32_t new_total = input->buffer_p.count + gRdwrCmd.transfered_so_far;
    uint32_t total = gRdwrCmd.header.command&bmSETWRITE ? 8 : gRdwrCmd.header.transfer_length+8;

    // don't do this for writes otherwise you'll truncate the ack packet instead of actual data 
    if(!(gRdwrCmd.header.command&bmSETWRITE) && gRdwrCmd.transfered_so_far < gRdwrCmd.header.transfer_length) { 
      // Because the FPGA can only transfer in multiples of 4 bytes,
      // check if this is the last packet and truncate it as necessary
      // to match what the host requested
      if(new_total >= gRdwrCmd.header.transfer_length) {
        log_debug("truncating %d %d\n", input->buffer_p.count, new_total - gRdwrCmd.header.transfer_length);
        input->buffer_p.count -= new_total - gRdwrCmd.header.transfer_length;
      }
    }
    status = CyU3PDmaChannelCommitBuffer (chHandle, input->buffer_p.count, 0);
    log_debug ( "C=%d\n", input->buffer_p.count );
    if (status != CY_U3P_SUCCESS)        {
      log_error("CyU3PDmaChannelCommitBuffer failed, Error code = %d\n", status);
    }
    gRdwrCmd.transfered_so_far += input->buffer_p.count;
    if (gRdwrCmd.transfered_so_far >= total) {
     gRdwrCmd.done=1; // got the ack packet on there now.
    }
  }

  log_debug ( "READ SLFIFO %d/%d done %d\n", gRdwrCmd.transfered_so_far, gRdwrCmd.header.transfer_length, gRdwrCmd.done );
}

/**
 * This callback used for mult of 4 reads/writes
 **/
void gpif2usb_auto_cb(CyU3PDmaChannel   *chHandle, /* Handle to the DMA channel. */
		 CyU3PDmaCbType_t  type,      /* Callback type.             */
		 CyU3PDmaCBInput_t *input)    /* Callback status.           */{

  if (!gRdwrCmd.done) {
      uint32_t total = gRdwrCmd.header.command & bmSETWRITE ? 8 : gRdwrCmd.header.transfer_length+8; 
      gRdwrCmd.transfered_so_far += input->buffer_p.count;
      if (gRdwrCmd.transfered_so_far >= total) {
        gRdwrCmd.done=1;
      }
  }
}

CyU3PReturnStatus_t slfifo_setup_cputop() {
  // Abstracting this so FDI can use the same channel.
  // and slfifo_cmd_start

  CyU3PDmaChannelConfig_t dmaCfg;
  CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

  /* Create a DMA MANUAL channel for CPU2P transfer. */
  dmaCfg.size           = sizeof(slfifo_cmd_t); //gRdwrCmd.ep_buffer_size;
  dmaCfg.count          = 1;
  dmaCfg.dmaMode        = CY_U3P_DMA_MODE_BYTE;
  dmaCfg.prodSckId      = CY_U3P_CPU_SOCKET_PROD;
  dmaCfg.consSckId      = CY_CPU_CONSUMER_PPORT_SOCKET;
  dmaCfg.notification   = 0;
  dmaCfg.cb             = 0;
  dmaCfg.prodHeader     = 0;
  dmaCfg.prodFooter     = 0;
  dmaCfg.consHeader     = 0;
  dmaCfg.prodAvailCount = 0;
  status |= CyU3PDmaChannelCreate (&glChHandleCPUtoP, CY_U3P_DMA_TYPE_MANUAL_OUT, &dmaCfg);
  if (status != CY_U3P_SUCCESS) {
    log_error("CyU3PDmaChannelCreate3 failed, Error code = %d\n", status );
  }
  return status;

}

CyU3PReturnStatus_t slfifo_setup(CyBool_t useAutoMode) {
  CyU3PDmaChannelConfig_t dmaCfg;
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

  log_debug("S a=%d m=%d\n", gSlFifoActive?1:0, useAutoMode?1:0);
  
  if (gSlFifoActive && useAutoMode != gSlFifoAutoMode ) {
    slfifo_teardown();
  }
  
  if (!gSlFifoActive) {
      log_debug(  "(Re)Setting up slfifo for auto mode: %d\n", useAutoMode ? 1 : 0 );
      CyU3PMemSet ((uint8_t *)&dmaCfg, 0, sizeof (dmaCfg));

      dmaCfg.size           = gRdwrCmd.ep_buffer_size;
      dmaCfg.count          = 2;
      dmaCfg.dmaMode        = CY_U3P_DMA_MODE_BYTE;
      dmaCfg.prodHeader     = 0;
      dmaCfg.prodFooter     = 0;
      dmaCfg.consHeader     = 0;
      dmaCfg.prodAvailCount = 0;

      /* Create a DMA MANUAL channel for P2U transfer. */
      dmaCfg.prodSckId      = CY_FX_PRODUCER_PPORT_SOCKET;
      dmaCfg.consSckId      = CY_FX_EP_CONSUMER_SOCKET;
      dmaCfg.notification   = CY_U3P_DMA_CB_PROD_EVENT;
      dmaCfg.cb             = useAutoMode ? gpif2usb_auto_cb : gpif2usb_cb;
      apiRetStatus |= CyU3PDmaChannelCreate (&glChHandlePtoU, useAutoMode ? CY_U3P_DMA_TYPE_AUTO_SIGNAL : CY_U3P_DMA_TYPE_MANUAL, &dmaCfg);
      if (apiRetStatus != CY_U3P_SUCCESS) {
        log_error("CyU3PDmaChannelCreate1 failed, Error code = %d\n", apiRetStatus);
        return apiRetStatus;
      }

      /* Create a DMA AUTO channel for U2P transfer. The writes can be auto because
       it just works regardless of whether the transfer size is a multiple of 4
      bytes. The read transfers have to be manual so that the buffer size can be
      truncated to the appropriate length if a transfer that is not a multiple of 
      4 is requested. */
      dmaCfg.prodSckId      = CY_FX_EP_PRODUCER_SOCKET;//CY_U3P_CPU_SOCKET_PROD;
      dmaCfg.consSckId      = CY_FX_CONSUMER_PPORT_SOCKET;
      dmaCfg.notification   = 0;
      dmaCfg.cb             = 0;

      apiRetStatus |= CyU3PDmaChannelCreate (&glChHandleUtoP, CY_U3P_DMA_TYPE_AUTO, &dmaCfg);
      if (apiRetStatus != CY_U3P_SUCCESS) {
        log_error("CyU3PDmaChannelCreate2 failed, Error code = %d\n", apiRetStatus);
        return apiRetStatus;
      }

      apiRetStatus == slfifo_setup_cputop();
      if (apiRetStatus != CY_U3P_SUCCESS) return apiRetStatus;

  }  
  
  // reset all the dma channels in prep for this new command
  apiRetStatus = CyU3PDmaChannelReset(&glChHandlePtoU);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    log_error("Channel Reset Failed, Error Code = %d\n",apiRetStatus);
  }
  apiRetStatus = CyU3PDmaChannelReset(&glChHandleUtoP);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    log_error("Channel Reset Failed, Error Code = %d\n",apiRetStatus);
  }
  apiRetStatus = CyU3PDmaChannelReset(&glChHandleCPUtoP);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    log_error("Channel Reset Failed, Error Code = %d\n",apiRetStatus);
  }

  CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);
  CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);

  /* Set DMA channel transfer size for each channel. */
  apiRetStatus = CyU3PDmaChannelSetXfer (&glChHandlePtoU, 0);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    log_error("CyU3PDmaChannelSetXfer1 Failed, Error code = %d\n", apiRetStatus);
  }
  apiRetStatus = CyU3PDmaChannelSetXfer (&glChHandleUtoP, 0);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    log_error("CyU3PDmaChannelSetXfer2 Failed, Error code = %d\n", apiRetStatus);
  }
  apiRetStatus = CyU3PDmaChannelSetXfer (&glChHandleCPUtoP, 0);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    log_error("CyU3PDmaChannelSetXfer3 Failed, Error code = %d\n", apiRetStatus);
  }
  
//  log_debug ("Sleepy Slfifo..");
//   CyU3PThreadSleep(10);
   


  gSlFifoActive = CyTrue;
  gSlFifoAutoMode = useAutoMode;
  
  log_debug("S\n");
  return apiRetStatus;
}

void slfifo_checkdone() {
 uint32_t prodCount, consCount;
 CyU3PDmaState_t state;
 CyU3PReturnStatus_t status;
 uint32_t total;
 if (!gSlFifoActive || gRdwrCmd.done)  return; 
 // check the tx counts
 status = CyU3PDmaChannelGetStatus ( &glChHandlePtoU, &state, &prodCount, &consCount );
 if (status) {
    log_warn ( "Error reading dma status." );
    gRdwrCmd.done=1;
    return;
 }

 total = gRdwrCmd.header.command & bmSETWRITE ? 8 : gRdwrCmd.header.transfer_length + 8;
 if ( consCount >= total ) {
  log_warn ( "Setting gRdwrCmd.done=1 consCount=%d gRdwrCmd.transfered_so_far=%d\n", consCount, 
      gRdwrCmd.transfered_so_far );
  gRdwrCmd.done=1; 
 }

}

/* This function tears down the DMA channels setup for CPU type handlers. */
void slfifo_teardown(void) {
  log_debug("TEARDOWN\n");
  /* Destroy the channels */
  CyU3PDmaChannelDestroy (&glChHandlePtoU);
  CyU3PDmaChannelDestroy (&glChHandleUtoP);
  CyU3PDmaChannelDestroy (&glChHandleCPUtoP);
  gSlFifoActive = CyFalse;
}


void slfifo_init(void) {
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

  /* setup gpif interface */
  CyU3PPibClock_t pibClock;

  /* Initialize the p-port block. */
  pibClock.clkDiv = 8;
  pibClock.clkSrc = CY_U3P_SYS_CLK;
  pibClock.isHalfDiv = CyFalse;
  /* Disable DLL for sync GPIF */
  pibClock.isDllEnable = CyFalse;
  apiRetStatus = CyU3PPibInit(CyTrue, &pibClock);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    log_error("P-port Initialization failed, Error Code = %d\n",apiRetStatus);
    error_handler(apiRetStatus);
  }

  /* Load the GPIF configuration for Slave FIFO sync mode. */
  apiRetStatus = CyU3PGpifLoad (&CyFxGpifConfig);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    log_error("CyU3PGpifLoad failed, Error Code = %d\n",apiRetStatus);
    error_handler(apiRetStatus);
  }

   /* Start the state machine. */
  apiRetStatus = CyU3PGpifSMStart (RESET, ALPHA_RESET);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    log_error("CyU3PGpifSMStart failed, Error Code = %d\n",apiRetStatus);
    error_handler(apiRetStatus);    
  }
  log_debug ( "gpif started\n" );


}
