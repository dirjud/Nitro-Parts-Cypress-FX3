

#include "log.h"

#include <cyu3uart.h>
#include <cyu3error.h>
#include "error_handler.h"



void init_uart_debug() {

  CyU3PUartConfig_t uartConfig;
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;


  /* Initialize the UART for printing debug messages */
  apiRetStatus = CyU3PUartInit();
  if (apiRetStatus != CY_U3P_SUCCESS) {
    /* Error handling */
    error_handler_0(apiRetStatus, CyFalse);
  }

  /* Set UART configuration */
  CyU3PMemSet ((uint8_t *)&uartConfig, 0, sizeof (uartConfig));
  uartConfig.baudRate = CY_U3P_UART_BAUDRATE_230400;
  uartConfig.stopBit = CY_U3P_UART_ONE_STOP_BIT;
  uartConfig.parity = CY_U3P_UART_NO_PARITY;
  uartConfig.txEnable = CyTrue;
  uartConfig.rxEnable = CyFalse;
  uartConfig.flowCtrl = CyFalse;
  uartConfig.isDma = CyTrue;

  apiRetStatus = CyU3PUartSetConfig (&uartConfig, NULL);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    error_handler_0(apiRetStatus, CyFalse);
  }

  /* Set the UART transfer to a really large value. */
  apiRetStatus = CyU3PUartTxSetBlockXfer (0xFFFFFFFFu);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    error_handler_0(apiRetStatus, CyFalse);
  }

  /* Initialize the debug module. */
  apiRetStatus = CyU3PDebugInit (CY_U3P_LPP_SOCKET_UART_CONS, 8);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    error_handler_0(apiRetStatus, CyFalse);
  }
  CyU3PDebugPreamble(CyFalse);
  CyU3PDebugEnable(0xFFFF);

  log_debug ( "Debugger Init.. should be working now.\n" );
}

#ifdef ENABLE_LOGGING
#ifdef USB_LOGGING

#include "rdwr.h"

CyBool_t glLogBoot=CyFalse;
uint8_t log_buffer[LOG_BUFFER_SIZE];
uint16_t glLogSize=0;
CyU3PMutex log_mutex;

void logging_boot() {
  if (!glLogBoot) {
      CyU3PMutexCreate(&log_mutex, CYU3P_NO_INHERIT);
      glLogBoot=CyTrue;
  }
}

uint16_t log_read(CyU3PDmaBuffer_t* buf) {
  CyU3PMutexGet(&log_mutex,0);
  uint16_t ret=0;
  uint16_t len = glLogSize > buf->count ? buf->count : glLogSize;
  switch (gRdwrCmd.header.reg_addr) {
    case 0:
      if (buf->count != 2) { ret=1; break;};
      CyU3PMemCopy(buf->buffer,(uint8_t*)&glLogSize,2);
      break;
    default:
      CyU3PMemCopy(buf->buffer,log_buffer,len);

      if (glLogSize>len) {
         uint16_t s=glLogSize-len;
         uint8_t* dst=log_buffer;
         uint8_t* src=log_buffer+len;
         while (s--) {
           *dst++ = *src++;
         }
      }
      glLogSize -= len;
  }

  CyU3PMutexPut(&log_mutex);
  return ret;
}

void log_stmt2(unsigned LEVEL, const uint8_t* stmt) {

  CyU3PMutexGet(&log_mutex,0);

    unsigned log_size = LOG_BUFFER_SIZE-glLogSize;
    uint16_t len=0;
    const uint8_t* p=stmt;
    while (*p++) ++len;
    len+=2; // one for level one for null
    if (len<=log_size) {
      log_buffer[glLogSize]=LEVEL;
      CyU3PMemCopy(log_buffer+glLogSize+1, stmt, len-1);
      glLogSize += len;
    }

  CyU3PMutexPut(&log_mutex);
}


#else


/* This function initializes the debug module. The debug prints
 * are routed to the UART and can be seen using a UART console
 * running at 115200 baud rate. */
void logging_boot () {
  init_uart_debug();
}

#endif


#endif
