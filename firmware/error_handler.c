/* Application Error Handler */
#include "error_handler.h"


#include <cyu3os.h>
#include <cyu3error.h>
#include <cyu3uart.h>
#include <cyu3spi.h>
#include <cyu3i2c.h>

#ifndef ENABLE_LOGGING
// logging always enabled for error handler
#define ENABLE_LOGGING
#define RECONFIG_IO_MATRIX
#endif
#include "log.h"

void error_handler_0(CyU3PReturnStatus_t apiRetStatus, CyBool_t init) {
  /* Application failed with the error code apiRetStatus */
  /* Add custom debug or recovery actions here */
  /* Loop Indefinitely */

#ifdef RECONFIG_IO_MATRIX
// io matrix didn't initialize so try again w/ uart enabled
// for debugging purpose
if (init) {
  CyU3PIoMatrixConfig_t io_cfg;
  CyU3PMemSet((uint8_t*)&io_cfg,0,sizeof(io_cfg));
  io_cfg.isDQ32Bit = CyTrue;
  io_cfg.useUart = CyTrue;

  CyU3PUartDeInit();
  CyU3PI2cDeInit();
  CyU3PSpiDeInit();
  CyU3PDeviceConfigureIOMatrix (&io_cfg);
  init_uart_debug();
}

#endif
  for (;;) {
    log_error("The app is stuck in the error_handler: %d\n", apiRetStatus );
    /* Thread sleep : 100 ms */
    CyU3PThreadSleep (1000);
  }
}

void error_handler(CyU3PReturnStatus_t apiRetStatus) {
  error_handler_0(apiRetStatus, CyTrue);
}
