#ifndef __CPU_HANDLER_H__
#define __CPU_HANDLER_H__

#include <cyu3error.h>

/* Multiplication factor used when allocating DMA buffers to reduce
   DMA callback frequency. */
#define CY_FX_DMA_SIZE_MULTIPLIER      (2)


void cpu_handler_read();
void cpu_handler_write();
/* Called at the start of any newly received cpu handler. */
void cpu_handler_cmd_start();
void cpu_handler_cmd_end();

/* This function sets up the DMA channels to pipe data to and from the
 * CPU so that cpu handlers can deals with it. */
CyU3PReturnStatus_t cpu_handler_setup(void);

/* This function tears down the DMA channels setup for CPU type handlers. */
void cpu_handler_teardown(void);

#endif

