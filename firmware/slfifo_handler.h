#ifndef __SLFIFO_HANDLER_H__
#define __SLFIFO_HANDLER_H__

#include <cyu3error.h>

/* Called at the start of any newly received cpu handler. */
typedef struct {
  uint16_t cmd;
  uint16_t buffer_length;
  uint16_t term_addr;
  uint16_t reserved;
  uint32_t reg_addr;
  uint32_t transfer_length;
} slfifo_cmd_t;


#define CY_FX_SLFIFO_DMA_BUF_COUNT      (4) /* Bulk channel buffer count */
//#define CY_FX_BULKSRCSINK_DMA_TX_SIZE        (0) /* DMA transfr size infinite */

/* Multiplication factor used when allocating DMA buffers to reduce
   DMA callback frequency. */
//#define CY_FX_DMA_SIZE_MULTIPLIER      (2)


/* Called at the start of any newly received cpu handler. */
void slfifo_cmd_start();

/* This function sets up the DMA channels to pipe data to and from the
 * CPU so that cpu handlers can deals with it. */
CyU3PReturnStatus_t slfifo_setup(CyBool_t useAutoCommit);

/* This function tears down the DMA channels setup for CPU type handlers. */
void slfifo_teardown(void);

void slfifo_init(void);

#endif

