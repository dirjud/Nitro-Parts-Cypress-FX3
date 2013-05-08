#ifndef __RDWR_H__
#define __RDWR_H__

#include "cyu3system.h"
#include "handlers.h"

extern io_handler_t io_handlers[];

typedef struct {
  rdwr_data_header_t header;   // current command
  io_handler_t *handler;       // current io_handler
  uint16_t ep_buffer_size;     // usb end point buffer size
  uint8_t done;                // has this command been handled?
  uint32_t transfered_so_far;  // used by handler to know how much data it has transfered so far
} rdwr_cmd_t;

CyBool_t handle_vendor_cmd(uint8_t  bRequest, uint8_t bReqType,
			   uint8_t  bType, uint8_t bTarget,
			   uint16_t wValue, uint16_t wIndex, 
			   uint16_t wLength);

void rdwr_teardown();

#endif
