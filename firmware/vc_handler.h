#ifndef __VC_HANDLER_H__
#define __VC_HANDLER_H__

#include "cyu3system.h"
#include "handlers.h"
CyBool_t handle_vendor_cmd(uint8_t  bRequest, uint8_t bReqType,
			   uint8_t  bType, uint8_t bTarget,
			   uint16_t wValue, uint16_t wIndex, 
			   uint16_t wLength);
void initHandlers();
void registerHandler(io_handler_t *handler);
void registerDefaultHandler(io_handler_t *handler);
void stopEventHandler();
#endif
