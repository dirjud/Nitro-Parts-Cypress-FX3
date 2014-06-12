/**
 * Low level DI support.  
 * reads and writes from slfifo terminals.
 * FX3 terminals simply expose an internal api for that terminal
 * and call that directly.
 */
#ifndef FIRMWARE_DI_H
#define FIRMWARE_DI_H

#include <cyu3system.h>

#include "handlers.h"

uint16_t di_get ( uint16_t term, uint32_t addr, uint32_t *val );
uint16_t di_set ( uint16_t term, uint32_t addr, uint32_t val );
uint16_t di_read ( uint16_t term, uint32_t addr, uint8_t *buf, uint32_t len );
uint16_t di_write ( uint16_t term, uint32_t addr, uint8_t *buf, uint32_t len );

extern io_handler_t firmware_di_handler;

uint16_t fdi_setup();
void fdi_teardown();
uint16_t fdi_handler_dmacb();

#endif
