/**
 * Copyright (C) 2013 BrooksEE, LLC.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 **/


#ifndef HANDLERS_H
#define HANDLERS_H
#include <cyu3system.h>
#include "vendor_commands.h"


/** API for boot/init callbacks **/
// called once at boot.
typedef void (*app_boot)(void);
// called each time the application starts
typedef void (*app_start)(void);
// called each time the applications stops
typedef void (*app_stop)(void);
// called every loop in the main thread 
// only if rdwr is done. (allows check for custom buffers etc)
typedef void (*app_main_loop_cb)(void);

typedef enum {
  APP_INIT_TERMINATOR = 0,
  APP_INIT_VALID = 1
} app_init_type;

typedef struct {
  app_init_type type;
  app_boot boot;
  app_start start;
  app_stop stop;
  app_main_loop_cb main_loop_cb;
} app_init_t;

extern app_init_t app_init[];

/**
 * API for IO handlers.
 *
 **/


/**
 * The RDWR vendor command populates the rdwr_data struct and calls an optional
 * init function for each terminal.  Handlers are in charge of the term_addr
 * and reg_addr of the rdwr_data structure.  A handler may increment the
 * reg_addr each time a byte is written for instance.
 **/

/**
 * boot handlers: void boot(uint16_t term_addr);
 * boot functions from the io_handlers struct are iterated at boot time.  If a
 * handler needs to do some initialazion 1 time, this is the function to use.  
 * It can be NULL otherwise.  Use the io_handler_init_func to initialize a
 * device for reads/writes at the beginning of each transaction.
 **/
typedef void (*io_handler_boot_func)(uint16_t);

/**
 *
 * READ handlers: uint16_t read_handler(CyU3PDmaBuffer_t *buf_p);
 *  Implements the read function of the handler. Puts data in in buf_p.
 *  Handlers should be capable of waiting forever.
 *
 *  Leave null to have this return dummy data.
 **/
typedef uint16_t (*io_handler_read_func)(CyU3PDmaBuffer_t *);

/**
 * WRITE handlers: uint16_t write_handler(CyU3PDmaBuffer_t *buf_p):
 *  The number of bytes available in the write buffer (EP2FIFOBUF) are stored in
 *  rdwr_data.bytes_avail.  The write_handler must handle the available bytes
 *  without blocking. The handler can return FALSE instead of TRUE to allow
 *  another pass at the same data if the write call would block.
 *  
 *  \return uint16_t: status code that gets OR'd with all other statuses
 *      and returned as part of the ack packet. 0 means success.
 * 
 *  Leave null to write dummy data.
 **/
typedef uint16_t (*io_handler_write_func)(CyU3PDmaBuffer_t *);


/**
 * Optional code to be run before a read/write transaction.  Function can be
 * NULL in read/write structure.  If this function is defined for a handler,
 * and it returns false, the RDWR setup will fail and the the data transfer
 * is not attempted.
 *
 * Example: 
 *  case: init handler is connected to device that is not configured properly
 *  result: return false to data read/write is not attempted to device.  
 **/
typedef uint16_t (*io_handler_init_func)();

/**
 * Optional code to run after a read/write transaction.  This function should
 * release any resources or if changes were made to the chip that could 
 * affect operation of another terminal, those changes should be reverted.
 *
 * Function is not actually called until right before the next rdwr transaction
 * is set to begin.
 *
 * Example:
 *  case: FPGA terminal puts IFCONFIG into slave fifo mode.
 *  result: this function should put it back out of SLAVE FIFO mode.
 **/
typedef void (*io_handler_uninit_func)();

/**
 * After a transaction is finished, the ack is sent back to the host.  This
 * function takes a pointer to checksum and status.  The function can be NULL
 * In the read/write structure.  Status must be 0 for a successful transaction
 * and checksum is the 16 bit sum (ignoring carry) of all bytes transferred.  A
 * NULL ack function causes a default ack of status/checksum both 0 to be sent.
 * 0 checksum only causes drivers to fail if the host is verifying checksums.
 **/
typedef uint16_t (*io_handler_status_func)();

/**
 * Same as io_handler_status.  Function can be NULL;
 **/
typedef uint16_t (*io_handler_chksum_func)();


/*
    Each io handler (below) must fall into a
    generic handler type.  The first field
    of the io hanlder must be set to the type.
 */
typedef uint16_t (*handler_setup_func)(); // setup for this handler 
typedef void (*handler_teardown_func)(); // teardown for this handler nullable
typedef uint16_t (*handler_start_func)(); //  nullable
typedef uint16_t (*handler_dma_cb_func)(); // call from data thread (nullable) return 0 to call in data loop
typedef CyBool_t (*handler_filter_func)(uint16_t);  // nullable if filter is non-null, allows filter of terminal address (more than one)

typedef struct {
    handler_setup_func handler_setup;
    handler_teardown_func handler_teardown;
    handler_start_func handler_start;
    handler_dma_cb_func handler_dma_cb;
    handler_filter_func handler_filter;
} handler_t;

// fx3 pre-provided handlers
extern handler_t glCpuHandler;
//extern handler_t glSlaveFifoHandler;
#ifdef FIRMWARE_DI
extern handler_t glFirmwareDIHandler;
#endif


/**
 *  rdwr vendor command handles a NULL-terminated array of these structures 
 *  when setting up the read/write transfer.
 **/
typedef struct {
  handler_t *handler;  // NULL for terminator
  uint16_t term_addr;
  io_handler_boot_func boot_handler; // once at boot
  io_handler_init_func init_handler; // before every transaction
  io_handler_read_func read_handler;
  io_handler_write_func write_handler;
  io_handler_status_func status_handler;
  io_handler_chksum_func chksum_handler;
  io_handler_uninit_func uninit_handler;
  void *userdata;
} io_handler_t;

/**
 * Individual firmare must define an array of 
 * handlers for the firmware to use when rdwr requests are received.
 **/
extern io_handler_t io_handlers[];

/**
 * For ease, this macro can help add handlers.  This macro 
 * is meant to be reused by handler macro definitions, easing creation of the io_handlers array. 
 *
 * Example:
 *  In some_handler.h:
 *      #define DECLARE_SOMEHANDLER(term) DECLARE_HANDLER(term, somehandler_boot, somehandler_init... etc)
 *  In your handlers.c file:
 *      io_handler code io_handlers[] = {
 *          DECLARE_SOMEHANDLER,
 *          DECLARE_ANOTHERHANDLER,... };
 **/
#define DECLARE_HANDLER(handler, term, boot_func, init_func, read_func, write_func, status_func, chksum_func, uninit_func, user_data) \
  {handler, term, boot_func, init_func, read_func, write_func, status_func, chksum_func, uninit_func, (void *) user_data} 

/**
 *  io_handlers must be null terminated.  If you don't have an address 0 terminator
 *  as the last handler, you can use the NULL handler
 **/
#define DECLARE_TERMINATOR \
  DECLARE_HANDLER(0,0,0,0,0,0,0,0,0,0)

#define DECLARE_DUMMY_HANDLER(TERM_ADDR) \
  DECLARE_HANDLER(&glCpuHandler,TERM_ADDR,0,0,0,0,0,0,0,0)

#endif

