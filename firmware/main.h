/*
 ## Cypress USB 3.0 Platform header file (cyfxbulklpautoenum.h)
 ## ===========================
 ##
 ##  Copyright Cypress Semiconductor Corporation, 2010-2011,
 ##  All Rights Reserved
 ##  UNPUBLISHED, LICENSED SOFTWARE.
 ##
 ##  CONFIDENTIAL AND PROPRIETARY INFORMATION
 ##  WHICH IS THE PROPERTY OF CYPRESS.
 ##
 ##  Use of this file is governed
 ##  by the license agreement included in the file
 ##
 ##     <install>/license/license.txt
 ##
 ##  where <install> is the Cypress software
 ##  installation root directory path.
 ##
 ## ===========================
*/

/* This file contains the externants used by the bulk loop application example */

#ifndef _INCLUDED_MAIN_H_
#define _INCLUDED_MAIN_H_

#include "cyu3types.h"
#include "cyu3usbconst.h"
#include "cyu3os.h"
#include "cyu3externcstart.h"

//#define CY_FX_NITRO_DMA_BUF_COUNT      (8)                       /* Bulk loop channel buffer count */
//#define CY_FX_NITRO_DMA_TX_SIZE        (0)                       /* DMA transfer size is set to infinite */

// NOTE
// if you find the dma channels failing to be created with error 16
// (out of memory), you probably forgot to modify the cyfxtx.c in the sdk
// to change the memory top so that the last 32k of memory reserved for
// boot os can be used to allocate buffer space.
// Restated... you must modify the fx3 sdk to get a usable firmware.

#define CY_FX_EP_BURST_LENGTH          (8)                      /* max burst length */
#define CY_FX_EP_BUF_COUNT             (2)                       /* num ep buffers */
#define CY_FX_DMA_SIZE_MULTIPLIER   (2)                          /* double buffer size to decrease latency */
#define CY_FX_NITRO_THREAD_STACK       (0x1000)                  /* Bulk loop application thread stack size */
#define CY_FX_NITRO_THREAD_PRIORITY    (8)                       /* Bulk loop application thread priority */

/* Endpoint and socket definitions for the bulkloop application */

/* To change the producer and consumer EP enter the appropriate EP numbers for the #defines.
 * In the case of IN endpoints enter EP number along with the direction bit.
 * For eg. EP 6 IN endpoint is 0x86
 *     and EP 6 OUT endpoint is 0x06.
 * To change sockets mention the appropriate socket number in the #defines. */

/* Note: For USB 2.0 the endpoints and corresponding sockets are one-to-one mapped
         i.e. EP 1 is mapped to UIB socket 1 and EP 2 to socket 2 so on */

#define CY_FX_EP_PRODUCER               0x01    /* EP 1 OUT */
#define CY_FX_EP_CONSUMER               0x81    /* EP 1 IN */

#define CY_FX_EP_PRODUCER_SOCKET        CY_U3P_UIB_SOCKET_PROD_1    /* Socket 1 is producer */
#define CY_FX_EP_CONSUMER_SOCKET        CY_U3P_UIB_SOCKET_CONS_1    /* Socket 1 is consumer */
/* Used with FX3 Silicon. */
#define CY_FX_PRODUCER_PPORT_SOCKET    CY_U3P_PIB_SOCKET_0    /* P-port Socket 0 is producer */
#define CY_FX_CONSUMER_PPORT_SOCKET    CY_U3P_PIB_SOCKET_3    /* P-port Socket 3 is consumer */
#define CY_CPU_CONSUMER_PPORT_SOCKET   CY_U3P_PIB_SOCKET_2    /* P-port Socket 2 is consumer */

/* Extern definitions for the USB Descriptors */
extern const uint8_t CyFxUSB20DeviceDscr[];
extern const uint8_t CyFxUSB30DeviceDscr[];
extern const uint8_t CyFxUSBDeviceQualDscr[];
extern const uint8_t* CyFxUSBFSConfigDscr[];
extern const uint8_t* CyFxUSBHSConfigDscr[];
extern const uint8_t CyFxUSBBOSDscr[];
extern const uint8_t* CyFxUSBSSConfigDscr[];
extern const uint8_t* CyFxUSBStringPtrs[];

void CyFxAppErrorHandler (CyU3PReturnStatus_t apiRetStatus);

extern CyU3PEvent glThreadEvent;       /* event to cause app thread to wake up */
#define NITRO_EVENT_VENDOR_CMD  (1<<0) /* mask for vendor commands */
#define NITRO_EVENT_DATA        (1<<1) /* DI transaction started. */
#define NITRO_EVENT_BREAK        (1<<2) /* break the main loop */
#define NITRO_EVENT_REBOOT       (1<<3) /* reboot the firmware */
#define NITRO_EVENT_USB2         (1<<4) /* glSSInit changed */ 

extern uint8_t glUsbConfiguration;
extern CyBool_t glIsApplnActive;

#include "cyu3externcend.h"

#endif /* _INCLUDED_MAIN_H_ */

/*[]*/
