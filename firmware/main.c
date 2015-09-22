/*
 ##  Copyright BrooksEE, LLC, 2013,
*/

#include "cyu3system.h"
#include "cyu3dma.h"
#include "cyu3error.h"
#include "cyu3i2c.h"
#include "main.h"
#include "cyu3usb.h"
#include "cyu3uart.h"
#include "rdwr.h"
#include "error_handler.h"
#include "cyu3gpio.h"
#include "slfifo_handler.h"
#include "cpu_handler.h"
#ifdef FIRMWARE_DI
#include "di.h" // fdi handler
#endif

#include "log.h"
#ifndef DEBUG_MAIN
#undef log_debug
#define log_debug(x,...) do {} while(0)
#endif


CyU3PThread NitroAppThread; /* Nitro application thread structure */
CyU3PThread NitroDataThread;
#ifdef FIRMWARE_DI
CyU3PThread NitroDIThread;
#endif

CyU3PEvent glThreadEvent;              /* event to cause app thread to wake up */

uint8_t glEp0Buffer[32] __attribute__ ((aligned (32))); /* Buffer used for sending EP0 data.    */
uint32_t glSetupDat0, glSetupDat1;      /* for handling vendor commands on app thread */


uint8_t glUsbDeviceStat = 0;            /* USB device status. Bus powered.      */
uint8_t glUsbConfiguration = 0;         /* Active USB configuration.            */
uint8_t glUsbInterface = 0;             /* Active USB interface.                */
uint8_t *glSelBuffer = 0;               /* Buffer to hold SEL values.           */

CyBool_t glIsApplnActive = CyFalse;     /* Whether the loopback application is active or not. */
extern rdwr_cmd_t gRdwrCmd;

void init_i2c() {
  CyU3PI2cConfig_t i2cConfig;
  CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

  /* Initialize and configure the I2C master module. */
  status = CyU3PI2cInit ();
  if (status != CY_U3P_SUCCESS) {
    log_error("Error initializing I2C: %d\n", status);
    return;
  }

  /* Start the I2C master block. The bit rate is set at 400KHz.
   * The data transfer is done via DMA. */
  CyU3PMemSet ((uint8_t *)&i2cConfig, 0, sizeof(i2cConfig));
  i2cConfig.bitRate    = 400000;  // 400KHz
  i2cConfig.busTimeout = 0xFFFFFFFF; // no timeout
  i2cConfig.dmaTimeout = 0xFFFF;
  i2cConfig.isDma      = CyFalse; // not a dma
  
  status = CyU3PI2cSetConfig (&i2cConfig, NULL);
  if (status != CY_U3P_SUCCESS) {
    log_error("Error setting I2C config: %d\n", status);
    return;
  }
}

// NOTE api doc sys no debug logging and nothing else that 
// would block in this interrupt.
void gpio_interrupt (uint8_t gpioId /* Indicates the pin that triggered the interrupt */ ) {

}

void init_gpio (void) {
    CyU3PGpioClock_t gpioClock;
    CyU3PGpioSimpleConfig_t gpioConfig;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    /* Init the GPIO module */
    gpioClock.fastClkDiv = 2;
    gpioClock.slowClkDiv = 0;
    gpioClock.simpleDiv = CY_U3P_GPIO_SIMPLE_DIV_BY_2;
    gpioClock.clkSrc = CY_U3P_SYS_CLK;
    gpioClock.halfDiv = 0;

    apiRetStatus = CyU3PGpioInit(&gpioClock, gpio_interrupt);
    if (apiRetStatus != 0) {
        /* Error Handling */
      log_error("CyU3PGpioInit failed, error code = %d\n", apiRetStatus);
      error_handler(apiRetStatus);
    }

//    /* Configure GPIO 45 as input with interrupt enabled for both edges */
//    gpioConfig.outValue = CyTrue;
//    gpioConfig.inputEn = CyTrue;
//    gpioConfig.driveLowEn = CyFalse;
//    gpioConfig.driveHighEn = CyFalse;
//    gpioConfig.intrMode = CY_U3P_GPIO_INTR_BOTH_EDGE;
//    apiRetStatus = CyU3PGpioSetSimpleConfig(45, &gpioConfig);
//    if (apiRetStatus != CY_U3P_SUCCESS)
//    {
//        /* Error handling */
//        CyU3PDebugPrint (4, "CyU3PGpioSetSimpleConfig failed, error code = %d\n",
//                apiRetStatus);
//        CyFxAppErrorHandler(apiRetStatus);
//    }

    /* Override GPIO 23 as this pin is associated with GPIF Control signal.
     * The IO cannot be selected as GPIO by CyU3PDeviceConfigureIOMatrix call
     * as it is part of the GPIF IOs. Override API call must be made with
     * caution as this will change the functionality of the pin. If the IO
     * line is used as part of GPIF and is connected to some external device,
     * then the line will no longer behave as a GPIF IO.. Here CTL4 line is
     * not used and so it is safe to override.  */

// NOTE check if this is really needed

/*    apiRetStatus = CyU3PDeviceGpioOverride (23, CyTrue);*/
/*    if (apiRetStatus != 0) {*/
/*        / * Error Handling * / */
/*      log_error("CyU3PDeviceGpioOverride failed, code = %d\n", apiRetStatus);*/
/*      error_handler(apiRetStatus);*/
/*    } */

    /* Configure GPIO 23 as output */
    gpioConfig.outValue    = CyTrue;
    gpioConfig.driveLowEn  = CyTrue;
    gpioConfig.driveHighEn = CyTrue;
    gpioConfig.inputEn     = CyFalse;
    gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
    apiRetStatus = CyU3PGpioSetSimpleConfig(23, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS) {
      /* Error handling */
      log_error("CyU3PGpioSetSimpleConfig(23) failed, code = %d\n", apiRetStatus);
      error_handler(apiRetStatus);
    }
    
    // hold prog_b down at app start
    gpioConfig.outValue=CyFalse;
    apiRetStatus = CyU3PGpioSetSimpleConfig(57, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS) {
      /* Error handling */
      log_error("CyU3PGpioSetSimpleConfig(57) failed, code = %d\n", apiRetStatus);
      error_handler(apiRetStatus);
    }
    
    //// drive VCON_EN low 
    gpioConfig.outValue = CyFalse;
    apiRetStatus = CyU3PGpioSetSimpleConfig(22, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS) {
      /* Error handling */
      log_error("CyU3PGpioSetSimpleConfig(22) failed, code = %d\n", apiRetStatus);
      error_handler(apiRetStatus);
    }

    // TODO - provide customizability in handlers?
    // drive LP_B high
    gpioConfig.outValue = CyTrue;
    apiRetStatus = CyU3PGpioSetSimpleConfig(26, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS) {
      /* Error handling */
      log_error("CyU3PGpioSetSimpleConfig(26) failed, code = %d\n", apiRetStatus);
      error_handler(apiRetStatus);
    }
   
    // drive V18_EN high
    gpioConfig.outValue = CyTrue;
    apiRetStatus = CyU3PGpioSetSimpleConfig(27, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS) {
      /* Error handling */
      log_error("CyU3PGpioSetSimpleConfig(27) failed, code = %d\n", apiRetStatus);
      error_handler(apiRetStatus);
    }
    

}



/* This function initializes the debug module. The debug prints
 * are routed to the UART and can be seen using a UART console
 * running at 115200 baud rate. */
void CyFxNitroApplnDebugInit (void) {
  CyU3PUartConfig_t uartConfig;
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

  /* Initialize the UART for printing debug messages */
  apiRetStatus = CyU3PUartInit();
  if (apiRetStatus != CY_U3P_SUCCESS) {
    /* Error handling */
    error_handler(apiRetStatus);
  }

  /* Set UART configuration */
  CyU3PMemSet ((uint8_t *)&uartConfig, 0, sizeof (uartConfig));
  uartConfig.baudRate = CY_U3P_UART_BAUDRATE_115200;
  uartConfig.stopBit = CY_U3P_UART_ONE_STOP_BIT;
  uartConfig.parity = CY_U3P_UART_NO_PARITY;
  uartConfig.txEnable = CyTrue;
  uartConfig.rxEnable = CyFalse;
  uartConfig.flowCtrl = CyFalse;
  uartConfig.isDma = CyTrue;
  
  apiRetStatus = CyU3PUartSetConfig (&uartConfig, NULL);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    error_handler(apiRetStatus);
  }

  /* Set the UART transfer to a really large value. */
  apiRetStatus = CyU3PUartTxSetBlockXfer (0xFFFFFFFFu);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    error_handler(apiRetStatus);
  }

  /* Initialize the debug module. */
  apiRetStatus = CyU3PDebugInit (CY_U3P_LPP_SOCKET_UART_CONS, 8);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    error_handler(apiRetStatus);
  }
  CyU3PDebugPreamble(CyFalse);
  CyU3PDebugEnable(0xFFFF);

  log_debug ( "Debugger Init.. should be working now.\n" );
}

/* This function starts the nitro application. This is called when a
 * SET_CONF event is received from the USB host. The endpoints are
 * configured in this function. */
void CyFxNitroApplnStart (void) {
  CyU3PEpConfig_t epCfg;
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
  CyU3PUSBSpeed_t usbSpeed = CyU3PUsbGetSpeed();
  int i;
  log_debug("Entering CyFxNitroApplnStart()\n");

  CyU3PUsbLPMDisable(); // no low power state for us.

  /* Based on the Bus Speed configure the endpoint packet size */
  log_debug("UsbSpeed: %d\n", usbSpeed);
  switch (usbSpeed) {
  case CY_U3P_FULL_SPEED:
    gRdwrCmd.ep_buffer_size = 64;
    break;

  case CY_U3P_HIGH_SPEED:
    gRdwrCmd.ep_buffer_size = 512;
    break;

  default: // NOTE if di_main starts up app then usb speed is invalid.
           // the value is ignored anyway.
  case  CY_U3P_SUPER_SPEED:
    gRdwrCmd.ep_buffer_size = 1024;
    break;
  }

  CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof (epCfg));
  epCfg.enable = CyTrue;
  epCfg.epType = CY_U3P_USB_EP_BULK;
  epCfg.burstLen = 1;
  epCfg.streams = 0;
  epCfg.pcktSize = gRdwrCmd.ep_buffer_size;
  
  /* Producer endpoint configuration */
  apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_PRODUCER, &epCfg);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    log_error("CyU3PSetEpConfig failed, Error code = %d\n", apiRetStatus);
    error_handler(apiRetStatus);
  }

  /* Consumer endpoint configuration */
  apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_CONSUMER, &epCfg);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    log_error("CyU3PSetEpConfig failed, Error code = %d\n", apiRetStatus);
    error_handler (apiRetStatus);
  }

  /* Flush the Endpoint memory */
  CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);
  CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);

  /* Update the status flag. */
  glIsApplnActive = CyTrue;

  /* Drop current U1/U2 enable state values. */
  glUsbDeviceStat = 0;


  i=0;
  while ( app_init[i].type ) {
    if (app_init[i].start) app_init[i].start();
    ++i;
  }

  log_info("CyFxNitroApplnStart() Complete\n");

}

/* This function stops the nitro application. This shall be called
 * whenever a RESET or DISCONNECT event is received from the USB
 * host. The endpoints are disabled and event handler is stopped by
 * this function. */
void CyFxNitroApplnStop (void) {
  CyU3PEpConfig_t epCfg;
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
  int i;
    
  log_debug("Entering CyFxNitroApplnStop()\n");

  /* Update the flag. */
  glIsApplnActive = CyFalse;
  gRdwrCmd.done=1; // just in case

  // clean up DMA channels and anything left by current event handler
  rdwr_teardown(); 

  /* Flush the endpoint memory */
  CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);
  CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);

  /* Disable endpoints. */
  CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof (epCfg));
  epCfg.enable = CyFalse;

  /* Producer endpoint configuration. */
  apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_PRODUCER, &epCfg);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    log_error("CyU3PSetEpConfig failed, Error code = %d\n", apiRetStatus);
    error_handler (apiRetStatus);
  }

  /* Consumer endpoint configuration. */
  apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_CONSUMER, &epCfg);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    log_error("CyU3PSetEpConfig failed, Error code = %d\n", apiRetStatus);
    error_handler (apiRetStatus);
  }
  i=0;
  while ( app_init[i].type ) {
    if (app_init[i].stop) app_init[i].stop();
    ++i;
  }
  log_debug ( "Exit ApplnStop\n"  );
}

/* Handle the CLEAR_FEATURE request. */
CyBool_t
CyFxUsbHandleClearFeature (
        uint8_t bTarget,
        uint16_t wValue,
        uint16_t wIndex)
{
    CyU3PUSBSpeed_t usbSpeed = CyU3PUsbGetSpeed();
    CyBool_t isHandled = CyFalse;

    if (bTarget == CY_U3P_USB_TARGET_DEVICE)
    {
        switch (wValue & 0xFF)
        {
            case CY_U3P_USB2_FS_REMOTE_WAKE:
                glUsbDeviceStat &= 0xFD;
                /* Fall-through here. */
            case CY_U3P_USB2_FS_TEST_MODE:
                if (usbSpeed != CY_U3P_SUPER_SPEED)
                {
                    isHandled = CyTrue;
                }
                break;

            case CY_U3P_USB3_FS_U1_ENABLE:
                if ((usbSpeed == CY_U3P_SUPER_SPEED) && (glIsApplnActive))
                {
                    glUsbDeviceStat &= 0xFB;
                    isHandled = CyTrue;
                }
                break;

            case CY_U3P_USB3_FS_U2_ENABLE:
                if ((usbSpeed == CY_U3P_SUPER_SPEED) && (glIsApplnActive))
                {
                    glUsbDeviceStat &= 0xF7;
                    isHandled = CyTrue;
                }
                break;

            default:
                isHandled = CyFalse;
                break;
        }
    }
    else if (bTarget == CY_U3P_USB_TARGET_INTF)
    {
        /* Need to handle CLEAR_FEATURE(FUNCTION_SUSPEND) requests. We allow this request to pass
           if the USB device is in configured state and fail it otherwise.  */
        if ((glIsApplnActive) && (wValue == 0))
            CyU3PUsbAckSetup ();
        else
            CyU3PUsbStall (0, CyTrue, CyFalse);

        isHandled = CyTrue;
    }
    else if ((bTarget == CY_U3P_USB_TARGET_ENDPT) &&
            (wValue == CY_U3P_USBX_FS_EP_HALT))
    {
        /* If the application is active make sure that all sockets and
         * DMA buffers are flushed and cleaned. If more than one enpoint
         * is linked to the same DMA channel, then reset all the affected
         * endpoint pipes. */
        if (glIsApplnActive)
        {
            log_debug ( "CLEAR EP - rdwr_teardown\n" );
            rdwr_teardown(); // will be all flushed for new transactions
        }
 
        /* Clear stall on the endpoint. */
        CyU3PUsbStall (wIndex, CyFalse, CyTrue);
        isHandled = CyTrue;
    }
    else
    {
        isHandled = CyFalse;
    }

    return isHandled;
}
/* Handle the SET_FEATURE request. */
CyBool_t
CyFxUsbHandleSetFeature (
        uint8_t bTarget,
        uint16_t wValue,
        uint16_t wIndex)
{
    CyU3PUSBSpeed_t usbSpeed = CyU3PUsbGetSpeed();
    CyBool_t isHandled = CyFalse;

    if (bTarget == CY_U3P_USB_TARGET_DEVICE)
    {
        switch (wValue & 0xFF)
        {
            case CY_U3P_USB2_FS_REMOTE_WAKE:
                glUsbDeviceStat |= 0x02;
                /* Fall-through here. */
            case CY_U3P_USB2_FS_TEST_MODE:
                if (usbSpeed != CY_U3P_SUPER_SPEED)
                {
                    isHandled = CyTrue;
                }
                break;

            case CY_U3P_USB3_FS_U1_ENABLE:
                if ((usbSpeed == CY_U3P_SUPER_SPEED) && (glIsApplnActive))
                {
                    glUsbDeviceStat |= 0x04;
                    isHandled = CyTrue;
                }
                break;

            case CY_U3P_USB3_FS_U2_ENABLE:
                if ((usbSpeed == CY_U3P_SUPER_SPEED) && (glIsApplnActive))
                {
                    glUsbDeviceStat |= 0x08;
                    isHandled = CyTrue;
                }
                break;

            default:
                isHandled = CyFalse;
                break;
        }
    }
    else if (bTarget == CY_U3P_USB_TARGET_INTF)
    {
        /* Need to handle SET_FEATURE(FUNCTION_SUSPEND) requests. We allow this request to pass
           if the USB device is in configured state and fail it otherwise.  */
        if ((glIsApplnActive) && (wValue == 0))
            CyU3PUsbAckSetup ();
        else
            CyU3PUsbStall (0, CyTrue, CyFalse);

        isHandled = CyTrue;
    }
    else if ((bTarget == CY_U3P_USB_TARGET_ENDPT) &&
            (wValue == CY_U3P_USBX_FS_EP_HALT))
    {
        /* Stall the endpoint */
        CyU3PUsbStall (wIndex, CyTrue, CyFalse);
        isHandled = CyTrue;
    }
    else
    {
        isHandled = CyFalse;
    }

    return isHandled;
}

/* Send the requested USB descriptor to the host. */
CyU3PReturnStatus_t
CyFxUsbSendDescriptor (uint16_t wValue, uint16_t wIndex, uint16_t wLength)
{
    uint16_t length = 0, index = 0;
    uint8_t *buffer = NULL;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    CyU3PUSBSpeed_t usbSpeed = CyU3PUsbGetSpeed ();

    /* The descriptor type is in the MS byte and the index is in
     * the LS byte. The index is useful only for string descriptor. */
    switch (wValue >> 8)
    {
        case CY_U3P_USB_DEVICE_DESCR:
            if (usbSpeed == CY_U3P_SUPER_SPEED)
            {
                buffer = (uint8_t *)CyFxUSB30DeviceDscr;
            }
            else
            {
                buffer = (uint8_t *)CyFxUSB20DeviceDscr;
            }
            length = buffer[0];
            break;

        case CY_U3P_BOS_DESCR:
            buffer = (uint8_t *)CyFxUSBBOSDscr;
            length = (buffer[2] | ((uint16_t)buffer[3] << 8));
            break;

        case CY_U3P_USB_DEVQUAL_DESCR:
            buffer = (uint8_t *)CyFxUSBDeviceQualDscr;
            length = buffer[0];
            break;

        case CY_U3P_USB_CONFIG_DESCR:
            if (usbSpeed == CY_U3P_SUPER_SPEED)
            {
                buffer = (uint8_t *)CyFxUSBSSConfigDscr;
            }
            else if (usbSpeed == CY_U3P_HIGH_SPEED)
            {
                buffer = (uint8_t *)CyFxUSBHSConfigDscr;
            }
            else /* CY_U3P_FULL_SPEED */
            {
                buffer = (uint8_t *)CyFxUSBFSConfigDscr;
            }
            buffer[1] = CY_U3P_USB_CONFIG_DESCR;                /* Mark as configuration descriptor. */
            length    = (buffer[2] | ((uint16_t)buffer[3] << 8));
            break;

        case CY_U3P_USB_OTHERSPEED_DESCR:
            if (usbSpeed == CY_U3P_HIGH_SPEED)
            {
                buffer = (uint8_t *)CyFxUSBFSConfigDscr;
            }
            else if (usbSpeed == CY_U3P_FULL_SPEED)
            {
                buffer = (uint8_t *)CyFxUSBHSConfigDscr;
            }
            else
            {
                /* Do nothing. buffer = NULL. */
            }

            if (buffer != NULL)
            {
                buffer[1] = CY_U3P_USB_OTHERSPEED_DESCR;        /* Mark as other speed configuration descriptor. */
                length    = (buffer[2] | ((uint16_t)buffer[3] << 8));
            }
            break;

        case CY_U3P_USB_STRING_DESCR:
            index = wValue & 0xFF;
            buffer = (uint8_t*)CyFxUSBStringPtrs[index];
            length = buffer[0];
            break;

        default:
            /* Do nothing. buffer = NULL. */
            break;
    }

    if (buffer != NULL)
    {
        /* Send only the minimum of actual descriptor length
         * and the requested length. */
        length = (wLength < length) ? wLength : length;
        status = CyU3PUsbSendEP0Data (length, buffer);
    }
    else
    {
        status = CY_U3P_ERROR_FAILURE;
    }

    return status;
}

CyBool_t handle_standard_setup_cmd(uint8_t  bRequest, uint8_t bReqType,
				   uint8_t  bType, uint8_t bTarget,
				   uint16_t wValue, uint16_t wIndex, 
				   uint16_t wLength) {
  CyBool_t isHandled = CyTrue;
  CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

  log_debug( "Entering handle_standard_setup_cmd (%d)\n", bRequest);

  /* Identify and handle setup request. */
  switch (bRequest) {

    /* This is a get status request. The response depends on the target.
     * If this is for the device, then we need to return the status
     * of the device. If this is for the endpoint, then return the stall
     * status of the endpoint. */
  case CY_U3P_USB_SC_GET_STATUS:
    CyU3PMemSet (glEp0Buffer, 0, sizeof(glEp0Buffer));
    if (bTarget == CY_U3P_USB_TARGET_DEVICE) {
      glEp0Buffer[0] = glUsbDeviceStat;
      status = CyU3PUsbSendEP0Data (wLength, glEp0Buffer);
    } else if (bTarget == CY_U3P_USB_TARGET_INTF) {
      /* Just send zeros. */
      status = CyU3PUsbSendEP0Data (wLength, glEp0Buffer);
    } else if (bTarget == CY_U3P_USB_TARGET_ENDPT) {
      CyBool_t isStall;
      status = CyU3PUsbGetEpCfg (wIndex, NULL, &isStall);
      if (status == CY_U3P_SUCCESS) {
	glEp0Buffer[0] = isStall;
	status = CyU3PUsbSendEP0Data (wLength, glEp0Buffer);
      }
    } else {
      isHandled = CyFalse;
    }
    break;

    /* This feature behaves differently depending upon the target.
     * If the target is device then, this request is for remote-wakeup,
     * test mode, U1_ENABLE and U2_ENABLE. If this is for the endpoint
     * then this is for clearing the endpoint stall. */
  case CY_U3P_USB_SC_CLEAR_FEATURE:
    isHandled = CyFxUsbHandleClearFeature (bTarget, wValue, wIndex);
    break;

    /* This feature behaves differently depending upon the target.
     * If the target is device then, this request is for remote-wakeup,
     * test mode, U1_ENABLE and U2_ENABLE. If this is for the endpoint
     * then this is for clearing the endpoint stall. */
  case CY_U3P_USB_SC_SET_FEATURE:
    isHandled = CyFxUsbHandleSetFeature (bTarget, wValue, wIndex);
    break;

    /* Return the requested descriptor. */
  case CY_U3P_USB_SC_GET_DESCRIPTOR:
    status = CyFxUsbSendDescriptor (wValue, wIndex, wLength);
    break;
    
  case CY_U3P_USB_SC_SET_DESCRIPTOR:
    /* ACK the request and do nothing. */
    break;
    
    /* Return the current selected configuration. */
  case CY_U3P_USB_SC_GET_CONFIGURATION:
    glEp0Buffer[0] = glUsbConfiguration;
    status = CyU3PUsbSendEP0Data (wLength, glEp0Buffer);
    break;

    /* Store the value for future use and start the application. */
  case CY_U3P_USB_SC_SET_CONFIGURATION:
    if (wValue == 1) {
      /* If the application is already active, then disable
       * it before re-enabling it. */
      glUsbConfiguration = wValue;
      if (glIsApplnActive) {
	CyFxNitroApplnStop ();
      }
      /* Start the loop back function. */
      CyFxNitroApplnStart ();
    } else {
      if (wValue == 0) {
	/* Stop the loop back function. */
	glUsbConfiguration = wValue;
	if (glIsApplnActive) {
	  CyFxNitroApplnStop ();
	}
      } else {
	/* Invalid configuration value. Fail the request. */
	CyU3PUsbStall (0, CyTrue, CyFalse);
      }
    }
    break;

    /* Return the current selected interface. */
  case CY_U3P_USB_SC_GET_INTERFACE:
    glEp0Buffer[0] = glUsbInterface;
    status = CyU3PUsbSendEP0Data (wLength, glEp0Buffer);
    break;

    /* Store the selected interface value. */
  case CY_U3P_USB_SC_SET_INTERFACE:
    glUsbInterface = wValue;
    break;

  case CY_U3P_USB_SC_SET_SEL:
    {
      if ((CyU3PUsbGetSpeed () == CY_U3P_SUPER_SPEED) && (wValue == 0) && (wIndex == 0) && (wLength == 6)) {
	status = CyU3PUsbGetEP0Data (32, glSelBuffer, 0);
      } else {
	isHandled = CyFalse;
      }
    }
    break;

  case CY_U3P_USB_SC_SET_ISOC_DELAY:
    {
      if ((CyU3PUsbGetSpeed () != CY_U3P_SUPER_SPEED) || (wIndex != 0) || (wLength != 0))
	isHandled = CyFalse;
    }
    break;
        
  default:
    isHandled = CyFalse;
    log_debug ( "Request not handled.\n" );
    break;
  }

  /* If there has been an error, stall EP0 to fail the transaction. */
  if ((isHandled != CyTrue) || (status != CY_U3P_SUCCESS)) {
    /* This is an unhandled setup command. Stall the EP. */
    CyU3PUsbStall (0, CyTrue, CyFalse);
  } else {
    CyU3PUsbAckSetup ();
  }

  log_debug ( "Returning from handle_standard_setup_cmd\n");
  return CyTrue;
}

/* Callback to handle the USB setup requests. */
CyBool_t CyFxNitroApplnUSBSetupCB (
        uint32_t setupdat0, /* SETUP Data 0 */
        uint32_t setupdat1  /* SETUP Data 1 */
				    ) {
    uint8_t  bRequest, bReqType;
    uint8_t  bType, bTarget;
    uint16_t wValue, wIndex, wLength;
    CyBool_t handled=CyFalse;
    
    //CyU3PDebugPrint(LOG_DEBUG, "Entering CyFxNitroApplnUSBSetupCB()\n");
    log_debug ( "Entering CyFxNitroApplnUSBSetupCB()\n");
    

    /* Decode the fields from the setup request. */
    bReqType = (setupdat0 & CY_U3P_USB_REQUEST_TYPE_MASK);
    bType    = (bReqType & CY_U3P_USB_TYPE_MASK);
    bTarget  = (bReqType & CY_U3P_USB_TARGET_MASK);
    bRequest = ((setupdat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS);
    wValue   = ((setupdat0 & CY_U3P_USB_VALUE_MASK)   >> CY_U3P_USB_VALUE_POS);
    wIndex   = ((setupdat1 & CY_U3P_USB_INDEX_MASK)   >> CY_U3P_USB_INDEX_POS);
    wLength  = ((setupdat1 & CY_U3P_USB_LENGTH_MASK)  >> CY_U3P_USB_LENGTH_POS);
    
    //log_debug("Setup Command Received:\n");
    //log_debug("  bReqType : 0x%x\n", bReqType);
    //log_debug("  bType    : 0x%x\n", bType);
    //log_debug("  bTarget  : 0x%x\n", bTarget);
    //log_debug("  bRequest : 0x%x\n", bRequest);
    //log_debug("  wValue   : 0x%x\n", wValue);
    //log_debug("  wIndex   : 0x%x\n", wIndex);
    //log_debug("  wLength  : 0x%x\n", wLength);

    

    switch (bType) {
    case CY_U3P_USB_STANDARD_RQT:
      handled= handle_standard_setup_cmd(bRequest, bReqType, bType, bTarget, wValue, wIndex, wLength);
      break;
    case CY_U3P_USB_VENDOR_RQT:
      handled = CyTrue;
      glSetupDat0=setupdat0;
      glSetupDat1=setupdat1;
      // TODO could we monitor entry/exit of handle event
      // and if there is a new one cancel the existing one?
      // we could do a get first to see if the flag is currently set
      // perhaps for instance
      CyU3PEventSet(&glThreadEvent, NITRO_EVENT_VENDOR_CMD, CYU3P_EVENT_OR);
      break;
    }
    
    log_debug ( "Vendor command handled: %d\n", handled ? 1 : 0);
    return handled;
}

/* This is the callback function to handle the USB events. */
void CyFxNitroApplnUSBEventCB (
    CyU3PUsbEventType_t evtype, /* Event type */
    uint16_t            evdata  /* Event data */
    ) {
  log_debug("Entering CyFxNitroApplnUSBEventCB\n");
//  log_debug("  evtype  : 0x%x\n", evtype);
//  log_debug("  evtdata : 0x%x\n", evdata);
  switch (evtype) {
  case CY_U3P_USB_EVENT_CONNECT:
   log_info ( "USB CONNECT\n" );
   break;
  case CY_U3P_USB_EVENT_RESET:
    log_info( "USB RESET\n" );
    /* Stop the loop back function. */
    {
      uint16_t phy, link;
      if (!CyU3PUsbGetErrorCounts( &phy, &link )) {
         if (phy>0||link>0) {
          log_warn ( "usb phy err=%d link err=%d\n", phy, link );
         }
      }

    }
    CyFxNitroApplnStop ();
    /* Drop current U1/U2 enable state values. */
    glUsbDeviceStat = 0;
    break;
  case CY_U3P_USB_EVENT_SUSPEND:
    log_info("SUSPEND\n" );
    break;
  case CY_U3P_USB_EVENT_VBUS_VALID:
    log_info("VBUS VALID\n");
    break;
  case CY_U3P_USB_EVENT_VBUS_REMOVED:
    log_info( "VBUS POWER REMOVED\n");
    break;    
  case CY_U3P_USB_EVENT_DISCONNECT:
    log_info ( "USB DISCONNECT\n" );
    /* Stop the loop back function. */
    CyFxNitroApplnStop ();
    /* Drop current U1/U2 enable state values. */
    glUsbDeviceStat = 0;
    break;
  case CY_U3P_USB_EVENT_SPEED:
    log_info ( "USB_EVENT_SPEED\n" );
    break;
  case CY_U3P_USB_EVENT_EP0_STAT_CPLT:
    log_debug( "EP0_STAT_CPLT\n" );
    break;
  case CY_U3P_USB_EVENT_EP_UNDERRUN:
    log_warn ( "USB DATA UNDERRUN\n" );
    break;
  default:
    log_info ( "Unhandled %d\n", evtype );
    break;
  }
}

/* Callback function to handle LPM requests from the USB 3.0 host. This function is invoked by the API
   whenever a state change from U0 -> U1 or U0 -> U2 happens. If we return CyTrue from this function, the
   FX3 device is retained in the low power state. If we return CyFalse, the FX3 device immediately tries
   to trigger an exit back to U0.

   This application does not have any state in which we should not allow U1/U2 transitions; and therefore
   the function always return CyTrue.
 */
CyBool_t CyFxNitroApplnLPMRqtCB (CyU3PUsbLinkPowerMode link_mode) {
  return CyTrue;
}

/* This function initializes the USB Module, sets the enumeration descriptors.
 * This function does not start the bulk streaming and this is done only when
 * SET_CONF event is received. */
void CyFxNitroApplnInit (void) {
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
  CyBool_t no_renum = CyFalse;

  log_debug("Entering CyFxNitroApplnInit\n");

  int i=0;
  while ( app_init[i].type != APP_INIT_TERMINATOR ) {
    if (app_init[i].boot) {
       app_init[i].boot();
    }
    ++i;
  }

  i=0;
  while (io_handlers[i].type != HANDLER_TYPE_TERMINATOR) {
    if (io_handlers[i].boot_handler) {
        io_handlers[i].boot_handler(io_handlers[i].term_addr);
    }
    ++i;
  }

  /* Start the USB functionality. */
  apiRetStatus = CyU3PUsbStart();
  if (apiRetStatus == CY_U3P_ERROR_NO_REENUM_REQUIRED) {
    no_renum = CyTrue;
  } else if (apiRetStatus != CY_U3P_SUCCESS) {
    log_error ("CyU3PUsbStart failed to Start, Error code = %d\n", apiRetStatus);
    error_handler(apiRetStatus);
  }

  /* Allocate a buffer to hold the SEL (System Exit Latency) values set by the host. */
  glSelBuffer = (uint8_t *)CyU3PDmaBufferAlloc (32);
  CyU3PMemSet (glSelBuffer, 0, 32);

  /* In this example the application handles all the setup packets.
   * But even in normal mode of enumeration, it is possible to register
   * DMA descriptors with the library and not handle inside the setup
   * callback. */
  CyU3PUsbRegisterSetupCallback(CyFxNitroApplnUSBSetupCB, CyFalse);

  /* Setup the callback to handle the USB events. */
  CyU3PUsbRegisterEventCallback(CyFxNitroApplnUSBEventCB);

  /* Register a callback to handle LPM requests from the USB 3.0 host. */
  CyU3PUsbRegisterLPMRequestCallback(CyFxNitroApplnLPMRqtCB);

  if (!no_renum) {
    /* Connect the USB Pins with super speed operation enabled. */
    apiRetStatus = CyU3PUsbSetTxSwing(127); // per Cypress tech phyerr doc
    log_debug ( "Tx Swing ret: %d\n" , apiRetStatus );
    apiRetStatus = CyU3PConnectState(CyTrue, CyTrue);
    if (apiRetStatus != CY_U3P_SUCCESS) {
      log_error( "USB Connect failed, Error code = %d\n", apiRetStatus);
      error_handler(apiRetStatus);
    }
  } else {
    /* USB connection is already active. Start the app again. */
    if (glIsApplnActive) {
      CyFxNitroApplnStop();
    }
    CyFxNitroApplnStart();
  }
}

/* Entry function for data thread */
void NitroDataThread_Entry (uint32_t input) {
  uint32_t eventStat;
  gRdwrCmd.done = 1; // not in a command
  while (CyTrue) {

    if (!gRdwrCmd.done) {
        if (gRdwrCmd.handler && gRdwrCmd.handler->type == HANDLER_TYPE_CPU) {
            if (!cpu_handler_dmacb())
                continue; // do this in a loop until dma cb puts the command back to done
        }
        #ifdef FIRMWARE_DI
        else if (gRdwrCmd.handler && gRdwrCmd.handler->type == HANDLER_TYPE_FDI) {
            if (!fdi_handler_dmacb())
                continue;
        }
        #endif
        else {
          log_debug ( "nd " );
        }
    }

    // sleep if we're not doing anything else the event breaks the sleep so data 
    // can be handled.
    CyU3PEventGet(&glThreadEvent, NITRO_EVENT_DATA, CYU3P_EVENT_OR_CLEAR, &eventStat, 1000);
  }
}


/* Entry function for the NitroAppThread. */
void NitroAppThread_Entry (uint32_t input) {

  CyU3PReturnStatus_t ret;
  uint32_t eventMask = NITRO_EVENT_VENDOR_CMD|NITRO_EVENT_BREAK; // can add more events
  uint32_t eventStat;

  /* Initialize the debug module */
  CyFxNitroApplnDebugInit();
  init_i2c();
  init_gpio();

  /* Initialize the bulk loop application */
  CyFxNitroApplnInit();


  log_info ( "Nitro Thread Entry\n" );
  
  for (;;) {
     log_debug ( "." );
     slfifo_checkdone();

     if (gRdwrCmd.done) {// ensures main thread mutex unlocked
         RDWR_DONE(CyTrue);
         int i=0;
         while ( app_init[i].type != 0 ) {
           if (app_init[i].main_loop_cb) app_init[i].main_loop_cb();
           ++i;
         }
     }

#ifdef ENABLE_LOGGING
    {
      uint16_t phy, link;
      if (CyU3PUsbGetSpeed() == CY_U3P_SUPER_SPEED) {   // no need to check if not plugged in to USB 3
         if (!CyU3PUsbGetErrorCounts( &phy, &link )) {
           if (phy>0||link>0) {
            log_warn( "usb phy err=%d link err=%d\n", phy, link );
           }
         } else {
            log_warn( "Err phy??\n" ); 
         }
      }
    } 
#endif

    ret = CyU3PEventGet(&glThreadEvent, eventMask, CYU3P_EVENT_OR_CLEAR, &eventStat, 1000);
    if (ret == CY_U3P_SUCCESS) {
        // handle event
        if (eventStat & NITRO_EVENT_VENDOR_CMD) {
            handle_vendor_cmd ( 
                ((glSetupDat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS), // bRequest
                glSetupDat0 & CY_U3P_USB_REQUEST_TYPE_MASK, // bReqType
                glSetupDat0 & CY_U3P_USB_REQUEST_TYPE_MASK & CY_U3P_USB_TYPE_MASK, // bType
                glSetupDat0 & CY_U3P_USB_REQUEST_TYPE_MASK & CY_U3P_USB_TARGET_MASK, // bTarget
                ((glSetupDat0 & CY_U3P_USB_VALUE_MASK)   >> CY_U3P_USB_VALUE_POS), // wValue
                ((glSetupDat1 & CY_U3P_USB_INDEX_MASK)   >> CY_U3P_USB_INDEX_POS), // wIndex
                ((glSetupDat1 & CY_U3P_USB_LENGTH_MASK)  >> CY_U3P_USB_LENGTH_POS) ); // wLength
        }
    }

/*    uint8_t curState;*/
/*    CyU3PGpifGetSMState(&curState);*/
/*    log_error("%d\n", curState );  */
  }
}

#ifdef FIRMWARE_DI
extern void di_main();
void NitroDIThread_Entry (uint32_t input) {
  log_info ( "DI Thread Entry\n" );
  CyU3PMutexCreate(&gRdwrCmd.rdwr_mutex, CYU3P_NO_INHERIT);
  di_main(); // defined 
}
#endif

/* Application define function which creates the threads. */
void CyFxApplicationDefine (void) {
  void *ptr = NULL;
  uint32_t ret = CY_U3P_SUCCESS;
  
  ret = CyU3PEventCreate(&glThreadEvent);
  if (ret != CY_U3P_SUCCESS) {
    while (1); // debugging not initialized yet.
  }

  /* Allocate the memory for the threads */
  ptr = CyU3PMemAlloc (CY_FX_NITRO_THREAD_STACK);

  /* Create the thread for the application */
  ret = CyU3PThreadCreate (&NitroAppThread, /* Bulk loop App Thread structure */
				     "21:Nitro",      /* Thread ID and Thread name */
				     NitroAppThread_Entry, /* Bulk loop App Thread Entry function */
				     0,      /* No input parameter to thread */
				     ptr,                                     /* Pointer to the allocated thread stack */
				     CY_FX_NITRO_THREAD_STACK,               /* Bulk loop App Thread stack size */
				     CY_FX_NITRO_THREAD_PRIORITY,            /* Bulk loop App Thread priority */
				     CY_FX_NITRO_THREAD_PRIORITY,            /* Bulk loop App Thread priority */
				     CYU3P_NO_TIME_SLICE,                     /* No time slice for the application thread */
				     CYU3P_AUTO_START                         /* Start the Thread immediately */
				     );


  /* Check the return code */
  if (ret != 0) {
    /* Thread Creation failed with the error code retThrdCreate */
    /* Add custom recovery or debug actions here */
    /* Application cannot continue */
    /* Loop indefinitely */
    while(1);
  }

  ptr = CyU3PMemAlloc (CY_FX_NITRO_THREAD_STACK);

  /* Create the thread for the application */
  ret = CyU3PThreadCreate (&NitroDataThread, /* Bulk loop App Thread structure */
				     "22:NitroData",      /* Thread ID and Thread name */
				     NitroDataThread_Entry, /* Bulk loop App Thread Entry function */
				     0,      /* No input parameter to thread */
				     ptr,                                    /* Pointer to the allocated thread stack */
				     CY_FX_NITRO_THREAD_STACK,               /* Bulk loop App Thread stack size */
				     CY_FX_NITRO_THREAD_PRIORITY+1,            /* Bulk loop App Thread priority */
				     CY_FX_NITRO_THREAD_PRIORITY+1,            /* Bulk loop App Thread priority */
				     CYU3P_NO_TIME_SLICE,                    /* No time slice for the application thread */
				     CYU3P_AUTO_START                        /* Start the Thread immediately */
				     );
  if (ret) while(1);

#ifdef FIRMWARE_DI
 ptr = CyU3PMemAlloc ( CY_FX_NITRO_THREAD_STACK); // memory for another thread 
 ret = CyU3PThreadCreate (&NitroDIThread, /* Bulk loop App Thread structure */
				     "23:NitroDI",      /* Thread ID and Thread name */
				     NitroDIThread_Entry, /* Bulk loop App Thread Entry function */
				     0,      /* No input parameter to thread */
				     ptr,                                     /* Pointer to the allocated thread stack */
				     CY_FX_NITRO_THREAD_STACK,               /* Bulk loop App Thread stack size */
				     CY_FX_NITRO_THREAD_PRIORITY+2,            /* Bulk loop App Thread priority */
				     CY_FX_NITRO_THREAD_PRIORITY+2,            /* Bulk loop App Thread priority */
				     CYU3P_NO_TIME_SLICE,                     /* No time slice for the application thread */
				     CYU3P_AUTO_START                         /* Start the Thread immediately */
				     );
 if (ret != 0) {
   while (1);
 }
#endif
}

CyU3PReturnStatus_t init_io() {
  /* Configure the IO matrix for the device. On the FX3 DVK board, the
   * COM port is connected to the IO(53:56). This means that either
   * DQ32 mode should be selected or lppMode should be set to
   * UART_ONLY. Here we are choosing UART_ONLY configuration. */
  CyU3PIoMatrixConfig_t io_cfg;
  io_cfg.isDQ32Bit = CyTrue;
  io_cfg.useUart   = CyTrue;
  io_cfg.useI2C    = CyTrue;
  io_cfg.useI2S    = CyFalse;
  io_cfg.useSpi    = CyFalse;
  io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_DEFAULT;
  
  // TODO these could change on different boards
  // 23 = HICS for fpga
  // 26 = LP_B
  // 27 = V18_EN
  // 57 = prog_b for the fpga
  io_cfg.gpioSimpleEn[0]  = (1<<22) | (1 << 23) | (1<<26) | (1<<27);
  io_cfg.gpioSimpleEn[1]  = (1 << (57-32));
  io_cfg.gpioComplexEn[0] = 0;
  io_cfg.gpioComplexEn[1] = 0;
  return  CyU3PDeviceConfigureIOMatrix (&io_cfg);
}

/* Main function */
int main (void) {
  CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

  /* Initialize the device */
  CyU3PSysClockConfig_t clockConfig;
  clockConfig.setSysClk400  = CyTrue;
  clockConfig.cpuClkDiv     = 2;
  clockConfig.dmaClkDiv     = 2;
  clockConfig.mmioClkDiv    = 2;
  clockConfig.useStandbyClk = CyFalse;
  clockConfig.clkSrc        = CY_U3P_SYS_CLK;
  status = CyU3PDeviceInit (&clockConfig);
  if (status != CY_U3P_SUCCESS) {
    goto handle_fatal_error;
  }

  /* Initialize the caches. Enable both Instruction and Data caches. */
  status = CyU3PDeviceCacheControl (CyTrue, CyFalse, CyFalse);
  if (status != CY_U3P_SUCCESS) {
    goto handle_fatal_error;
  }

  status = init_io();
  if (status != CY_U3P_SUCCESS) {
    goto handle_fatal_error;
  }

  /* This is a non returnable call for initializing the RTOS kernel */
  CyU3PKernelEntry ();

  /* Dummy return to make the compiler happy */
  return 0;

 handle_fatal_error:
  /* Cannot recover from this error. */
  while (1);
}
