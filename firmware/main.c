/*
 ##  Copyright BrooksEE, LLC, 2013,
*/

#include "cyu3system.h"
#include "cyu3dma.h"
#include "cyu3error.h"
#include "cyu3i2c.h"
#include "main.h"
#include "cyu3usb.h"
#include "rdwr.h"
#include "error_handler.h"
#include "cyu3gpio.h"
#include "log.h"

#ifndef DEBUG_MAIN
#undef log_debug
#define log_debug(x,...) do {} while(0)
#endif


#ifndef GPIO_INTERRUPT
#define GPIO_INTERRUPT 0
#else
void GPIO_INTERRUPT (uint8_t);
#endif

#ifndef SS_INIT
CyBool_t glSSInit=CyTrue;
#else
CyBool_t glSSInit=SS_INIT;
#endif


#ifdef UXN1340
// board specific mux for usb-c connector
#define GPIO_USB3_SS_MUX 50
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
#define NUM_INTERFACES 10      // increase if necessary
uint8_t glInterfaceAltSettings[NUM_INTERFACES] = {0};  /* Active USB interfaced.      */
uint8_t *glSelBuffer = 0;               /* Buffer to hold SEL values.           */

CyBool_t glIsApplnActive = CyFalse;     /* Whether the loopback application is active or not. */

extern rdwr_cmd_t gRdwrCmd;

CyU3PReturnStatus_t init_io();


#ifndef NITRO_INTERFACE_IDX
// custom firmware might potentially place the nitro interface
// after another interface in which case it should define
// what the interface number is in order to shut down the endpoints
// on interface change etc.
#define NITRO_INTERFACE_IDX 0
#endif

/**
 * set interface callback.  Allows firmware
 * to be notified if an interface is selected.
 **/
#ifndef INTF_CALLBACK
#define INTF_CALLBACK default_set_interface
void CyFxNitroApplnStart (); // predefs
void CyFxNitroApplnStop ();
void default_set_interface(uint8_t interface, uint8_t alt) {
  log_info ( "Set Interface %d Alt %d\n", interface, alt);
  if (interface==NITRO_INTERFACE_IDX) {
    if (alt && !glIsApplnActive)
      CyFxNitroApplnStart();
    if (!alt && glIsApplnActive)
      CyFxNitroApplnStop();
  }
}
#else
void INTF_CALLBACK(uint8_t, uint8_t);
#endif

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

void init_gpio (void) {
    CyU3PGpioClock_t gpioClock;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
#ifdef UXN1340
    CyU3PGpioSimpleConfig_t gpioConfig;
#endif

    // TODO should these be registers in the fx3 term
    // that allow firmware to customize?
    // or perhaps #defines to customize at compile time?
    /* Init the GPIO module */
    gpioClock.fastClkDiv = 2;
    gpioClock.slowClkDiv = 0;
    gpioClock.simpleDiv = CY_U3P_GPIO_SIMPLE_DIV_BY_2;
    gpioClock.clkSrc = CY_U3P_SYS_CLK;
    gpioClock.halfDiv = 0;

    apiRetStatus = CyU3PGpioInit(&gpioClock, GPIO_INTERRUPT);
    if (apiRetStatus != 0) {
        /* Error Handling */
      log_error("CyU3PGpioInit failed, error code = %d\n", apiRetStatus);
      error_handler(apiRetStatus);
    }
    log_debug ( "GPIO block initialized\n");

#ifdef UXN1340
    apiRetStatus = CyU3PDeviceGpioOverride (GPIO_USB3_SS_MUX, CyTrue); // Start High
    if (apiRetStatus) log_error ( "Fail to override usb3_ss_mux: %d\n", apiRetStatus);

    gpioConfig.outValue = CyTrue;
    gpioConfig.driveLowEn = CyTrue;
    gpioConfig.driveHighEn = CyTrue;
    gpioConfig.inputEn = CyFalse;
    gpioConfig.intrMode = CY_U3P_GPIO_NO_INTR;
    apiRetStatus = CyU3PGpioSetSimpleConfig(GPIO_USB3_SS_MUX, &gpioConfig);
    if (apiRetStatus) log_error ( "Fail to config usb3_ss_mux: %d\n",apiRetStatus);
#endif
}

/* This function starts the nitro application. This is called when a
 * SET_CONF event is received from the USB host. The endpoints are
 * configured in this function. */
void CyFxNitroApplnStart (void) {
  CyU3PEpConfig_t epCfg;
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
  CyU3PUSBSpeed_t usbSpeed = CyU3PUsbGetSpeed();
  int i;
  log_info("Entering CyFxNitroApplnStart() usbSpeed: ");

  CyU3PUsbLPMDisable(); // no low power state for us.

  /* Based on the Bus Speed configure the endpoint packet size */
  switch (usbSpeed) {
  case CY_U3P_FULL_SPEED:
    log_info ( "full\n" );
    gRdwrCmd.ep_buffer_size = 64;
    break;

  case CY_U3P_HIGH_SPEED:
    log_info("high\n");
    gRdwrCmd.ep_buffer_size = 512;
    break;

  default: // NOTE if di_main starts up app then usb speed is invalid.
           // the value is ignored anyway.
  case  CY_U3P_SUPER_SPEED:
    log_info("SS\n");
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

  epCfg.burstLen = gRdwrCmd.ep_buffer_size == 1024 ? CY_FX_EP_BURST_LENGTH : 1; // for the read endpoint we can burst in usb3 mode

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
    uint8_t dscrType = wValue>>8;
    uint8_t dscrIdx = wValue&0xff;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    CyU3PUSBSpeed_t usbSpeed = CyU3PUsbGetSpeed ();

    /* The descriptor type is in the MS byte and the index is in
     * the LS byte. The index is useful only for string descriptor. */
    switch (dscrType)
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
                buffer = CyFxUSBSSConfigDscr[dscrIdx];
            }
            else if (usbSpeed == CY_U3P_HIGH_SPEED)
            {
                buffer = CyFxUSBHSConfigDscr[dscrIdx];
            }
            else /* CY_U3P_FULL_SPEED */
            {
                buffer = CyFxUSBFSConfigDscr[dscrIdx];
            }
            length = (buffer[2] | ((uint16_t)buffer[3] << 8));
            break;

        case CY_U3P_USB_OTHERSPEED_DESCR:
            if (usbSpeed == CY_U3P_HIGH_SPEED)
            {
                buffer = CyFxUSBFSConfigDscr[dscrIdx];
            }
            else if (usbSpeed == CY_U3P_FULL_SPEED)
            {
                buffer = CyFxUSBHSConfigDscr[dscrIdx];
            }
            else
            {
                /* Do nothing. buffer = NULL. */
            }

            if (buffer != NULL)
            {
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
    glUsbConfiguration = wValue;
    if (glUsbConfiguration == 0) {
      if (glIsApplnActive) {
        CyFxNitroApplnStop ();
      }
    } else {
      if (glIsApplnActive) {
        CyFxNitroApplnStop ();
      }
      /* Start the loop back function. */
      // up to plugins to check glUsbConfiguration
      // and set up endpoints/config appropriately
      CyFxNitroApplnStart ();
    }
    break;

    /* Return the current selected interface. */
  case CY_U3P_USB_SC_GET_INTERFACE:
    if (wIndex>=NUM_INTERFACES) {
      log_error ( "Get Interface %d, NUM_INTERFACES: %d\n", wIndex, NUM_INTERFACES );
      isHandled=CyFalse;
    } else {
      glEp0Buffer[0] = glInterfaceAltSettings[wIndex];
      status = CyU3PUsbSendEP0Data (wLength, glEp0Buffer);
    }
    break;

    /* Store the selected interface value. */
  case CY_U3P_USB_SC_SET_INTERFACE:
    if (wIndex>=NUM_INTERFACES) {
      log_error ( "Set Interface %d, NUM_INTERFACES: %d\n", wIndex, NUM_INTERFACES);
      isHandled=CyFalse;
    } else {
      glInterfaceAltSettings[wIndex] = wValue;
      INTF_CALLBACK ( wIndex, wValue );
    }
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
    log_info ( "USB_EVENT_SPEED, speed: %d\n", CyU3PUsbGetSpeed() );
    break;
  case CY_U3P_USB_EVENT_EP0_STAT_CPLT:
    log_debug( "EP0_STAT_CPLT\n" );
    break;
  case CY_U3P_USB_EVENT_EP_UNDERRUN:
    log_warn ( "USB DATA UNDERRUN\n" );
    break;
  case CY_U3P_USB_EVENT_USB3_LNKFAIL:
    log_warn ( "USB3 LINK FAIL\n");
    break;
  case CY_U3P_USB_EVENT_SS_COMP_ENTRY:
    log_warn ( "USB3 compliance entry\n");
    break;
  case CY_U3P_USB_EVENT_SS_COMP_EXIT:
    log_warn ( "USB3 compliance exit\n");
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

void init_usb() {
    // should be called when usb is not connected or errors
    CyU3PReturnStatus_t apiRetStatus=CyU3PUsbControlUsb2Support(!glSSInit);
    if (apiRetStatus) log_error("Fail to set usb2 support to %d: %d\n", glSSInit?0:1,apiRetStatus);

    apiRetStatus = CyU3PConnectState(CyTrue, glSSInit);
#ifdef UXN1340
    CyU3PUsbControlUsb2Support(CyTrue); // turn that back on in case we need it later
    if (glSSInit && CyU3PUsbGetSpeed() != CY_U3P_SUPER_SPEED) {
        log_info ( "First usb connect fail.\n");
        CyU3PConnectState(CyFalse,CyFalse);
        CyU3PGpioSetValue(GPIO_USB3_SS_MUX,0); // try the other way

        apiRetStatus = CyU3PConnectState(CyTrue,CyTrue);
    }
#endif
    if (apiRetStatus) {
        log_error( "USB Connect failed: %d\n", apiRetStatus);
        error_handler(apiRetStatus);
    }
    log_info ( "USB Connect state: %d\n", CyU3PUsbGetSpeed());
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
  while (io_handlers[i].handler != 0) {
    log_debug ( "handler %d boot %s\n", i, io_handlers[i].boot_handler ? "yes" : "no");
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
    //apiRetStatus = CyU3PUsbSetTxSwing(127); // per Cypress tech phyerr doc
    //log_debug ( "Tx Swing ret: %d\n" , apiRetStatus );
    #ifdef CX3
    // ON THE CX3, the power input is the same as the batt input.
    // if we have lower voltage than 5v (iPhone) the usb bus won't
    // turn on without enabling the batt input.
    // for FX3 however they are different inputs and enabling VBatt
    // causes the FX3 to not power the usb bus.
    apiRetStatus = CyU3PUsbVBattEnable(CyTrue);
    apiRetStatus |= CyU3PUsbControlVBusDetect ( CyFalse, CyTrue );
    #endif
    init_usb();

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
        if (gRdwrCmd.io_handler && gRdwrCmd.io_handler->handler->handler_dma_cb) {
            if (!gRdwrCmd.io_handler->handler->handler_dma_cb())
                continue; // do this in a loop until dma cb puts the command back to done
        }
        else {
          log_debug ( "nd (%d/%d)", gRdwrCmd.transfered_so_far,gRdwrCmd.header.transfer_length );
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
  uint32_t eventMask = NITRO_EVENT_VENDOR_CMD|NITRO_EVENT_BREAK|NITRO_EVENT_REBOOT|NITRO_EVENT_USB2; // can add more events
  uint32_t eventStat;

  /* Initialize the debug and other io modules module */
  ret=init_io();
  #ifdef ENABLE_LOGGING
  logging_boot();
  #endif
  log_info ("io matrix init %d\n", ret);
  init_i2c();
  init_gpio();

  /* Initialize the bulk loop application */
  CyFxNitroApplnInit();

  log_info ( "Nitro Thread Entry\n" );
  for (;;) {
    #ifndef USB_LOGGING
     log_info ( "." );
    #endif

     // CyU3PGpioSimpleSetValue ( 48, io48_val);
     // io48_val = !io48_val;

     CyU3PSysWatchDogClear();
     //slfifo_checkdone();
     // TODO put this in the mail_loop_cb
     // ok for other main loop cb not to have rdwr_done?

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

        if (eventStat & NITRO_EVENT_REBOOT) {
            CyU3PThreadSleep(500);
            #ifdef CX3
            CyU3PCx3DeviceReset(CyFalse,CyFalse); // additional reset for mipi stuff
            #else
            CyU3PDeviceReset(CyFalse); // cold boot from prom
            #endif
            // doesn't return
        }

        if (eventStat & NITRO_EVENT_USB2) {
            // disconnect usb lines

            CyU3PThreadSleep(500);
            CyU3PConnectState(CyFalse,CyFalse);
            init_usb();

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

 CyU3PSysWatchDogConfigure ( CyTrue, 2000 );

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
  CyU3PMemSet((uint8_t*)&io_cfg,0,sizeof(io_cfg));

#ifdef CX3
  io_cfg.isDQ32Bit = CyFalse;
  io_cfg.useUart   = CyTrue;
  io_cfg.useI2C    = CyTrue;
  io_cfg.useI2S    = CyFalse;
  io_cfg.useSpi    = CyTrue;
#else
  io_cfg.isDQ32Bit = CyTrue;
  io_cfg.useUart = CyTrue;
  io_cfg.useI2C = CyTrue;
#endif
  io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_DEFAULT;
  return CyU3PDeviceConfigureIOMatrix (&io_cfg);
}

#ifndef SYS_CLK_400
#define SYS_CLK_400 CyTrue
#endif

/* Main function */
int main (void) {
  CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

  /* Initialize the device */
  CyU3PSysClockConfig_t clockConfig;
  clockConfig.setSysClk400  = SYS_CLK_400;
  clockConfig.cpuClkDiv     = 2; // clkSrc/2 = 201.6 or 192
  clockConfig.dmaClkDiv     = 2; // cpuClk/2 = 100.8 or 96
  clockConfig.mmioClkDiv    = 2; // cpuClk/2 = 100.8 or 96
  clockConfig.useStandbyClk = CyFalse;
  clockConfig.clkSrc        = CY_U3P_SYS_CLK; // if sys_clk_400 then 403.2 else 384
  status = CyU3PDeviceInit (&clockConfig);
  if (status != CY_U3P_SUCCESS) {
    error_handler(status);
  }

  /* Initialize the caches. Enable both Instruction and Data caches. */
  status = CyU3PDeviceCacheControl (CyTrue, CyFalse, CyFalse);
  if (status != CY_U3P_SUCCESS) {
    error_handler(status);
  }

  //status = init_io();

  if (status != CY_U3P_SUCCESS) {
   error_handler(status);
  }

  /* This is a non returnable call for initializing the RTOS kernel */
  CyU3PKernelEntry ();
  return 0;
}
