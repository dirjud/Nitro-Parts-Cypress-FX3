
/**
 * This file is the default fx3 i2c prom implementation.
 * For other proms or storage locations, simply exclude
 * this file from config.mk and include your file
 * with get_serial and set_serial instead.
 */

/**
 * serial number is 16 bytes ensured by rdwr.c.
 **/

#include <cyu3i2c.h>

#include "log.h"

#include "fx3_terminals.h"

uint16_t set_serial(uint8_t* serial) {
    CyU3PI2cPreamble_t preamble;
    uint32_t reg_addr = FX3_PROM_SERIALNUM0_0;
    CyU3PReturnStatus_t status;

    preamble.length    = 3;
    preamble.buffer[0] = (TERM_FX3_PROM<<1)|8;
    preamble.buffer[1] = (uint8_t)(reg_addr >> 8);
    preamble.buffer[2] = (uint8_t)(reg_addr & 0xFF);
    preamble.ctrlMask  = 0x0000;
    status = CyU3PI2cTransmitBytes(&preamble, serial, 16, 1);
    if(status) {
        log_error("Error writing serial num to I2C (%d)\n", status);
    }
    return status;
}

/**
 * Buf ensured to by 16 bytes by rdwr.c
 * 16 bytes is 8 unicode chars.
 **/
uint16_t get_serial(uint8_t* buf) {

    CyU3PI2cPreamble_t preamble;
    uint32_t reg_addr = FX3_PROM_SERIALNUM0_0;
    CyU3PReturnStatus_t status;

    preamble.length    = 4;
    preamble.buffer[0] = (TERM_FX3_PROM<<1)|8;
    preamble.buffer[1] = (uint8_t)(reg_addr >> 8);
    preamble.buffer[2] = (uint8_t)(reg_addr & 0xFF);
    preamble.buffer[3] = (TERM_FX3_PROM<<1)|9;
    preamble.ctrlMask  = 0x0004;
    status = CyU3PI2cReceiveBytes (&preamble, buf, 16, 1);
    if(status) {
        log_error("Error reading serial num from prom (%d)\n", status);
    }
    return status;
}
