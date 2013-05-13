/**
 * Copyright (C) 2013 BrooksEE, LLC 
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

// NOTE this is a temporary fix while the current sdk 1.2.3 has a bug
// restoring GPIO pins.  Next SDK Register modifcation should be able
// to be removed
//
#ifndef __GPIO_H
#define __GPIO_H

#define GPIO_RESTORE(pin) \
    CyU3PGpioDisable(pin);\
    CyU3PDeviceGpioRestore(pin);\
    if (pin < 32) { \
      uint32_t temp; \
      CyU3PReadDeviceRegisters ((uvint32_t *)0xe005100c, 1, &temp); \
      temp &= ~(1 << pin); \
      CyU3PWriteDeviceRegisters ((uvint32_t *)0xe005100c, 1, &temp); \
    }

#endif

