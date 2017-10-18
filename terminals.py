#
# Copyright (C) 2009 Ubixum, Inc.
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
#

import nitro
from nitro import DeviceInterface, Terminal, Register, SubReg

prom_di = nitro.load_di ( "Microchip/M24XX/M24XX.xml" )
fx3_prom_term = prom_di['M24XX'].clone()
fx3_prom_term.name = 'FX3_PROM'
fx3_prom_term.addr = 0x50
fx3_prom_term.add_child (
    Register (
        name='serialnum',
        addr=0x20000-16, # place serial number at last 16 bytes of eeprom (stored as unicode (16 bit) on prom for device descriptor compat.
        comment="Location of serial number stored on eeprom.",
        mode="write",
        width=16, # unicode characters
        array=8) ) # 8 characters (str type not implemented yet)


di=DeviceInterface(
    name="fx3",
    terminal_list=[
        Terminal(
            name="DUMMY_FX3",
            comment = "Read/write dummy data from this terminal",
            addr = 5,
            regAddrWidth=16,
            regDataWidth=8,
            ),
        Terminal(
            name='FX3',
            comment='Special FX3 functions.',
            addr=0x100,
            regAddrWidth=16,
            regDataWidth=16,
            register_list = [
                Register(name="version",
                         mode="read",
                         comment="Application Firmware Version",
                         subregs=[ SubReg ( name="minor",
                                        width=8,
                                        comment="Firmware minor version." ),
                                   SubReg ( name="major",
                                        width=8,
                                        comment="Firmware major version." )
                                  ]
                ),
                Register(name="usbver",
                         mode="read",
                         comment="USB device descriptor information" ,
                         subregs=[ SubReg ( name="minor",
                                        width=8,
                                        comment="Nitro protocol minor version." ),
                                   SubReg ( name="major",
                                        width=8,
                                        comment="Nitro protocol major version." )
                                  ] ),
                Register(name='USB3',
                         mode='read',
                         comment="Return 1 if device enumerated in usb3 mode"),
                Register(name='rdwr_init_stat',
                         mode='read',
                         comment="Status of last trans init" ),
                Register(name='force_usb2',
                         mode="write",
                         init=0,
                         comment="write 1 to cause device to not connect usb3 lines."),
             ]
         ),
         Terminal(
            name="LOG",
            comment="Logging terminal if USB_LOGGING enabled when firmware compiled.",
            regAddrWidth=16,
            regDataWidth=16,
            addr=245,
            register_list=[
                Register(name="count",
                         mode="read",
                         comment="count of bytes available in log buffer.",
                         width=16),
                Register(name="log",
                         mode="read",
                         width=8,
                         comment="read from this register to drain the buffer."),
            ]

         ),
         fx3_prom_term
     ]
)
