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
                Register(name="gpif_clk_div",
                         mode="write",
                         comment="clock divider for gpif, valid values are 2-1024",
                         init=8,
                         type="int"),
                Register(name="gpif_clk_halfdiv",
                         mode="write",
                         init=0,
                         comment="if 1, gpif divider is half div and provides values from 2.5 to 256.5, else normal divider.",
                         type="int" ),
                Register(name="gpif_clk_src",
                         mode="write",
                         init="3",
                         comment="0 = sys_clk/16, 1 = sys_clk/4, 2= sys_clk/2, 3=sys_clk"  ),
                Register(name="gpif_clk_trigger",
                         type="trigger",
                         comment="write to this register to re-initialize gpif interface.  The gpif registers do not cause a gpif reset." ),
                Register(name="gpif_drive_strength",
                         mode="write",
                         init="2",
                         comment="change the drive strength for the gpif. 0=1/4 strength. 1=1/2 strength, 2=3/4 strenght, 3=full strength." ),

             ]
         ),
         fx3_prom_term
     ]
)

