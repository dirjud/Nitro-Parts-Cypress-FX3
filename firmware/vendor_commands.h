/**
 * Copyright (C) 2009 Ubixum, Inc. 
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

#ifndef VENDOR_COMMANDS_H
#define VENDOR_COMMANDS_H


#define bmSETWRITE 8
/** 
 * command codes for rdwr_data_t.command
 **/
enum NITRO_COMMAND {
   COMMAND_READ,
   COMMAND_WRITE=bmSETWRITE,
   COMMAND_GET=1,
   COMMAND_SET=bmSETWRITE|1
};

// structure for passing data from rdwr vendor command to rdwr handlers
#ifdef WIN32
#pragma pack(push)
#pragma pack(1)
#endif

/**
 * This is the payload data for the vendor command to initiate a read, write,
 * get, or set.
 **/
typedef struct {
  /**
   * The command.  A NITRO_COMMAND.
   **/
  uint8_t command;

  /**
   * The terminal address.
   **/
  uint16_t term_addr;

  /**
   * The register address
   **/
  uint32_t reg_addr;

  /**
   * Total transfer length
   **/
  uint32_t transfer_length;
  
} 
#ifdef __GNUC__
 __attribute__((__packed__))
#endif
rdwr_data_header_t;



enum NITRO_VC { 

 // implemented by fx2 core
 VC_RDWR_RAM=0xa0,
 
 /**
  * type 0xc0
  *  returns sizeof(rdwr_data_t) bytes 
  *  returns the rdwr_data structure
  **/

 VC_RDWR_STAT=0xb1,

/**
 * type 0x40 to initiate a read or a write
 *
 * value = ep_addr
 * index = rdwr_addr
 * length = 5
 *  send 1 byte (0=read 1 = write, 2=get, 3=set) followed by 4 bytes for length (send 4 bytes little-endian!)
 **/
VC_HI_RDWR=0xb4,

/**
 * type 0x40
 * Cause the device to renumerate
 **/
VC_RENUM=0xb5,


/**
 * type 0x40 -> set the serial
 * type 0xc0 <- get the serial
 * length = 8
 * \return 8 byte serial number 
 **/
VC_SERIAL=0xb6

};

#define ACK_PKT_ID 0xA50F
typedef struct {
  uint16_t id;
  uint16_t checksum;
  uint16_t status;
  uint16_t reserved;
}
#ifdef __GNUC__
 __attribute__((__packed__))
#endif
ack_pkt_t;


#define LITTLE_ENDIAN_16(x) ((((uint16_t) (x))<<8) | (((uint16_t) (x)) >> 8))

#endif
