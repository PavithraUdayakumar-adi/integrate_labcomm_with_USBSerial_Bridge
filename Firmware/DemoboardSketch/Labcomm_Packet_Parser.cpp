/*
 Labcomm: a packet protocol over serial links

 @verbatim
 This code implements a simple packet protocol over the Arduino Zero SerialUSB.
 @endverbatim

 REVISION HISTORY
 $Revision: 4639 $
 $Date: 2016-01-29 16:42:40 -0500 (Fri, 29 Jan 2016) $

 Copyright (c) 2018, Linear Technology Corp.(LTC)
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those
 of the authors and should not be interpreted as representing official policies,
 either expressed or implied, of Linear Technology Corp.

 */

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include <Arduino.h>

#include "Linduino.h"
#include "demoboard.h"
#include "Labcomm_Packet_Parser.h"
#include "Labcomm_dispatcher.h"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
typedef enum {
    COMM_GUI_STATE_WAITING,                   // Waiting for first byte of Start of Command
    COMM_GUI_STATE_WAITING_BYTE2,             // Waiting for second byte of Start of Command
    COMM_GUI_STATE_COMMAND_RECEIVING_HEADER,  // Receiving bytes in the header.
    COMM_GUI_STATE_COMMAND_RECEIVING_DATA,    // Receiving data bytes identified by the header.
    COMM_GUI_STATE_COMMAND_VERIFYING,         // Receiving the crc to compare to calculated crc.
    COMM_GUI_STATE_COMMAND_PROCESSING,        // Processing the data bytes for the specified module_id.
    COMM_GUI_STATE_RESPONSE_SENDING,          // Sending the response to the command.
    COMM_GUI_NUM_STATES,
} COMM_GUI_STATE_TYPE;

#define COMM_GUI_START_OF_PACKET  (0x4C54)

typedef struct __attribute__((packed))
{
    uint16_t start_of_packet;
    uint16_t src_id;
    uint16_t dest_id;
    uint16_t num_bytes;  // length(payload + CRC16)
} COMM_GUI_HEADER_TYPE;

// CRC applied to both header and data to ensure packet validity
#define COMM_GUI_CRC_TYPE uint16_t
#if (COMM_GUI_START_OF_PACKET == 0x4C54)
  #define COMM_GUI_CRC_SEED  0x8F34
#else
  #error COMM_GUI_CRC_SEED definition must be coordinated with codes used for Start of Packet.
#endif

#define COMM_GUI_SAFETY_TIMEOUT   (1000)

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Variables for parser state machine.
static COMM_GUI_STATE_TYPE comm_gui_state;
static COMM_GUI_CRC_TYPE comm_gui_crc;

// Variables to detect if gui comm is interrupted in the middle of a packet so that, so that comms will not be locked up forever.
static uint32_t comm_gui_safety_timer;
static COMM_GUI_STATE_TYPE comm_gui_state_last;

// Storage for receiving/building packets.
static uint8_t comm_gui_receive_buffer[RECEIVE_BFR_LEN];
static uint16_t comm_gui_bytes_peeked;
static COMM_GUI_HEADER_TYPE comm_gui_header;

const uint16_t PROGMEM crc16_table[] = {
  0x0000,0xC0C1,0xC181,0x0140,0xC301,0x03C0,0x0280,0xC241,0xC601,0x06C0,0x0780,0xC741,0x0500,0xC5C1,0xC481,0x0440,
  0xCC01,0x0CC0,0x0D80,0xCD41,0x0F00,0xCFC1,0xCE81,0x0E40,0x0A00,0xCAC1,0xCB81,0x0B40,0xC901,0x09C0,0x0880,0xC841,
  0xD801,0x18C0,0x1980,0xD941,0x1B00,0xDBC1,0xDA81,0x1A40,0x1E00,0xDEC1,0xDF81,0x1F40,0xDD01,0x1DC0,0x1C80,0xDC41,
  0x1400,0xD4C1,0xD581,0x1540,0xD701,0x17C0,0x1680,0xD641,0xD201,0x12C0,0x1380,0xD341,0x1100,0xD1C1,0xD081,0x1040,
  0xF001,0x30C0,0x3180,0xF141,0x3300,0xF3C1,0xF281,0x3240,0x3600,0xF6C1,0xF781,0x3740,0xF501,0x35C0,0x3480,0xF441,
  0x3C00,0xFCC1,0xFD81,0x3D40,0xFF01,0x3FC0,0x3E80,0xFE41,0xFA01,0x3AC0,0x3B80,0xFB41,0x3900,0xF9C1,0xF881,0x3840,
  0x2800,0xE8C1,0xE981,0x2940,0xEB01,0x2BC0,0x2A80,0xEA41,0xEE01,0x2EC0,0x2F80,0xEF41,0x2D00,0xEDC1,0xEC81,0x2C40,
  0xE401,0x24C0,0x2580,0xE541,0x2700,0xE7C1,0xE681,0x2640,0x2200,0xE2C1,0xE381,0x2340,0xE101,0x21C0,0x2080,0xE041,
  0xA001,0x60C0,0x6180,0xA141,0x6300,0xA3C1,0xA281,0x6240,0x6600,0xA6C1,0xA781,0x6740,0xA501,0x65C0,0x6480,0xA441,
  0x6C00,0xACC1,0xAD81,0x6D40,0xAF01,0x6FC0,0x6E80,0xAE41,0xAA01,0x6AC0,0x6B80,0xAB41,0x6900,0xA9C1,0xA881,0x6840,
  0x7800,0xB8C1,0xB981,0x7940,0xBB01,0x7BC0,0x7A80,0xBA41,0xBE01,0x7EC0,0x7F80,0xBF41,0x7D00,0xBDC1,0xBC81,0x7C40,
  0xB401,0x74C0,0x7580,0xB541,0x7700,0xB7C1,0xB681,0x7640,0x7200,0xB2C1,0xB381,0x7340,0xB101,0x71C0,0x7080,0xB041,
  0x5000,0x90C1,0x9181,0x5140,0x9301,0x53C0,0x5280,0x9241,0x9601,0x56C0,0x5780,0x9741,0x5500,0x95C1,0x9481,0x5440,
  0x9C01,0x5CC0,0x5D80,0x9D41,0x5F00,0x9FC1,0x9E81,0x5E40,0x5A00,0x9AC1,0x9B81,0x5B40,0x9901,0x59C0,0x5880,0x9841,
  0x8801,0x48C0,0x4980,0x8941,0x4B00,0x8BC1,0x8A81,0x4A40,0x4E00,0x8EC1,0x8F81,0x4F40,0x8D01,0x4DC0,0x4C80,0x8C41,
  0x4400,0x84C1,0x8581,0x4540,0x8701,0x47C0,0x4680,0x8641,0x8201,0x42C0,0x4380,0x8341,0x4100,0x81C1,0x8081,0x4040
};

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void comm_gui_state_machine_reset();
boolean comm_gui_parse(t_fifo_buffer *buf_in, uint16_t timeout);
uint16_t comm_gui_crc16_update(uint16_t crc, uint8_t data);

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Comm_GUI_Init(void)
{
  comm_gui_state_machine_reset();
  comm_gui_safety_timer = millis();
  return;
}

boolean Labcomm_Send_Packet(t_fifo_buffer *buf_out, uint16_t src_id, uint16_t dest_id, uint8_t * data, uint16_t length)
{
  uint16_t crc;
  uint16_t i;
  uint16_t sof = COMM_GUI_START_OF_PACKET;

  if (fifoBuf_getFree(buf_out) > (length + sizeof(comm_gui_header) + sizeof(comm_gui_crc)))
  {
    //there is space to transmit now

    crc = COMM_GUI_CRC_SEED;
    fifoBuf_putByte(buf_out, UPPER_BYTE(sof));
    fifoBuf_putByte(buf_out, LOWER_BYTE(sof));

    // src_id
    crc = comm_gui_crc16_update(crc, UPPER_BYTE(src_id));
    fifoBuf_putByte(buf_out, UPPER_BYTE(src_id));
    crc = comm_gui_crc16_update(crc, LOWER_BYTE(src_id));
    fifoBuf_putByte(buf_out, LOWER_BYTE(src_id));

    // dest_id
    crc = comm_gui_crc16_update(crc, UPPER_BYTE(dest_id));
    fifoBuf_putByte(buf_out, UPPER_BYTE(dest_id));
    crc = comm_gui_crc16_update(crc, LOWER_BYTE(dest_id));
    fifoBuf_putByte(buf_out, LOWER_BYTE(dest_id));

    // length
    crc = comm_gui_crc16_update(crc, UPPER_BYTE(length));
    fifoBuf_putByte(buf_out, UPPER_BYTE(length));
    crc = comm_gui_crc16_update(crc, LOWER_BYTE(length));
    fifoBuf_putByte(buf_out, LOWER_BYTE(length));

    // data
    for (i=0;i<length;i++)
      crc = comm_gui_crc16_update(crc, data[i]);
    fifoBuf_putData(buf_out, data, length);

    //crc
    fifoBuf_putByte(buf_out, UPPER_BYTE(crc));
    fifoBuf_putByte(buf_out, LOWER_BYTE(crc));

    return true;
  }
  else
  {
    return false;
  }
}

boolean Labcomm_Receive_Packet(t_fifo_buffer *buf_in, uint16_t dest_id, uint16_t src_id, uint8_t *data, uint16_t *length, uint16_t timeout)
{
  uint16_t result;
  uint8_t index;

  // Check for a packet from the input buffer and return it if the desired src_id, dest_id, and <length.
  if(true == comm_gui_parse(buf_in, timeout))
  {
    if((comm_gui_header.dest_id == dest_id) &&
       (comm_gui_header.src_id == src_id) &&
       (comm_gui_header.num_bytes <= *length))
    {
      *length = comm_gui_header.num_bytes;
      memcpy(data, comm_gui_receive_buffer, comm_gui_header.num_bytes);
      comm_gui_state_machine_reset(); // todo - we need to do this when the packet is processed, which at this point it is, but if whatever called this can't send a response could be a problem.
      return true;
    }
  }

  return false;
}

// todo - we no longer process multiple packets in the input buffer in one shot.
boolean Labcomm_Handle_Comms(t_fifo_buffer *buf_in, t_fifo_buffer *buf_out)
{
  uint16_t result;

  digitalWrite(DEBUG5, HIGH);

  // Check for a packet from the input buffer and attempt to process it if found.
  if(true == comm_gui_parse(buf_in, COMM_GUI_SAFETY_TIMEOUT))
  {
    uint16_t module_id = comm_gui_header.dest_id;

    uint8_t response_buffer[TRANSMIT_BFR_LEN];
    uint16_t response_size = 0; // response size returned by the response function.

    if(DO_SEND_RESPONSE == dispatch_Labcomm_packet(module_id, comm_gui_receive_buffer, comm_gui_header.num_bytes, response_buffer, &response_size))
    {
      // Try sending the response if output buffer has room.
      comm_gui_header.dest_id = comm_gui_header.src_id;
      comm_gui_header.src_id = module_id;
      if(true == Labcomm_Send_Packet(buf_out, comm_gui_header.src_id, comm_gui_header.dest_id, response_buffer, response_size))
      {
        // Can not reset or else we'll return true below.  That'll cause the next byte to get thrown away if there's a partial second packet in the serial buffer.
        // todo - might be better to make this state machine run again instead, to start the processing of the next packet instead.
        comm_gui_state = COMM_GUI_STATE_RESPONSE_SENDING;
      }
      else
      {
        comm_gui_state_machine_reset(); // sending packet has error
      }
    }
    else
    {
      // Silently drop the packet and don't send any response.
      comm_gui_state_machine_reset(); // function to process this data has error
    }
  }
  // Return true if still receiving packet from the input buffer.
  return (comm_gui_state != COMM_GUI_STATE_WAITING);
}

// Ensure 16bit values are transmitted in Network Byte Order
uint16_t Labcomm_Swap_Byte_Order_16(uint16_t data)
{
  uint16_t temp;
  ((uint8_t*)(&temp))[0] = UPPER_BYTE(data);
  ((uint8_t*)(&temp))[1] = LOWER_BYTE(data);

  return temp;
}

// Ensure 32bit values are transmitted in Network Byte Order
uint32_t Labcomm_Swap_Byte_Order_32(uint32_t data)
{
  uint32_t temp;
  ((uint8_t*)&(temp))[0] = UPPER_BYTE(UPPER_WORD(data));
  ((uint8_t*)&(temp))[1] = LOWER_BYTE(UPPER_WORD(data));
  ((uint8_t*)&(temp))[2] = UPPER_BYTE(LOWER_WORD(data));
  ((uint8_t*)&(temp))[3] = LOWER_BYTE(LOWER_WORD(data));

  return temp;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void comm_gui_state_machine_reset()
{
    comm_gui_header.start_of_packet = COMM_GUI_START_OF_PACKET;
    comm_gui_bytes_peeked = 0;
    comm_gui_crc = 0;

    comm_gui_state = COMM_GUI_STATE_WAITING;
    comm_gui_state_last = COMM_GUI_STATE_WAITING;
}

// Get characters from serial port and determine if they are part of a GUI Comm packet.
// Return true if data was received from a valid packet.
// The goal is to leave serial input untouched if not receiving a packet so that another comm interface can also exist.
// If a packet is detected, however, and fails a validity check it is acceptable for all of the bytes in the bad packet to be eaten.
// If all of the bytes in a command are available from the serial port, this function should run to completion.  It should not require
// being called multiple times.  The most elegant way to handle this is with gotos so be prepared to see some.
boolean comm_gui_parse(t_fifo_buffer *buf_in, uint16_t timeout)
{
  digitalWrite(DEBUG8, HIGH);

  // Check to ensure we don't lock up in one state forever if for some reason Comm is interrupted in the middle of a packet.
  {
      uint32_t new_time = millis();
      if((comm_gui_state != comm_gui_state_last) || (comm_gui_state == COMM_GUI_STATE_WAITING))
      {
          comm_gui_state_last = comm_gui_state;
          comm_gui_safety_timer = new_time;
      }
      else if((new_time - comm_gui_safety_timer) >= timeout)
      {
        comm_gui_state_machine_reset();
        comm_gui_safety_timer = new_time;
      }
  }

  // Determine if the serial input is part of a packet used for communication with the GUI.
  switch (comm_gui_state)
  {
    case COMM_GUI_STATE_RESPONSE_SENDING: // todo - might not need this state if Comm_Gui_Handle_Comms() is modified such that it processes multiple packets in a row.
      comm_gui_state_machine_reset();
    case COMM_GUI_STATE_WAITING:
comm_gui_check_start:
      // Check for start of packet.
      if(fifoBuf_getBytePeek(buf_in) != UPPER_BYTE(comm_gui_header.start_of_packet))
      {
        break; // This isn't the start of a packet.
      }
      comm_gui_state = COMM_GUI_STATE_WAITING_BYTE2;
      // fallthrough

    case COMM_GUI_STATE_WAITING_BYTE2:
      if(fifoBuf_getUsed(buf_in) < sizeof(comm_gui_header.start_of_packet))
      {
        break; // This could be a start of a packet, two bytes haven't been received yet.
      }
      else if(fifoBuf_getBytePeekAhead(buf_in, 1) != LOWER_BYTE(comm_gui_header.start_of_packet))
      {
        comm_gui_state_machine_reset();
        break; // This isn't the start of a packet.  First byte matched, but second byte did not.
      }

      // Start of Packet received.  Read and validate the rest.
      comm_gui_bytes_peeked = sizeof(comm_gui_header.start_of_packet);
      comm_gui_crc = COMM_GUI_CRC_SEED;
      comm_gui_state = COMM_GUI_STATE_COMMAND_RECEIVING_HEADER;
      // fallthrough

    case COMM_GUI_STATE_COMMAND_RECEIVING_HEADER:
comm_gui_receive_header:
      while((fifoBuf_getUsed(buf_in) - comm_gui_bytes_peeked) > 0)
      {
        uint8_t* rxbuf_ptr = (uint8_t*)&comm_gui_header;
        uint8_t rxbyte = fifoBuf_getBytePeekAhead(buf_in, comm_gui_bytes_peeked);
        comm_gui_crc = comm_gui_crc16_update(comm_gui_crc, rxbyte);
        rxbuf_ptr[comm_gui_bytes_peeked] = rxbyte;
        comm_gui_bytes_peeked++;
        if(comm_gui_bytes_peeked >= sizeof(comm_gui_header))
        {
          // Reorder the bytes to the native endianness and advance state.
          comm_gui_header.src_id = Labcomm_Swap_Byte_Order_16(comm_gui_header.src_id);
          comm_gui_header.dest_id = Labcomm_Swap_Byte_Order_16(comm_gui_header.dest_id);
          comm_gui_header.num_bytes = Labcomm_Swap_Byte_Order_16(comm_gui_header.num_bytes);

          // Abort if attempting to receive more bytes than possible
          if(comm_gui_header.num_bytes > sizeof(comm_gui_receive_buffer))
          {
            comm_gui_state_machine_reset();
            break;
          }
          else if(comm_gui_header.num_bytes > 0)
          {
            comm_gui_state = COMM_GUI_STATE_COMMAND_RECEIVING_DATA;
            goto comm_gui_receive_data;
          }
          else
          {
            comm_gui_state = COMM_GUI_STATE_COMMAND_VERIFYING;
            goto comm_gui_verifying;
          }
        }
      }
      break;

    case COMM_GUI_STATE_COMMAND_RECEIVING_DATA:
comm_gui_receive_data:
      while((fifoBuf_getUsed(buf_in) - comm_gui_bytes_peeked) > 0)
      {
        uint8_t rxbyte = fifoBuf_getBytePeekAhead(buf_in, comm_gui_bytes_peeked);
        comm_gui_crc = comm_gui_crc16_update(comm_gui_crc, rxbyte);
        comm_gui_receive_buffer[comm_gui_bytes_peeked - sizeof(comm_gui_header)] = rxbyte;
        comm_gui_bytes_peeked++;
        if(comm_gui_bytes_peeked >= (comm_gui_header.num_bytes + sizeof(comm_gui_header)))
        {
          comm_gui_state = COMM_GUI_STATE_COMMAND_VERIFYING;
          goto comm_gui_verifying;
        }
      }
      break;

    case COMM_GUI_STATE_COMMAND_VERIFYING:
comm_gui_verifying:
      if ((fifoBuf_getUsed(buf_in) - comm_gui_bytes_peeked) < sizeof(comm_gui_crc)) {
        break;
      }

      if((fifoBuf_getBytePeekAhead(buf_in, comm_gui_bytes_peeked) != UPPER_BYTE(comm_gui_crc)) ||
          (fifoBuf_getBytePeekAhead(buf_in, comm_gui_bytes_peeked+1) != LOWER_BYTE(comm_gui_crc)))
      {
        comm_gui_state_machine_reset();
        break;
      }

      // Packet parsed and moved into local memory, remove from input buffer.
      comm_gui_bytes_peeked += sizeof(comm_gui_crc);
      fifoBuf_removeData(buf_in, comm_gui_bytes_peeked);
      comm_gui_state = COMM_GUI_STATE_COMMAND_PROCESSING;
      // fallthrough

    case COMM_GUI_STATE_COMMAND_PROCESSING:
comm_gui_processing:
      // todo - we still need some sort of state to prevent the next packet from getting received while the last one isn't processed.
      break;

  }

  digitalWrite(DEBUG8, LOW);

  // Return true if we are actively processing a packet.
  return (comm_gui_state == COMM_GUI_STATE_COMMAND_PROCESSING);
}

uint16_t comm_gui_crc16_update(uint16_t crc, uint8_t data)
{
  return ((crc >> 8) ^ crc16_table[(crc ^ data) & 0xff]);
}
