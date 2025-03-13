/*
  Linear Technology Demonstration Board ID Module

  @verbatim
  Demoboard Identification Labcomm module. Handles dest_id 0x0002.
  @endverbatim

  REVISION HISTORY
  $Revision: 4639 $
  $Date: 2016-01-29 16:42:40 -0500 (Fri, 29 Jan 2016) $

  Copyright (c) 2014, Linear Technology Corp.(LTC)
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
#include "Labcomm_Demoboard_ID_module.h"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#define NUM_ID_STRINGS 2

typedef struct __attribute__((packed))
{
  uint16_t board_number; // The demo board number (DC####) in numeric format.
  uint32_t timestamp_ticks_per_sec; // The number of timestamp ticks per second, used by software to convert timestamps to real time.
  uint8_t num_strings; // The number of strings returned to identify the demo board.
  uint8_t string_length[NUM_ID_STRINGS]; // The length of the strings returned to identify the demo board.
  char id_string[DEMOBOARD_ID_STRING_SIZE]; // The response to the old "I" DC590 command.
  char ltc_string[DEMOBOARD_ID_STRING_SIZE]; // The response to the old "i" DC590 command.
} DEMOBOARD_ID_MODULE_RESPONSE_TYPE;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Comm_Gui_Data_Demo_Board_Init(void)
{
  digitalWrite(DEBUG18, HIGH);
  return;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// No command packet payload needed.
// This always responds over Labcomm with a DEMOBOARD_ID_MODULE_RESPONSE_TYPE payload
// which contains demoboard "DCxxxx" number, ID strings, etc. as defined above.

RESPONSE_ACTION Labcomm_Board_ID_handler(uint8_t* command_data_ptr, uint16_t command_data_size,
                                         uint8_t* response_data_ptr, uint16_t* response_data_size)
{
  UNUSED(command_data_ptr);
  UNUSED(command_data_size);
  DEMOBOARD_ID_MODULE_RESPONSE_TYPE* response_struct_ptr = (DEMOBOARD_ID_MODULE_RESPONSE_TYPE*)response_data_ptr;

  // Populate the Data
  response_struct_ptr->board_number = Labcomm_Swap_Byte_Order_16(2038);
  response_struct_ptr->timestamp_ticks_per_sec = Labcomm_Swap_Byte_Order_32(US_PER_SEC);
  response_struct_ptr->num_strings = NUM_ID_STRINGS;
  for (uint8_t string_num = 0; string_num < NUM_ID_STRINGS; string_num++)
  {
    response_struct_ptr->string_length[string_num] = DEMOBOARD_ID_STRING_SIZE;
  }
  strcpy_P(response_struct_ptr->id_string, id_struct.id_string);
  strcpy_P(response_struct_ptr->ltc_string, id_struct.ltc_string);

  // Set the Response Data Size
  *response_data_size = sizeof(DEMOBOARD_ID_MODULE_RESPONSE_TYPE);
  return DO_SEND_RESPONSE;
}
