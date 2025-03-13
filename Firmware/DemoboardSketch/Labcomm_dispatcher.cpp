/*
  Labcomm: a packet protocol over a serial link

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
#include "Labcomm_dispatcher.h"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

typedef struct
{
  LABCOMM_MODULE_ID_TYPE module_id;
  void (*init_function)(void);
  RESPONSE_ACTION (*handler_function)(uint8_t* command_data_ptr, uint16_t command_data_size, uint8_t* response_data_ptr, uint16_t* response_data_size);
} MODULE_FUNCTION_TABLE;

typedef struct __attribute__((packed))
{
  uint16_t module_ids[0];      // dest_ids supported by this demo board.
} MODULE_IDS_RESPONSE_TYPE;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
int16_t get_module_function_table_index(uint16_t module_id);
RESPONSE_ACTION Labcomm_list_all_modules(uint8_t* command_data_ptr, uint16_t command_data_size, uint8_t* response_data_ptr, uint16_t* response_data_size);

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
const MODULE_FUNCTION_TABLE module_function_table[] =
{
  {LABCOMM_MODULE_ID_MODULE_IDS, NULL, Labcomm_list_all_modules},
  {LABCOMM_MODULE_ID_DEMO_BOARD, Comm_Gui_Data_Demo_Board_Init, Labcomm_Board_ID_handler},
  {LABCOMM_MODULE_ID_SMBUS, Labcomm_SMBus_Init, Labcomm_SMBus_handler},
  {LABCOMM_MODULE_ID_NONE, NULL, NULL}
};

void Labcomm_dispatcher_init(void)
{
  uint8_t index = 0;
  do
  {
    if(NULL != module_function_table[index].init_function)
    {
      module_function_table[index].init_function();
    }
    index++;
  } while(module_function_table[index].module_id != LABCOMM_MODULE_ID_NONE);
  return;
}

RESPONSE_ACTION dispatch_Labcomm_packet(uint16_t module_id, uint8_t* command_data_ptr, uint16_t command_data_size, uint8_t* response_data_ptr, uint16_t* response_data_size)
{
  int16_t index = get_module_function_table_index(module_id);
  RESPONSE_ACTION retval = NO_RESPONSE;  // Default no response unless handler function requests it.

  // Look up handler function for module_id and call it if it exists.
  if(index > -1)
  {
    if(NULL == module_function_table[index].handler_function)
    {
      // TODO: Report a "module not implemented" error to the yet-to-be-written error module.
      retval = NO_RESPONSE;
    }
    else
    {
      retval =  module_function_table[index].handler_function(command_data_ptr, command_data_size,
                                                              response_data_ptr, response_data_size);
    }
  }
  return retval;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
int16_t get_module_function_table_index(uint16_t module_id)
{
  // Searches for destination module_id in global module_function_table.
  // Returns -1 if module_id not found in table, else
  // returns positive int16 where module_function_table[int16] is for the destination module
  const MODULE_FUNCTION_TABLE *table_ptr = module_function_table;
  int16_t index = 0;
  while(table_ptr->module_id != LABCOMM_MODULE_ID_NONE)
  {
    if(table_ptr->module_id == module_id)
    {
      return index;
    }
    table_ptr++;
    index++;
  }
  // dest_id doesn't match any module_id in the module_function_table.
  return -1;
}

RESPONSE_ACTION Labcomm_list_all_modules(uint8_t* command_data_ptr, uint16_t command_data_size,
                                         uint8_t* response_data_ptr, uint16_t* response_data_size)
{
  UNUSED(command_data_ptr);  // Arguments from the dispatcher that we don't need.
  UNUSED(command_data_size);
  MODULE_IDS_RESPONSE_TYPE* response_struct_ptr = (MODULE_IDS_RESPONSE_TYPE*)response_data_ptr;

  // Build the Response
  const MODULE_FUNCTION_TABLE *table_ptr = module_function_table;
  uint8_t index = 0;

  // Print each ID in table.
  uint16_t module_id;
  do
  {
    module_id = table_ptr->module_id;
    response_struct_ptr->module_ids[index] = Labcomm_Swap_Byte_Order_16(module_id);
    index++;
    table_ptr++;
  } while(module_id != LABCOMM_MODULE_ID_NONE);
  *response_data_size = index * sizeof(*response_struct_ptr->module_ids);

  return DO_SEND_RESPONSE;
}
