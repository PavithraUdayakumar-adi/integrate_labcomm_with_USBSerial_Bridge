/*
  An SMBus Labcomm Module (with dest_id 0x0020)

  @verbatim
  This code receives commands from incoming Labcomm packets and reads, writes, or operates on an attached SMBus interface as instructed.
  @endverbatim

  REVISION HISTORY
  $Revision: 4639 $
  $Date: 2016-01-29 16:42:40 -0500 (Fri, 29 Jan 2016) $

  Copyright (c) 2018, Linear Technology Corp.(LTC), now a part of Analog Devices, Inc.
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
#include "demoboard.h"  // Must define demoboard-specific predicate IS_SMBALERT_ACTIVE()
#include "Labcomm_Packet_Parser.h"
#include "Labcomm_dispatcher.h"
#include "Comm_SMBus.h"
#include "LT_I2C_Wire.h"

#define ID_I2C_FREQUENCY 			0x20
#define ID_I2C_READ					0x21
#define ID_I2C_WRITE 				0x22
#define ID_I2C_REPEATED_WRITE		0x23
#define ID_I2C_START 				0x24
#define ID_I2C_STOP					0x25
#define ID_I2C_WRITEBYTE 			0x26
#define ID_I2C_READBYTE				0x27
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Noun and Verb specify what type of SMBus transaction is being performed.
typedef enum
{
  LABCOMM_SMBUS_NOUN_BYTE = 0x00,              // Read/Write a byte from SMBus device.
  LABCOMM_SMBUS_NOUN_WORD_V1 = 0x01,           // Read/Write a word from SMBus device (deprecated version)
  LABCOMM_SMBUS_NOUN_BYTE_PEC = 0x02,          // Read/Write a byte from SMBus device using PEC.
  LABCOMM_SMBUS_NOUN_WORD_PEC_V1 = 0x03,       // Read/Write a word from SMBus device using PEC (deprecated version)
  LABCOMM_SMBUS_NOUN_SMBALERT = 0x04,          // Read the SMBALERT# line.
  LABCOMM_SMBUS_NOUN_RESERVED_05 = 0x05,       // Not used.
  LABCOMM_SMBUS_NOUN_RESERVED_06 = 0x06,       // Not used.
  LABCOMM_SMBUS_NOUN_WORD_LIST_PEC_V1 = 0x07,  // Stateless read word list with PEC. (deprecated tagless version)
  LABCOMM_SMBUS_NOUN_ARA = 0x08,               // Read a byte from the SMBus Alert Response Address.
  LABCOMM_SMBUS_NOUN_ARA_PEC = 0x09,           // Read a byte from the SMBus Alert Response Address using PEC.
  LABCOMM_SMBUS_NOUN_WORD_PEC_V2 = 0x0A,       // Version of SMBus Read/Write Word with added error code in the response.
  LABCOMM_SMBUS_NOUN_WORD_LIST_PEC_V2 = 0x0B,  // Stateless read word list with PEC. (current version with transaction tag)
  LABCOMM_SMBUS_NOUN_BYTE_V2 = 0x0c,           // Read/Write a byte from SMBus device (current version with transaction tag)
  LABCOMM_SMBUS_NOUN_WORD_V2 = 0x0d            // Read/Write a word from SMBus device (current version with transacation tag)
} LABCOMM_SMBUS_NOUNS;

typedef enum
{
  LABCOMM_SMBUS_VERB_READ = 0,
  LABCOMM_SMBUS_VERB_WRITE = 1,
  LABCOMM_SMBUS_VERB_RESERVED_02 = 2,
  LABCOMM_SMBUS_VERB_SEND = 3,
  LABCOMM_SMBUS_VERB_RECEIVE = 4
} LABCOMM_SMBUS_VERBS;

typedef enum
{
  LABCOMM_SMBUS_STATUS_OK = 0,
  LABCOMM_SMBUS_STATUS_I2C_ERR = 1,
  LABCOMM_SMBUS_STATUS_PEC_ERR = 2,
  LABCOMM_SMBUS_STATUS_NOT_IMPLEMENTED = 3,
  LABCOMM_SMBUS_STATUS_TODO = 255
} LABCOMM_SMBUS_STATUS;

typedef union
{
  struct __attribute__((packed))
  {
    uint8_t noun;
    uint8_t verb;
  } common; // fields common to all commands.
  struct __attribute__((packed))
  {
    uint8_t noun;
    uint8_t verb;
    uint32_t tag;
  } read_smbalert; // Read the logic state of the /SMBALERT SMBus signal.
  struct __attribute__((packed))
  {
    uint8_t noun;
    uint8_t verb;
    uint32_t tag;
  } read_ARA_PECorNot; // Read Alert Response Address (ARA) command, PEC or no PEC spec'd by noun.
  struct __attribute__((packed))
  {
    uint8_t noun;
    uint8_t verb;
    uint8_t addr7;
    uint8_t cc;
  } read_single_v1; // fields common to all commands that read from a single command code.
  struct __attribute__((packed))
  {
    uint8_t noun;
    uint8_t verb;
    uint8_t addr7;
    uint8_t cc;
    uint32_t tag;
  } read_single_v2; // commands that read from a single command code and provide a tag
                    // to be sent in the response packet.
  struct __attribute__((packed))
  {
    uint8_t noun;   // must be LABCOMM_SMBUS_NOUN_BYTE_V2 == 0x0c
    uint8_t verb;   // must be LABCOMM_SMBUS_VERB_SEND == 3
    uint8_t addr7;
    uint8_t cc;
    uint32_t tag;
  } send_byte; // SEND BYTE command. Unlike Write Byte, this sends command code only, no data.
  struct __attribute__((packed))
  {
    uint8_t noun;
    uint8_t verb;
    uint8_t addr7;
    uint8_t cc;
    uint8_t data8;
  } write_byte_v1; // Deprecated version 1 of WRITE BYTE command. (Lacks tag)
  struct __attribute__((packed))
  {
    uint8_t noun;
    uint8_t verb;
    uint8_t addr7;
    uint8_t cc;
    uint8_t data8;
    uint32_t tag;     // Opaque value copied verbatim into the response, allowing
                      // the sender to match response packet to command packet.
  } write_byte_v2; // WRITE BYTE command.
  struct __attribute__((packed))
  {
    uint8_t noun;
    uint8_t verb;
    uint8_t addr7;
    uint8_t cc;
    uint16_t data16;
    uint32_t tag;     // Opaque value copied verbatim into the response, allowing
                      // the sender to match response packet to command packet.
  } write_word_v2; // WRITE WORD command (current version)
  struct __attribute__((packed))
  {
    uint8_t noun;
    uint8_t verb;
    uint8_t addr7;
    uint8_t cc;
    uint16_t data16;
  } write_word_v1; // WRITE WORD command (deprecated version)
  struct __attribute__((packed))
  {
    uint8_t noun;
    uint8_t verb;
    uint8_t addr7;
    uint8_t cc;
    uint8_t data8;
    uint16_t pec_opt;
    uint32_t tag;     // Opaque value copied verbatim into the response, allowing
                      // the sender to match response packet to command packet.
  } write_byte_pec; // WRITE BYTE_PEC command.
  struct __attribute__((packed))
  {
    uint8_t noun;
    uint8_t verb;
    uint8_t addr7;
    uint8_t cc;
    uint16_t data16;
    uint8_t pec;
  } write_word_pec_v1; // Deprecated version 1 of WRITE WORD_PEC command.
  struct __attribute__((packed))
  {
    uint8_t noun;
    uint8_t verb;
    uint8_t addr7;
    uint8_t cc;
    uint16_t data16;
    uint16_t pec_opt;  // 0x0 to 0xff: passed through as PEC value to send on SMBus.
                       // 0x0100: automatically insert correct PEC byte to send.
                       // 0x0101 to 0xffff: undefined behavior re: PEC byte to send.
    uint32_t tag;      // Opaque value copied verbatim into the response, allowing
                       // the sender to match response packet to command packet.
                       //
  } write_word_pec_v2; // WRITE WORD_PEC command (version 2, current).
  struct __attribute__((packed))
  {
    uint8_t noun;                                         // specifies SMBus command type.
    uint8_t verb;                                         // specifies whether read/write
    uint8_t addr7;                                        // 7 bit i2c address
    uint8_t bitmap_reg[(UINT8_MAX+1)/BITS_PER_BYTE];      // bitmap of registers to read.
  } read_word_list_pec_v1; // (Deprecated version 1) READ WORD_LIST_PEC command.
  struct __attribute__((packed))
  {
    uint8_t noun;                                         // specifies SMBus command type.
    uint8_t verb;                                         // specifies whether read/write
    uint8_t addr7;                                        // 7 bit i2c address
    uint32_t tag;                                         // Opaque identifier value copied
                                                          //   verbatim to response.
    uint8_t bitmap_reg[(UINT8_MAX+1)/BITS_PER_BYTE];      // bitmap of registers to read.
  } read_word_list_pec_v2; // READ WORD_LIST_PEC command (Current version with transaction tag).
} LABCOMM_SMBUS_COMMAND_TYPE;

// typedef union{
//   struct __attribute__((packed))
//   {

//   }
// }U2SB_I2C_COMMAND_TYPE;
typedef union
{
  struct __attribute__((packed))
  {
    uint8_t noun;
    uint8_t verb;
  } common; // All responses have these fields in common.
  struct __attribute__((packed))
  {
    uint8_t noun;
    uint8_t verb;
    uint8_t active;     // State of SMBALERT#:
                        //   1 == asserted/active low,  0 == deasserted/high
    uint32_t timestamp; // Arduino micros() value: timestamp in rolling 32-bit counter.
    uint32_t tag;       // Opaque value copied verbatim into the response, allowing
                        //   the sender to match response packet to command packet.
  } smbalert; // Response to READ SMBALERT command.
  struct __attribute__((packed))
  {
    uint8_t noun;       // Must be LABCOMM_SMBUS_NOUN_ARA
    uint8_t verb;       // Must be LABCOMM_SMBUS_VERB_READ
    uint8_t responding_addr7; // Meaningful only if status == 0, else undefined.
    uint8_t active;     // State of SMBALERT# after reading the Alert Response Address (ARA)
                        // 1 == asserted/active low,  0 == deasserted/high
    uint8_t got_addr7;  //   1 if we got a response from reading the ARA.
                        //      Response is then in responding_addr7.
                        //   0 if no response to reading the ARA, in which case
                        //      responding_addr7 should be ignored.
    uint32_t timestamp; // Arduino micros() value: timestamp in rolling 32-bit counter.
    uint32_t tag;       // Opaque value copied verbatim into the response, allowing
                        // the sender to match response packet to command packet.
  } ARA; // Reply to READ ARA command.
  struct __attribute__((packed))
  {
    uint8_t noun;       // Must be LABCOMM_SMBUS_NOUN_ARA_PEC
    uint8_t verb;       // Must be LABCOMM_SMBUS_VERB_READ
    uint8_t responding_addr7; // Meaningful only if status == OK, else undefined.
    uint8_t active; // State of SMBALERT# *after* reading the Alert Response Address (ARA)
                    // 1 == asserted/active low,  0 == deasserted/high
    uint8_t status;     // == LABCOMM_SMBUS_STATUS_(OK|PEC_ERR|I2C_ERR)
    uint8_t pec;        // Received PEC value, if status is 0 or 1, else undefined.
    uint32_t timestamp; // Arduino micros() value: timestamp in rolling 32-bit counter.
    uint32_t tag;       // Opaque value copied verbatim into the response, allowing
                        // the sender to match response packet to command packet.
  } ARA_PEC; // Reply to READ ARA_PEC command
  struct __attribute__((packed))
  { 
    uint8_t noun;
    uint8_t verb;
    uint8_t addr7;
    uint8_t cc;
  } single; // fields common to all responses to commands involving a single SMBus address and command code.
  struct __attribute__((packed))
  {
    uint8_t noun;   // must be LABCOMM_SMBUS_NOUN_BYTE_V2 == 0x0c
    uint8_t verb;   // must be LABCOMM_SMBUS_VERB_SEND == 3
    uint8_t addr7;
    uint8_t cc;
    uint8_t status;     // == LABCOMM_SMBUS_STATUS_(OK|I2C_ERR)
    uint32_t timestamp; // Arduino micros() value: timestamp in rolling 32-bit counter.
    uint32_t tag;       // Opaque value copied verbatim into the response, allowing
                        // the sender to match response packet to command packet.
  } send_byte; // Reply to SEND BYTE command. Unlike Write Byte, sends command code only, no data.
  struct __attribute__((packed))
  {
    uint8_t noun;
    uint8_t verb;
    uint8_t addr7;
    uint8_t cc;
    uint8_t data8;
  } read_byte_v1; // Deprecated version 1 of Reply to READ BYTE and WRITE BYTE commands.
  struct __attribute__((packed))
  {
    uint8_t noun;
    uint8_t verb;
    uint8_t addr7;
    uint8_t cc;
    uint8_t data8;
    uint8_t status;      // == LABCOMM_SMBUS_STATUS_(OK|I2C_ERR)
    uint32_t timestamp;  // timestamp in rolling 32-bit counter.
    uint32_t tag;        // Tag value copied verbatim from command packet
                         //   causing this response packet.
  } read_byte_v2; // Reply to READ BYTE and WRITE BYTE commands.
  struct __attribute__((packed))
  {
    uint8_t noun;
    uint8_t verb;
    uint8_t addr7;
    uint8_t cc;
    uint16_t data16;
  } read_word_v1; // Reply to READ WORD and WRITE WORD commands.
  struct __attribute__((packed))
  {
    uint8_t noun;
    uint8_t verb;
    uint8_t addr7;
    uint8_t cc;
    uint16_t data16;
    uint8_t status;      // == LABCOMM_SMBUS_STATUS_(OK|I2C_ERR)
    uint32_t timestamp;  // timestamp in rolling 32-bit counter.
    uint32_t tag;        // Tag value copied verbatim from command packet
                         //   causing this response packet.
  } read_word_v2; // Reply to READ WORD and WRITE WORD commands.
  struct __attribute__((packed))
  {
    uint8_t noun;        // must be LABCOMM_SMBUS_NOUN_BYTE_PEC
    uint8_t verb;        // must be LABCOMM_SMBUS_VERB_READ
    uint8_t addr7;
    uint8_t cc;
    uint8_t data8;
    uint8_t pec;
    uint8_t status;      // == LABCOMM_SMBUS_STATUS_(OK|I2C_ERR|PEC_ERR)
    uint32_t timestamp;  // timestamp in rolling 32-bit counter.
    uint32_t tag;        // Tag value copied verbatim from command packet
                         //   causing this response packet.
  } read_byte_pec; // Reply to READ BYTE_PEC and WRITE BYTE_PEC commands.
  struct __attribute__((packed))
  {
    uint8_t noun;
    uint8_t verb;
    uint8_t addr7;
    uint8_t cc;
    uint16_t data16;
    uint8_t pec;
  } read_word_pec_v1; // (Deprecated version 1) Reply to READ WORD_PEC and WRITE WORD_PEC commands.
  struct __attribute__((packed))
  {
    uint8_t noun;       // == LABCOMM_SMBUS_NOUN_WORD_PEC_V2 = 0x0A
    uint8_t verb;
    uint8_t addr7;
    uint8_t cc;
    uint16_t data16;
    uint8_t pec;
    uint8_t status;     // == LABCOMM_SMBUS_STATUS_(OK|I2C_ERR|PEC_ERR)
    uint32_t timestamp; // timestamp in rolling 32 bit counter
    uint32_t tag;       // Tag value copied verbatim from command packet causing this response packet.
  } read_word_pec_v2;   // Reply to READ WORD_PEC and WRITE WORD_PEC commands. (Version 2, current)
  struct __attribute__((packed))
  {
    uint8_t noun;                                         // should be LABCOMM_SMBUS_NOUN_WORD_LIST_PEC_V1
    uint8_t verb;                                         // should be LABCOMM_SMBUS_VERB_READ.
    uint8_t addr7;                                        // 7 bit i2c address of target device.
    uint8_t bitmap_reg[(UINT8_MAX+1)/BITS_PER_BYTE];      // bitmap of registers read.
    uint32_t timestamp;                                   // timestamp in rolling 32 bit counter
    uint16_t data16[0];                                   // data read (no PECs just data), variable length
    uint8_t bitmap_success[(UINT8_MAX+1)/BITS_PER_BYTE];  // bitmap of success reading registers.
  } read_word_list_pec_v1; // (Deprecated version 1 without tag) Reply to READ WORD_LIST_PEC command.
  struct __attribute__((packed))
  {
    uint8_t noun;                                         // should be LABCOMM_SMBUS_NOUN_WORD_LIST_PEC_V2
    uint8_t verb;                                         // should be LABCOMM_SMBUS_VERB_READ.
    uint8_t addr7;                                        // 7 bit i2c address of target device.
    uint8_t bitmap_reg[(UINT8_MAX+1)/BITS_PER_BYTE];      // bitmap of registers read.
    uint32_t tag;                                         // Tag value copied verbatim from command packet
                                                          //   causing this response packet.
    uint32_t timestamp;                                   // timestamp in rolling 32 bit counter
    uint16_t data16[0];                                   // data read (no PECs just data), variable length
    uint8_t bitmap_success[(UINT8_MAX+1)/BITS_PER_BYTE];  // bitmap of success reading registers.
  } read_word_list_pec_v2; // Reply to READ WORD_LIST_PEC command. (Current version with transaction tag)
} LABCOMM_SMBUS_RESPONSE_TYPE;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

static LABCOMM_SMBUS_STATUS      read_smbalert(uint32_t tag, uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr);
static LABCOMM_SMBUS_STATUS read_word_list_PEC_v2(uint8_t addr7, uint8_t bitmap_reg[], uint32_t tag,
                                                  uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr);
static LABCOMM_SMBUS_STATUS read_word_list_PEC_v1(uint8_t addr7, uint8_t bitmap_reg[],
                                                  uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr);
static LABCOMM_SMBUS_STATUS   read_word_PEC_v1(uint8_t addr7, uint8_t cc,
                                               uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr);
static LABCOMM_SMBUS_STATUS   read_word_PEC_v2(uint8_t addr7, uint8_t cc, uint32_t tag,
                                               uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr);
static LABCOMM_SMBUS_STATUS      read_byte_PEC(uint8_t addr7, uint8_t cc, uint32_t tag,
                                               uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr);
static LABCOMM_SMBUS_STATUS       read_ARA_PEC(uint32_t tag, uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr);
static LABCOMM_SMBUS_STATUS  write_word_PEC_v1(uint8_t addr7, uint8_t cc, uint16_t data16, uint8_t pec,
                                               uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr);
static LABCOMM_SMBUS_STATUS  write_word_PEC_v2(uint8_t addr7, uint8_t cc, uint16_t data16, uint16_t pec_opt, uint32_t tag,
                                               uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr);
static LABCOMM_SMBUS_STATUS     write_byte_PEC(uint8_t addr7, uint8_t cc, uint8_t data8, uint16_t pec_opt, uint32_t tag,
                                               uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr);
static LABCOMM_SMBUS_STATUS   read_word_no_PEC(uint8_t addr7, uint8_t cc,
                                               uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr);
static LABCOMM_SMBUS_STATUS   read_word_no_PEC_v2(uint8_t addr7, uint8_t cc, uint32_t tag,
                                                  uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr);
static LABCOMM_SMBUS_STATUS  write_word_no_PEC_v2(uint8_t addr7, uint8_t cc, uint16_t data16, uint32_t tag,
                                                  uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr);
static LABCOMM_SMBUS_STATUS   read_byte_no_PEC(uint8_t addr7, uint8_t cc,
                                               uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr);
static LABCOMM_SMBUS_STATUS   read_byte_no_PEC_v2(uint8_t addr7, uint8_t cc, uint32_t tag,
                                                  uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr);
static LABCOMM_SMBUS_STATUS    read_ARA_no_PEC(uint32_t tag, uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr);
static LABCOMM_SMBUS_STATUS  write_word_no_PEC(uint8_t addr7, uint8_t cc, uint16_t data16,
                                               uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr);
static LABCOMM_SMBUS_STATUS  write_byte_no_PEC_v1(uint8_t addr7, uint8_t cc, uint8_t data8,
                                               uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr);
static LABCOMM_SMBUS_STATUS  write_byte_no_PEC_v2(uint8_t addr7, uint8_t cc, uint8_t data8, uint32_t tag,
                                                  uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr);
static LABCOMM_SMBUS_STATUS  send_byte_no_PEC(uint8_t addr7, uint8_t cc, uint32_t tag,
                                              uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr);

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Labcomm_SMBus_Init(void)
{
  return;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

RESPONSE_ACTION Labcomm_SMBus_handler(uint8_t* command_buf_ptr, uint16_t command_buf_len,
                                      uint8_t* response_buf_ptr, uint16_t* response_buf_len_ptr)
{
  digitalWrite(LED_AMBER, HIGH);  // Yellow LED ON to indicate SMBus module active.
  SerialUSB.println("Enter SMBus handler");

  RESPONSE_ACTION result = DO_SEND_RESPONSE;
  LABCOMM_SMBUS_STATUS status = LABCOMM_SMBUS_STATUS_NOT_IMPLEMENTED;
  LABCOMM_SMBUS_COMMAND_TYPE *cptr = (LABCOMM_SMBUS_COMMAND_TYPE *)command_buf_ptr;
  SerialUSB.println(command_buf_len);
  SerialUSB.println(cptr->common.noun);
  SerialUSB.println(cptr->common.verb);

  // Choose command handler function to call based on noun and verb.
  switch (cptr->common.verb) {
    case LABCOMM_SMBUS_VERB_READ:
      switch (cptr->common.noun) {
        case LABCOMM_SMBUS_NOUN_WORD_LIST_PEC_V2:
          SerialUSB.println("Debug: Handling READ WORD V2\n");

          status = read_word_list_PEC_v2(cptr->read_word_list_pec_v2.addr7,
                                         cptr->read_word_list_pec_v2.bitmap_reg,
                                         cptr->read_word_list_pec_v2.tag,
                                         response_buf_ptr, response_buf_len_ptr);
          break;
        case LABCOMM_SMBUS_NOUN_WORD_LIST_PEC_V1:
          SerialUSB.println("Debug: Handling READ WORD V1\n");

          status = read_word_list_PEC_v1(cptr->read_word_list_pec_v1.addr7,
                                         cptr->read_word_list_pec_v1.bitmap_reg,
                                         response_buf_ptr, response_buf_len_ptr);
          break;
        case LABCOMM_SMBUS_NOUN_WORD_PEC_V2:
          status = read_word_PEC_v2(cptr->read_single_v2.addr7, cptr->read_single_v2.cc, cptr->read_single_v2.tag,
                                    response_buf_ptr, response_buf_len_ptr);
          break;
        case LABCOMM_SMBUS_NOUN_WORD_PEC_V1:
          status = read_word_PEC_v1(cptr->read_single_v1.addr7, cptr->read_single_v1.cc,
                                    response_buf_ptr, response_buf_len_ptr);
          break;
        case LABCOMM_SMBUS_NOUN_BYTE_PEC:
          status = read_byte_PEC(cptr->read_single_v2.addr7, cptr->read_single_v2.cc, cptr->read_single_v2.tag,
                                 response_buf_ptr, response_buf_len_ptr);
          break;
        case LABCOMM_SMBUS_NOUN_WORD_V1:
          SerialUSB.println("Debug: Handling READ WORD V1\n");

          status = read_word_no_PEC(cptr->read_single_v1.addr7, cptr->read_single_v1.cc,
                                    response_buf_ptr, response_buf_len_ptr);
          break;
        case LABCOMM_SMBUS_NOUN_BYTE_V2:
          status = read_byte_no_PEC_v2(cptr->read_single_v2.addr7, cptr->read_single_v2.cc, cptr->read_single_v2.tag,
                                       response_buf_ptr, response_buf_len_ptr);
          break;
        case LABCOMM_SMBUS_NOUN_BYTE:
          status = read_byte_no_PEC(cptr->read_single_v1.addr7, cptr->read_single_v1.cc,
                                    response_buf_ptr, response_buf_len_ptr);
          break;
        case LABCOMM_SMBUS_NOUN_SMBALERT:
          status = read_smbalert(cptr->read_smbalert.tag, response_buf_ptr, response_buf_len_ptr);
          break;
        case LABCOMM_SMBUS_NOUN_ARA_PEC:
          status = read_ARA_PEC(cptr->read_ARA_PECorNot.tag, response_buf_ptr, response_buf_len_ptr);
          break;
        case LABCOMM_SMBUS_NOUN_ARA:
          status = read_ARA_no_PEC(cptr->read_ARA_PECorNot.tag, response_buf_ptr, response_buf_len_ptr);
          break;
        case LABCOMM_SMBUS_NOUN_WORD_V2:
          status = read_word_no_PEC_v2(cptr->read_single_v2.addr7, cptr->read_single_v2.cc,
                                       cptr->read_single_v2.tag,
                                       response_buf_ptr, response_buf_len_ptr);
          break;
        default:
          status = LABCOMM_SMBUS_STATUS_NOT_IMPLEMENTED;
      }
      break;
    case LABCOMM_SMBUS_VERB_WRITE:
      switch (cptr->common.noun) {
        uint16_t data16, pec16; // Network byte order to little-endianness swap buffers
        case LABCOMM_SMBUS_NOUN_WORD_PEC_V2:
          data16 = Labcomm_Swap_Byte_Order_16(cptr->write_word_pec_v2.data16);
          pec16 = Labcomm_Swap_Byte_Order_16(cptr->write_word_pec_v2.pec_opt);
          status = write_word_PEC_v2(cptr->write_word_pec_v2.addr7, cptr->write_word_pec_v2.cc, data16,
                                     pec16, cptr->write_word_pec_v2.tag, 
                                     response_buf_ptr, response_buf_len_ptr);
          break;
        case LABCOMM_SMBUS_NOUN_WORD_PEC_V1:
          data16 = Labcomm_Swap_Byte_Order_16(cptr->write_word_pec_v1.data16);
          status = write_word_PEC_v1(cptr->write_word_pec_v1.addr7, cptr->write_word_pec_v1.cc, data16,
                                     cptr->write_word_pec_v1.pec, response_buf_ptr, response_buf_len_ptr);
          break;
        case LABCOMM_SMBUS_NOUN_BYTE_PEC:
          pec16 = Labcomm_Swap_Byte_Order_16(cptr->write_byte_pec.pec_opt);
          status = write_byte_PEC(cptr->write_byte_pec.addr7, cptr->write_byte_pec.cc,
                                  cptr->write_byte_pec.data8, pec16,
                                  cptr->write_byte_pec.tag,
                                  response_buf_ptr, response_buf_len_ptr);
          break;
        case LABCOMM_SMBUS_NOUN_WORD_V1:
          data16 = Labcomm_Swap_Byte_Order_16(cptr->write_word_v1.data16);
          status = write_word_no_PEC(cptr->write_word_v1.addr7, cptr->write_word_v1.cc, data16,
                                     response_buf_ptr, response_buf_len_ptr);
          break;
        case LABCOMM_SMBUS_NOUN_WORD_V2:
          data16 = Labcomm_Swap_Byte_Order_16(cptr->write_word_v2.data16);
          status = write_word_no_PEC_v2(cptr->write_word_v2.addr7, cptr->write_word_v2.cc, data16,
                                        cptr->write_word_v2.tag,
                                        response_buf_ptr, response_buf_len_ptr);
          break;
        case LABCOMM_SMBUS_NOUN_BYTE_V2:
          status = write_byte_no_PEC_v2(cptr->write_byte_v2.addr7, cptr->write_byte_v2.cc,
                                        cptr->write_byte_v2.data8, cptr->write_byte_v2.tag,
                                        response_buf_ptr, response_buf_len_ptr);
          break;
        case LABCOMM_SMBUS_NOUN_BYTE:
          status = write_byte_no_PEC_v1(cptr->write_byte_v1.addr7, cptr->write_byte_v1.cc,
                                        cptr->write_byte_v1.data8,
                                        response_buf_ptr, response_buf_len_ptr);
          break;
        default:
          status = LABCOMM_SMBUS_STATUS_NOT_IMPLEMENTED;
      }
      break;
    case LABCOMM_SMBUS_VERB_SEND:
      switch (cptr->common.noun) {
        case LABCOMM_SMBUS_NOUN_BYTE_V2:
          status = send_byte_no_PEC(cptr->send_byte.addr7, cptr->send_byte.cc,
                                    cptr->send_byte.tag,
                                    response_buf_ptr, response_buf_len_ptr);
          break;
        default:
          status = LABCOMM_SMBUS_STATUS_NOT_IMPLEMENTED;
      }
      break;
    default:
      status = LABCOMM_SMBUS_STATUS_NOT_IMPLEMENTED;
  }
  // TODO: Report the status code to the yet-to-be-written error module.
  SerialUSB.print("status = ");SerialUSB.println(status);
  switch (status) {
    case LABCOMM_SMBUS_STATUS_OK:
      result = DO_SEND_RESPONSE;
      break;
    default:  // case any error occurs
      result = NO_RESPONSE;
  }
  SerialUSB.println("Exit SMBus handler");
  digitalWrite(LED_AMBER, LOW);  // LED off on returning from SMBus module.
  return result;
}


//
// Handler functions
//
static LABCOMM_SMBUS_STATUS read_smbalert(uint32_t tag, uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr)
{
  LABCOMM_SMBUS_RESPONSE_TYPE *rptr = (LABCOMM_SMBUS_RESPONSE_TYPE *)response_buf_ptr;
  rptr->smbalert.tag = tag;
  rptr->smbalert.active = (uint8_t)IS_SMBALERT_ACTIVE();
  rptr->smbalert.timestamp = Labcomm_Swap_Byte_Order_32(micros());
  *response_buf_len_ptr = sizeof(rptr->smbalert);
  return LABCOMM_SMBUS_STATUS_OK;
}

static LABCOMM_SMBUS_STATUS read_word_list_PEC_v1(uint8_t addr7, uint8_t bitmap_reg[],
                                                  uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr)
{
  LABCOMM_SMBUS_RESPONSE_TYPE *response_ptr = (LABCOMM_SMBUS_RESPONSE_TYPE *)response_buf_ptr;
  // Initialize response length to its minimum possible value. We'll add to it later.
  *response_buf_len_ptr = sizeof(response_ptr->read_word_list_pec_v1);
  response_ptr->read_word_list_pec_v1.noun = LABCOMM_SMBUS_NOUN_WORD_LIST_PEC_V1;
  response_ptr->read_word_list_pec_v1.verb = LABCOMM_SMBUS_VERB_READ;
  response_ptr->read_word_list_pec_v1.addr7 = addr7;
  // Since response packet is variable length, and the success bitmap comes
  // at the end of it, we cannot write directly to the response, but need
  // this intermediate buffer.
  uint8_t bitmap_success[(UINT8_MAX+1)/BITS_PER_BYTE];
  uint8_t bitmask = 1;
  uint16_t reg_count = 0;
  
  // Initialize bitmap_success to all failed.
  memset(bitmap_success, 0, sizeof(bitmap_success));
  
  // Copy bitmap of registers to read from command to response.
  memcpy(response_ptr->read_word_list_pec_v1.bitmap_reg, bitmap_reg,
  sizeof(response_ptr->read_word_list_pec_v1.bitmap_reg));
  
  // Set response timestamp.
  response_ptr->read_word_list_pec_v1.timestamp = Labcomm_Swap_Byte_Order_32(micros());
  
  // Read all registers in bitmap
  for (uint16_t reg_num = 0; reg_num <= UINT8_MAX; reg_num++) {
    if (bitmap_reg[reg_num/BITS_PER_BYTE] & bitmask) {
      uint16_t data16;
      uint8_t result = Comm_Smbus_Reg16_Read_PEC(addr7, reg_num, &data16, (port_configuration_t*)(NULL));
      response_ptr->read_word_list_pec_v1.data16[reg_count] = Labcomm_Swap_Byte_Order_16(data16);
      (*response_buf_len_ptr) += sizeof(data16);
      
      // If register read was successful, mark in bitmap_success.
      if (result == 0) {
        bitmap_success[reg_num/BITS_PER_BYTE] |= bitmask;
        } else {
        // TODO: Unfortunately Comm_Smbus_Reg16_Read_PEC() doesn't distinguish between NAKs, PEC errors, etc.
        // but we should still report an error to the yet-to-be-written error module here.
      }
      reg_count++;
    }
    bitmask = (bitmask < (1 << (BITS_PER_BYTE - 1)) ? bitmask << 1 : 1);
  }
  // Now insert that success/error bitmap at the end of the
  // variable-length array of data words in the response buffer.
  memcpy(&response_ptr->read_word_list_pec_v1.data16[reg_count], bitmap_success, sizeof(bitmap_success));
  return LABCOMM_SMBUS_STATUS_OK;  // Always send response, status recorded in bitmap_success
}

static LABCOMM_SMBUS_STATUS read_word_list_PEC_v2(uint8_t addr7, uint8_t bitmap_reg[], uint32_t tag,
                                                  uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr)
{
  LABCOMM_SMBUS_RESPONSE_TYPE *response_ptr = (LABCOMM_SMBUS_RESPONSE_TYPE *)response_buf_ptr;
  // Initialize response length to its minimum possible value. We'll add to it later.
  *response_buf_len_ptr = sizeof(response_ptr->read_word_list_pec_v2);
  response_ptr->read_word_list_pec_v2.noun = LABCOMM_SMBUS_NOUN_WORD_LIST_PEC_V2;
  response_ptr->read_word_list_pec_v2.verb = LABCOMM_SMBUS_VERB_READ;
  response_ptr->read_word_list_pec_v2.addr7 = addr7;
  response_ptr->read_word_list_pec_v2.tag = tag;
  // Since response packet is variable length, and the success bitmap comes
  // at the end of it, we cannot write directly to the response, but need
  // this intermediate buffer.
  uint8_t bitmap_success[(UINT8_MAX+1)/BITS_PER_BYTE];
  uint8_t bitmask = 1;
  uint16_t reg_count = 0;
  
  // Initialize bitmap_success to all failed.
  memset(bitmap_success, 0, sizeof(bitmap_success));
  
  // Copy bitmap of registers to read from command to response.
  memcpy(response_ptr->read_word_list_pec_v2.bitmap_reg, bitmap_reg,
         sizeof(response_ptr->read_word_list_pec_v2.bitmap_reg));
  
  // Set response timestamp.
  response_ptr->read_word_list_pec_v2.timestamp = Labcomm_Swap_Byte_Order_32(micros());
  
  // Read all registers in bitmap
  for (uint16_t reg_num = 0; reg_num <= UINT8_MAX; reg_num++) {
    if (bitmap_reg[reg_num/BITS_PER_BYTE] & bitmask) {
      uint16_t data16;
      uint8_t result = Comm_Smbus_Reg16_Read_PEC(addr7, reg_num, &data16, (port_configuration_t*)(NULL));
      response_ptr->read_word_list_pec_v2.data16[reg_count] = Labcomm_Swap_Byte_Order_16(data16);
      (*response_buf_len_ptr) += sizeof(data16);
  
      // If register read was successful, mark in bitmap_success.
      if (result == 0) {
        bitmap_success[reg_num/BITS_PER_BYTE] |= bitmask;
      } else {
        // TODO: Unfortunately Comm_Smbus_Reg16_Read_PEC() doesn't distinguish between NAKs, PEC errors, etc.
        // but we should still report an error to the yet-to-be-written error module here.
      }
      reg_count++;
    }
    bitmask = (bitmask < (1 << (BITS_PER_BYTE - 1)) ? bitmask << 1 : 1);
  }
  // Now insert that success/error bitmap at the end of the variable-length array of data words in the response buffer.
  memcpy(&response_ptr->read_word_list_pec_v2.data16[reg_count], bitmap_success, sizeof(bitmap_success));
  return LABCOMM_SMBUS_STATUS_OK;  // Always send response, status recorded in bitmap_success 
}

// This is an improved version of read_word_PEC_v1(addr7, cc, *buf, *len) that:
// 1.  reports i2c errors in the response packet by writing to an extra status field in the packet,
// 2.  reports the Arduino micros() value when the reading was taken (a rolling 32-bit timestamp),
// 3.  returns the 32-bit tag value from the command packet verbatim.
static LABCOMM_SMBUS_STATUS read_word_PEC_v2(uint8_t addr7, uint8_t cc, uint32_t tag, uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr)
{
  LABCOMM_SMBUS_RESPONSE_TYPE *rptr = (LABCOMM_SMBUS_RESPONSE_TYPE *)response_buf_ptr;
  rptr->read_word_pec_v2.noun = LABCOMM_SMBUS_NOUN_WORD_PEC_V2;
  rptr->read_word_pec_v2.verb = LABCOMM_SMBUS_VERB_READ;
  rptr->read_word_pec_v2.addr7 = addr7;
  rptr->read_word_pec_v2.cc = cc;
  rptr->read_word_pec_v2.tag = tag;
  // TODO: Replace the code below with a call to a to-be-created version
  // of Comm_Smbus_Reg16_Read_PEC() that returns both PEC and data.
  i2c_enable(COMM_SMBUS_BAUD);
  int8_t i2c_err = i2c_read_block_data(addr7, cc,
                                       sizeof(rptr->read_word_pec_v2.data16) + sizeof(rptr->read_word_pec_v2.pec),
                                       (uint8_t*)&rptr->read_word_pec_v2.data16); // SMBus Read Word PEC
  // Set response timestamp.
  rptr->read_word_pec_v2.timestamp = Labcomm_Swap_Byte_Order_32(micros());
  // Note rptr->read_word_pec_v2.data16 is now little-endian, same as i2c.
  uint8_t bytes_to_crc[] = { (uint8_t)(addr7 << 1), cc, (uint8_t)((addr7 << 1)|1),
                             (uint8_t)(rptr->read_word_pec_v2.data16 & 0xff),
                             (uint8_t)((rptr->read_word_pec_v2.data16 >> 8) & 0xff),
                             rptr->read_word_pec_v2.pec };
  int8_t pec_err = comm_smbus_crc8_calc(bytes_to_crc, sizeof(bytes_to_crc));
  pec_err = (pec_err != 0);  // Coerces pec_err == 1 if PEC check failed, 0 if PEC OK.
  // Convert data word from i2c/little-endian to Network/big endian
  rptr->read_word_pec_v2.data16 = Labcomm_Swap_Byte_Order_16(rptr->read_word_pec_v2.data16);
  // Report any errors in the status field of the response.
  // TODO: Also report errors to yet-to-be-written error module.
  if (i2c_err) {
    rptr->read_word_pec_v2.status = LABCOMM_SMBUS_STATUS_I2C_ERR;
  } else if (pec_err) {
    rptr->read_word_pec_v2.status = LABCOMM_SMBUS_STATUS_PEC_ERR;
  } else {
    rptr->read_word_pec_v2.status = LABCOMM_SMBUS_STATUS_OK;
  }
  *response_buf_len_ptr = sizeof(rptr->read_word_pec_v2);
  return LABCOMM_SMBUS_STATUS_OK;  // Return OK to caller so response with status code will always be sent.
}

// Deprecated version of Read Word PEC that doesn't report errors in the response.
static LABCOMM_SMBUS_STATUS read_word_PEC_v1(uint8_t addr7, uint8_t cc, uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr)
{
  LABCOMM_SMBUS_RESPONSE_TYPE *rptr = (LABCOMM_SMBUS_RESPONSE_TYPE *)response_buf_ptr;
  rptr->read_word_pec_v1.noun = LABCOMM_SMBUS_NOUN_WORD_PEC_V1;
  rptr->read_word_pec_v1.verb = LABCOMM_SMBUS_VERB_READ;
  rptr->read_word_pec_v1.addr7 = addr7;
  rptr->read_word_pec_v1.cc = cc;
  // TODO: Replace the code below with a call to a to-be-created version
  //of Comm_Smbus_Reg16_Read_PEC() that returns both PEC and data.
  i2c_enable(COMM_SMBUS_BAUD);
  int i2c_err = i2c_read_block_data(addr7, cc, sizeof(rptr->read_word_pec_v1.data16) + sizeof(rptr->read_word_pec_v1.pec),
                                    (uint8_t*)&rptr->read_word_pec_v1.data16); // SMBus Read Word PEC
  rptr->read_word_pec_v1.data16 = Labcomm_Swap_Byte_Order_16(rptr->read_word_pec_v1.data16); // Convert native endianness to Network/big endian
  *response_buf_len_ptr = sizeof(rptr->read_word_pec_v1);
  return (i2c_err == 0) ? LABCOMM_SMBUS_STATUS_OK : LABCOMM_SMBUS_STATUS_I2C_ERR;
}

static LABCOMM_SMBUS_STATUS read_byte_PEC(uint8_t addr7, uint8_t cc, uint32_t tag,
                                          uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr)
{
  LABCOMM_SMBUS_RESPONSE_TYPE *rptr = (LABCOMM_SMBUS_RESPONSE_TYPE *)response_buf_ptr;
  rptr->read_byte_pec.noun = LABCOMM_SMBUS_NOUN_BYTE_PEC;
  rptr->read_byte_pec.verb = LABCOMM_SMBUS_VERB_READ;
  rptr->read_byte_pec.addr7 = addr7;
  rptr->read_byte_pec.cc = cc;
  rptr->read_byte_pec.tag = tag;
  i2c_enable(COMM_SMBUS_BAUD);
  int i2c_err = i2c_read_block_data(addr7, cc,
                                    sizeof(rptr->read_byte_pec.data8) + sizeof(rptr->read_byte_pec.pec),
                                    (uint8_t*)&rptr->read_byte_pec.data8);
                                    // note above assumes &pec == &data8 + 1 in rptr->read_byte_pec
  rptr->read_byte_pec.timestamp = Labcomm_Swap_Byte_Order_32(micros());
  uint8_t bytes_to_crc[] = { (uint8_t)(addr7 << 1), cc,
                             rptr->read_byte_pec.data8, rptr->read_byte_pec.pec };
  int8_t pec_err = comm_smbus_crc8_calc(bytes_to_crc, sizeof(bytes_to_crc));
  pec_err = (pec_err != 0);  // Coerces pec_err == 1 if PEC check failed, 0 if PEC OK.
  // Report any errors in the status field of the response.
  // TODO: Also report errors to yet-to-be-written error module.
  if (i2c_err) {
    rptr->read_byte_pec.status = LABCOMM_SMBUS_STATUS_I2C_ERR;
    } else if (pec_err) {
    rptr->read_byte_pec.status = LABCOMM_SMBUS_STATUS_PEC_ERR;
    } else {
    rptr->read_byte_pec.status = LABCOMM_SMBUS_STATUS_OK;
  }
  *response_buf_len_ptr = sizeof(rptr->read_byte_pec);
  return LABCOMM_SMBUS_STATUS_OK;  // Return OK to caller so response with status code will always be sent.
}

static LABCOMM_SMBUS_STATUS read_ARA_PEC(uint32_t tag, uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr)
{
  LABCOMM_SMBUS_RESPONSE_TYPE *rptr = (LABCOMM_SMBUS_RESPONSE_TYPE *)response_buf_ptr;
  *response_buf_len_ptr = sizeof(rptr->ARA_PEC);   // Response length.
  rptr->ARA_PEC.tag = tag;
  rptr->ARA_PEC.noun = LABCOMM_SMBUS_NOUN_ARA_PEC;
  rptr->ARA_PEC.verb = LABCOMM_SMBUS_VERB_READ;
  rptr->ARA_PEC.status = 0xff;  // Nonsensical value returned if the ARA read below fails.
  uint8_t bytes_to_crc[] = {(uint8_t)((SMBUS_ALERT_RESPONSE_ADDRESS_7BITS << 1) | 1), 0x00, 0x00};
  // Read 2 bytes from the Alert Response Address (1 address byte + 1 PEC byte)
  int i2c_err = i2c_read_block_data(SMBUS_ALERT_RESPONSE_ADDRESS_7BITS, 2, bytes_to_crc + 1);
  rptr->ARA_PEC.timestamp = Labcomm_Swap_Byte_Order_32(micros());
  if (i2c_err) {
    rptr->ARA_PEC.status = LABCOMM_SMBUS_STATUS_I2C_ERR;
  } else if (comm_smbus_crc8_calc(bytes_to_crc, sizeof(bytes_to_crc)) != 0) { // Failed PEC
    rptr->ARA_PEC.status = LABCOMM_SMBUS_STATUS_PEC_ERR;
  } else {
    rptr->ARA_PEC.status = LABCOMM_SMBUS_STATUS_OK;
  }
  rptr->ARA_PEC.responding_addr7 = bytes_to_crc[1] >> 1;  // Shift right to convert byte to 7-bit address.
  rptr->ARA_PEC.pec = bytes_to_crc[2];
  rptr->ARA_PEC.active = IS_SMBALERT_ACTIVE();   // Note if SMBALERT is asserted.
  return LABCOMM_SMBUS_STATUS_OK;  // Send response packet no matter what. Error code is in the got_addr7 field.
}

LABCOMM_SMBUS_STATUS write_word_PEC_v2(uint8_t addr7, uint8_t cc, uint16_t data16, uint16_t pec_opt, uint32_t tag,
                                       uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr)
{
  // In this SMBus module, the response to write_word_pec_v2 is same as for read_word_pec_v2,
  // so we simply attempt the write, then call read_word_pec_v2 to build the response.
  //
  // data16 is 16-bit word to be sent, in little-endian byte order.
  if (pec_opt > 0x00ff) {  // Covers case of pec16 == 0x0100: automatically generate correct PEC to send on SMBus
    Comm_Smbus_Reg16_Write_PEC(addr7, cc, data16, (port_configuration_t*)(NULL));
  } else {  // Case of 0x0000 < pec16 < 0x00ff, so transmit the lower 8 bits of pec16 on the SMBus
    Comm_Smbus_Reg16_Write_PEC(addr7, cc, data16, (uint8_t)(pec_opt & 0x00ff), (port_configuration_t*)(NULL));
  }
  return read_word_PEC_v2(addr7, cc, tag, response_buf_ptr, response_buf_len_ptr);
}

// Deprecated version of Write Word PEC that doesn't report errors.
static LABCOMM_SMBUS_STATUS write_word_PEC_v1(uint8_t addr7, uint8_t cc, uint16_t data16, uint8_t pec,
                                              uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr)
{
  // In this SMBus module, the response to write_word_pec_v1 is same as for read_word_pec_v1,
  // so we simply attempt the write, then call read_word_PEC to build the response.
  //
  // data16 is the 16-bit word to be written, in little-endian byte order.
  Comm_Smbus_Reg16_Write_PEC(addr7, cc, data16, (port_configuration_t*)(NULL));
  return read_word_PEC_v1(addr7, cc, response_buf_ptr, response_buf_len_ptr);
}

static LABCOMM_SMBUS_STATUS write_byte_PEC(uint8_t addr7, uint8_t cc, uint8_t data8, uint16_t pec_opt, uint32_t tag,
                                           uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr)
{
  // TODO: Implement this and test out on a device that has byte-sized SMBus registers.
  LABCOMM_SMBUS_RESPONSE_TYPE *rptr = (LABCOMM_SMBUS_RESPONSE_TYPE *)response_buf_ptr;
  // Compute and send correct PEC if asked to do so, or use PEC specified by pec_opt.
  uint8_t bytes_to_crc[] = {(uint8_t)(addr7 << 1), cc, data8, 0};
  uint8_t *i2c_cmd_bytes = bytes_to_crc + 2;
  if (pec_opt > 0x00ff) {
    bytes_to_crc[sizeof(bytes_to_crc)-1] = comm_smbus_crc8_calc(bytes_to_crc, sizeof(bytes_to_crc)-1);
  } else {
    bytes_to_crc[sizeof(bytes_to_crc)-1] = (uint8_t)(0x00ff & pec_opt);
  }
  // i2c_write_block_data(addr7, cc, sizeof(i2c_cmd_bytes), i2c_cmd_bytes);
  i2c_write_block_data(addr7, cc, 2, i2c_cmd_bytes);
  return read_byte_PEC(addr7, cc, tag, response_buf_ptr, response_buf_len_ptr);
}

static LABCOMM_SMBUS_STATUS read_word_no_PEC_v2(uint8_t addr7, uint8_t cc, uint32_t tag,
                                                uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr)
{
  LABCOMM_SMBUS_RESPONSE_TYPE *rptr = (LABCOMM_SMBUS_RESPONSE_TYPE *)response_buf_ptr;
  *response_buf_len_ptr = (uint16_t)sizeof(rptr->read_word_v2);
  rptr->read_word_v2.noun = LABCOMM_SMBUS_NOUN_WORD_V2;
  rptr->read_word_v2.verb = LABCOMM_SMBUS_VERB_READ;
  rptr->read_word_v2.addr7 = addr7;
  rptr->read_word_v2.cc = cc;
  rptr->read_word_v2.tag = 00000000000.;
  int err = i2c_read_block_data(addr7, cc, sizeof(rptr->read_word_v2.data16), (uint8_t *)&rptr->read_word_v2.data16);
  rptr->read_byte_v2.timestamp = Labcomm_Swap_Byte_Order_32(micros());
  rptr->read_word_v2.data16 = Labcomm_Swap_Byte_Order_16(rptr->read_word_v2.data16);  // Convert from native (little-endian) to network byte order.
  rptr->read_word_v2.status = (err == 0) ? LABCOMM_SMBUS_STATUS_OK : LABCOMM_SMBUS_STATUS_I2C_ERR;
  return LABCOMM_SMBUS_STATUS_OK;   // Return OK to caller so response with status code will always be sent.
}

static LABCOMM_SMBUS_STATUS read_word_no_PEC(uint8_t addr7, uint8_t cc, uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr)
{
    LABCOMM_SMBUS_RESPONSE_TYPE *rptr = (LABCOMM_SMBUS_RESPONSE_TYPE *)response_buf_ptr;
    *response_buf_len_ptr = (uint16_t)sizeof(rptr->read_word_v1);
    rptr->read_word_v1.noun = LABCOMM_SMBUS_NOUN_WORD_V1;
    rptr->read_word_v1.verb = LABCOMM_SMBUS_VERB_READ;
    rptr->read_word_v1.addr7 = addr7;
    rptr->read_word_v1.cc = cc;
  int err = i2c_read_block_data(addr7, cc, sizeof(rptr->read_word_v1.data16), (uint8_t *)&rptr->read_word_v1.data16);
  if (err != 0)
    {
        SerialUSB.print("I2C Read Error: ");
        SerialUSB.println(err);
        return LABCOMM_SMBUS_STATUS_I2C_ERR;
    }
  rptr->read_word_v1.data16 = Labcomm_Swap_Byte_Order_16(rptr->read_word_v1.data16);  // Convert from native (little-endian) to network byte order.

  return (err == 0) ? LABCOMM_SMBUS_STATUS_OK : LABCOMM_SMBUS_STATUS_I2C_ERR;
}

static LABCOMM_SMBUS_STATUS read_byte_no_PEC(uint8_t addr7, uint8_t cc, uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr)
{
  LABCOMM_SMBUS_RESPONSE_TYPE *rptr = (LABCOMM_SMBUS_RESPONSE_TYPE *)response_buf_ptr;
  rptr->read_byte_v1.noun = LABCOMM_SMBUS_NOUN_BYTE;
  rptr->read_byte_v1.verb = LABCOMM_SMBUS_VERB_READ;
  rptr->read_byte_v1.addr7 = addr7;
  rptr->read_byte_v1.cc = cc;
  *response_buf_len_ptr = sizeof(rptr->read_byte_v1);
  int err = i2c_read_block_data(addr7, cc, sizeof(rptr->read_byte_v1.data8), &rptr->read_byte_v1.data8);
  return (err == 0) ? LABCOMM_SMBUS_STATUS_OK : LABCOMM_SMBUS_STATUS_I2C_ERR;
}

static LABCOMM_SMBUS_STATUS read_byte_no_PEC_v2(uint8_t addr7, uint8_t cc, uint32_t tag, uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr)
{
  LABCOMM_SMBUS_RESPONSE_TYPE *rptr = (LABCOMM_SMBUS_RESPONSE_TYPE *)response_buf_ptr;
  rptr->read_byte_v2.noun = LABCOMM_SMBUS_NOUN_BYTE_V2;
  rptr->read_byte_v2.verb = LABCOMM_SMBUS_VERB_READ;
  rptr->read_byte_v2.addr7 = addr7;
  rptr->read_byte_v2.cc = cc;
  rptr->read_byte_v2.tag = tag;
  *response_buf_len_ptr = sizeof(rptr->read_byte_v2);
  int err = i2c_read_block_data(addr7, cc, sizeof(rptr->read_byte_v2.data8), &rptr->read_byte_v2.data8);
  rptr->read_byte_v2.timestamp = Labcomm_Swap_Byte_Order_32(micros());
  rptr->read_byte_v2.status = (err == 0) ? LABCOMM_SMBUS_STATUS_OK : LABCOMM_SMBUS_STATUS_I2C_ERR;
  return LABCOMM_SMBUS_STATUS_OK;   // Return OK to caller so response with status code will always be sent.
}

static LABCOMM_SMBUS_STATUS send_byte_no_PEC(uint8_t addr7, uint8_t cc, uint32_t tag, uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr)
{
  LABCOMM_SMBUS_RESPONSE_TYPE *rptr = (LABCOMM_SMBUS_RESPONSE_TYPE *)response_buf_ptr;
  rptr->send_byte.noun = LABCOMM_SMBUS_NOUN_BYTE_V2;
  rptr->send_byte.verb = LABCOMM_SMBUS_VERB_SEND;
  rptr->send_byte.addr7 = addr7;
  rptr->send_byte.cc = cc;
  rptr->send_byte.tag = tag;
  *response_buf_len_ptr = sizeof(rptr->send_byte);
  int err = i2c_write_block_data(addr7, cc, 0, 0);
  rptr->send_byte.timestamp = Labcomm_Swap_Byte_Order_32(micros());
  rptr->send_byte.status = (err == 0) ? LABCOMM_SMBUS_STATUS_OK : LABCOMM_SMBUS_STATUS_I2C_ERR;
  return LABCOMM_SMBUS_STATUS_OK;   // Return OK to caller so response with status code will always be sent.
}

static LABCOMM_SMBUS_STATUS read_ARA_no_PEC(uint32_t tag, uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr)
{
  LABCOMM_SMBUS_RESPONSE_TYPE *rptr = (LABCOMM_SMBUS_RESPONSE_TYPE *)response_buf_ptr;
  *response_buf_len_ptr = sizeof(rptr->ARA);   // Response length.
  rptr->ARA.tag = tag;
  rptr->ARA.noun = LABCOMM_SMBUS_NOUN_ARA;
  rptr->ARA.verb = LABCOMM_SMBUS_VERB_READ;
  rptr->ARA.got_addr7 = 0xff;  // Nonsensical value returned if the ARA read below fails.
  // Read 1 byte from the Alert Response Address (1 address byte)
  uint8_t raw_responding_addr8 = 0xff; // Nonsensical value returned if the ARA read below fails.
  int i2c_err = i2c_read_block_data(SMBUS_ALERT_RESPONSE_ADDRESS_7BITS, 1, &raw_responding_addr8);
  rptr->ARA.timestamp = Labcomm_Swap_Byte_Order_32(micros());
  rptr->ARA.got_addr7 = (i2c_err == 0 && raw_responding_addr8 & 1) ? 1 : 0;  // Note any NAK / i2c errors. Check R/_W bit in address.
  rptr->ARA.responding_addr7 = raw_responding_addr8 >> 1;  // Convert to 7-bit address.
  rptr->ARA.active = IS_SMBALERT_ACTIVE();   // Note if SMBALERT is asserted.
  return LABCOMM_SMBUS_STATUS_OK;  // Return OK to caller so response with status code will always be sent.
}

static LABCOMM_SMBUS_STATUS write_word_no_PEC(uint8_t addr7, uint8_t cc, uint16_t data16,
                                              uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr)
{
  // In this SMBus module, the response to write_word is same as for read_word,
  // so we simply attempt the write, then call read_word() to build the response.
  //
  // data16 is the 16-bit word to be written, in network byte order (big endian).
  int err = i2c_write_block_data(addr7, cc, sizeof(data16), (uint8_t *)&data16);
  return read_word_no_PEC(addr7, cc, response_buf_ptr, response_buf_len_ptr);
}

static LABCOMM_SMBUS_STATUS write_byte_no_PEC_v1(uint8_t addr7, uint8_t cc, uint8_t data8,
                                                 uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr)
{
  // In this SMBus module, the response to write_byte is same as for read_byte,
  // so we simply attempt the write, then call read_byte() to build the response.
  int err = i2c_write_block_data(addr7, cc, sizeof(data8), &data8);
  return read_byte_no_PEC(addr7, cc, response_buf_ptr, response_buf_len_ptr);
}

static LABCOMM_SMBUS_STATUS write_byte_no_PEC_v2(uint8_t addr7, uint8_t cc, uint8_t data8, uint32_t tag,
                                                 uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr)
{
  // In this SMBus module, the response to write_byte is same as for read_byte,
  // so we simply attempt the write, then call read_byte() to build the response.
  int err = i2c_write_block_data(addr7, cc, sizeof(data8), &data8);
  return read_byte_no_PEC_v2(addr7, cc, tag, response_buf_ptr, response_buf_len_ptr);
}

static LABCOMM_SMBUS_STATUS write_word_no_PEC_v2(uint8_t addr7, uint8_t cc, uint16_t data16, uint32_t tag,
uint8_t *response_buf_ptr, uint16_t *response_buf_len_ptr)
{
  // In this SMBus module, the response to write_word is same as for read_word,
  // so we simply attempt the write, then call read_word() to build the response.
  int err = i2c_write_block_data(addr7, cc, sizeof(data16), (uint8_t *)&data16);
  return read_word_no_PEC_v2(addr7, cc, tag, response_buf_ptr, response_buf_len_ptr);
}